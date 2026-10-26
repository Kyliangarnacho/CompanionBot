"""Task-scoped Nav2 intent gate. No ROS, actuator, pose, or estimator authority.

Nav2 supplies unshaped body-forward velocity / left-positive yaw rate. The
existing Pi Ruckig / ReferenceBlock chain remains the sole trajectory owner.
All leases and receive watchdogs use this process's monotonic clock; stamped
command freshness additionally uses the explicitly shared simulation clock.
"""
from __future__ import annotations

from collections import deque
import math
import threading
import time

from companionbot_follow.core.rolling_reference import MotionIntent, TargetObservation


class NavigationMotionAdapter:
    def __init__(self, *, map_id: str, map_version: str,
                 max_velocity_m_s: float = .25, max_yaw_rate_rad_s: float = .5,
                 command_timeout_s: float = .3, health_timeout_s: float = .35,
                 clock=time.perf_counter):
        if (not map_id or not map_version or not all(math.isfinite(v) and v > 0 for v in
                (max_velocity_m_s, max_yaw_rate_rad_s, command_timeout_s, health_timeout_s))
                or max_velocity_m_s > .6 or max_yaw_rate_rad_s > .5):
            raise ValueError("invalid_navigation_envelope")
        self.map_id, self.map_version = map_id, map_version
        self.max_velocity_m_s, self.max_yaw_rate_rad_s = max_velocity_m_s, max_yaw_rate_rad_s
        self.command_timeout_s, self.health_timeout_s = command_timeout_s, health_timeout_s
        self.clock, self._lock = clock, threading.RLock()
        self.authority_epoch = 0
        self.task_id = self.source_epoch = None
        self._watermarks = {}
        self._lease_until = 0.
        self._granted_source_time = 0.
        self._latest = None
        self._healthy = False
        self._health_reason, self._health_at = "navigation_health_unavailable", -math.inf
        self._stop_serial = 0
        self._pending_stop = None
        self.reason = "no_navigation_authority"
        self._mcu_state, self._mcu_reason = None, "mcu_status_unavailable"
        self._owned_stop_reasons = set()
        self.events = deque(maxlen=2048)

    def _event(self, event, **values):
        self.events.append(dict(event=event, received_at_s=self.clock(), **values))

    def _stop(self, reason, *, force=False):
        had_authority = self.task_id is not None
        self.task_id = self.source_epoch = None
        self._latest = None
        self._lease_until = 0.
        self.reason = reason
        if had_authority or force:
            self.authority_epoch += 1
            self._stop_serial += 1
            self._pending_stop = (self._stop_serial, reason)
            self._owned_stop_reasons.add(reason)
            self._event("navigation_authority_revoked", reason=reason,
                        authority_epoch=self.authority_epoch)

    def stop(self, reason="navigation_stop"):
        """Revoke immediately; the packet link sends HARD safety and clears epochs."""
        with self._lock:
            self._stop(str(reason))

    def update_health(self, available: bool, reason: str = "", *, now_s=None):
        """Caller owns scan / odom / localization validation, never GT.

        ``now_s`` is accepted only as diagnostic source-clock context. Watchdog
        age always uses local monotonic receipt time.
        """
        with self._lock:
            self._healthy, self._health_reason = bool(available), reason or "navigation_health_unavailable"
            self._health_at = self.clock()
            if not self._healthy:
                self._stop(self._health_reason)

    def update_mcu_status(self, state, reason):
        """Simulation link supplies actual received MCU safety status, not GT."""
        with self._lock:
            self._mcu_state, self._mcu_reason = state, reason
            if state != "NORMAL" and reason not in self._owned_stop_reasons:
                self._stop("navigation_mcu_safety:" + str(reason))

    def _mcu_ready(self):
        return (self._mcu_state == "NORMAL" or self._mcu_state == "SAFE_HOLD"
                and self._mcu_reason in self._owned_stop_reasons)

    def control(self, *, task_id: str, source_epoch: str, sequence: int,
                command: str, lease_duration_s: float, map_id: str,
                map_version: str, now_s: float):
        """Typed service implementation. ACQUIRE is explicit software rearm intent.

        It cannot clear independent MCU safety or physical gates. After a fault
        a new ACQUIRE and a command newer than the grant are required.
        """
        with self._lock:
            def answer(ok, reason):
                return dict(accepted=ok, reason=reason, authority_epoch=self.authority_epoch)
            if (not task_id or len(task_id) > 80 or not source_epoch or len(source_epoch) > 128
                    or isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0
                    or not math.isfinite(now_s)):
                return answer(False, "invalid_navigation_identity")
            key = (task_id, source_epoch)
            if sequence <= self._watermarks.get(key, -1):
                return answer(False, "stale_navigation_control")
            if command not in ("CHECK", "CHECK_STOPPED", "ACQUIRE", "RENEW", "RELEASE", "STOP", "WAIT"):
                return answer(False, "unknown_navigation_control")
            self._watermarks[key] = sequence
            if len(self._watermarks) > 256:
                self._watermarks.pop(next(iter(self._watermarks)))
            if command in ("STOP", "WAIT"):
                self._stop("navigation_" + command.lower(), force=True)
                return answer(True, "navigation_authority_stopped")
            if (map_id, map_version) != (self.map_id, self.map_version):
                return answer(False, "navigation_map_version_mismatch")
            if command == "CHECK_STOPPED":
                return answer(self._mcu_state == "SAFE_HOLD" and self.task_id is None,
                              "mcu_safe_hold" if self._mcu_state == "SAFE_HOLD" else "mcu_stop_pending")
            if command == "CHECK":
                self._check(now_s)
                healthy = self._healthy and self.clock() - self._health_at <= self.health_timeout_s
                return answer(healthy and self._mcu_ready(), "navigation_ready" if healthy and self._mcu_ready()
                              else self._health_reason if not healthy else self._mcu_reason)
            owner = (self.task_id, self.source_epoch)
            if command == "RELEASE":
                if owner != key:
                    return answer(False, "navigation_owner_mismatch")
                self._stop("navigation_task_released")
                return answer(True, "navigation_authority_released")
            if not math.isfinite(lease_duration_s) or not .1 <= lease_duration_s <= 2.:
                return answer(False, "invalid_navigation_lease")
            self._check(now_s)
            if not self._healthy or self.clock() - self._health_at > self.health_timeout_s:
                return answer(False, self._health_reason)
            if not self._mcu_ready():
                return answer(False, self._mcu_reason)
            if self.task_id is not None and owner != key:
                return answer(False, "motion_authority_busy")
            if command == "RENEW" and owner != key:
                return answer(False, "navigation_owner_mismatch")
            if self.task_id is None:
                self.authority_epoch += 1
                self.task_id, self.source_epoch = key
                self._granted_source_time = now_s
                self._latest = None
                self._event("navigation_authority_acquired", task_id=task_id,
                            source_epoch=source_epoch, authority_epoch=self.authority_epoch)
            self._lease_until = self.clock() + lease_duration_s
            self.reason = "navigation_authorized"
            return answer(True, self.reason)

    def _check(self, now_s):
        if self.task_id is None:
            return
        if self.clock() >= self._lease_until:
            self._stop("navigation_lease_expired")
        elif not self._healthy or self.clock() - self._health_at > self.health_timeout_s:
            self._stop(self._health_reason if not self._healthy else "navigation_health_timeout")
        elif self._latest is not None:
            stamp, received, _, _ = self._latest
            if now_s < stamp - .02:
                self._stop("navigation_clock_discontinuity")
            elif now_s - stamp > self.command_timeout_s or self.clock() - received > self.command_timeout_s:
                self._stop("navigation_command_expired")

    def receive_cmd(self, linear_m_s: float, yaw_rad_s: float, *, source_time_s: float,
                    now_s: float, source_id: str = "nav2:controller_server"):
        """Ingress after ROS Graph publisher / frame / nonplanar-component checks.

        No queue exists. Rejected or pre-grant messages cannot later execute.
        Nav2 reverse is disabled; a reverse request is a fault, never silently
        converted into a different motion. Saturation remains visible.
        """
        with self._lock:
            if not all(math.isfinite(v) for v in (linear_m_s, yaw_rad_s, source_time_s, now_s)):
                self._stop("invalid_navigation_twist")
                return False
            self._check(now_s)
            if self.task_id is None:
                return False
            if (source_time_s < self._granted_source_time - 1e-9
                    or not -.02 <= now_s - source_time_s <= self.command_timeout_s
                    or self._latest and source_time_s <= self._latest[0]):
                self._event("navigation_command_rejected", reason="stale_or_unmapped_command",
                            source_time_s=source_time_s, task_id=self.task_id)
                return False
            if linear_m_s < -1e-9:
                self._stop("navigation_reverse_not_authorized")
                return False
            linear = max(0., min(self.max_velocity_m_s, linear_m_s))
            yaw = max(-self.max_yaw_rate_rad_s, min(self.max_yaw_rate_rad_s, yaw_rad_s))
            self._latest = (source_time_s, self.clock(), linear, yaw)
            self._event("navigation_command", task_id=self.task_id, source_id=source_id,
                        source_time_s=source_time_s, requested_v_m_s=linear_m_s,
                        requested_yaw_rad_s=yaw_rad_s, limited_v_m_s=linear,
                        limited_yaw_rad_s=yaw, saturated=(linear != linear_m_s or yaw != yaw_rad_s))
            return True

    def sample(self, now_s: float):
        with self._lock:
            self._check(now_s)
            v, yaw = (self._latest[2:] if self.task_id and self._latest else (0., 0.))
            source_time = self._latest[0] if self.task_id and self._latest else float(now_s)
            return MotionIntent(source_time, v, yaw)

    def take_stop(self):
        with self._lock:
            result, self._pending_stop = self._pending_stop, None
            return result

    def snapshot(self, *, include_events=True):
        with self._lock:
            result = dict(task_id=self.task_id, source_epoch=self.source_epoch,
                        authority_epoch=self.authority_epoch, reason=self.reason,
                        mcu_state=self._mcu_state, mcu_reason=self._mcu_reason,
                        map_id=self.map_id, map_version=self.map_version,
                        authorized=self.task_id is not None, retained_events=len(self.events))
            if include_events:
                result["events"] = list(self.events)
            return result


class NavigationIntentReader:
    """Explicit intent carrier required by the existing planner's logging API.

    x/y below encode requested v/yaw, as in existing mapping_motion. They are
    never published as measured target observations or consumed by perception.
    """
    def __init__(self, adapter):
        self.adapter, self.time_s = adapter, 0.

    def read(self):
        intent = self.adapter.sample(self.time_s)
        return TargetObservation(intent.source_time_s, intent.linear_velocity_target_m_s,
                                 intent.yaw_rate_target_rad_s)

    @staticmethod
    def intent(carrier):
        return MotionIntent(carrier.capture_time_s, carrier.x_forward_m, carrier.y_left_m)
