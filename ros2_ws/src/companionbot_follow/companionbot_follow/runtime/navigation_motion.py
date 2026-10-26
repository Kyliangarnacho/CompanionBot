"""Thin compatibility DTO adapter for the sole C++ motion authority owner.

ROS/source time and local monotonic receipt time stay separate. This module
contains no task/lease/epoch state machine and has no Python runtime fallback.
The frozen Python implementation lives only under tests/oracles for parity.
"""
from __future__ import annotations

import time

from companionbot_follow import _native_runtime
from companionbot_follow.core.rolling_reference import MotionIntent, TargetObservation


class NavigationMotionAdapter:
    def __init__(self, *, map_id: str, map_version: str,
                 max_velocity_m_s: float = .25, max_yaw_rate_rad_s: float = .5,
                 command_timeout_s: float = .3, health_timeout_s: float = .35,
                 clock=time.perf_counter, follow_config=None):
        self.clock = clock
        self.native_authority = _native_runtime.MotionAuthority(
            map_id, map_version, max_velocity_m_s, max_yaw_rate_rad_s,
            command_timeout_s, health_timeout_s, follow_config)

    @property
    def map_id(self):
        return self.native_authority.map_id

    @property
    def map_version(self):
        return self.native_authority.map_version

    @property
    def max_velocity_m_s(self):
        return self.native_authority.max_velocity_m_s

    @property
    def max_yaw_rate_rad_s(self):
        return self.native_authority.max_yaw_rate_rad_s

    @property
    def command_timeout_s(self):
        return self.native_authority.command_timeout_s

    @property
    def health_timeout_s(self):
        return self.native_authority.health_timeout_s

    def stop(self, reason="navigation_stop"):
        self.native_authority.stop(str(reason), self.clock())

    def update_health(self, available: bool, reason: str = "", *, now_s=None):
        # now_s remains diagnostic source-clock context; watchdogs use receipt.
        self.native_authority.update_health(bool(available), reason, self.clock())

    def update_mcu_status(self, state, reason):
        self.native_authority.update_mcu_status(state, reason, self.clock())

    def control(self, *, task_id: str, source_epoch: str, sequence: int,
                command: str, lease_duration_s: float, map_id: str,
                map_version: str, now_s: float):
        if isinstance(sequence, bool) or not isinstance(sequence, int) or not 0 <= sequence < 2**64:
            return dict(accepted=False, reason="invalid_navigation_identity",
                        authority_epoch=self.native_authority.snapshot(False)["authority_epoch"])
        return self.native_authority.control(
            task_id, source_epoch, sequence, command, lease_duration_s,
            map_id, map_version, now_s, self.clock())

    def receive_cmd(self, linear_m_s: float, yaw_rad_s: float, *, source_time_s: float,
                    now_s: float, source_id: str = "nav2:controller_server"):
        return self.native_authority.receive_cmd(
            linear_m_s, yaw_rad_s, source_time_s, now_s, self.clock(), source_id)

    def sample(self, now_s: float):
        intent = self.native_authority.sample(now_s, self.clock())
        return MotionIntent(intent.source_time_s, intent.linear_velocity_target_m_s,
                            intent.yaw_rate_target_rad_s)

    def take_stop(self):
        return self.native_authority.take_stop()

    def snapshot(self, *, include_events=True):
        return self.native_authority.snapshot(include_events)


class NavigationIntentReader:
    """Explicit intent carrier required by the existing planner's logging API.

    x/y encode requested v/yaw and are never measured target observations.
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
