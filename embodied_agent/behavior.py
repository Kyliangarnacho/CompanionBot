"""Deterministic high-level authorization; the only backend is a fake in V1.

No velocity, PWM, torque, controller object or perception oracle crosses this API.
The real adapter must provide timestamped state and acknowledge execution itself.
"""
from __future__ import annotations

import asyncio
from enum import Enum
import time
from typing import Callable, Literal, Protocol
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, Field, model_validator


class Intent(str, Enum):
    FOLLOW = "FOLLOW"
    WAIT = "WAIT"
    STOP_REQUEST = "STOP_REQUEST"
    GUIDE_TO = "GUIDE_TO"


class BehaviorRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    intent: Intent
    destination: str | None = Field(default=None, min_length=1, max_length=80)

    @model_validator(mode="after")
    def destination_rule(self) -> "BehaviorRequest":
        if self.intent == Intent.GUIDE_TO:
            if self.destination is None or not self.destination.strip():
                raise ValueError("GUIDE_TO requires a destination ID")
        elif self.destination is not None:
            raise ValueError("Only GUIDE_TO accepts a destination")
        return self


class RobotSnapshot(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)
    observed_at_s: float
    connected: bool = False
    execution_ready: bool = False
    safety_ok: bool = False
    master_locked: bool = False
    backend: str = "unconnected"


class Feedback(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    status: Literal["ACCEPT", "REJECT", "CANCEL", "COMPLETED", "FAILED"]
    reason: str
    intent: Intent | None = None
    task_id: str | None = None
    backend: str = "fake"


class RobotBackend(Protocol):
    async def latest_state(self) -> RobotSnapshot: ...
    async def request(self, task_id: str, request: BehaviorRequest) -> Feedback: ...
    async def cancel(self, task_id: str) -> Feedback: ...
    async def status(self, task_id: str) -> Feedback: ...


class NavigationBackend(Protocol):
    async def start(self, task_id: str, destination: str) -> Feedback: ...
    async def cancel(self, task_id: str) -> Feedback: ...
    async def status(self, task_id: str) -> Feedback: ...


class FakeNavigationBackend:
    def __init__(self, destinations: set[str]) -> None:
        self.destinations = frozenset(destinations)
        self.tasks: dict[str, Feedback] = {}

    async def start(self, task_id: str, destination: str) -> Feedback:
        if destination not in self.destinations:
            return Feedback(status="REJECT", reason="unknown_destination", intent=Intent.GUIDE_TO)
        result = Feedback(status="ACCEPT", reason="fake_navigation_started",
                          task_id=task_id, intent=Intent.GUIDE_TO)
        self.tasks[task_id] = result
        return result

    async def status(self, task_id: str) -> Feedback:
        return self.tasks.get(task_id, Feedback(status="REJECT", reason="unknown_task", task_id=task_id))

    async def cancel(self, task_id: str) -> Feedback:
        previous = await self.status(task_id)
        if previous.status != "ACCEPT":
            return previous
        result = Feedback(status="CANCEL", reason="fake_navigation_cancelled",
                          task_id=task_id, intent=Intent.GUIDE_TO)
        self.tasks[task_id] = result
        return result

    def complete(self, task_id: str, *, success: bool = True) -> Feedback:
        previous = self.tasks[task_id]
        if previous.status != "ACCEPT":
            return previous
        result = Feedback(status="COMPLETED" if success else "FAILED",
                          reason="fake_arrival" if success else "fake_navigation_failure",
                          task_id=task_id, intent=Intent.GUIDE_TO)
        self.tasks[task_id] = result
        return result


class FakeRobotBackend:
    def __init__(self, clock: Callable[[], float] = time.perf_counter) -> None:
        self.clock = clock
        self.connected = self.execution_ready = self.safety_ok = self.master_locked = True
        self.state_age_s = 0.0
        self.tasks: dict[str, Feedback] = {}
        self.requests: list[BehaviorRequest] = []

    async def latest_state(self) -> RobotSnapshot:
        return RobotSnapshot(observed_at_s=self.clock() - self.state_age_s,
                             connected=self.connected, execution_ready=self.execution_ready,
                             safety_ok=self.safety_ok, master_locked=self.master_locked, backend="fake")

    async def request(self, task_id: str, request: BehaviorRequest) -> Feedback:
        self.requests.append(request)
        result = Feedback(status="ACCEPT", reason="fake_behavior_accepted",
                          intent=request.intent, task_id=task_id)
        self.tasks[task_id] = result
        return result

    async def cancel(self, task_id: str) -> Feedback:
        previous = await self.status(task_id)
        if previous.status != "ACCEPT":
            return previous
        result = Feedback(status="CANCEL", reason="fake_behavior_cancelled",
                          task_id=task_id, intent=previous.intent)
        self.tasks[task_id] = result
        return result

    async def status(self, task_id: str) -> Feedback:
        return self.tasks.get(task_id, Feedback(status="REJECT", reason="unknown_task", task_id=task_id))


class BehaviorSupervisor:
    def __init__(self, robot: RobotBackend, navigation: NavigationBackend, *,
                 max_state_age_s: float = 0.5, clock: Callable[[], float] = time.perf_counter) -> None:
        self.robot, self.navigation = robot, navigation
        self.max_state_age_s, self.clock = max_state_age_s, clock
        self.active_task: str | None = None
        self._owners: dict[str, RobotBackend | NavigationBackend] = {}
        self._lock = asyncio.Lock()
        self.events: list[Feedback] = []
        self.on_feedback: Callable[[Feedback], None] | None = None

    def _record(self, result: Feedback) -> Feedback:
        self.events.append(result)
        if self.on_feedback:
            self.on_feedback(result)
        return result

    async def submit(self, request: BehaviorRequest, *, authorized: Callable[[], bool] = lambda: True,
                     on_dispatch: Callable[[str], None] | None = None) -> Feedback:
        async with self._lock:
            if not authorized():
                return self._record(Feedback(status="REJECT", reason="interaction_cancelled", intent=request.intent))
            if self.active_task and request.intent not in (Intent.STOP_REQUEST, Intent.WAIT):
                try:
                    previous = await self._owners[self.active_task].status(self.active_task)
                except Exception:
                    return self._record(Feedback(status="REJECT", reason="previous_task_status_unavailable", intent=request.intent))
                if previous.status == "ACCEPT" and previous.intent not in (Intent.WAIT, Intent.STOP_REQUEST):
                    return self._record(Feedback(status="REJECT", reason="task_busy_cancel_first", intent=request.intent))
                if previous.status != "ACCEPT":
                    self._record(previous)
                    self.active_task = None
            if self.active_task:
                try:
                    cancellation = await self._owners[self.active_task].cancel(self.active_task)
                except Exception:
                    cancellation = Feedback(status="FAILED", reason="backend_cancel_failure", task_id=self.active_task)
                self._record(cancellation)
                if cancellation.status in ("CANCEL", "COMPLETED"):
                    self.active_task = None
                elif request.intent not in (Intent.STOP_REQUEST, Intent.WAIT):
                    return self._record(Feedback(status="REJECT", reason="previous_task_cancel_unconfirmed", intent=request.intent))
            # Check robot state after all potentially slow status/cancellation awaits,
            # immediately before dispatch. STOP/WAIT remain available without sensing.
            if request.intent not in (Intent.STOP_REQUEST, Intent.WAIT):
                try:
                    state = await self.robot.latest_state()
                except Exception:
                    return self._record(Feedback(status="REJECT", reason="state_unavailable", intent=request.intent))
                age = self.clock() - state.observed_at_s
                reason = None
                if not 0 <= age <= self.max_state_age_s:
                    reason = "stale_robot_state"
                elif not state.connected or not state.execution_ready or not state.safety_ok:
                    reason = "robot_not_ready"
                elif request.intent == Intent.FOLLOW and not state.master_locked:
                    reason = "master_not_locked"
                if reason:
                    return self._record(Feedback(status="REJECT", reason=reason, intent=request.intent))
            if not authorized():
                return self._record(Feedback(status="REJECT", reason="interaction_cancelled", intent=request.intent))
            task_id = uuid4().hex
            owner = self.navigation if request.intent == Intent.GUIDE_TO else self.robot
            self._owners[task_id] = owner
            if on_dispatch:
                on_dispatch(task_id)
            revoked = False

            async def commit() -> Feedback:
                try:
                    if request.intent == Intent.GUIDE_TO:
                        result = await self.navigation.start(task_id, request.destination)
                    else:
                        result = await self.robot.request(task_id, request)
                except Exception:
                    # Missing acknowledgement does not prove the backend did nothing.
                    if self.active_task is None:
                        self.active_task = task_id
                    return self._record(Feedback(status="FAILED", reason="backend_ack_unavailable",
                                                 intent=request.intent, task_id=task_id))
                self._record(result)
                if result.status == "ACCEPT":
                    # Keep an unresolved older navigation cancellation visible even
                    # after a safe STOP/WAIT request is accepted by the robot backend.
                    if self.active_task is None:
                        self.active_task = task_id
                    # Finish a dispatched command's acknowledgement despite cancellation.
                    # The real backend contract must bound acknowledgement latency and
                    # make IDs idempotent; Stage 8 only exercises the fake backend.
                    if revoked or not authorized():
                        try:
                            result = await owner.cancel(task_id)
                        except Exception:
                            result = Feedback(status="FAILED", reason="backend_cancel_failure", task_id=task_id)
                        if result.status in ("CANCEL", "COMPLETED") and self.active_task == task_id:
                            self.active_task = None
                        self._record(result)
                return result

            pending = asyncio.create_task(commit())
            try:
                return await asyncio.shield(pending)
            except asyncio.CancelledError:
                # Do not abandon an in-flight backend accept with an orphaned action.
                revoked = True
                await pending
                raise

    async def cancel(self, task_id: str | None = None) -> Feedback:
        async with self._lock:
            task_id = task_id or self.active_task
            if not task_id or task_id not in self._owners:
                return self._record(Feedback(status="REJECT", reason="no_active_task"))
            try:
                result = await self._owners[task_id].cancel(task_id)
            except Exception:
                result = Feedback(status="FAILED", reason="backend_cancel_failure", task_id=task_id)
            if result.status in ("CANCEL", "COMPLETED") and self.active_task == task_id:
                self.active_task = None
            return self._record(result)

    async def status(self, task_id: str | None = None) -> Feedback:
        task_id = task_id or self.active_task
        if not task_id or task_id not in self._owners:
            return Feedback(status="REJECT", reason="no_active_task")
        try:
            result = await self._owners[task_id].status(task_id)
        except Exception:
            return self._record(Feedback(status="FAILED", reason="backend_status_unavailable", task_id=task_id))
        if result.status != "ACCEPT" and self.active_task == task_id:
            self.active_task = None
            self._record(result)
        return result
