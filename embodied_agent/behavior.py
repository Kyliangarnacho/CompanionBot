"""Deterministic high-level authorization; the only backend is a fake in V1.

No velocity, PWM, torque, controller object or perception oracle crosses this API.
The real adapter must provide timestamped state and acknowledge execution itself.
"""
from __future__ import annotations

import asyncio
from collections import OrderedDict
from enum import Enum
import json
import time
from typing import Callable, Literal, Protocol
from uuid import uuid4

from pydantic import AliasChoices, BaseModel, ConfigDict, Field, model_validator


class Intent(str, Enum):
    FOLLOW = "FOLLOW"
    WAIT = "WAIT"
    STOP_REQUEST = "STOP_REQUEST"
    GUIDE_TO = "GUIDE_TO"


class BehaviorRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    intent: Intent
    destination_id: str | None = Field(default=None, min_length=1, max_length=80,
                                     validation_alias=AliasChoices("destination_id", "destination"))
    resolution_id: str | None = None

    @model_validator(mode="before")
    @classmethod
    def encoded_object(cls, value):
        if isinstance(value, str):
            if len(value) > 4096:
                raise ValueError("tool_object_too_large")
            value = json.loads(value)
        return value

    @property
    def destination(self):
        """v1 Python compatibility; v2 serialization always uses destination_id."""
        return self.destination_id

    @model_validator(mode="after")
    def destination_rule(self) -> "BehaviorRequest":
        if self.intent == Intent.GUIDE_TO:
            if self.destination is None or not self.destination.strip():
                raise ValueError("GUIDE_TO requires a destination ID")
        elif self.destination is not None:
            raise ValueError("Only GUIDE_TO accepts a destination")
        if self.resolution_id is not None and self.intent != Intent.GUIDE_TO:
            raise ValueError("Only GUIDE_TO accepts a resolution")
        return self


class RobotSnapshot(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)
    observed_at_s: float
    connected: bool = False
    execution_ready: bool = False
    safety_ok: bool = False
    master_locked: bool = False
    backend: str = "unconnected"
    contract_version: Literal[1, 2] = 1
    source_id: str = "legacy:robot"
    source_epoch: str = "legacy"
    sequence: int | None = Field(default=None, ge=0, strict=True)
    received_at_s: float | None = None
    clock_domain: str = "host_perf_counter"
    time_semantics: str = "state_observed"
    produced_at_s: float | None = None
    produced_clock_domain: str | None = None


class Feedback(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)
    status: Literal["ACCEPT", "RUNNING", "CANCEL_REQUESTED", "UNKNOWN", "REJECT", "CANCEL", "COMPLETED", "FAILED"]
    reason: str
    intent: Intent | None = None
    task_id: str | None = None
    backend: str = "fake"
    destination_id: str | None = None
    input_id: str | None = None
    turn_id: str | None = None
    request_id: str | None = None
    progress: float | None = Field(default=None, ge=0, le=1)
    contract_version: Literal[1, 2] = 1
    source_id: str = "legacy:backend"
    source_epoch: str = "legacy"
    sequence: int | None = Field(default=None, ge=0, strict=True)
    observed_at_s: float | None = None
    received_at_s: float | None = None
    clock_domain: str = "host_perf_counter"
    time_semantics: str = "backend_event"


TERMINAL = {"CANCEL", "COMPLETED", "FAILED", "REJECT"}
IN_FLIGHT = {"ACCEPT", "RUNNING", "CANCEL_REQUESTED", "UNKNOWN"}


def retain_tasks(tasks, limit=256):
    for key in list(tasks):
        if len(tasks) <= limit:
            break
        if tasks[key].status in TERMINAL:
            del tasks[key]


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
    def __init__(self, destinations: set[str], *, cancel_immediate: bool = True,
                 clock: Callable = time.perf_counter) -> None:
        self.destinations = frozenset(destinations)
        self.tasks: dict[str, Feedback] = {}
        self.cancel_immediate = cancel_immediate
        self.clock, self.source_epoch = clock, uuid4().hex

    async def start(self, task_id: str, destination: str) -> Feedback:
        if task_id in self.tasks:
            previous = self.tasks[task_id]
            return previous if previous.destination_id == destination else Feedback(
                status="REJECT", reason="duplicate_task_conflict", task_id=task_id)
        if destination not in self.destinations:
            return Feedback(status="REJECT", reason="unknown_destination", intent=Intent.GUIDE_TO, task_id=task_id)
        if sum(r.status in IN_FLIGHT for r in self.tasks.values()) >= 256:
            return Feedback(status="REJECT", reason="backend_capacity", task_id=task_id)
        result = Feedback(status="ACCEPT", reason="fake_navigation_started",
                          task_id=task_id, intent=Intent.GUIDE_TO, destination_id=destination,
                          contract_version=2, source_id="fake:navigation", source_epoch=self.source_epoch,
                          sequence=0, observed_at_s=self.clock())
        self.tasks[task_id] = result
        retain_tasks(self.tasks)
        return result

    async def status(self, task_id: str) -> Feedback:
        result = self.tasks.get(task_id, Feedback(status="REJECT", reason="unknown_task", task_id=task_id))
        return result.model_copy(update={"observed_at_s": self.clock()})

    async def cancel(self, task_id: str) -> Feedback:
        previous = await self.status(task_id)
        if previous.status not in IN_FLIGHT:
            return previous
        if previous.status == "CANCEL_REQUESTED":
            return previous
        result = previous.model_copy(update={"status": "CANCEL" if self.cancel_immediate else "CANCEL_REQUESTED",
                                             "reason": "fake_navigation_cancelled" if self.cancel_immediate else "fake_cancel_requested",
                                             "sequence": (previous.sequence or 0) + 1, "observed_at_s": self.clock()})
        self.tasks[task_id] = result
        return result

    def acknowledge_cancel(self, task_id: str) -> Feedback:
        previous = self.tasks[task_id]
        if previous.status != "CANCEL_REQUESTED":
            return previous
        result = previous.model_copy(update={"status": "CANCEL", "reason": "fake_cancel_completed",
                                             "sequence": (previous.sequence or 0) + 1, "observed_at_s": self.clock()})
        self.tasks[task_id] = result
        return result

    def complete(self, task_id: str, *, success: bool = True) -> Feedback:
        previous = self.tasks[task_id]
        if previous.status not in IN_FLIGHT:
            return previous
        result = previous.model_copy(update={"status": "COMPLETED" if success else "FAILED",
                          "reason": "fake_arrival" if success else "fake_navigation_failure",
                          "sequence": (previous.sequence or 0) + 1, "progress": 1.0 if success else previous.progress, "observed_at_s": self.clock()})
        self.tasks[task_id] = result
        return result

    def advance(self, task_id: str, progress: float) -> Feedback:
        previous = self.tasks[task_id]
        if previous.status not in ("ACCEPT", "RUNNING"):
            return previous
        result = Feedback.model_validate(previous.model_dump() | {
            "status": "RUNNING", "reason": "fake_navigation_progress", "progress": progress,
            "sequence": (previous.sequence or 0) + 1, "observed_at_s": self.clock()})
        self.tasks[task_id] = result
        return result


class FakeRobotBackend:
    def __init__(self, clock: Callable[[], float] = time.perf_counter) -> None:
        self.clock = clock
        self.connected = self.execution_ready = self.safety_ok = self.master_locked = True
        self.state_age_s = 0.0
        self.tasks: dict[str, Feedback] = {}
        self.requests: list[BehaviorRequest] = []
        self.source_epoch = uuid4().hex
        self.sequence = 0

    async def latest_state(self) -> RobotSnapshot:
        self.sequence += 1
        return RobotSnapshot(observed_at_s=self.clock() - self.state_age_s,
                             connected=self.connected, execution_ready=self.execution_ready,
                             safety_ok=self.safety_ok, master_locked=self.master_locked, backend="fake",
                             source_id="fake:robot", source_epoch=self.source_epoch, sequence=self.sequence,
                             received_at_s=self.clock(), contract_version=2)

    async def request(self, task_id: str, request: BehaviorRequest) -> Feedback:
        if task_id in self.tasks:
            return self.tasks[task_id] if self.tasks[task_id].intent == request.intent else Feedback(
                status="REJECT", reason="duplicate_task_conflict", task_id=task_id)
        self.requests.append(request)
        self.requests[:] = self.requests[-256:]
        result = Feedback(status="ACCEPT", reason="fake_behavior_accepted",
                          intent=request.intent, task_id=task_id, sequence=0,
                          contract_version=2, source_id="fake:robot", source_epoch=self.source_epoch,
                          observed_at_s=self.clock())
        self.tasks[task_id] = result
        retain_tasks(self.tasks)
        return result

    async def cancel(self, task_id: str) -> Feedback:
        previous = await self.status(task_id)
        if previous.status not in IN_FLIGHT:
            return previous
        result = previous.model_copy(update={"status": "CANCEL", "reason": "fake_behavior_cancelled",
                                             "sequence": (previous.sequence or 0) + 1, "observed_at_s": self.clock()})
        self.tasks[task_id] = result
        return result

    async def status(self, task_id: str) -> Feedback:
        result = self.tasks.get(task_id, Feedback(status="REJECT", reason="unknown_task", task_id=task_id))
        return result.model_copy(update={"observed_at_s": self.clock()})


class BehaviorSupervisor:
    """Bounded task ledger. Generation lifetime never represents task completion.

    Backend adapters must be cooperative with deadline cancellation and idempotent
    on task_id. Timeout means UNKNOWN, never proof that no motion was started.
    """
    def __init__(self, robot: RobotBackend, navigation: NavigationBackend, *,
                 max_state_age_s: float = 0.5, clock: Callable[[], float] = time.perf_counter,
                 backend_timeout_s: float = 2, max_tasks: int = 256, task_timeout_s: float = 300) -> None:
        self.robot, self.navigation = robot, navigation
        self.max_state_age_s, self.clock = max_state_age_s, clock
        self.backend_timeout_s, self.max_tasks = backend_timeout_s, max_tasks
        self.task_timeout_s = task_timeout_s
        self.active_task: str | None = None
        self._owners: dict[str, RobotBackend | NavigationBackend] = {}
        self._requests: dict[str, BehaviorRequest] = {}
        self._contexts: dict[str, dict] = {}
        self._results: OrderedDict[str, Feedback] = OrderedDict()
        self._dedup: dict[str, str] = {}
        self._state_watermark = None
        self._feedback_watermarks: dict[str, tuple] = {}
        self._started_at: dict[str, float] = {}
        self._expired: set[str] = set()
        self.destination_valid: Callable[[str], bool] = lambda d: d in getattr(navigation, 'destinations', ())
        self._lock = asyncio.Lock()
        self.events: list[Feedback] = []
        self.on_feedback: Callable[[Feedback], None] | None = None

    def _record(self, result: Feedback) -> Feedback:
        self.events.append(result)
        self.events[:] = self.events[-512:]
        if self.on_feedback:
            self.on_feedback(result)
        return result

    def _unknown(self, task_id, reason):
        return Feedback(status='UNKNOWN', reason=reason, task_id=task_id,
                        intent=self._requests[task_id].intent,
                        destination_id=self._requests[task_id].destination_id,
                        received_at_s=self.clock(), **self._contexts[task_id])

    def _apply(self, task_id: str, result: Feedback) -> Feedback:
        """Reject mismatched/stale events before they can mutate the active task."""
        request = self._requests[task_id]
        prior = self._results.get(task_id)
        invalid = (result.task_id != task_id or result.intent not in (None, request.intent)
                   or result.destination_id not in (None, request.destination_id)
                   or result.clock_domain != 'host_perf_counter'
                   or any(getattr(result, k) not in (None, v) for k, v in self._contexts[task_id].items())
                   or result.contract_version == 2 and (result.sequence is None or result.intent != request.intent or
                      request.intent == Intent.GUIDE_TO and result.destination_id != request.destination_id))
        if result.observed_at_s is not None and not 0 <= self.clock() - result.observed_at_s <= self.backend_timeout_s:
            invalid = True
        reason = 'backend_feedback_id_or_clock_mismatch' if invalid else None
        if prior and prior.contract_version == 2 and result.contract_version == 1 and result.status != 'UNKNOWN':
            reason = 'backend_contract_downgrade'
        watermark = self._feedback_watermarks.get(task_id)
        if not reason and result.contract_version == 2:
            stamp = (result.source_id, result.source_epoch, result.sequence)
            if watermark and stamp[:2] != watermark[:2]:
                reason = 'backend_source_restart_requires_reconciliation'
            elif watermark and stamp[2] < watermark[2]:
                reason = 'stale_or_duplicate_feedback'
        if prior and not reason:
            if prior.status in TERMINAL and result.status != prior.status:
                reason = 'terminal_task_feedback'
            elif prior.source_id == result.source_id and prior.source_epoch == result.source_epoch:
                if prior.sequence is not None and result.sequence is not None and result.sequence <= prior.sequence:
                    # Status polling returns an unchanged snapshot; it is idempotent.
                    if result.sequence == prior.sequence and result.status == prior.status and result.progress == prior.progress:
                        return prior
                    reason = 'stale_or_duplicate_feedback'
            elif prior.contract_version == 2 and result.contract_version == 2:
                reason = 'backend_source_restart_requires_reconciliation'
            if prior.status == 'CANCEL_REQUESTED' and result.status in ('ACCEPT', 'RUNNING'):
                reason = 'cancel_pending_feedback'
            if prior.status == 'RUNNING' and result.status == 'ACCEPT':
                reason = 'backward_task_feedback'
            if prior.progress is not None and result.progress is not None and result.progress < prior.progress:
                reason = 'backward_task_progress'
        if reason:
            self._record(Feedback(status='REJECT', reason=reason, task_id=task_id,
                                  intent=request.intent, **self._contexts[task_id]))
            if prior:
                return prior
            result = self._unknown(task_id, reason)
        result = result.model_copy(update={**self._contexts[task_id], 'intent': request.intent,
            'destination_id': request.destination_id, 'received_at_s': self.clock()})
        self._results[task_id] = result
        if result.contract_version == 2:
            self._feedback_watermarks[task_id] = (result.source_id, result.source_epoch, result.sequence)
        if result.status in TERMINAL and self.active_task == task_id:
            self.active_task = None
            self.active_task = next((key for key, value in reversed(self._results.items())
                                     if key != task_id and value.status in IN_FLIGHT), None)
        if prior != result:
            self._record(result)
        return result

    async def _backend_call(self, task_id, operation):
        try:
            result = await asyncio.wait_for(operation, self.backend_timeout_s)
        except Exception:
            result = self._unknown(task_id, 'backend_ack_unavailable')
        if not isinstance(result, Feedback):
            result = self._unknown(task_id, 'backend_feedback_contract_mismatch')
        return self._apply(task_id, result)

    def _prune(self):
        for key in list(self._results):
            if len(self._owners) < self.max_tasks:
                break
            if self._results[key].status in TERMINAL and key != self.active_task:
                del self._results[key]
                self._owners.pop(key, None)
                self._requests.pop(key, None)
                context = self._contexts.pop(key, {})
                self._dedup.pop(context.get('request_id'), None)
                self._feedback_watermarks.pop(key, None)
                self._started_at.pop(key, None)
                self._expired.discard(key)

    def _state_reason(self, state, intent):
        if state.clock_domain != 'host_perf_counter' or state.time_semantics != 'state_observed':
            return 'robot_clock_unmapped'
        if not 0 <= self.clock() - state.observed_at_s <= self.max_state_age_s:
            return 'stale_robot_state'
        if state.contract_version == 2 and state.sequence is None:
            return 'robot_state_sequence_missing'
        if state.sequence is not None:
            stamp = (state.source_id, state.source_epoch, state.sequence, state.observed_at_s)
            previous = self._state_watermark
            if previous and stamp[:2] == previous[:2] and (stamp[2] <= previous[2] or stamp[3] < previous[3]):
                return 'duplicate_or_out_of_order_robot_state'
            if previous and stamp[:2] != previous[:2] and self.active_task:
                return 'robot_source_restart_during_task'
            self._state_watermark = stamp
        if not state.connected or not state.execution_ready or not state.safety_ok:
            return 'robot_not_ready'
        if intent == Intent.FOLLOW and not state.master_locked:
            return 'master_not_locked'
        return None

    async def submit(self, request: BehaviorRequest, *, authorized: Callable[[], bool] = lambda: True,
                     target_authorized: Callable[[], bool] | None = None,
                     on_dispatch: Callable[[str], None] | None = None,
                     input_id: str | None = None, turn_id: str | None = None,
                     request_id: str | None = None) -> Feedback:
        async with self._lock:
            context = dict(input_id=input_id, turn_id=turn_id, request_id=request_id)
            def reject(reason):
                return self._record(Feedback(status='REJECT', reason=reason, intent=request.intent,
                                            destination_id=request.destination_id, **context))
            if not authorized():
                return reject('interaction_cancelled')
            if request_id in self._dedup:
                existing = self._dedup[request_id]
                if self._requests[existing] != request:
                    return reject('duplicate_request_conflict')
                return self._results[existing]
            if request.intent == Intent.GUIDE_TO:
                if not self.destination_valid(request.destination_id):
                    return reject('unknown_destination')
                if target_authorized is not None and not target_authorized():
                    return reject('unresolved_or_expired_destination')
            safe = request.intent in (Intent.STOP_REQUEST, Intent.WAIT)
            if self.active_task and not safe:
                old_id = self.active_task
                previous = await self._backend_call(old_id, self._owners[old_id].status(old_id))
                if previous.status in IN_FLIGHT and previous.intent not in (Intent.WAIT, Intent.STOP_REQUEST):
                    return reject('task_busy_cancel_first')
            if self.active_task:
                old_id = self.active_task
                cancellation = await self._backend_call(old_id, self._owners[old_id].cancel(old_id))
                if cancellation.status not in TERMINAL and not safe:
                    return reject('previous_task_cancel_unconfirmed')
            if not safe:
                try:
                    state = await asyncio.wait_for(self.robot.latest_state(), self.backend_timeout_s)
                except Exception:
                    return reject('state_unavailable')
                reason = self._state_reason(state, request.intent)
                if reason:
                    return reject(reason)
            if not authorized():
                return reject('interaction_cancelled')
            if target_authorized is not None and not target_authorized():
                return reject('unresolved_or_expired_destination')
            self._prune()
            if len(self._owners) >= self.max_tasks:
                return reject('supervisor_capacity')
            task_id = uuid4().hex
            owner = self.navigation if request.intent == Intent.GUIDE_TO else self.robot
            self._owners[task_id], self._requests[task_id], self._contexts[task_id] = owner, request, context
            self._started_at[task_id] = self.clock()
            if request_id:
                self._dedup[request_id] = task_id
            if self.active_task is None:
                self.active_task = task_id
            if on_dispatch:
                on_dispatch(task_id)
            revoked = False
            async def commit():
                operation = (self.navigation.start(task_id, request.destination_id) if request.intent == Intent.GUIDE_TO
                             else self.robot.request(task_id, request))
                result = await self._backend_call(task_id, operation)
                if result.status in IN_FLIGHT and (revoked or not authorized()):
                    result = await self._backend_call(task_id, owner.cancel(task_id))
                return result
            pending = asyncio.create_task(commit())
            try:
                return await asyncio.shield(pending)
            except asyncio.CancelledError:
                revoked = True
                await pending
                raise

    async def cancel(self, task_id: str | None = None) -> Feedback:
        async with self._lock:
            task_id = task_id or self.active_task
            if not task_id or task_id not in self._owners:
                return self._record(Feedback(status='REJECT', reason='no_active_task'))
            if self._results.get(task_id) and self._results[task_id].status in TERMINAL:
                return self._results[task_id]
            return await self._backend_call(task_id, self._owners[task_id].cancel(task_id))

    async def status(self, task_id: str | None = None) -> Feedback:
        async with self._lock:
            task_id = task_id or self.active_task
            if not task_id or task_id not in self._owners:
                return Feedback(status='REJECT', reason='no_active_task')
            return await self._backend_call(task_id, self._owners[task_id].status(task_id))

    async def receive_feedback(self, result: Feedback) -> Feedback:
        """Adapter callback ingress, independent of the Agent or session turn."""
        async with self._lock:
            if not result.task_id or result.task_id not in self._owners:
                return self._record(Feedback(status='REJECT', reason='unknown_or_expired_task_feedback',
                                            task_id=result.task_id))
            return self._apply(result.task_id, result)

    async def poll(self):
        # A safe STOP may coexist with an unresolved older navigation task.
        for task_id in list(self._owners):
            if self._results.get(task_id) and self._results[task_id].status in IN_FLIGHT:
                if task_id not in self._expired and self.clock() - self._started_at[task_id] >= self.task_timeout_s:
                    self._expired.add(task_id)
                    self._record(Feedback(status='CANCEL_REQUESTED', reason='task_deadline_expired', task_id=task_id,
                                          intent=self._requests[task_id].intent, **self._contexts[task_id]))
                    await self.cancel(task_id)
                else:
                    await self.status(task_id)

    async def cancel_all(self):
        for task_id in list(self._owners):
            if self._results.get(task_id) and self._results[task_id].status in IN_FLIGHT:
                await self.cancel(task_id)
