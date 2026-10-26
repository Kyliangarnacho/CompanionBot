"""Opt-in stock Nav2 Action adapter for the existing Supervisor contracts.

No map coordinates come from the language model. No velocity or actuator API
exists here. The server-local motion lease is revoked before Action cancellation.
"""
import asyncio
import math
import threading
import time
from uuid import uuid4

from companionbot_behavior.core.behavior import Feedback, Intent, RobotSnapshot, IN_FLIGHT, retain_tasks
from companionbot_contracts.navigation_map import load_map_binding as load_bindings


class Nav2NavigationBackend:
    def progress_from_distance(self, task_id, distance):
        if not math.isfinite(distance) or distance < 0:
            return None
        if distance > .01:
            self.initial_distance.setdefault(task_id, distance)
        baseline = self.initial_distance.get(task_id)
        return None if baseline is None else max(0., min(.99, 1.-distance/baseline))

    def __init__(self, bindings, *, use_sim_time=True):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose
        from companionbot_interfaces.srv import NavigationControl
        from lifecycle_msgs.srv import GetState
        self.binding = load_bindings(bindings)
        self.destinations = frozenset(self.binding["destinations"])
        self.tasks, self.handles, self.monitors = {}, {}, {}
        self.source_epoch, self.sequence, self.control_sequence = uuid4().hex, 0, 0
        self.error, self.active = None, None
        self._events_lock = threading.RLock()
        self.initial_distance = {}
        self.context = Context(); rclpy.init(args=[], context=self.context)
        self.node = Node("agent_navigation_backend", namespace="companionbot", context=self.context,
            parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)])
        self.action = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")
        self.control_client = self.node.create_client(NavigationControl, "/companionbot/navigation/control")
        self.lifecycle_clients = [self.node.create_client(GetState, "/"+name+"/get_state")
            for name in ("map_server", "amcl", "planner_server", "controller_server", "bt_navigator")]
        from visualization_msgs.msg import MarkerArray
        from rclpy.qos import QoSProfile, DurabilityPolicy
        self.targets_pub = self.node.create_publisher(MarkerArray, "/companionbot/navigation/targets",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.targets_published = False
        self.executor = SingleThreadedExecutor(context=self.context); self.executor.add_node(self.node)
        def spin():
            try:
                self.executor.spin()
            except BaseException as error:
                self.error = error
        self.thread = threading.Thread(target=spin, name="nav2-action-executor", daemon=True)
        self.thread.start()

    def publish_targets(self):
        if self.targets_published:
            return
        from visualization_msgs.msg import Marker, MarkerArray
        markers=[]
        for index,(name,target) in enumerate(self.binding["destinations"].items()):
            arrow=Marker();arrow.header.frame_id="map";arrow.header.stamp=self.node.get_clock().now().to_msg()
            arrow.ns=self.binding["map_id"]+":"+self.binding["map_version"];arrow.id=index*2;arrow.type=Marker.ARROW
            arrow.pose.position.x=float(target["x_m"]);arrow.pose.position.y=float(target["y_m"]);arrow.pose.position.z=.03
            arrow.pose.orientation.z=math.sin(target["yaw_rad"]/2);arrow.pose.orientation.w=math.cos(target["yaw_rad"]/2)
            arrow.scale.x,arrow.scale.y,arrow.scale.z=.35,.06,.06
            arrow.color.r,arrow.color.g,arrow.color.b,arrow.color.a=.95,.55,.1,1.
            text=Marker();text.header=arrow.header;text.ns=arrow.ns;text.id=index*2+1;text.type=Marker.TEXT_VIEW_FACING
            text.pose.position.x=arrow.pose.position.x;text.pose.position.y=arrow.pose.position.y;text.pose.position.z=.28
            text.pose.orientation.w=1.;text.scale.z=.16;text.text=name
            text.color.r,text.color.g,text.color.b,text.color.a=.95,.65,.15,1.
            markers.extend([arrow,text])
        self.targets_pub.publish(MarkerArray(markers=markers));self.targets_published=True

    async def wait(self, future, timeout=1.):
        deadline = time.perf_counter()+timeout
        while not future.done():
            if self.error:
                raise RuntimeError("navigation_ros_executor_failed") from self.error
            if time.perf_counter() >= deadline:
                raise TimeoutError("navigation_ros_operation_timeout")
            await asyncio.sleep(.01)
        return future.result()

    async def control(self, task_id, command):
        from companionbot_interfaces.srv import NavigationControl
        if not self.control_client.service_is_ready():
            raise RuntimeError("navigation_authority_service_unavailable")
        self.control_sequence += 1
        request = NavigationControl.Request(task_id=task_id, source_epoch=self.source_epoch,
            sequence=self.control_sequence, command=command, lease_duration_s=1.2,
            map_id=self.binding["map_id"], map_version=self.binding["map_version"])
        return await self.wait(self.control_client.call_async(request), .8)

    async def navigation_active(self):
        """Discovery precedes activation: only stock ACTIVE nodes can execute."""
        from lifecycle_msgs.srv import GetState
        from lifecycle_msgs.msg import State
        if self.error is not None or not self.action.server_is_ready():
            return False
        if not all(client.service_is_ready() for client in self.lifecycle_clients):
            return False
        try:
            states = await asyncio.gather(*(self.wait(client.call_async(GetState.Request()), .3)
                for client in self.lifecycle_clients))
        except (RuntimeError, TimeoutError):
            return False
        return all(result.current_state.id == State.PRIMARY_STATE_ACTIVE for result in states)

    def event(self, task_id, status, reason, destination=None, progress=None):
        with self._events_lock:
            return self._event(task_id, status, reason, destination, progress)

    def _event(self, task_id, status, reason, destination=None, progress=None):
        self.sequence += 1
        previous = self.tasks.get(task_id)
        result = Feedback(status=status, reason=reason, task_id=task_id, intent=Intent.GUIDE_TO,
            destination_id=destination or (previous.destination_id if previous else None),
            backend="nav2_simulation", contract_version=2, source_id="nav2:action",
            source_epoch=self.source_epoch, sequence=self.sequence, observed_at_s=time.perf_counter(),
            progress=progress if progress is not None else (previous.progress if previous else None))
        self.tasks[task_id] = result; retain_tasks(self.tasks)
        return result

    async def start(self, task_id, destination):
        from nav2_msgs.action import NavigateToPose
        if task_id in self.tasks:
            previous = self.tasks[task_id]
            if previous.destination_id == destination:
                return previous
            return Feedback(status="REJECT", reason="duplicate_task_conflict", task_id=task_id)
        if destination not in self.destinations or self.active is not None:
            return self.event(task_id, "REJECT", "unknown_destination_or_motion_busy", destination)
        if not await self.navigation_active():
            return self.event(task_id, "REJECT", "nav2_lifecycle_or_action_unavailable", destination)
        granted = False
        pending = None
        try:
            answer = await self.control(task_id, "ACQUIRE")
            if not answer.accepted:
                return self.event(task_id, "REJECT", answer.reason, destination)
            granted, self.active = True, task_id
            self.publish_targets()
            target = self.binding["destinations"][destination]
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id, goal.pose.header.stamp = "map", self.node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = float(target["x_m"]), float(target["y_m"])
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(target["yaw_rad"]/2), math.cos(target["yaw_rad"]/2)
            self.event(task_id, "ACCEPT", "navigation_authority_acquired", destination)
            def feedback(message):
                with self._events_lock:
                    previous = self.tasks.get(task_id)
                    if previous is None or previous.status not in ("ACCEPT", "RUNNING"):
                        return
                    distance = float(message.feedback.distance_remaining)
                    # BT feedback may initially contain its default zero before
                    # FollowPath publishes a path distance. That is unavailable
                    # progress, not 99% arrival. SUCCEEDED alone produces 1.0.
                    progress = self.progress_from_distance(task_id, distance)
                    self._event(task_id, "RUNNING", "nav2_feedback", progress=progress)
            pending = self.action.send_goal_async(goal, feedback_callback=feedback)
            handle = await self.wait(pending, .8)
            if not handle.accepted:
                await self.control(task_id, "RELEASE"); self.active = None
                return self.event(task_id, "REJECT", "nav2_goal_rejected", destination)
            self.handles[task_id] = handle
            self.monitors[task_id] = asyncio.create_task(self.monitor(task_id, handle))
            return self.event(task_id, "ACCEPT", "nav2_goal_accepted", destination)
        except BaseException:
            if granted:
                try:
                    await asyncio.shield(self.control(task_id, "STOP"))
                finally:
                    self.active = None
                    if pending is not None:
                        pending.add_done_callback(lambda f: f.result().cancel_goal_async() if f.result().accepted else None)
            raise

    async def monitor(self, task_id, handle):
        future = handle.get_result_async()
        try:
            self.event(task_id, "RUNNING", "nav2_executing")
            renew_due = 0.
            while not future.done():
                if time.perf_counter() >= renew_due:
                    answer = await self.control(task_id, "RENEW")
                    if not answer.accepted:
                        raise RuntimeError("navigation_authority_lost:"+answer.reason)
                    renew_due = time.perf_counter()+.35
                # Terminal results must release authority before the 0.3s
                # command watchdog, independently of the lease renewal period.
                await asyncio.sleep(.01)
            result = future.result()
            status = {4: "COMPLETED", 5: "CANCEL", 6: "FAILED"}.get(result.status, "FAILED")
            reason = {4: "nav2_succeeded", 5: "nav2_canceled", 6: "nav2_aborted"}.get(result.status, "nav2_unknown_result")
            self.event(task_id, status, reason+":"+str(result.result.error_code), progress=1. if result.status == 4 else None)
        except asyncio.CancelledError:
            raise
        except Exception as error:
            self.event(task_id, "FAILED", str(error))
            try:
                await self.control(task_id, "STOP")
                await self.wait(handle.cancel_goal_async())
            except Exception:
                pass  # Lease expiry is independently enforced by the motion owner.
        finally:
            try:
                if self.active == task_id:
                    try:
                        await self.control(task_id, "RELEASE")
                    finally:
                        self.active = None
            finally:
                self.monitors.pop(task_id, None)
                self.handles.pop(task_id, None)
                self.initial_distance.pop(task_id, None)

    async def cancel(self, task_id):
        previous = self.tasks.get(task_id)
        if previous is None:
            return Feedback(status="REJECT", reason="unknown_task", task_id=task_id)
        if previous.status not in IN_FLIGHT:
            return previous
        # Retain the handle across awaited STOP/monitor cleanup, including a
        # simultaneous terminal result; acknowledgement still comes from Nav2.
        handle = self.handles.get(task_id)
        answer = await self.control(task_id, "STOP")
        if not answer.accepted:
            return self.event(task_id, "UNKNOWN", "navigation_stop_not_acknowledged")
        monitor = self.monitors.pop(task_id, None)
        self.active = None
        if monitor:
            monitor.cancel(); await asyncio.gather(monitor, return_exceptions=True)
        if handle:
            await self.wait(handle.cancel_goal_async())
            result = await self.wait(handle.get_result_async(), 1.)
            if result.status == 5:
                return self.event(task_id, "CANCEL", "nav2_canceled_after_motion_revoked")
            if result.status == 4:
                return self.event(task_id, "COMPLETED", "nav2_finished_before_cancel", progress=1.)
        return self.event(task_id, "CANCEL_REQUESTED", "motion_revoked_awaiting_nav2_cancel")

    async def status(self, task_id):
        result = self.tasks.get(task_id)
        if result is None:
            return Feedback(status="REJECT", reason="unknown_task", task_id=task_id)
        return result.model_copy(update={"observed_at_s": time.perf_counter()})

    async def close(self):
        try:
            if self.active:
                await self.cancel(self.active)
        finally:
            for task in self.monitors.values():
                task.cancel()
            await asyncio.gather(*self.monitors.values(), return_exceptions=True)
            self.executor.shutdown(timeout_sec=2.)
            self.thread.join(timeout=2.)
            self.node.destroy_node(); self.context.try_shutdown()
            if self.thread.is_alive():
                raise RuntimeError("navigation_executor_failed_to_stop")


class NavigationRobotBackend:
    """Existing Supervisor robot side of the same native motion authority."""
    def __init__(self, navigation):
        self.navigation, self.tasks, self.sequence = navigation, {}, 0
        self.follow_monitors = {}
        self.active_follow = None

    def follow_event(self, task_id, status, reason):
        self.sequence += 1
        result = Feedback(status=status, reason=reason, task_id=task_id, intent=Intent.FOLLOW,
            backend="nav2_simulation", contract_version=2, source_id="native:follow_authority",
            source_epoch=self.navigation.source_epoch, sequence=self.sequence,
            observed_at_s=time.perf_counter())
        self.tasks[task_id] = result
        retain_tasks(self.tasks)
        return result

    async def latest_state(self):
        self.sequence += 1
        requested_at = time.perf_counter()
        try:
            answer = await self.navigation.control("supervisor_readiness", "CHECK")
            safe = answer.accepted
            connected = True
            follow = await self.navigation.control("supervisor_follow_readiness", "CHECK_FOLLOW")
            master_locked = follow.accepted
        except (RuntimeError, TimeoutError):
            connected = safe = False
            master_locked = False
        ready = safe and await self.navigation.navigation_active()
        if ready:
            self.navigation.publish_targets()
        now = time.perf_counter()
        return RobotSnapshot(observed_at_s=requested_at, connected=connected, execution_ready=ready, safety_ok=safe,
            master_locked=master_locked, backend="nav2_simulation", contract_version=2,
            source_id="nav2:authority_check", source_epoch=self.navigation.source_epoch,
            sequence=self.sequence, received_at_s=now)

    async def request(self, task_id, request):
        if request.intent == Intent.FOLLOW:
            previous = self.tasks.get(task_id)
            if previous is not None:
                return previous
            answer = await self.navigation.control(task_id, "ACQUIRE_FOLLOW")
            if not answer.accepted:
                return self.follow_event(task_id, "REJECT", answer.reason)
            self.active_follow = task_id
            result = self.follow_event(task_id, "ACCEPT", "native_follow_authority_acquired")
            self.follow_monitors[task_id] = asyncio.create_task(self.monitor_follow(task_id))
            return result
        if request.intent not in (Intent.STOP_REQUEST, Intent.WAIT):
            return Feedback(status="REJECT", reason="navigation_mode_follow_unavailable", task_id=task_id)
        command = "STOP" if request.intent == Intent.STOP_REQUEST else "WAIT"
        answer = await self.navigation.control(task_id, command)
        result = Feedback(status="ACCEPT" if answer.accepted else "FAILED", reason=answer.reason,
            task_id=task_id, intent=request.intent, backend="nav2_simulation", contract_version=2,
            source_id="nav2:authority_check", source_epoch=self.navigation.source_epoch,
            sequence=self.sequence, observed_at_s=time.perf_counter())
        self.tasks[task_id] = result
        return result

    async def monitor_follow(self, task_id):
        try:
            while self.active_follow == task_id:
                answer = await self.navigation.control(task_id, "RENEW_FOLLOW")
                if not answer.accepted:
                    self.follow_event(task_id, "FAILED", "follow_authority_lost:"+answer.reason)
                    return
                self.follow_event(task_id, "RUNNING", "native_visual_follow_executing")
                await asyncio.sleep(.35)
        except asyncio.CancelledError:
            raise
        except Exception as error:
            self.follow_event(task_id, "FAILED", str(error))
            try:
                await self.navigation.control(task_id, "STOP")
            except Exception:
                pass  # Native lease/measurement expiry independently revokes.
        finally:
            try:
                if self.active_follow == task_id:
                    try:
                        await self.navigation.control(task_id, "RELEASE")
                    finally:
                        self.active_follow = None
            finally:
                self.follow_monitors.pop(task_id, None)

    async def status(self, task_id):
        result = self.tasks.get(task_id, Feedback(status="REJECT", reason="unknown_task", task_id=task_id))
        if result.intent == Intent.FOLLOW:
            return result.model_copy(update={"observed_at_s": time.perf_counter()})
        if result.status == "ACCEPT":
            answer = await self.navigation.control(task_id, "CHECK_STOPPED")
            if answer.accepted:
                result = result.model_copy(update={"status": "COMPLETED", "reason": "mcu_safe_hold_observed",
                    "sequence": (result.sequence or 0)+1, "observed_at_s": time.perf_counter()})
                self.tasks[task_id] = result
        return result

    async def cancel(self, task_id):
        result = self.tasks.get(task_id)
        if result is not None and result.intent == Intent.FOLLOW:
            if result.status not in IN_FLIGHT:
                return result
            answer = await self.navigation.control(task_id, "STOP")
            if not answer.accepted:
                return self.follow_event(task_id, "UNKNOWN", "follow_stop_not_acknowledged")
            self.active_follow = None
            monitor = self.follow_monitors.pop(task_id, None)
            if monitor:
                monitor.cancel()
                await asyncio.gather(monitor, return_exceptions=True)
            return self.follow_event(task_id, "CANCEL", "follow_authority_revoked")
        return await self.status(task_id)

    async def close(self):
        try:
            if self.active_follow:
                await self.cancel(self.active_follow)
        finally:
            for monitor in self.follow_monitors.values():
                monitor.cancel()
            await asyncio.gather(*self.follow_monitors.values(), return_exceptions=True)
            self.follow_monitors.clear()
