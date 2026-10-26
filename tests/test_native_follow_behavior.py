"""Business task lifecycle against explicit service replies; no simulated motion."""
import asyncio
from types import SimpleNamespace

from companionbot_behavior.core.behavior import BehaviorRequest, Intent
from companionbot_behavior.runtime.nav2_backend import NavigationRobotBackend


class ServiceFixture:
    source_epoch='behavior_test'
    def __init__(self):
        self.calls=[]
        self.follow_ready=False
        self.renew=True
        self.stop=True
    async def control(self,task,command):
        self.calls.append((task,command))
        accepted=(self.follow_ready if command in ('CHECK_FOLLOW','ACQUIRE_FOLLOW') else
                  self.renew if command=='RENEW_FOLLOW' else self.stop if command=='STOP' else True)
        return SimpleNamespace(accepted=accepted,reason='fixture_ready' if accepted else 'fixture_unavailable')
    async def navigation_active(self): return True
    def publish_targets(self): pass


def test_master_readiness_comes_from_native_check_and_failed_follow_never_starts():
    async def run():
        service=ServiceFixture(); robot=NavigationRobotBackend(service)
        assert not (await robot.latest_state()).master_locked
        result=await robot.request('first',BehaviorRequest(intent=Intent.FOLLOW))
        assert result.status=='REJECT' and not robot.follow_monitors
        service.follow_ready=True
        assert (await robot.latest_state()).master_locked
        await robot.close()
    asyncio.run(run())


def test_follow_cancel_revokes_before_monitor_cleanup_and_never_completes_motion():
    async def run():
        service=ServiceFixture();service.follow_ready=True
        robot=NavigationRobotBackend(service)
        assert (await robot.request('first',BehaviorRequest(intent=Intent.FOLLOW))).status=='ACCEPT'
        await asyncio.sleep(0)
        result=await robot.cancel('first')
        assert result.status=='CANCEL' and result.reason=='follow_authority_revoked'
        at=next(i for i,row in enumerate(service.calls) if row==('first','STOP'))
        await asyncio.sleep(0)
        assert not any(c=='RENEW_FOLLOW' for _,c in service.calls[at+1:])
        assert robot.active_follow is None and (await robot.status('first')).status=='CANCEL'
        await robot.close()
    asyncio.run(run())


def test_native_follow_revocation_becomes_failed_task_without_implicit_reacquire():
    async def run():
        service=ServiceFixture();service.follow_ready=True;service.renew=False
        robot=NavigationRobotBackend(service)
        await robot.request('first',BehaviorRequest(intent=Intent.FOLLOW))
        await asyncio.sleep(0);await asyncio.sleep(0)
        assert (await robot.status('first')).status=='FAILED'
        assert sum(c=='ACQUIRE_FOLLOW' for _,c in service.calls)==1
        assert robot.active_follow is None
        await robot.close()
    asyncio.run(run())


def test_finished_follow_monitors_do_not_accumulate_across_tasks():
    async def run():
        service=ServiceFixture();service.follow_ready=True;service.renew=False
        robot=NavigationRobotBackend(service)
        for index in range(300):
            await robot.request(str(index),BehaviorRequest(intent=Intent.FOLLOW))
            await asyncio.sleep(0);await asyncio.sleep(0)
        assert not robot.follow_monitors and robot.active_follow is None
        assert len(robot.tasks)<=256
        await robot.close()
    asyncio.run(run())


def test_nav2_cancel_retains_action_handle_across_monitor_cleanup():
    """STOP and Action completion can interleave; result still needs real handle ack."""
    import threading
    from companionbot_behavior.core.behavior import Feedback
    from companionbot_behavior.runtime.nav2_backend import Nav2NavigationBackend
    async def run():
        backend=Nav2NavigationBackend.__new__(Nav2NavigationBackend)
        backend.sequence=0;backend.source_epoch='fixture'
        backend._events_lock=threading.RLock();backend.active='task'
        backend.tasks={'task':Feedback(status='RUNNING',reason='fixture',task_id='task',
            intent=Intent.GUIDE_TO,destination_id='service_desk')}
        backend.initial_distance={}
        class Handle:
            canceled=False
            def cancel_goal_async(self):
                self.canceled=True
                future=asyncio.get_running_loop().create_future();future.set_result(True)
                return future
            def get_result_async(self):
                future=asyncio.get_running_loop().create_future()
                future.set_result(SimpleNamespace(status=5));return future
        handle=Handle();backend.handles={'task':handle}
        async def cleanup():
            try: await asyncio.Future()
            finally: backend.handles.pop('task',None)
        monitor=asyncio.create_task(cleanup());backend.monitors={'task':monitor}
        async def control(*_):
            await asyncio.sleep(0)
            return SimpleNamespace(accepted=True)
        async def wait(future,*_): return future.result()
        backend.control,backend.wait=control,wait
        result=await backend.cancel('task')
        assert result.status=='CANCEL' and handle.canceled
        assert not backend.handles and not backend.monitors
    asyncio.run(run())
