"""Existing Agent tool/Supervisor/real-Action integration, deterministic LLM fixture.

No external model call, velocity, pose or simulator control. This diagnostic is
not a second formal interaction entry and makes no claim about LLM language
quality. The formal app selects the same backend with --navigation-mode nav2.
"""
import argparse
import asyncio
import json
from pathlib import Path
import time

from pydantic_ai.models.function import FunctionModel, DeltaToolCall
from companionbot_behavior.core.behavior import BehaviorSupervisor
from companionbot_behavior.core.catalog import JsonCatalogProvider
from companionbot_behavior.core.config import AgentConfig
from companionbot_behavior.core.interaction import AgentSession
from companionbot_behavior.core.runtime import AgentRuntime
from companionbot_behavior.runtime.nav2_backend import Nav2NavigationBackend, NavigationRobotBackend


async def run(args):
    out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    catalog=JsonCatalogProvider.load(args.catalog)
    destination=catalog.destinations[args.destination]
    async def fixture(messages, info):
        returns=[p for p in messages[-1].parts if p.part_kind=="tool-return"]
        if not returns:
            yield {0:DeltaToolCall(name="lookup_knowledge",json_args=json.dumps(
                dict(query=destination.name,target_kind="destination")),tool_call_id="fixture-catalog")}
        else:
            found=next((p.content for p in returns if p.tool_name=="lookup_knowledge"),None)
            if isinstance(found,dict) and found.get("resolution"):
                resolution=found["resolution"]
                yield {0:DeltaToolCall(name="request_behavior",json_args=json.dumps(dict(request=dict(
                    intent="GUIDE_TO",destination_id=resolution["destination_id"],resolution_id=resolution["resolution_id"]))),
                    tool_call_id="fixture-guide")}
            else:
                yield "导航请求已提交，等待实际任务反馈。"
    navigation=Nav2NavigationBackend(args.bindings)
    robot=NavigationRobotBackend(navigation)
    supervisor=BehaviorSupervisor(robot,navigation)
    runtime=AgentRuntime(FunctionModel(stream_function=fixture,model_name="deterministic_navigation_fixture"),
        supervisor,AgentConfig(),catalog=catalog,telemetry=out/"agent.jsonl")
    session=AgentSession(runtime)
    summary=dict(passed=False,scope="existing Agent tools and Supervisor; deterministic model fixture; actual Nav2/MuJoCo execution",physical_hardware=False)
    try:
        deadline=time.perf_counter()+10.
        while time.perf_counter()<deadline:
            state=await robot.latest_state()
            if state.execution_ready:
                break
            await asyncio.sleep(.1)
        if not state.execution_ready:
            raise RuntimeError("Agent_navigation_readiness_unavailable")
        await session.receive("你好小柒，带我去"+destination.name)
        outcome=await asyncio.wait_for(session.wait(),timeout=15.)
        summary["agent_turn_status"]=outcome.status
        summary["agent_metrics"]=outcome.metrics
        task=supervisor.active_task
        summary["task_id"]=task
        if task is None:
            raise RuntimeError("Agent_did_not_create_authorized_navigation_task")
        initial=await supervisor.status(task);summary["initial_feedback"]=initial.model_dump(mode="json")
        deadline=time.perf_counter()+args.timeout
        started=time.perf_counter();canceled=False
        while time.perf_counter()<deadline:
            if args.cancel_after is not None and not canceled and time.perf_counter()-started>=args.cancel_after:
                await session.receive("/cancel");canceled=True
            await supervisor.poll();feedback=await supervisor.status(task)
            if feedback.status in ("COMPLETED","FAILED","CANCEL","REJECT"):
                break
            await asyncio.sleep(.1)
        summary["final_feedback"]=feedback.model_dump(mode="json")
        before=navigation.control_sequence
        duplicate=await navigation.start(task,args.destination)
        summary["duplicate_task"]=dict(status=duplicate.status,task_id=duplicate.task_id,
            control_requests_added=navigation.control_sequence-before)
        expected="CANCEL" if args.cancel_after is not None else "COMPLETED"
        summary["passed"]=(feedback.status==expected and feedback.backend=="nav2_simulation"
            and duplicate.status==feedback.status and navigation.control_sequence==before)
        if not summary["passed"]:
            raise RuntimeError("Agent_navigation_not_completed:"+feedback.reason)
    except BaseException as error:
        summary["error"]=repr(error)
        raise
    finally:
        await session.close();await navigation.close()
        (out/"summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False)+"\n")
    print(json.dumps(summary,ensure_ascii=False),flush=True)


def main():
    cli=argparse.ArgumentParser(description=__doc__);cli.add_argument("--bindings",type=Path,required=True)
    cli.add_argument("--catalog",type=Path,required=True);cli.add_argument("--destination",default="entrance")
    cli.add_argument("--output-dir",type=Path,required=True);cli.add_argument("--timeout",type=float,default=100.)
    cli.add_argument("--cancel-after",type=float)
    asyncio.run(run(cli.parse_args()))


if __name__=="__main__":main()
