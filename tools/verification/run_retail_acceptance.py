"""Finite headless acceptance orchestration, not a formal robot interaction entry.

Stock planning verifies every explicit destination before movement. Every goal
then uses real Actions and the same task lease as the Agent adapter. No velocity,
pose, joint, TF or sensor injection is performed by this diagnostic.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bindings",type=Path,required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--domain-id",default=75,type=int)
    parser.add_argument("--duration",default=300.,type=float)
    parser.add_argument("--goals",nargs="+",default=["bookshelf_west","bookshelf_east","service_desk"])
    parser.add_argument("--mode",choices=("navigate","cancel"),default="navigate")
    parser.add_argument("--agent-goal",default="")
    parser.add_argument("--agent-only",action="store_true")
    parser.add_argument("--agent-cancel-after",type=float)
    args=parser.parse_args();out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    binding=json.loads(args.bindings.read_text());env=dict(os.environ,ROS_DOMAIN_ID=str(args.domain_id),ROS_LOG_DIR=str(out/"ros_logs"))
    report=dict(passed=False,scope="actual MuJoCo physics and stock Nav2; WSL host only",commands=[])
    launch=[sys.executable,"/opt/ros/jazzy/bin/ros2","launch","companionbot_bringup","retail_simulation.launch.py",
        "mode:=localization","set_initial_pose:=true","duration_s:="+str(args.duration),"finish_grace_s:=3.0",
        "record_truth:=true","map_yaml:="+binding["map_yaml"],"navigation_map_id:="+binding["map_id"],
        "navigation_bindings:="+str(args.bindings.resolve()),
        "navigation_map_version:="+binding["map_version"],"output_dir:="+str(out/"simulation")]
    handle=(out/"launch.txt").open("w");process=subprocess.Popen(launch,env=env,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
    report["commands"].append(launch)
    try:
        plan=[sys.executable,"tools/verification/verify_retail_navigation.py","--mode","plan","--bindings",str(args.bindings.resolve()),
            "--output-dir",str(out/"planning"),"--goals",*binding["destinations"]]
        report["commands"].append(plan)
        with (out/"planning.txt").open("w") as log:
            planned=subprocess.run(plan,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=100.)
        if planned.returncode:
            raise RuntimeError("stock_goal_reachability_failed; inspect planning evidence")
        proof=json.loads((out/"planning/summary.json").read_text())
        for target in proof["results"]:
            if target["status"]!=4 or not target["path"]["poses"]:
                raise RuntimeError("unreachable_semantic_destination")
            binding["destinations"][target["destination_id"]]["reachability_verified"]=True
        binding["reachability_evidence"]=str(out/"planning/summary.json")
        verified=out/"bindings.json";verified.write_text(json.dumps(binding,indent=2)+"\n")
        navigate=[sys.executable,"tools/verification/verify_retail_navigation.py","--mode",args.mode,"--bindings",str(verified),
            "--output-dir",str(out/"navigation"),"--goals",*args.goals]
        if not args.agent_only:
            report["commands"].append(navigate)
            with (out/"navigation.txt").open("w") as log:
                navigated=subprocess.run(navigate,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=args.duration+30.)
            report["navigation_exit_code"]=navigated.returncode
            if navigated.returncode:
                raise RuntimeError("physical_navigation_failed; inspect Action/sensor/authority evidence")
        if args.agent_goal:
            agent=[sys.executable,"tools/verification/verify_agent_navigation.py","--bindings",str(verified),
                "--catalog","models/minisegway/retail_navigation/config/catalog.json",
                "--destination",args.agent_goal,"--output-dir",str(out/"agent")]
            if args.agent_cancel_after is not None:
                agent += ["--cancel-after",str(args.agent_cancel_after)]
            report["commands"].append(agent)
            with (out/"agent.txt").open("w") as log:
                guided=subprocess.run(agent,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=125.)
            report["agent_exit_code"]=guided.returncode
            if guided.returncode:
                raise RuntimeError("Agent_navigation_failed; inspect real backend evidence")
        deadline=time.perf_counter()+args.duration+30.
        while process.poll() is None and time.perf_counter()<deadline:
            time.sleep(.2)
        if process.poll() is None:
            raise TimeoutError("finite_navigation_runtime_did_not_exit")
        if process.returncode:
            raise RuntimeError("required_navigation_runtime_failed")
        from verify_spatial_foundation import control_gates
        result=json.loads((out/"simulation/frozen_result_posthoc.json").read_text())
        report["control_gates"]=control_gates(result,args.duration)
        if not report["control_gates"]["passed"]:
            raise RuntimeError("frozen_navigation_control_gates_failed")
        report["passed"]=True
    except BaseException as error:
        report["error"]=repr(error)
        raise
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=15.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=8.)
        handle.close();report["launch_exit_code"]=process.returncode
        (out/"summary.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report),flush=True)


if __name__=="__main__":main()
