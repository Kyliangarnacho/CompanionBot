"""Headless typed ROS input acceptance of the sole formal app in Nav2 mode."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import MarkerArray
from companionbot_interfaces.msg import RobotSnapshot, BehaviorFeedback
from companionbot_interfaces.srv import UserInput, NavigationControl
from companionbot_contracts.navigation_map import load_map_binding
from companionbot_contracts.stamps import source_stamp
from verify_spatial_foundation import control_gates


def physical_goal_evidence(out,binding,feedback):
    """Independent post-hoc sink, called only after the physical run exits."""
    import numpy as np
    rows=[json.loads(line) for line in (out/"simulation/truth_posthoc.jsonl").read_text().splitlines()]
    t=np.array([r["time_s"] for r in rows]);p=np.array([r["position_ros"][:2] for r in rows])
    heading=rows[0]["rpy_ros"][2];c,s=math.cos(heading),math.sin(heading)
    xy=(p-p[0])@np.array([[c,-s],[s,c]]);yaw=np.unwrap([r["rpy_ros"][2] for r in rows])-heading
    releases={r["task_id"]:r["ros_observation_time_s"] for r in
        map(json.loads,(out/"simulation/navigation_controls.jsonl").read_text().splitlines()) if r["command"]=="RELEASE" and r["accepted"]}
    limits=json.loads(Path("models/minisegway/retail_navigation/acceptance.json").read_text())["navigation"]
    evidence=[]
    for f in feedback:
        if f["status"]!="COMPLETED":continue
        end=releases[f["task_id"]];target=binding["destinations"][f["destination_id"]]
        actual=np.array([np.interp(end,t,xy[:,i]) for i in range(2)])
        settled=np.array([np.interp(end+3.,t,xy[:,i]) for i in range(2)])
        errors=[float(np.linalg.norm(q-[target["x_m"],target["y_m"]])) for q in (actual,settled)]
        angle=np.interp(end,t,yaw)-target["yaw_rad"];angle=abs(math.atan2(math.sin(angle),math.cos(angle)))
        settled_angle=np.interp(end+3.,t,yaw)-target["yaw_rad"];settled_angle=abs(math.atan2(math.sin(settled_angle),math.cos(settled_angle)))
        evidence.append(dict(task_id=f["task_id"],destination_id=f["destination_id"],release_source_time_s=end,
            position_error_m=errors[0],position_error_after_3s_m=errors[1],yaw_error_rad=angle,
            yaw_error_after_3s_rad=settled_angle,
            passed=max(errors)<=limits["goal_position_error_limit_m"] and max(angle,settled_angle)<=limits["goal_yaw_error_limit_rad"]))
    return dict(gt_posthoc_only=True,initial_alignment_only=True,goals=evidence,
        obstacle_contact_samples=sum(bool(r.get("obstacle_contacts_posthoc")) for r in rows),
        passed=bool(evidence) and all(r["passed"] for r in evidence) and not any(r.get("obstacle_contacts_posthoc") for r in rows))


def main():
    cli=argparse.ArgumentParser(description=__doc__);cli.add_argument("--bindings",type=Path,required=True)
    cli.add_argument("--output-dir",type=Path,required=True)
    cli.add_argument("--visual-config",type=Path)
    cli.add_argument("--expect-follow-unavailable",action="store_true",
        help="Verify real perception's failed depth certificate cannot authorize FOLLOW; does not certify following")
    cli.add_argument("--domain-id",type=int,default=91)
    args=cli.parse_args();out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    if args.expect_follow_unavailable and not args.visual_config:
        cli.error('--expect-follow-unavailable requires --visual-config')
    binding=load_map_binding(args.bindings);os.environ.update(ROS_DOMAIN_ID=str(args.domain_id),ROS_LOG_DIR=str(out/"ros_logs"),PYDANTIC_AI_NO_BANNER="1")
    rclpy.init();node=Node("formal_navigation_acceptance");states=[];feedback=[];targets=[]
    node.create_subscription(RobotSnapshot,"/companionbot/robot/state",lambda m:states.append(m),10)
    node.create_subscription(BehaviorFeedback,"/companionbot/behavior/feedback",lambda m:feedback.append(m),32)
    node.create_subscription(MarkerArray,"/companionbot/navigation/targets",lambda m:targets.append(m),
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    input_client=node.create_client(UserInput,"/companionbot/interaction/input")
    authority_client=node.create_client(NavigationControl,"/companionbot/navigation/control")
    handles=[];processes=[];resource_sampler=None
    summary=dict(passed=False,scope="formal app + offline original FunctionModel + actual Nav2/physics; no GUI/hardware")
    def start(label,command,stdin=None):
        handle=(out/(label+".txt")).open("w");handles.append(handle)
        p=subprocess.Popen(command,stdin=stdin,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True,text=True);processes.append(p);return p
    def spin_until(predicate,timeout):
        end=time.perf_counter()+timeout
        while time.perf_counter()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate():return
            if any(p.poll() is not None for p in processes):raise RuntimeError("formal_acceptance_process_exited_early")
        raise TimeoutError("formal_acceptance_deadline")
    try:
        sim=start("launch",[sys.executable,"/opt/ros/jazzy/bin/ros2","launch","companionbot_bringup","retail_simulation.launch.py",
            "mode:=localization","navigation_bindings:="+str(args.bindings.resolve()),"set_initial_pose:=true",
            "duration_s:=60.0","finish_grace_s:=3.0","record_truth:=true","output_dir:="+str(out/"simulation"),
            *(["visual_companion_config:="+str(args.visual_config.resolve())] if args.visual_config else [])])
        from runtime_resources import RuntimeResourceSampler
        resource_sampler=RuntimeResourceSampler(out/'resource_profile',root=sim,label='simulation_graph').start()
        app=start("formal_app",[sys.executable,"scripts/demo_stage8_interaction.py","--fake","--no-camera","--headless","--no-mic","--no-tts","--ros",
            "--navigation-mode","nav2","--navigation-bindings",str(args.bindings.resolve()),
            "--catalog-file","models/minisegway/retail_navigation/config/catalog.json","--output-dir",str(out/"interaction")],subprocess.PIPE)
        resource_sampler.add_child(app,label='formal_agent')
        spin_until(lambda:any(m.execution_ready and m.backend=="nav2_simulation" for m in states),25.)
        ready=states[-1]
        if ready.hardware_execution_ready:raise RuntimeError("simulation_misreported_hardware_ready")
        spin_until(input_client.service_is_ready,5.)
        def submit(text,sequence):
            request=UserInput.Request(text=text,meta=source_stamp("formal_navigation_acceptance:text","formal_navigation_service",sequence,
                time.perf_counter(),time_semantics="text_submitted"))
            future=input_client.call_async(request);spin_until(future.done,5.)
            response=future.result()
            if not response.queued:raise RuntimeError("formal_input_rejected:"+response.reason)
            return response.input_id
        if args.expect_follow_unavailable:
            summary['follow_input_id']=submit('你好小柒，跟着我走',1)
            spin_until(lambda:any(m.intent=='FOLLOW' and m.status=='REJECT' for m in feedback),8.)
            summary['failed_depth_follow_rejected']=True
            summary['visual_follow_acceptance']=False
            request=NavigationControl.Request(task_id='verify_unavailable_follow',
                source_epoch='formal_visual_acceptance',sequence=1,command='CHECK_FOLLOW',
                lease_duration_s=1.2,map_id=binding['map_id'],map_version=binding['map_version'])
            future=authority_client.call_async(request);spin_until(future.done,3.)
            summary['native_follow_readiness']=dict(accepted=future.result().accepted,reason=future.result().reason)
            if future.result().accepted:raise RuntimeError('unverified_depth_authorized_native_follow')
        input_offset=1 if args.expect_follow_unavailable else 0
        if args.visual_config:
            state_count=len(states)
            spin_until(lambda:len(states)>state_count and states[-1].execution_ready,8.)
        summary["input_id"]=submit("你好小柒，带我去服务台",1+input_offset)
        spin_until(lambda:any(m.status=="COMPLETED" and m.destination_id=="service_desk" and m.backend=="nav2_simulation" for m in feedback),120. if args.visual_config else 40.)
        first_task=next(m.task_id for m in feedback if m.status=="COMPLETED" and m.destination_id=="service_desk")
        # Action success starts an actual MCU brake. A repeated task must wait
        # for its SAFE_HOLD acknowledgement; never weaken the rearm gate.
        stop_deadline=time.perf_counter()+6.;sequence=0
        while time.perf_counter()<stop_deadline:
            sequence+=1
            request=NavigationControl.Request(task_id="formal_repeat_readiness",source_epoch="formal_navigation_service",
                sequence=sequence,command="CHECK_STOPPED",lease_duration_s=1.2,
                map_id=binding["map_id"],map_version=binding["map_version"])
            future=authority_client.call_async(request);spin_until(future.done,2.)
            if future.result().accepted:break
            delay=time.perf_counter()+.1;spin_until(lambda:time.perf_counter()>=delay,.5)
        else:raise TimeoutError("formal_repeat_waiting_for_actual_mcu_hold")
        state_count=len(states)
        spin_until(lambda:len(states)>state_count and states[-1].execution_ready,3.)
        summary["repeat_input_id"]=submit("带我去服务台",2+input_offset)
        spin_until(lambda:any(m.status=="COMPLETED" and m.destination_id=="service_desk" and m.task_id!=first_task for m in feedback),10.)
        summary["repeated_destination_completed_with_new_task"]=True
        if not any(m.status in ("ACCEPT","RUNNING") for m in feedback):raise RuntimeError("missing_nonterminal_task_feedback")
        if not targets or len(targets[-1].markers)!=2*len(binding["destinations"]):raise RuntimeError("missing_bound_goal_visualization")
        if any(m.header.frame_id!="map" for m in targets[-1].markers):raise RuntimeError("incorrect_target_visualization_frame")
        summary["marker_count"]=len(targets[-1].markers)
        summary["feedback"]=[dict(status=m.status,reason=m.reason,task_id=m.task_id,destination_id=m.destination_id,backend=m.backend) for m in feedback
            if m.status in ("ACCEPT","COMPLETED","FAILED","CANCEL")]
        submit("/quit",3+input_offset);app.wait(timeout=12.)
        if app.returncode:raise RuntimeError("formal_app_cleanup_failed")
        # Only the finite physical runtime remains; do not interpret the app's
        # intentional exit as a failure while waiting for its final evidence.
        processes.remove(app)
        spin_until(lambda:sim.poll() is not None,200. if args.visual_config else 70.)
        if sim.returncode:raise RuntimeError("formal_simulation_failed")
        result=json.loads((out/"simulation/frozen_result_posthoc.json").read_text());summary["control_gates"]=control_gates(result,60.)
        authority=json.loads((out/"simulation/motion_authority.json").read_text())
        summary["unexpected_takeovers"]=[e for e in authority["safety_events"] if e["event"]=="safety_takeover"
            and e["reason"]!="navigation_task_released"]
        summary["physical_goals_posthoc"]=physical_goal_evidence(out,binding,summary["feedback"])
        summary["passed"]=(summary["control_gates"]["passed"] and summary["physical_goals_posthoc"]["passed"]
            and not summary["unexpected_takeovers"])
    except BaseException as error:
        summary["error"]=repr(error);raise
    finally:
        for p in processes:
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=12.)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=8.)
        for handle in handles:handle.close()
        if resource_sampler is not None:
            try:
                resource_sampler.close()
                summary['resource_profile']='resource_profile/summary.json'
            except Exception as error:
                summary['resource_profile_error']=repr(error)
        (out/"summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False)+"\n")
        node.destroy_node();rclpy.try_shutdown()
    print(json.dumps(summary,ensure_ascii=False),flush=True)


if __name__=="__main__":main()
