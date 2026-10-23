"""Actual DDS/source interruption, cancellation and explicit reissue acceptance.

Only this script's own AMCL/controller children may be paused. The robot stays
on the original physics/reference/MCU chain. No synthetic pose/TF/scan is sent.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from uuid import uuid4

import psutil
import rclpy
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from verify_retail_navigation import NavigationProbe
from verify_spatial_foundation import control_gates


def run_case(root_out, binding_path, fault, domain):
    out=root_out/fault;out.mkdir(parents=True,exist_ok=True)
    binding=json.loads(binding_path.read_text())
    os.environ.update(ROS_DOMAIN_ID=str(domain),ROS_LOG_DIR=str(out/"ros_logs"))
    rclpy.init();probe=NavigationProbe(out);probe.binding=binding
    options=[sys.executable,"/opt/ros/jazzy/bin/ros2","launch","companionbot_bringup","retail_simulation.launch.py",
        "mode:=localization","set_initial_pose:=true","duration_s:=65.0","finish_grace_s:=3.0","record_truth:=true",
        "navigation_bindings:="+str(binding_path),"output_dir:="+str(out/"simulation")]
    if fault=="scan_dropout":
        options += ["scan_dropout_start_s:=20.0","scan_dropout_duration_s:=1.2"]
    handle=(out/"launch.txt").open("w");process=subprocess.Popen(options,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
    report=dict(fault=fault,passed=False,phase="startup",scope="actual ROS/DDS + original MuJoCo/reference/MCU; no hardware")
    paused=None
    def until_source(target, timeout=40.):
        end=time.perf_counter()+timeout
        while probe.now()<target and time.perf_counter()<end:
            probe.spin()
            if process.poll() is not None:raise RuntimeError("fault_graph_exited_early")
        if probe.now()<target:raise TimeoutError("fault_source_clock_stalled")
    try:
        report["lifecycle"]=probe.wait_ready()
        report["startup_source_time_s"]=probe.now()
        report["phase"]="initial_goal"
        probe.task=uuid4().hex
        answer=probe.control("ACQUIRE")
        if not answer.accepted:raise RuntimeError("fault_fixture_authority_rejected:"+answer.reason)
        goal=NavigateToPose.Goal(pose=probe.goal_pose(binding["destinations"]["bookshelf_west"]))
        goal_handle=probe.wait(probe.nav.send_goal_async(goal,feedback_callback=probe.feedback),5.)
        if not goal_handle.accepted:raise RuntimeError("fault_fixture_goal_rejected")
        future=goal_handle.get_result_async()
        until_source(19.8)
        if future.done():raise RuntimeError("goal_ended_before_planned_fault")
        if probe.last_odom.twist.twist.linear.x<.1:
            raise RuntimeError("fault_not_in_actual_forward_motion")
        report["pre_fault_velocity_hat_m_s"]=probe.last_odom.twist.twist.linear.x
        report["phase"]="fault_injection"
        if fault in ("command_dropout","localization_loss"):
            wanted="controller_server" if fault=="command_dropout" else "amcl"
            children=[p for p in psutil.Process(process.pid).children(recursive=True) if p.name()==wanted]
            if len(children)!=1:raise RuntimeError("owned_fault_process_not_unique")
            paused=children[0];paused.send_signal(signal.SIGSTOP)
            report["paused_owned_process"]={"pid":paused.pid,"name":wanted}
        elif fault in ("STOP","CANCEL"):
            probe.control("STOP");probe.task=None
            probe.wait(goal_handle.cancel_goal_async(),4.)
        fault_time=probe.now();report["fault_source_time_s"]=fault_time
        until_source(21.3)
        if paused is not None:
            paused.send_signal(signal.SIGCONT);paused=None
        if probe.task:
            # The failed freshness gate must revoke the old lease itself.
            # Do not inject a second STOP and mask a missing watchdog action.
            if not probe.renew_error:
                raise RuntimeError("watchdog_did_not_revoke_old_task")
            probe.task=None;probe.lease_active=False
        if not future.done():probe.wait(goal_handle.cancel_goal_async(),4.)
        result=probe.wait(future,5.);report["interrupted_action_status"]=result.status
        velocity=[]
        report["phase"]="settling"
        while probe.now()<28.:
            probe.spin()
            if probe.now()>=26.:
                velocity.append(abs(probe.last_odom.twist.twist.linear.x))
        report["settled_velocity_hat_peak_m_s"]=max(velocity)
        if max(velocity)>.03:raise RuntimeError("fault_robot_not_settled_before_reissue")
        # This is a fresh, explicit task. Old commands cannot resume the lease.
        report["phase"]="explicit_reissue"
        report["reissue"]=probe.navigate("display_area",binding["destinations"]["display_area"],35.)
        if report["reissue"]["status"]!=4:raise RuntimeError("explicit_reissue_failed")
        # Planning is not an actuator command. Verify both a normal path and an
        # occupied shelf goal after reissue, with no motion lease held.
        report["planner_checks"]=[]
        for name,target,expected in (("service_desk",binding["destinations"]["service_desk"],4),
                ("inside_actual_shelf",dict(x_m=1.4,y_m=-1.45,yaw_rad=0.),6)):
            request=ComputePathToPose.Goal(goal=probe.goal_pose(target),planner_id="GridBased",use_start=False)
            wall=time.perf_counter();planned=probe.wait(probe.planner.send_goal_async(request),3.)
            if not planned.accepted:raise RuntimeError("planner_check_action_rejected")
            result=probe.wait(planned.get_result_async(),5.)
            report["planner_checks"].append(dict(goal=name,status=result.status,error_code=result.result.error_code,
                host_roundtrip_s=time.perf_counter()-wall,path_poses=len(result.result.path.poses)))
            if result.status!=expected:raise RuntimeError("planner_obstacle_failure_semantics_incorrect")
        # Diagnostic-only unbound target inside a real collision shelf. It is
        # never added to the venue Catalog or advertised as reachable. Exercise
        # NavigateToPose failure and lease release, not just ComputePathToPose.
        report["blocked_navigation"]=probe.navigate("diagnostic_occupied_shelf",
            dict(x_m=1.4,y_m=-1.45,yaw_rad=0.),8.)
        if report["blocked_navigation"]["status"]!=6:
            raise RuntimeError("occupied_shelf_navigation_did_not_fail")
        deadline=time.perf_counter()+90.
        report["phase"]="finite_completion"
        while process.poll() is None and time.perf_counter()<deadline:
            probe.spin()
        if process.poll() is None:raise TimeoutError("finite_fault_graph_did_not_exit")
        if process.returncode:raise RuntimeError("required_fault_graph_failed")
        physical=json.loads((out/"simulation/frozen_result_posthoc.json").read_text())
        report["control_gates"]=control_gates(physical,65.)
        authority=json.loads((out/"simulation/motion_authority.json").read_text())
        report["safety_takeovers"]=[e for e in authority["safety_events"] if e["event"]=="safety_takeover"]
        report["unexpected_takeovers"]=[e for e in report["safety_takeovers"] if e["reason"] not in
            ("navigation_stop","navigation_task_released","navigation_command_expired",
             "navigation_localization_tf_unavailable","navigation_scan_unavailable")]
        if report["unexpected_takeovers"]:
            raise RuntimeError("unexpected_independent_mcu_takeover")
        expected={"command_dropout":"navigation_command_expired","localization_loss":"navigation_localization_tf_unavailable",
                  "scan_dropout":"navigation_scan_unavailable","STOP":"navigation_stop","CANCEL":"navigation_stop"}[fault]
        if not any(e["reason"]==expected for e in report["safety_takeovers"]):
            raise RuntimeError("expected_freshness_stop_not_observed:"+expected)
        report["passed"]=report["control_gates"]["passed"]
    except BaseException as error:
        report["error"]=repr(error)
    finally:
        if paused is not None:
            try:paused.send_signal(signal.SIGCONT)
            except psutil.Error:pass
        try:probe.close()
        except Exception as error:report["cleanup_error"]=repr(error);report["passed"]=False
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=15.)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=8.)
        handle.close();probe.destroy_node();rclpy.try_shutdown()
        report["launch_exit_code"]=process.returncode
        (out/"summary.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report),flush=True)
    return report


def main():
    cli=argparse.ArgumentParser(description=__doc__);cli.add_argument("--bindings",type=Path,required=True)
    cli.add_argument("--output-dir",type=Path,required=True)
    cli.add_argument("--faults",nargs="+",default=["STOP","CANCEL","command_dropout","scan_dropout","localization_loss"])
    cli.add_argument("--domain-id",type=int,default=83);args=cli.parse_args()
    out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    cases=[run_case(out,args.bindings.resolve(),fault,args.domain_id+i) for i,fault in enumerate(args.faults)]
    summary=dict(passed=all(c["passed"] for c in cases),cases=cases)
    (out/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    if not summary["passed"]:raise SystemExit(1)


if __name__=="__main__":main()
