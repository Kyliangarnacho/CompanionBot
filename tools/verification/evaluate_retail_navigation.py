"""Independent post-hoc physical navigation metrics. Never imported by runtime."""
import argparse
import json
import math
from pathlib import Path
import re

import numpy as np
from scipy.ndimage import map_coordinates
import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt


def performance(out):
    """Observed clocks/CPU only; never infer RPP CPU time from message age."""
    report={"platform_scope":"WSL host, not Pi 5; processes filtered by this launch PID",
            "rpp_internal_compute_latency":"not separately instrumented"}
    own=set(map(int,re.findall(r"process started with pid \[(\d+)\]",(out/"launch.txt").read_text())))
    resources=json.loads((out/"navigation/resources.json").read_text())
    report["processes"]={}
    for pid in sorted(own):
        rows=[r for r in resources if r["pid"]==pid]
        if len(rows)<2:continue
        duration=rows[-1]["wall_s"]-rows[0]["wall_s"]
        report["processes"][rows[0]["name"]]=dict(pid=pid,observed_wall_s=duration,
            peak_rss_mib=max(r["rss_bytes"] for r in rows)/1048576,
            cpu_percent_one_core=100*(rows[-1]["cpu_s"]-rows[0]["cpu_s"])/duration)
    report["streams"]={}
    for name in ("scan","odom","command"):
        rows=[json.loads(line) for line in (out/"navigation"/(name+".jsonl")).read_text().splitlines()]
        if len(rows)<2:continue
        arrival=np.array([r["arrival_monotonic_s"] for r in rows])
        stamp=np.array([r["message"]["header"]["stamp"]["sec"]+r["message"]["header"]["stamp"]["nanosec"]*1e-9 for r in rows])
        report["streams"][name]=dict(count=len(rows),observed_wall_hz=(len(rows)-1)/(arrival[-1]-arrival[0]),
            observed_source_to_wall_ratio=(stamp[-1]-stamp[0])/(arrival[-1]-arrival[0]),
            source_interval_p95_s=float(np.quantile(np.diff(stamp),.95)))
    plans=json.loads((out/"planning/summary.json").read_text())["results"]
    report["stock_planner_reported_sim_duration_s"]={r["destination_id"]:r["planning_time"]["sec"]+r["planning_time"]["nanosec"]*1e-9 for r in plans}
    report["planner_duration_clock_note"]="Nav2 reports simulated ROS time; zero is unresolved at clock publication cadence, not zero CPU time. Host Action RTT is separately measured by the ROS fault probe."
    authority=json.loads((out/"simulation/motion_authority.json").read_text())
    epochs=authority.get("producer_epochs",[{"events":authority["producer_events"]}])
    compute=[r["compute_time_s"] for epoch in epochs for r in epoch["events"] if r["event"]=="block_generated"]
    report["reference_block_generation"]={"samples":len(compute),"epoch_coverage":"all" if "producer_epochs" in authority else "last only",
        "p95_wall_s":float(np.quantile(compute,.95)),"maximum_wall_s":max(compute)}
    report["controller_rate_warning_count"]=(out/"launch.txt").read_text().count("Control loop missed its desired rate")
    return report


def main():
    cli=argparse.ArgumentParser(description=__doc__);cli.add_argument("session",type=Path)
    cli.add_argument("--world-audit",type=Path,required=True);args=cli.parse_args();out=args.session.resolve()
    root=Path(__file__).resolve().parents[2]
    limits=json.loads((root/"models/minisegway/retail_navigation/acceptance.json").read_text())["navigation"]
    nav=json.loads((out/"navigation/summary.json").read_text())
    truth=[json.loads(l) for l in (out/"simulation/truth_posthoc.jsonl").read_text().splitlines()]
    t=np.array([r["time_s"] for r in truth]);position=np.array([r["position_ros"][:2] for r in truth])
    heading=truth[0]["rpy_ros"][2];rotation=np.array([[math.cos(heading),-math.sin(heading)],[math.sin(heading),math.cos(heading)]])
    xy=(position-position[0])@rotation;yaw=np.unwrap([r["rpy_ros"][2] for r in truth])-heading
    world=np.load(args.world_audit/"collision_projection_posthoc.npz")
    spacing=float(np.diff(world["x_m"])[0]);coordinates=np.array([(position[:,1]-world["y_m"][0])/spacing,(position[:,0]-world["x_m"][0])/spacing])
    center_clearance=map_coordinates(world["clearance_m"],coordinates,order=1,mode="constant",cval=0.)
    radius=json.loads((root/"ros2_ws/src/companionbot_bringup/config/nav2_geometry.json").read_text())["physical_radius_bound_m"]
    report=dict(gt_posthoc_only=True,alignment="one initial origin/yaw; no best fit",goals=[],
        obstacle_contact_samples=sum(bool(r.get("obstacle_contacts_posthoc")) for r in truth),
        minimum_conservative_obstacle_clearance_m=float(np.min(center_clearance)-radius-spacing*math.sqrt(2)/2),
        clearance_basis="canonical contact raster; full attitude-invariant robot radius; cell uncertainty subtracted")
    for result in nav["results"]:
        target=result["target"];start,end=result["start_source_time_s"],result["source_time_s"]
        actual=np.array([np.interp(end,t,xy[:,i]) for i in range(2)])
        settled=np.array([np.interp(end+3.,t,xy[:,i]) for i in range(2)])
        angle=np.interp(end,t,yaw);error=math.atan2(math.sin(angle-target["yaw_rad"]),math.cos(angle-target["yaw_rad"]))
        settled_angle=np.interp(end+3.,t,yaw)-target["yaw_rad"]
        settled_angle=abs(math.atan2(math.sin(settled_angle),math.cos(settled_angle)))
        mask=(t>=start)&(t<=end)
        row=dict(destination_id=result["destination_id"],status=result["status"],
            position_error_m=float(np.linalg.norm(actual-[target["x_m"],target["y_m"]])),
            settled_position_error_after_3s_m=float(np.linalg.norm(settled-[target["x_m"],target["y_m"]])),
            yaw_error_rad=abs(float(error)),settled_yaw_error_after_3s_rad=settled_angle,
            path_length_m=float(np.linalg.norm(np.diff(xy[mask],axis=0),axis=1).sum()),
            elapsed_source_s=end-start,elapsed_wall_s=result["elapsed_wall_s"],actual_xy_m=actual.tolist())
        row["passed"]=(row["status"]==4 and max(row["position_error_m"],row["settled_position_error_after_3s_m"])<=limits["goal_position_error_limit_m"]
                       and max(row["yaw_error_rad"],settled_angle)<=limits["goal_yaw_error_limit_rad"])
        report["goals"].append(row)
    authority=json.loads((out/"simulation/motion_authority.json").read_text())
    takeovers=[e for e in authority["safety_events"] if e["event"]=="safety_takeover"]
    report["expected_task_handover_count"]=sum(e["reason"]=="navigation_task_released" for e in takeovers)
    report["unexpected_takeovers"]=[e for e in takeovers if e["reason"]!="navigation_task_released"]
    agent_summary=out/"agent/summary.json"
    if agent_summary.exists():
        agent=json.loads(agent_summary.read_text())
        name=agent.get("final_feedback",{}).get("destination_id")
        binding=json.loads((out/"bindings.json").read_text())
        if name in binding["destinations"]:
            target=binding["destinations"][name]
            error=float(np.linalg.norm(xy[-1]-[target["x_m"],target["y_m"]]))
            angle=abs(math.atan2(math.sin(yaw[-1]-target["yaw_rad"]),math.cos(yaw[-1]-target["yaw_rad"])))
            report["agent_final_settled_goal"]=dict(destination_id=name,position_error_m=error,yaw_error_rad=angle,
                evaluation_source_time_s=float(t[-1]),method="final physical hold after completed Agent goal, no later motion task",
                passed=agent["passed"] and error<=limits["goal_position_error_limit_m"] and angle<=limits["goal_yaw_error_limit_rad"])
    report["passed"]=(all(g["passed"] for g in report["goals"]) and not report["unexpected_takeovers"]
        and report["obstacle_contact_samples"]==0 and json.loads((out/"summary.json").read_text())["control_gates"]["passed"]
        and report.get("agent_final_settled_goal",{}).get("passed",True))
    report["performance"]=performance(out)
    fig,ax=plt.subplots(figsize=(10,9));ax.imshow(world["occupied"],origin="lower",cmap="Greys",alpha=.5,
        extent=[world["x_m"][0],world["x_m"][-1],world["y_m"][0],world["y_m"][-1]])
    ax.plot(xy[:,0],xy[:,1],color="#007f8b",label="Physical trajectory (post-hoc)",linewidth=1.5)
    for r in nav["results"]:
        g=r["target"];ax.plot(g["x_m"],g["y_m"],"o",color="#cc7842");ax.annotate(r["destination_id"],(g["x_m"],g["y_m"]),xytext=(4,4),textcoords="offset points")
    ax.set_aspect("equal");ax.legend();ax.set(xlabel="map x [m]",ylabel="map y [m]",title="Real Nav2 Actions through original balance control")
    fig.tight_layout();fig.savefig(out/"physical_navigation.png",dpi=180);plt.close(fig)
    (out/"evaluation_posthoc.json").write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report),flush=True)
    if not report["passed"]:raise SystemExit(1)


if __name__=="__main__":main()
