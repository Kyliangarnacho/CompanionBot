"""Post-hoc retail map/state evaluation; no runtime or localization consumer."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt, map_coordinates
import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt

from evaluate_slam_map import rows, pose_rows


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("session", type=Path)
    cli.add_argument("--world-audit", type=Path, required=True)
    args = cli.parse_args(); out=args.session.resolve()
    limits=json.loads((Path(__file__).resolve().parents[2]/"models/minisegway/retail_navigation/acceptance.json").read_text())["mapping"]
    truth=rows(out/"simulation/truth_posthoc.jsonl");states=rows(out/"simulation/sensorized_state.jsonl")
    origin=np.asarray(truth[0]["position_ros"][:2]);heading=truth[0]["rpy_ros"][2]
    rotation=np.array([[math.cos(heading),-math.sin(heading)],[math.sin(heading),math.cos(heading)]])
    xy=(np.array([t["position_ros"][:2] for t in truth])-origin)@rotation
    yaw=np.unwrap([t["rpy_ros"][2] for t in truth])-heading
    ts=np.array([t["time_s"] for t in truth]);gt=np.column_stack([xy,yaw])
    metrics=dict(gt_posthoc_only=True,alignment="initial origin/yaw only; no trajectory fitting",
        path_length_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),
        pitch_peak_deg=float(np.rad2deg(max(abs(s["pitch_rad"]) for s in states))),
        xy_span_m=np.ptp(xy,axis=0).tolist())
    trajectories={}
    for key in ("pose","odom"):
        values=pose_rows(rows(out/(key+".jsonl")),nested=True)
        target=np.column_stack([np.interp(values[:,0],ts,gt[:,i]) for i in range(3)])
        error=values[:,1:]-target;error[:,2]=np.arctan2(np.sin(error[:,2]),np.cos(error[:,2]))
        distance=np.linalg.norm(error[:,:2],axis=1)
        metrics[key]=dict(samples=len(values),position_rmse_m=float(np.sqrt(np.mean(distance**2))),
            position_p95_m=float(np.percentile(distance,95)),position_peak_m=float(distance.max()),
            yaw_rmse_rad=float(np.sqrt(np.mean(error[:,2]**2))))
        trajectories[key]=values
    message=rows(out/"map.jsonl")[-1];info=message["info"];res=info["resolution"]
    grid=np.asarray(message["data"]).reshape(info["height"],info["width"])
    yy,xx=np.nonzero(grid>=65)
    occupied=np.column_stack([info["origin"]["position"]["x"]+(xx+.5)*res,
                              info["origin"]["position"]["y"]+(yy+.5)*res])
    world=np.load(args.world_audit/"collision_projection_posthoc.npz")
    # Conservative 0.5..22cm physical obstacle projection. Score occupied cells
    # against real contact geometry; never feed this evaluation raster to SLAM.
    distance=distance_transform_edt(~world["occupied"])*float(np.diff(world["x_m"])[0])
    global_xy=occupied@rotation.T+origin
    coordinates=np.vstack([(global_xy[:,1]-world["y_m"][0])/.025,(global_xy[:,0]-world["x_m"][0])/.025])
    errors=map_coordinates(distance,coordinates,order=1,mode="constant",cval=10.)
    metrics["map"]=dict(resolution_m=res,shape_cells=list(grid.shape),occupied_cells=len(occupied),
        known_cells=int(np.sum(grid>=0)),occupied_geometry_p95_m=float(np.percentile(errors,95)),
        occupied_precision_within_0p10m=float(np.mean(errors<=.10)),
        comparison="occupied cells to independent conservative physical obstacle projection")
    arrivals=json.loads((out/"arrivals.json").read_text()); resources=json.loads((out/"resources.json").read_text())
    metrics["performance"]={}
    for key in ("scan","odom","pose","map"):
        a=arrivals.get(key,[])
        if len(a)>1:metrics["performance"][key+"_hz_wall"]=(len(a)-1)/(a[-1]-a[0])
    if len(arrivals["scan"])>1:
        metrics["performance"]["observed_sim_realtime_factor"]=(ts[-1]-.1)/(arrivals["scan"][-1]-arrivals["scan"][0])
    for pid in {r["pid"] for r in resources}:
        p=[r for r in resources if r["pid"]==pid]
        if len(p)>1:
            metrics["performance"][str(pid)]=dict(name=p[0]["name"],peak_rss_mib=max(r["rss_bytes"] for r in p)/2**20,
                cpu_percent_one_core=100*(p[-1]["cpu_s"]-p[0]["cpu_s"])/(p[-1]["wall_s"]-p[0]["wall_s"]))
    metrics["checks"]={"geometry_p95":metrics["map"]["occupied_geometry_p95_m"]<=limits["occupied_geometry_p95_limit_m"],
        "occupied_precision":metrics["map"]["occupied_precision_within_0p10m"]>=limits["occupied_precision_within_0p10m_min"],
        "slam_position":metrics["pose"]["position_rmse_m"]<=limits["slam_position_rmse_limit_m"],
        "slam_yaw":metrics["pose"]["yaw_rmse_rad"]<=limits["slam_yaw_rmse_limit_rad"],
        "frozen_control":json.loads((out/"summary.json").read_text())["control_gates"]["passed"]}
    metrics["passed"]=all(metrics["checks"].values())
    fig,ax=plt.subplots(figsize=(10,10)); extent=[info["origin"]["position"]["x"],info["origin"]["position"]["x"]+info["width"]*res,
        info["origin"]["position"]["y"],info["origin"]["position"]["y"]+info["height"]*res]
    ax.imshow(grid,origin="lower",extent=extent,cmap="gray_r",vmin=-1,vmax=100)
    ax.plot(xy[:,0],xy[:,1],label="Physical trajectory (post-hoc)",linewidth=1)
    ax.plot(trajectories["pose"][:,1],trajectories["pose"][:,2],label="SLAM",linewidth=1)
    ax.set_aspect("equal");ax.legend();ax.set(xlabel="map x [m]",ylabel="map y [m]",title="AWS Bookstore: measured SLAM map and physical path")
    fig.tight_layout();fig.savefig(out/"map_evaluation.png",dpi=180);plt.close(fig)
    (out/"evaluation.json").write_text(json.dumps(metrics,indent=2)+"\n");print(json.dumps(metrics),flush=True)
    if not metrics["passed"]:raise SystemExit(1)


if __name__=="__main__":main()
