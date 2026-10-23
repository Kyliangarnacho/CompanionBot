"""Offline asset/physics audit, never a source of robot localization or maps.

Projects the actual convex contact geometry through the robot's height slab,
using SciPy hulls and raster morphology. This evaluation grid is not published
to SLAM, AMCL, costmaps or a controller. Real scans use VirtualPlanarLidar.
"""
import argparse
import json
import math
from pathlib import Path
import time

import mujoco
import numpy as np
from scipy.spatial import ConvexHull
from scipy.ndimage import distance_transform_edt, label
from matplotlib.path import Path as PolygonPath
import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt

from companionbot_contracts.paths import repository_root
from companionbot_contracts.spatial_geometry import load_spatial_geometry
from companionbot_simulation.core.retail_scene import prepare_retail_model
from companionbot_simulation.core.spatial_scene import VirtualPlanarLidar


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    out = args.output_dir.resolve(); out.mkdir(parents=True, exist_ok=True)
    root, geometry = repository_root(), load_spatial_geometry()
    path = prepare_retail_model(root/"models/minisegway/mini_segway.xml", out/"bookstore_robot.xml")
    started = time.perf_counter()
    model = mujoco.MjModel.from_xml_path(str(path)); data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    resolution = .025
    xs, ys = np.arange(-8., 8.025, resolution), np.arange(-7.6, 7.625, resolution)
    occupied = np.zeros((len(ys), len(xs)), dtype=bool)
    nconvex = 0
    for gid in np.flatnonzero(model.geom_group == 4):
        mid = model.geom_dataid[gid]
        start, count = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        local = model.mesh_vert[start:start+count]
        native = local @ data.geom_xmat[gid].reshape(3, 3).T + data.geom_xpos[gid]
        vertices = native @ geometry.mj_to_ros.T
        if vertices[:, 2].min() > .22 or vertices[:, 2].max() < .005:
            continue
        fstart, fcount = model.mesh_faceadr[mid], model.mesh_facenum[mid]
        faces = model.mesh_face[fstart:fstart+fcount]
        edges = np.unique(np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [0, 2]]]), axis=1), axis=0)
        inside = vertices[(vertices[:, 2] >= .005) & (vertices[:, 2] <= .22)]
        points = [inside]
        for z in (.005, .22):
            a, b = vertices[edges[:, 0]], vertices[edges[:, 1]]
            mask = ((a[:, 2]-z)*(b[:, 2]-z) <= 0) & (abs(a[:, 2]-b[:, 2]) > 1e-12)
            a, b = a[mask], b[mask]
            fraction = (z-a[:, 2])/(b[:, 2]-a[:, 2])
            points.append(a + fraction[:, None]*(b-a))
        points = np.concatenate(points)[:, :2]
        if len(points) < 3:
            continue
        try:
            polygon = points[ConvexHull(points).vertices]
        except Exception as error:
            raise RuntimeError("contact_projection_failed:"+model.geom(gid).name) from error
        ix = np.flatnonzero((xs >= polygon[:, 0].min()-resolution) & (xs <= polygon[:, 0].max()+resolution))
        iy = np.flatnonzero((ys >= polygon[:, 1].min()-resolution) & (ys <= polygon[:, 1].max()+resolution))
        if not len(ix) or not len(iy):
            continue
        xx, yy = np.meshgrid(xs[ix], ys[iy])
        # A raster cell intersecting a thin 2mm wall is occupied even when its
        # centre is outside the wall. Path's radius is twice the contour margin:
        # expand by the cell circumradius before centre sampling. Otherwise
        # axis-aligned walls can vanish from the independent evaluation grid.
        occupied[np.ix_(iy, ix)] |= PolygonPath(polygon).contains_points(
            np.column_stack((xx.ravel(), yy.ravel())), radius=resolution*math.sqrt(2)).reshape(xx.shape)
        nconvex += 1
    clearance = distance_transform_edt(~occupied)*resolution
    radius = json.loads((root/"ros2_ws/src/companionbot_bringup/config/nav2_geometry.json").read_text())["robot_radius_m"]
    passable = clearance > radius+resolution*math.sqrt(2)
    components, count = label(passable)
    def location(x, y):
        row, column = round((y-ys[0])/resolution), round((x-xs[0])/resolution)
        return dict(clearance_m=float(clearance[row, column]), passable=bool(passable[row, column]), component=int(components[row, column]))
    origin = location(0, 0)
    probes = {name: dict(x_m=x, y_m=y, **location(x, y)) for name, x, y in
              (("origin", 0, 0), ("east", 5, 0), ("north_east", 5, 2), ("north_west", -5, 2),
               ("west", -5, 0), ("south_west", -5, -6), ("south_center", 0, -6),
               ("service_approach", 0, 2.8), ("entrance_approach", -2.5, 6.2))}
    lidar = VirtualPlanarLidar(model); scan = lidar.sample(model, data)
    finite = np.asarray(scan.ranges_m); finite = finite[np.isfinite(finite) & (finite > 0)]
    timings = []
    for _ in range(20):
        t = time.perf_counter(); lidar.sample(model, data); timings.append(time.perf_counter()-t)
    summary = dict(passed=bool(origin["passable"] and len(finite) > 180),
        scope="offline static geometry and sensor audit; evaluation grid never consumed by runtime navigation",
        models=26, instances=138, ngeom=model.ngeom, nmesh=model.nmesh, ntex=model.ntex,
        physics_dt_s=model.opt.timestep, robot_radius_m=radius, projected_contact_geoms=nconvex,
        collision_projection_z_m=[.005, .22], raster_resolution_m=resolution,
        raster_rule="conservative cell intersection; polygon expanded by cell circumradius",
        passable_components=count, origin=origin, probes=probes,
        lidar=dict(rays=len(scan.ranges_m), finite_returns=len(finite), min_range_m=float(finite.min()),
                   max_range_m=float(finite.max()), sample_mean_wall_s=float(np.mean(timings)),
                   sample_p95_wall_s=float(np.percentile(timings, 95))),
        audit_wall_s=time.perf_counter()-started)
    np.savez_compressed(out/"collision_projection_posthoc.npz", occupied=occupied, passable=passable,
                        clearance_m=clearance, x_m=xs, y_m=ys)
    fig, ax = plt.subplots(figsize=(11, 10))
    ax.imshow(occupied, extent=[xs[0], xs[-1], ys[0], ys[-1]], origin="lower", cmap="Greys", interpolation="nearest")
    ax.contour(xs, ys, passable, levels=[.5], colors=["#cf8b26"], linewidths=.5)
    for name, entry in probes.items():
        ax.plot(entry["x_m"], entry["y_m"], "o", color="#087f8c" if entry["passable"] else "#c22d39")
        ax.annotate(name, (entry["x_m"], entry["y_m"]), xytext=(4, 4), textcoords="offset points", fontsize=8)
    ax.set(xlabel="ROS world x [m]", ylabel="ROS world y [m]", title="AWS Bookstore: actual contact geometry through robot height\nOrange boundary includes robot footprint and raster uncertainty")
    ax.set_aspect("equal"); fig.tight_layout(); fig.savefig(out/"collision_projection_posthoc.png", dpi=180); plt.close(fig)
    (out/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    print(json.dumps(summary), flush=True)
    if not summary["passed"]:
        raise RuntimeError("bookstore_static_geometry_or_lidar_audit_failed")


if __name__ == "__main__":
    main()
