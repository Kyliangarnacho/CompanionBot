"""Offline map geometry / localization evaluation. Never imported by runtime."""
import argparse
import json
from pathlib import Path
import math
import subprocess

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException
from geometry_msgs.msg import TransformStamped
from rosidl_runtime_py.set_message import set_message_fields


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def seconds(stamp):
    return stamp['sec']+stamp['nanosec']*1e-9


def yaw(q):
    return Rotation.from_quat([q[k] for k in ('x','y','z','w')]).as_euler('xyz')[2]


def pose_rows(messages, nested=False):
    return np.array([[seconds(m['header']['stamp']),
        (m['pose']['pose'] if nested else m['pose'])['position']['x'],
        (m['pose']['pose'] if nested else m['pose'])['position']['y'],
        yaw((m['pose']['pose'] if nested else m['pose'])['orientation'])] for m in messages])


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('session', type=Path)
    args = cli.parse_args()
    out = args.session.resolve()
    root = Path(__file__).resolve().parents[2]
    limits = json.loads((root/'models/minisegway/slam_toolbox_integration/acceptance.json').read_text())
    inspector = out.parent/'inspect_slam_pose_graph'
    source = root/'tools/verification/inspect_slam_pose_graph.cpp'
    if not inspector.exists() or inspector.stat().st_mtime < source.stat().st_mtime:
        includes = ['-I/opt/ros/jazzy/include','-I/usr/include/eigen3']
        includes += ['-I'+str(p) for p in Path('/opt/ros/jazzy/include').iterdir() if p.is_dir()]
        subprocess.run(['c++','-std=c++17','-O2',*includes,str(source),'-L/opt/ros/jazzy/lib',
            '-Wl,-rpath,/opt/ros/jazzy/lib','-lkartoSlamToolbox','-lrclcpp','-lrclcpp_lifecycle',
            '-lrcutils','-lboost_serialization','-o',str(inspector)],check=True)
    with (out/'inspect.txt').open('w') as handle:
        subprocess.run([str(inspector),str(out/'maps/indoor'),str(out/'pose_graph.json')],
            stdout=handle,stderr=subprocess.STDOUT,check=True)
    truth = rows(out/'simulation/truth_posthoc.jsonl')
    states = rows(out/'simulation/sensorized_state.jsonl')
    origin = np.asarray(truth[0]['position_ros'][:2])
    heading = truth[0]['rpy_ros'][2]
    rotation = np.array([[math.cos(heading),-math.sin(heading)],[math.sin(heading),math.cos(heading)]])
    gt_xy = (np.asarray([r['position_ros'][:2] for r in truth])-origin) @ rotation
    gt_yaw = np.unwrap([r['rpy_ros'][2] for r in truth])-heading
    gt_t = np.array([r['time_s'] for r in truth])
    gt = np.column_stack([gt_xy,gt_yaw])
    metrics = {'alignment':'one initial sensor/GT origin and yaw only; no trajectory or map best fit',
        'gt_posthoc_only':True,
        'path_length_m':float(np.linalg.norm(np.diff(gt_xy,axis=0),axis=1).sum()),
        'xy_span_m':np.ptp(gt_xy,axis=0).tolist(),
        'yaw_accumulated_rad':float(gt_yaw[-1]-gt_yaw[0]),
        'pitch_abs_peak_deg':float(np.rad2deg(max(abs(s['pitch_rad']) for s in states)))}
    histories = {}
    for key in ('pose','odom'):
        data = pose_rows(rows(out/(key+'.jsonl')), nested=True)
        target = np.column_stack([np.interp(data[:,0],gt_t,gt[:,i]) for i in range(3)])
        error = data[:,1:]-target
        error[:,2] = np.arctan2(np.sin(error[:,2]),np.cos(error[:,2]))
        xy = np.linalg.norm(error[:,:2],axis=1)
        metrics[key] = {'samples':len(data), 'position_rmse_m':float(np.sqrt(np.mean(xy**2))),
            'position_p95_m':float(np.percentile(xy,95)), 'position_peak_m':float(xy.max()),
            'yaw_rmse_rad':float(np.sqrt(np.mean(error[:,2]**2))),
            'yaw_peak_rad':float(np.abs(error[:,2]).max())}
        histories[key] = data
    m = rows(out/'map.jsonl')[-1]
    info = m['info']; res = info['resolution']
    grid = np.array(m['data']).reshape(info['height'],info['width'])
    yy, xx = np.indices(grid.shape)
    points = np.column_stack([info['origin']['position']['x']+(xx.ravel()+.5)*res,
                              info['origin']['position']['y']+(yy.ravel()+.5)*res])
    occupied = points[grid.ravel()>=65]
    # Independent simulator scene metadata stays in this offline evaluator.
    scene = json.loads((out/'simulation/indoor_model.json').read_text())
    distances = []
    for name, center, half in scene['walls_ros_startup']:
        local = occupied @ rotation.T + origin - np.array(center[:2])
        d = np.abs(local)-half[:2]
        distances.append(np.linalg.norm(np.maximum(d,0),axis=1)+np.minimum(np.max(d,axis=1),0))
    distance = np.min(np.abs(distances),axis=0)
    # Sample the inside faces of all four room walls; box faces are scored by
    # occupied precision but do not artificially increase room completeness.
    walls = {name:(np.array(c),np.array(h)) for name,c,h in scene['walls_ros_startup']}
    xmin = walls['room_back'][0][0]+walls['room_back'][1][0]
    xmax = walls['room_front'][0][0]-walls['room_front'][1][0]
    ymin = walls['room_right'][0][1]+walls['room_right'][1][1]
    ymax = walls['room_left'][0][1]-walls['room_left'][1][1]
    xs,ys = np.arange(xmin,xmax,res),np.arange(ymin,ymax,res)
    expected_world = np.vstack([np.column_stack([xs,np.full(len(xs),ymin)]),
        np.column_stack([xs,np.full(len(xs),ymax)]),np.column_stack([np.full(len(ys),xmin),ys]),
        np.column_stack([np.full(len(ys),xmax),ys])])
    expected = (expected_world-origin) @ rotation
    boundary_dist = cKDTree(occupied).query(expected)[0]
    world_points = points @ rotation.T+origin
    inside = ((world_points[:,0]>xmin+.1)&(world_points[:,0]<xmax-.1)
              &(world_points[:,1]>ymin+.1)&(world_points[:,1]<ymax-.1))
    metrics['map'] = {'resolution_m':res,'shape_cells':list(grid.shape),
        'occupied_cells':len(occupied), 'occupied_geometry_p50_m':float(np.percentile(distance,50)),
        'occupied_geometry_p95_m':float(np.percentile(distance,95)),
        'occupied_precision_within_0p10m':float(np.mean(distance<=.1)),
        'room_boundary_coverage_within_0p10m':float(np.mean(boundary_dist<=.1)),
        'known_interior_fraction':float(np.mean(grid.ravel()[inside]>=0))}
    arrivals = json.loads((out/'arrivals.json').read_text())
    scans = rows(out/'scan.jsonl')
    buffer = Buffer(cache_time=Duration(seconds=400))
    for key, static in [('tf_static',True),('tf',False)]:
        for message in rows(out/(key+'.jsonl')):
            for raw in message['transforms']:
                transform = TransformStamped();set_message_fields(transform,raw)
                (buffer.set_transform_static if static else buffer.set_transform)(transform,'offline_actual_dds')
    covered, missing = 0, []
    for scan in scans:
        t = seconds(scan['header']['stamp'])
        try:
            buffer.lookup_transform('odom',scan['header']['frame_id'],Time(nanoseconds=round(t*1e9)))
            covered += 1
        except TransformException as error:
            missing.append({'time_s':t,'reason':str(error)})
    metrics['source_validity'] = {'scans_with_source_time_tf':covered,'scans_received':len(scans),
        'tf_coverage_fraction':covered/len(scans),'unavailable':missing,
        'generated_scans':round(gt_t[-1]/.1),
        'received_fraction_of_generated':len(scans)/round(gt_t[-1]/.1)}
    # Independent physical ray check; GT is used only here, after the run.
    lidar = scene['sensor']
    mount_rotation = Rotation.from_euler('xyz',lidar['rpy_rad']).as_matrix()
    floor_counts, finite_counts = [], []
    for scan in scans:
        t = seconds(scan['header']['stamp'])
        index = int(np.argmin(abs(gt_t-t)))
        if abs(gt_t[index]-t)>.001:continue
        body = Rotation.from_euler('xyz',truth[index]['rpy_ros']).as_matrix()
        height = truth[index]['position_ros'][2]+(body @ np.array(lidar['xyz_m']))[2]
        angles=scan['angle_min']+np.arange(len(scan['ranges']))*scan['angle_increment']
        directions=np.column_stack([np.cos(angles),np.sin(angles),np.zeros_like(angles)]) @ (body@mount_rotation).T
        ranges=np.array(scan['ranges']);valid=np.isfinite(ranges)&(ranges>=scan['range_min'])&(ranges<=scan['range_max'])
        z=height+directions[valid,2]*ranges[valid]
        floor_counts.append(int(np.sum(abs(z)<.02)));finite_counts.append(int(valid.sum()))
    metrics['physical_scan_posthoc']={'finite_returns':sum(finite_counts),
        'floor_returns_within_2cm':sum(floor_counts),
        'floor_fraction':sum(floor_counts)/sum(finite_counts),
        'floor_returns_peak_per_scan':max(floor_counts),
        'runtime_compensation':'none; raw ranges and physical TF retained'}
    scan_arrival = {round(seconds(m['header']['stamp']),6):t for m,t in zip(scans,arrivals['scan'])}
    latency = [t-scan_arrival[round(seconds(m['header']['stamp']),6)]
        for m,t in zip(rows(out/'pose.jsonl'),arrivals['pose'])
        if round(seconds(m['header']['stamp']),6) in scan_arrival]
    metrics['performance'] = {'scan_to_pose_arrival_wall_s_p50':float(np.percentile(latency,50)),
        'scan_to_pose_arrival_wall_s_p95':float(np.percentile(latency,95)),
        'pose_hz_wall':(len(arrivals['pose'])-1)/(arrivals['pose'][-1]-arrivals['pose'][0]),
        'map_hz_wall':(len(arrivals['map'])-1)/(arrivals['map'][-1]-arrivals['map'][0])}
    resource = json.loads((out/'resources.json').read_text())
    metrics['performance']['processes'] = {}
    for pid in sorted({r['pid'] for r in resource}):
        rs = [r for r in resource if r['pid']==pid]
        if len(rs)<2: continue
        metrics['performance']['processes'][str(pid)] = {'name':rs[0]['name'],
            'mean_cpu_percent_one_core':100*(rs[-1]['cpu_s']-rs[0]['cpu_s'])/(rs[-1]['wall_s']-rs[0]['wall_s']),
            'peak_rss_mib':max(r['rss_bytes'] for r in rs)/2**20}
    authorities = {}
    for message in rows(out/'diagnostics.jsonl'):
        for status in message['status']:
            if status['name'].endswith('/tf_authority'):
                assert status['level'] != 2, status
                for value in status['values']:
                    if not value['key'].startswith('meta.'):
                        authorities.setdefault(value['key'],set()).add(value['value'])
    metrics['tf_authorities'] = {k:sorted(v) for k,v in authorities.items()}
    graph = json.loads((out/'pose_graph.json').read_text())
    cross_lap = [e for e in graph['edges'] if abs(e['source_time_s']-e['target_time_s'])>20]
    published = {round(v[0],6):v[1:3] for v in histories['pose']}
    changes = [np.linalg.norm(np.asarray(s['pose'][:2])-published[round(s['time_s'],6)])
        for s in graph['scans'] if round(s['time_s'],6) in published]
    # Fresh mapping has no deserialize/manual-closure calls. In upstream 2.8.5
    # TryCloseLoop -> CorrectPoses is the path that optimizes historical nodes.
    # Cross-lap constraints AND backward pose correction establish real closure.
    metrics['pose_graph'] = {'nodes':len(graph['scans']),'edges':len(graph['edges']),
        'cross_lap_edges_over_20s':len(cross_lap),
        'historical_pose_correction_peak_m':float(max(changes,default=0.)),
        'evidence':'installed Karto deserialization; cross-lap edges and historical correction; not a guessed event count'}
    checks = {
        'geometry_p95':metrics['map']['occupied_geometry_p95_m']<=limits['occupied_geometry_p95_limit_m'],
        'occupied_precision':metrics['map']['occupied_precision_within_0p10m']>=limits['occupied_precision_within_0p10m_min'],
        'room_coverage':metrics['map']['room_boundary_coverage_within_0p10m']>=limits['room_boundary_coverage_within_0p10m_min'],
        'known_interior':metrics['map']['known_interior_fraction']>=limits['known_interior_fraction_min'],
        'position_rmse':metrics['pose']['position_rmse_m']<=limits['slam_position_rmse_limit_m'],
        'yaw_rmse':metrics['pose']['yaw_rmse_rad']<=limits['slam_yaw_rmse_limit_rad'],
        'actual_motion':metrics['path_length_m']>=limits['actual_path_length_min_m']
            and all(v>=limits['actual_xy_span_each_min_m'] for v in metrics['xy_span_m']),
        'map_authority':len(authorities.get('odom',[]))==1 and '|slam_toolbox|/tf|' in next(iter(authorities.get('odom',[])),''),
        'loop_closure':len(cross_lap)>0 and max(changes,default=0.)>1e-5,
        'frozen_control':json.loads((out/'summary.json').read_text())['control_gates']['passed'],
        'motion_authority':json.loads((out/'summary.json').read_text()).get('motion_authority_passed',False),
        'source_time_tf':metrics['source_validity']['tf_coverage_fraction']>=limits['source_time_tf_coverage_min'],
    }
    checks = {k:bool(v) for k,v in checks.items()}
    metrics['checks'],metrics['passed'] = checks,all(checks.values())
    (out/'evaluation.json').write_text(json.dumps(metrics,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(13,5))
    axes[0].imshow(grid,origin='lower',cmap='gray_r',vmin=-1,vmax=100,
        extent=[info['origin']['position']['x'],info['origin']['position']['x']+info['width']*res,
                info['origin']['position']['y'],info['origin']['position']['y']+info['height']*res])
    axes[0].plot(expected[:,0],expected[:,1],'.',ms=1,label='independent physical walls')
    axes[0].plot(gt_xy[:,0],gt_xy[:,1],label='GT post-hoc')
    axes[0].plot(histories['pose'][:,1],histories['pose'][:,2],label='SLAM')
    axes[0].axis('equal');axes[0].legend();axes[0].set_title('Occupancy and physical geometry')
    axes[1].plot([s['time_s'] for s in states],np.rad2deg([s['pitch_rad'] for s in states]))
    axes[1].set(xlabel='source time [s]',ylabel='estimated pitch [deg]',title='Physical pitch retained')
    fig.tight_layout();fig.savefig(out/'map_evaluation.png',dpi=160);plt.close(fig)
    print(json.dumps(metrics,indent=2))
    if not metrics['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
