"""Actual DDS SLAM acceptance, with independent post-hoc physical evaluation.

This diagnostic never publishes motion, sensors or robot TF. The simulation
owns its reproducible reference sequence and all existing MCU safety gates.
"""
import argparse
from collections import defaultdict
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import platform
import psutil
import shutil
import hashlib
from ament_index_python.packages import get_package_share_directory

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from rosidl_runtime_py.convert import message_to_ordereddict


def seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--output-dir', type=Path, required=True)
    cli.add_argument('--scenario', default='mapping')
    cli.add_argument('--duration', type=float)
    cli.add_argument('--extra', action='append', default=[])
    cli.add_argument('--feature-launch', default='slam_simulation.launch.py')
    cli.add_argument('--baseline', action='store_true')
    args = cli.parse_args()
    if args.duration is None:
        from companionbot_contracts.paths import repository_root
        args.duration = (json.loads((repository_root()/'ros2_ws/src/companionbot_bringup/config/mapping_route.json').read_text())['duration_s']
                         if args.scenario == 'mapping' else 14.)
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    parameter_source = Path(get_package_share_directory('companionbot_bringup'))/'config/slam.yaml'
    for option in args.extra:
        if option.startswith('slam_params_file:='):
            parameter_source = Path(option.split(':=',1)[1]).resolve()
    shutil.copyfile(parameter_source,out/'slam_parameters.yaml')
    (out/'input_manifest.json').write_text(json.dumps({'slam_parameter_source':str(parameter_source),
        'slam_parameter_sha256':hashlib.sha256((out/'slam_parameters.yaml').read_bytes()).hexdigest(),
        'extra_launch_arguments':args.extra},indent=2)+'\n')
    os.environ['ROS_LOG_DIR'] = str(out/'ros_logs')
    os.environ.setdefault('ROS_DOMAIN_ID', '71')
    rclpy.init()
    node = Node('slam_acceptance_probe')
    messages = defaultdict(list)
    arrivals = defaultdict(list)
    resources = []
    buffer = Buffer(cache_time=Duration(seconds=400))
    def tf(message, static=False):
        for transform in message.transforms:
            (buffer.set_transform_static if static else buffer.set_transform)(transform, 'transport_audited')
        messages['tf_static' if static else 'tf'].append(message_to_ordereddict(message))
    node.create_subscription(TFMessage, '/tf', tf, 100)
    node.create_subscription(TFMessage, '/tf_static', lambda m: tf(m, True),
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for key, cls, topic, qos in [
        ('map', OccupancyGrid, '/map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ('pose', PoseWithCovarianceStamped, '/pose', 10),
        ('odom', Odometry, '/companionbot/odom', qos_profile_sensor_data),
        ('scan', LaserScan, '/companionbot/scan', qos_profile_sensor_data),
        ('diagnostics', DiagnosticArray, '/companionbot/diagnostics', 100),
    ]:
        def receive(m, key=key):
            messages[key].append(m)
            arrivals[key].append(time.monotonic())
        node.create_subscription(cls, topic, receive, qos)
    processes, handles = [], []
    def launch(label, name, options):
        handle = (out/(label+'.txt')).open('w'); handles.append(handle)
        process = subprocess.Popen([sys.executable, '/opt/ros/jazzy/bin/ros2', 'launch',
            'companionbot_bringup', name, *options], stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process
    summary = {'passed': False, 'scenario': args.scenario,
        'platform':{'uname':platform.uname()._asdict(), 'cpu_count':os.cpu_count(),
                    'python':sys.executable, 'physical_hardware':False}}
    try:
        if args.baseline:
            launch('slam', 'slam.launch.py', ['slam_params_file:='+str(out/'slam_parameters.yaml')])
            process = launch('simulation', 'spatial_simulation.launch.py', [
                'scenario:='+args.scenario, 'duration_s:='+str(args.duration), 'output_dir:='+str(out/'simulation'),
                'finish_grace_s:=5.0', *args.extra])
        else:
            process = launch('launch', args.feature_launch, ['scenario:='+args.scenario,
                'duration_s:='+str(args.duration), 'output_dir:='+str(out/'simulation'), 'record_truth:=true',
                *args.extra,'slam_params_file:='+str(out/'slam_parameters.yaml')])
        deadline = time.monotonic() + max(60., args.duration*2+40)
        next_resource = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if time.monotonic() >= next_resource:
                next_resource = time.monotonic()+.5
                for child in psutil.Process(process.pid).children(recursive=True) if process.poll() is None else []:
                    try:
                        cpu = child.cpu_times()
                        resources.append({'wall_s':time.monotonic(), 'pid':child.pid, 'name':child.name(),
                            'cpu_s':cpu.user+cpu.system, 'rss_bytes':child.memory_info().rss})
                    except psutil.NoSuchProcess:
                        pass
            if (out/'simulation/frozen_result_posthoc.json').exists() and messages['map']:
                # Allow upstream's wall-timed occupancy update to finish.
                end = time.monotonic()+3
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.02)
                break
            if process.poll() is not None:
                break
        summary['counts'] = {k:len(v) for k,v in messages.items()}
        summary['nodes'] = node.get_node_names_and_namespaces()
        summary['topics'] = {topic:types for topic,types in node.get_topic_names_and_types()}
        assert messages['map'], 'no actual OccupancyGrid; inspect launch log'
        assert (out/'simulation/frozen_result_posthoc.json').exists(), 'simulation terminated before full duration'
        assert process.poll() is None or process.returncode == 0, 'required runtime failed'
        if not args.baseline:
            from verify_spatial_foundation import control_gates
            summary['control_gates'] = control_gates(json.loads((out/'simulation/frozen_result_posthoc.json').read_text()), args.duration)
            assert summary['control_gates']['passed'], summary['control_gates']
            if args.scenario == 'mapping':
                authority = json.loads((out/'simulation/motion_authority.json').read_text())
                summary['motion_authority'] = {k:authority[k] for k in ('arbiter_state','arbiter_reason','safety_events')}
                summary['motion_authority_passed'] = authority['arbiter_state']=='NORMAL' and not authority['safety_events']
                assert summary['motion_authority_passed'], summary['motion_authority']
            map_prefix = out/'maps/indoor'
            completed = subprocess.run(['bash', 'scripts/save_slam_map.sh', str(map_prefix)],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=55)
            (out/'save.txt').write_text(completed.stdout)
            assert completed.returncode == 0, completed.stdout
        latest = messages['map'][-1]
        summary['map'] = {'width':latest.info.width, 'height':latest.info.height,
            'resolution':latest.info.resolution, 'occupied_cells':sum(v>=65 for v in latest.data),
            'free_cells':sum(0<=v<25 for v in latest.data)}
        summary['passed'] = True
    except BaseException as error:
        summary['error'] = repr(error)
        raise
    finally:
        for p in processes:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
                try: p.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGTERM); p.wait(timeout=8)
        summary['exit_codes'] = [p.returncode for p in processes]
        for key in ('map', 'pose', 'odom', 'scan', 'diagnostics'):
            (out/(key+'.jsonl')).write_text(''.join(json.dumps(message_to_ordereddict(m))+'\n' for m in messages[key]))
        for key in ('tf', 'tf_static'):
            (out/(key+'.jsonl')).write_text(''.join(json.dumps(m)+'\n' for m in messages[key]))
        (out/'resources.json').write_text(json.dumps(resources)+'\n')
        (out/'arrivals.json').write_text(json.dumps(arrivals)+'\n')
        if any(p.returncode not in (0, -signal.SIGINT) for p in processes):
            summary['passed'] = False
        for h in handles: h.close()
        (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        node.destroy_node(); rclpy.try_shutdown()
    print(json.dumps(summary), flush=True)
    if not summary['passed']:
        raise RuntimeError('SLAM integration failed; inspect saved evidence')


if __name__ == '__main__':
    main()
