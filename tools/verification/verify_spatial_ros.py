"""Headless actual DDS/TF/lifecycle acceptance; never a robot command source.

Uses an isolated local ROS domain and finite MuJoCo diagnostics. Fault injection
is explicitly labelled and applies only to this test graph.
"""
import argparse
from collections import defaultdict
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu, LaserScan
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from rosidl_runtime_py.convert import message_to_ordereddict
from companionbot_interfaces.msg import BalanceState
from companionbot_contracts.paths import repository_root
from verify_runtime_ros import graph


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def quaternion(value):
    return [value.x, value.y, value.z, value.w]


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--output-dir', type=Path, required=True)
    cli.add_argument('--normal-evidence', type=Path, help='Reuse an already checked mixed graph; run new fault graphs')
    args = cli.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = '101'
    os.environ['ROS_LOG_DIR'] = str(output/'ros_logs')
    rclpy.init()
    node = Node('spatial_acceptance_probe')
    records, processes, handles = {}, [], []
    messages, transforms, authorities = defaultdict(list), {}, defaultdict(set)
    buffer = Buffer(cache_time=Duration(seconds=60))
    diagnostics = []
    def tf(message, static=False):
        for t in message.transforms:
            transforms[(t.child_frame_id, ns(t.header.stamp))] = t
            (buffer.set_transform_static if static else buffer.set_transform)(t, 'checked_by_cpp_auditor')
    node.create_subscription(TFMessage, '/tf', tf, 100)
    node.create_subscription(TFMessage, '/tf_static', lambda m: tf(m, True),
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for key, cls, suffix in [('state', BalanceState, 'state/balance'), ('odom', Odometry, 'odom'),
                            ('joints', JointState, 'joint_states'), ('imu', Imu, 'imu/data_raw'),
                            ('scan', LaserScan, 'scan')]:
        node.create_subscription(cls, '/spatialcheck/'+suffix,
                                 lambda m, key=key: messages[key].append(m), qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray, '/spatialcheck/diagnostics', lambda m: diagnostics.append(m), 10)
    clocks = []
    node.create_subscription(Clock, '/clock', lambda m: clocks.append(ns(m.clock)), 10)

    def spin_until(predicate, seconds=30):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate():
                return
        raise TimeoutError('spatial DDS deadline')

    def launch(label, *options):
        log = (output/(label+'.txt')).open('w')
        handles.append(log)
        process = subprocess.Popen([sys.executable, '/opt/ros/jazzy/bin/ros2', 'launch', 'companionbot_bringup',
            'spatial_simulation.launch.py', 'namespace:=spatialcheck', 'output_dir:='+str(output/label),
            *options], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    def finish(process, label, failure=False):
        spin_until(lambda: process.poll() is not None, 55)
        log = (output/(label+'.txt')).read_text()
        if failure:
            assert process.returncode != 0, log
        else:
            assert process.returncode == 0 and 'process has died' not in log and 'Traceback' not in log, log
            runtime = json.loads((output/label/'runtime.json').read_text())
            assert runtime['state'] == 'CLOSED' and not runtime['error'], runtime
            records[label+'_runtime'] = runtime
        spin_until(lambda: not any(n == '/spatialcheck' for _, n in node.get_node_names_and_namespaces()))
        return log

    try:
        if args.normal_evidence:
            source = args.normal_evidence.resolve()
            previous = json.loads((source/'summary.json').read_text())
            assert previous['mixed_runtime']['state'] == 'CLOSED' and not previous['mixed_runtime']['error']
            assert previous['mixed']['matched_state_odom'] >= 400
            records.update({k:previous[k] for k in ('mixed', 'mixed_runtime')})
            records['normal_evidence_source'] = str(source)
        else:
            process = launch('mixed', 'scenario:=mixed', 'duration_s:=14.0', 'provisional_camera:=true')
            spin_until(lambda: len(messages['odom']) >= 30 or process.poll() is not None)
            assert process.poll() is None, (output/'mixed.txt').read_text()
            snapshot = graph(node)
            (output/'graph.json').write_text(json.dumps(snapshot, indent=2))
            finish(process, 'mixed')
            for key in ('state', 'odom', 'joints', 'imu', 'scan'):
                assert len(messages[key]) >= (80 if key == 'scan' else 400), (key, len(messages[key]))
                stamps = [ns(m.header.stamp) for m in messages[key]]
                assert all(b > a for a, b in zip(stamps, stamps[1:])), key
                with (output/(key+'.jsonl')).open('w') as stream:
                    for m in messages[key]:
                        stream.write(json.dumps(message_to_ordereddict(m))+'\n')
            assert all(b > a for a, b in zip(clocks, clocks[1:]))
            states = {ns(s.header.stamp): s for s in messages['state']}
            for state in states.values():
                assert abs(ns(state.header.stamp)*1e-9-state.meta.source_time_s) < 1e-9
                assert abs(state.imu_measurement_age_s-.001) < 1e-8 and state.imu_valid and not state.imu_stale
            pairs = 0
            for odom in messages['odom']:
                stamp = ns(odom.header.stamp)
                assert odom.header.frame_id == 'odom' and odom.child_frame_id == 'base_footprint'
                if stamp not in states:
                    continue  # independent best-effort DDS delivery is recorded below
                pairs += 1
                t = transforms[('base_footprint', stamp)]
                assert t.transform.translation.x == odom.pose.pose.position.x
                assert t.transform.translation.y == odom.pose.pose.position.y
                assert quaternion(t.transform.rotation) == quaternion(odom.pose.pose.orientation)
                t = transforms[('base_link', stamp)]
                expected = Rotation.from_euler('xyz', [states[stamp].roll_rad, states[stamp].pitch_rad, 0.])
                assert (expected.inv()*Rotation.from_quat(quaternion(t.transform.rotation))).magnitude() < 1e-12
                assert np.linalg.eigvalsh(np.array(odom.pose.covariance).reshape(6,6)).min() >= -1e-12
            assert pairs >= 400
            for joint in messages['joints']:
                state = states.get(ns(joint.header.stamp))
                if state:
                    assert joint.name == ['left_wheel_joint', 'right_wheel_joint']
                    assert list(joint.position) == list(state.wheel_relative_angle_rad)[::-1]
                    assert list(joint.velocity) == list(state.wheel_relative_velocity_rad_s)[::-1]
                    assert not joint.effort
            for imu in messages['imu']:
                assert imu.header.frame_id == 'imu_link' and imu.orientation_covariance[0] == -1
                assert all(math.isfinite(x) for x in [imu.linear_acceleration.x, imu.linear_acceleration.y,
                                                    imu.linear_acceleration.z, imu.angular_velocity.z])
            coverage = {}
            first_dynamic_s = max(min(t for (child, t) in transforms if child == frame)
                                  for frame in ('base_footprint', 'base_link'))*1e-9
            for key in ('imu', 'scan'):
                missing = []
                for m in messages[key]:
                    if key == 'scan':
                        assert m.header.frame_id == 'lidar_link' and m.time_increment == 0
                        assert len(m.ranges) == 360 and m.range_min > 0 and m.range_max > m.range_min
                        assert abs(m.angle_max-(m.angle_min+359*m.angle_increment)) < 1e-6
                    if not buffer.can_transform('odom', m.header.frame_id, Time.from_msg(m.header.stamp)):
                        missing.append(ns(m.header.stamp)*1e-9)
                # First IMU exposure precedes the first estimate horizon by 1ms.
                # Preserve that startup limitation; no restamping or identity TF.
                assert all(t < first_dynamic_s for t in missing), (key, missing)
                coverage[key] = {'count': len(messages[key]), 'first_dynamic_tf_s': first_dynamic_s,
                                 'missing_source_times_s': missing}
            children = {'base_footprint', 'base_link', 'left_wheel_link', 'right_wheel_link', 'imu_link',
                        'lidar_link', 'camera_link', 'camera_optical_frame'}
            for diagnostic in diagnostics:
                for status in diagnostic.status:
                    if status.name.endswith('/tf_authority'):
                        assert status.level != DiagnosticStatus.ERROR, status.message
                        for value in status.values:
                            if value.key in children:
                                authorities[value.key].add(value.value)
            assert set(authorities) == children and all(len(v) == 1 for v in authorities.values())
            endpoints = {t['name']: t for t in snapshot['topics']}
            for topic in ('/clock', '/spatialcheck/state/balance', '/spatialcheck/odom', '/spatialcheck/joint_states',
                          '/spatialcheck/imu/data_raw', '/spatialcheck/scan'):
                assert len(endpoints[topic]['publishers']) == 1, topic
            assert not any('ground_truth' in name or 'cmd_vel' in name for name in endpoints)
            # Camera closure comes from the existing provisional optical extrinsic.
            camera = buffer.lookup_transform('base_link', 'camera_optical_frame', Time())
            mount = json.loads((repository_root()/'models/minisegway/stage7/config/final_demo.json').read_text())['camera_extrinsic']
            assert np.allclose([camera.transform.translation.x, camera.transform.translation.y, camera.transform.translation.z],
                               mount['translation_body_camera_m'], atol=1e-12)
            assert np.allclose(Rotation.from_quat(quaternion(camera.transform.rotation)).as_matrix(),
                               mount['rotation_body_camera'], atol=1e-12)
            records['mixed'] = {'counts': {k:len(v) for k,v in messages.items()}, 'matched_state_odom': pairs,
                                'source_time_tf_coverage': coverage,
                                'graph_pending_observations_max': max([int(v.value) for d in diagnostics
                                    for s in d.status if s.name.endswith('/tf_authority') for v in s.values
                                    if v.key == 'meta.graph_pending_observations'] or [0]),
                                'tf_authorities': {k:sorted(v) for k,v in authorities.items()}}

        # A new graph/epoch requires a fresh estimator. Dropout must inhibit outputs.
        messages.clear()
        process = launch('imu_dropout', 'duration_s:=6.0', 'imu_dropout_at_s:=3.0')
        log = finish(process, 'imu_dropout', failure=True)
        assert ('sensor_invalid_or_stale' in log or 'imu_packet_sample_gap_or_reversal' in log), log
        assert messages['odom'] and max(ns(m.header.stamp)*1e-9 for m in messages['odom']) <= 3.02
        records['imu_dropout'] = {'nonzero_exit': process.returncode,
                                 'last_odom_source_s': max(ns(m.header.stamp)*1e-9 for m in messages['odom'])}

        for label, parent, reason in [('tf_conflict', 'wrong_parent', 'tf_parent_conflict:base_link'),
                                      ('tf_duplicate', 'base_footprint', 'tf_multiple_authorities:base_link')]:
            messages.clear()
            process = launch(label, 'duration_s:=6.0')
            spin_until(lambda: len(messages['odom']) >= 30 or process.poll() is not None)
            assert process.poll() is None, (output/(label+'.txt')).read_text()
            injector = node.create_publisher(TFMessage, '/tf', 10)
            bad = TransformStamped()
            bad.header.frame_id, bad.child_frame_id = parent, 'base_link'
            bad.transform.rotation.w = 1.
            for _ in range(20):
                injector.publish(TFMessage(transforms=[bad]))
                rclpy.spin_once(node, timeout_sec=.02)
            log = finish(process, label, failure=True)
            assert reason in log, log
            node.destroy_publisher(injector)
            records[label] = {'nonzero_exit':process.returncode, 'rejection':reason}

        messages.clear()
        process = launch('interrupted', 'duration_s:=20.0')
        spin_until(lambda: len(messages['odom']) >= 30 or process.poll() is not None)
        assert process.poll() is None, (output/'interrupted.txt').read_text()
        os.killpg(process.pid, signal.SIGINT)
        spin_until(lambda: process.poll() is not None)
        spin_until(lambda: not any(n == '/spatialcheck' for _, n in node.get_node_names_and_namespaces()))
        runtime = json.loads((output/'interrupted/runtime.json').read_text())
        assert runtime['state'] == 'CLOSED' and not runtime['error'], runtime
        records['interrupted'] = runtime
        records['passed'] = True
    except BaseException as error:
        records['error'] = {'type':type(error).__name__, 'message':str(error)}
        raise
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=8)
        for handle in handles:
            handle.close()
        (output/'summary.json').write_text(json.dumps(records, indent=2)+'\n')
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps({'passed':True, 'output':str(output)}))


if __name__ == '__main__':
    main()
