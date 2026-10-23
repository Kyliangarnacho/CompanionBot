"""Typed DDS protocol fixtures for spatial_state; no plant, GUI or hardware."""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from companionbot_interfaces.msg import BalanceState
from companionbot_contracts.spatial_geometry import load_spatial_geometry
from companionbot_contracts.stamps import source_stamp


def stamp(t):
    n = round(t*1e9)
    return Time(sec=n//1_000_000_000, nanosec=n%1_000_000_000)


def state(t, sequence, epoch='fixture-epoch'):
    m = BalanceState()
    m.header.stamp, m.header.frame_id = stamp(t), 'base_link'
    m.meta = source_stamp('fixture:spatial', epoch, sequence, t, clock_domain='mujoco_sim',
                          clock_id='sim:'+epoch, time_semantics='control_sample')
    m.spatial_contract_version, m.geometry_id = 1, load_spatial_geometry().geometry_id
    m.ros_time_mapping_id = 'identity:sim:'+epoch
    m.imu_sample_time_s, m.imu_available_time_s, m.imu_measurement_age_s = t-.001, t, .001
    m.imu_valid, m.encoder_valid = True, True
    return m


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--output-dir', type=Path, required=True)
    args = cli.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'], os.environ['ROS_LOG_DIR'] = '103', str(output/'ros_logs')
    rclpy.init()
    node = Node('spatial_protocol_fixture')
    publisher = node.create_publisher(BalanceState, '/spatialfixture/state/balance', qos_profile_sensor_data)
    clock = node.create_publisher(Clock, '/clock', 10)
    odometry = []
    node.create_subscription(Odometry, '/spatialfixture/odom', lambda m: odometry.append(m), qos_profile_sensor_data)
    processes, records = [], {}

    def until(predicate, seconds=12):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate(): return
        raise TimeoutError('protocol fixture deadline')

    def spin_for(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=.01)

    cases = {
        'source_epoch': ('source_or_time_mapping_changed', lambda m: setattr(m.meta, 'source_epoch', 'reboot')),
        'future_time': ('outside_ros_window', None),
        'duplicate_sequence': ('sequence_time_reversal', lambda m: setattr(m.meta, 'sequence', 10)),
        'wrong_frame': ('state_schema_source_frame_or_geometry', lambda m: setattr(m.header, 'frame_id', 'map')),
        'wrong_geometry': ('state_schema_source_frame_or_geometry', lambda m: setattr(m, 'geometry_id', 'wrong')),
        'uncertain_mapping': ('missing_or_uncertain', lambda m: setattr(m, 'ros_time_uncertainty_s', .01)),
        'invalid_encoder': ('sensor_invalid_or_stale', lambda m: setattr(m, 'encoder_valid', False)),
        'nonfinite': ('nonfinite_or_malformed', lambda m: setattr(m, 'traveled_distance_m', float('nan'))),
        'pause_then_timeout': ('source_timeout_while_clock_advances', None),
        'duplicate_publisher': ('multiple_balance_state_publishers', None),
    }
    try:
        for label, (reason, mutation) in cases.items():
            odometry.clear()
            log = (output/(label+'.txt')).open('w')
            executable = Path(os.environ['COMPANIONBOT_ROOT'])/'ros2_ws/install/companionbot_control/lib/companionbot_control/spatial_state'
            process = subprocess.Popen([str(executable), '--ros-args', '-r', '__ns:=/spatialfixture',
                '-p', 'use_sim_time:=true', '-p', 'expected_source_id:=fixture:spatial'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(process)
            duplicate = None
            try:
                until(lambda: publisher.get_subscription_count() == 1 or process.poll() is not None)
                assert process.poll() is None, (output/(label+'.txt')).read_text()
                clock.publish(Clock(clock=stamp(.02)))
                spin_for(.1)
                publisher.publish(state(.02, 10))
                until(lambda: bool(odometry) or process.poll() is not None)
                assert process.poll() is None and len(odometry) == 1
                invalid = state(.04, 20)
                if label == 'pause_then_timeout':
                    spin_for(.35)
                    assert process.poll() is None and len(odometry) == 1, 'paused clock advanced spatial output'
                    clock.publish(Clock(clock=stamp(.4)))
                elif label == 'duplicate_publisher':
                    duplicate = node.create_publisher(BalanceState, '/spatialfixture/state/balance', qos_profile_sensor_data)
                else:
                    clock.publish(Clock(clock=stamp(.04)))
                    spin_for(.05)
                    if label == 'future_time': invalid = state(1., 20)
                    else: mutation(invalid)
                    publisher.publish(invalid)
                until(lambda: process.poll() is not None)
                log.flush()
                evidence = (output/(label+'.txt')).read_text()
                assert process.returncode != 0 and reason in evidence, evidence
                assert len(odometry) == 1, 'invalid state published new odometry'
                records[label] = {'passed':True, 'exit_code':process.returncode,
                                  'rejection':reason, 'valid_odom_count':len(odometry)}
            finally:
                if duplicate: node.destroy_publisher(duplicate)
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=8)
                log.close()
            until(lambda: publisher.get_subscription_count() == 0)
        records['passed'] = True
    except BaseException as error:
        records['error'] = {'type':type(error).__name__, 'message':str(error)}
        raise
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=8)
        (output/'summary.json').write_text(json.dumps(records, indent=2)+'\n')
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps({'passed':True, 'cases':len(cases)}))


if __name__ == '__main__': main()
