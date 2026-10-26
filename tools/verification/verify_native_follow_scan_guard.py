"""Actual local DDS FOLLOW scan guard fixture; no physics or actuator execution.

Reuses the established Nav2 ingress fixture and its isolated-domain setup.
Master validity and required braking distance are explicitly labeled software
inputs here. Raw scan values travel through DDS unchanged; they are not model
observations, camera/depth certification or a simulated physical stopping test.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from companionbot_follow.runtime.native_follow import authority_configuration
from verify_native_navigation_ingress import Fixture, stamp


class FollowFixture(Fixture):
    def __init__(self, output):
        super().__init__(output, follow_config=authority_configuration())
        self.profile = 'finite_360'
        self.follow_sequence = 0
        self.echoes = deque(maxlen=64)
        self.echo = self.node.create_subscription(LaserScan, '/companionbot/scan',
            lambda message: self.echoes.append(message), qos_profile_sensor_data)
        self.required_clearance_m = .5  # Declared software fixture input, not C99 braking evidence.

    def scan_message(self):
        message = LaserScan()
        message.header.frame_id = 'lidar_link'; message.header.stamp = stamp(self.source)
        message.range_min, message.range_max = .05, 12.
        message.angle_min, message.angle_increment = -math.pi, math.pi/180
        count = {'missing_359':359, 'missing_358':358, 'half_fov':180,
                 'inclusive_361':361}.get(self.profile, 360)
        values = [2.] * count
        if self.profile in ('mixed_inf', 'nan', 'negative_inf'):
            values = [math.inf] * count
            values[0], values[count//2] = 4.5, 2.
            if self.profile == 'nan': values[90] = math.nan
            if self.profile == 'negative_inf': values[90] = -math.inf
        if self.profile == 'angle_header_mismatch':
            message.angle_max = message.angle_min + (count-2)*message.angle_increment
        else:
            message.angle_max = message.angle_min + (count-1)*message.angle_increment
        message.scan_time = .1  # Canonical source describes 10 Hz; this DDS fixture republishes 50 Hz.
        message.ranges = values
        return message

    def publish_sensors(self):
        super().publish_sensors()
        receipt = time.perf_counter()
        self.follow_sequence += 1
        owner = self.adapter.native_authority
        owner.update_follow_braking(self.required_clearance_m, receipt)
        owner.update_follow_health(True, '', 'software_camera_fixture', self.follow_sequence,
            self.source, self.source, receipt)
        snapshot = owner.snapshot(False)
        if snapshot['mode'] == 'FOLLOW':
            owner.receive_follow_cmd(snapshot['task_id'], snapshot['source_epoch'],
                snapshot['authority_epoch'], 'software_camera_fixture', self.follow_sequence,
                self.source, self.source, receipt, .45, .1)

    def await_clearance(self, expected):
        after = self.source
        self.until(lambda: (self.adapter.snapshot(include_events=False)['follow_obstacle_source_time_s'] is not None
            and self.adapter.snapshot(include_events=False)['follow_obstacle_source_time_s'] > after
            and abs(self.adapter.snapshot(include_events=False)['follow_obstacle_clearance_m']-expected) < 1e-6))

    def latest_echo(self):
        self.until(lambda: bool(self.echoes))
        # Every profile is repeated until observed; compare angle/count and
        # special-value locations, not the previous profile's queued echo.
        wanted = self.scan_message()
        def matches(message):
            return (len(message.ranges) == len(wanted.ranges) and
                np.array_equal(np.asarray(message.ranges, dtype=np.float32),
                               np.asarray(wanted.ranges, dtype=np.float32), equal_nan=True) and
                abs(message.angle_max-wanted.angle_max) < 1e-6)
        self.until(lambda: any(matches(message) for message in self.echoes))
        return next(message for message in reversed(self.echoes) if matches(message))


def run(output):
    output.mkdir(parents=True, exist_ok=False)
    fixture = FollowFixture(output)
    checks, cases, raw_scans = {}, [], {}
    runtime = authority = error = None
    try:
        fixture.until(lambda: fixture.client.service_is_ready() and
            fixture.runtime.snapshot()['reason'] == 'navigation_health_valid')
        fixture.await_clearance(2.)
        checks['finite_complete_scan_readiness'] = fixture.control('CHECK_FOLLOW').accepted
        assert checks['finite_complete_scan_readiness']
        assert fixture.control('ACQUIRE_FOLLOW').accepted
        fixture.until(lambda: fixture.adapter.sample(fixture.source).linear_velocity_target_m_s == .45)
        checks['follow_envelope_preserved'] = True
        # Approved Nav2 callbacks may remain in flight during a mode switch.
        # Their malformed task-local input must not revoke active FOLLOW.
        before_rejected = fixture.runtime.snapshot()['commands_rejected']
        fixture.command(nonplanar=True)
        fixture.until(lambda: fixture.runtime.snapshot()['commands_rejected'] > before_rejected)
        checks['nonplanar_nav2_does_not_revoke_follow'] = fixture.adapter.snapshot()['authorized']
        fixture.command(frame='wrong_frame')
        fixture.pump(.06)
        checks['wrong_frame_nav2_does_not_revoke_follow'] = fixture.adapter.snapshot()['authorized']
        for profile in ('mixed_inf', 'inclusive_361', 'nan', 'negative_inf', 'missing_359',
                        'missing_358', 'half_fov', 'angle_header_mismatch'):
            fixture.profile = profile
            expected = 2. if profile in ('mixed_inf', 'inclusive_361') else 0.
            fixture.await_clearance(expected)
            echo = fixture.latest_echo()
            observed = np.asarray(echo.ranges, dtype=np.float32)
            raw_scans[profile] = observed
            answer = fixture.control('CHECK_FOLLOW', task='readiness_'+profile)
            snapshot = fixture.adapter.snapshot(include_events=False)
            row = dict(profile=profile, expected_clearance_lower_bound_m=expected,
                actual_clearance_lower_bound_m=snapshot['follow_obstacle_clearance_m'],
                scan_count=len(echo.ranges), raw_positive_inf_count=int(np.isposinf(observed).sum()),
                raw_negative_inf_count=int(np.isneginf(observed).sum()), raw_nan_count=int(np.isnan(observed).sum()),
                check_follow_accepted=answer.accepted, check_follow_reason=answer.reason,
                authorized=snapshot['authorized'], mode=snapshot['mode'])
            cases.append(row)
            checks[profile+'_guard'] = (abs(row['actual_clearance_lower_bound_m']-expected)<1e-6
                and answer.accepted == (expected > fixture.required_clearance_m))
            if profile == 'mixed_inf':
                checks['positive_infinity_preserved_on_dds'] = row['raw_positive_inf_count'] == 358
                checks['mixed_inf_keeps_follow_authorized'] = snapshot['authorized']
            if expected == 0.:
                checks[profile+'_not_authorized'] = not snapshot['authorized']
            # Recovery of valid scan facts never re-grants a revoked task.
            if expected == 0.:
                fixture.profile = 'mixed_inf'
                fixture.await_clearance(2.)
                fixture.pump(.04)
                checks[profile+'_no_implicit_recovery'] = not fixture.adapter.snapshot()['authorized']
                assert fixture.control('ACQUIRE_FOLLOW', task='explicit_after_'+profile).accepted
                fixture.until(lambda: fixture.adapter.sample(fixture.source).linear_velocity_target_m_s == .45)
        authority = fixture.adapter.snapshot()
    except BaseException as exception:
        error = repr(exception)
        raise
    finally:
        if authority is None: authority = fixture.adapter.snapshot()
        runtime = fixture.close()
        np.savez_compressed(output/'raw_scans_received.npz', **raw_scans)
        report = dict(scope='actual_local_DDS_software_guard_fixture_no_physics_no_actuator',
            hardware_execution_ready=False, checks=checks, cases=cases,
            passed=error is None and len(cases) == 8 and bool(checks) and all(checks.values()), error=error,
            fixture_inputs=dict(required_clearance_m=fixture.required_clearance_m,
                follow_intent_m_s=.45, perception_health='declared software fixture, no model inference',
                scan_publication_period_s=.02, source_scan_time_field_s=.1,
                authoritative_configuration='companionbot_follow.runtime.native_follow.authority_configuration',
                raw_scan_semantics='REP117 +Inf observed no return within range_max; NaN/-Inf unavailable/too-close'),
            runtime=runtime, authority=authority)
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    assert report['passed'], report
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    result = run(parser.parse_args().output_dir.resolve())
    print(json.dumps(result, indent=2))
