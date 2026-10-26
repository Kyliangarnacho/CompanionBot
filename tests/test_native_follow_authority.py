"""Deterministic software tests for one native FOLLOW/GUIDE_TO authority.

Fixture ages are explicit. Frozen controller/governor parameters are not tuned;
these tests exercise authorization and observation provenance, not ML accuracy.
"""
import json
from pathlib import Path

import pytest

from companionbot_follow import _native_runtime as native

CONFIG = json.loads((Path(__file__).resolve().parents[1] /
    'models/minisegway/stage6/config/stage6_4_2d_follow_config.json').read_text())


class Fixture:
    def __init__(self, enabled=True):
        self.source, self.receipt, self.sequence = 5., 10., 0
        follow = native.FollowAuthorityConfig(CONFIG['governor']['max_abs_velocity_m_s'],
            CONFIG['yaw']['max_rate_rad_s'], .3, CONFIG['yaw']['freshness_timeout_s'], .35, .15) if enabled else None
        self.owner = native.MotionAuthority('bookstore', 'v1', follow_config=follow)
        self.owner.update_health(True, '', self.receipt)
        self.owner.update_mcu_status('NORMAL', '', self.receipt)
        if enabled:
            self.owner.update_follow_obstacle(10., self.source, self.source, self.receipt)
            self.owner.update_follow_braking(.5, self.receipt)

    def control(self, command='ACQUIRE_FOLLOW', task='task', task_epoch='agent', **kwargs):
        self.sequence += 1
        return self.owner.control(task, task_epoch, self.sequence, command, 1.2,
            'bookstore', 'v1', kwargs.get('source', self.source), kwargs.get('receipt', self.receipt))

    def health(self, sequence=1, valid=True, reason='', epoch='camera', **kwargs):
        return self.owner.update_follow_health(valid, reason, epoch, sequence,
            kwargs.get('source', self.source), kwargs.get('now', self.source),
            kwargs.get('receipt', self.receipt))

    def advance(self, dt=.05):
        self.source += dt; self.receipt += dt
        self.owner.update_health(True, '', self.receipt)
        self.owner.update_follow_obstacle(10., self.source, self.source, self.receipt)
        self.owner.update_follow_braking(.5, self.receipt)

    def command(self, sequence=2, epoch='camera', velocity=.5, yaw=.1, **kwargs):
        snapshot = self.owner.snapshot(False)
        return self.owner.receive_follow_cmd(kwargs.get('task', 'task'), kwargs.get('task_epoch', 'agent'),
            kwargs.get('authority_epoch', snapshot['authority_epoch']), epoch, sequence,
            kwargs.get('source', self.source), kwargs.get('now', self.source),
            kwargs.get('receipt', self.receipt), velocity, yaw)

    def follow(self):
        assert self.health()
        assert self.control()['accepted']
        self.advance()
        assert self.health(2)
        assert self.command()


def test_follow_requires_explicit_configuration_and_real_health():
    f = Fixture(False)
    assert not f.health()
    answer = f.control('CHECK_FOLLOW')
    assert not answer['accepted'] and answer['reason'] == 'follow_disabled'
    assert f.control('CHECK')['accepted']
    f = Fixture()
    assert not f.control('CHECK_FOLLOW')['accepted']
    assert not f.control()['accepted']
    assert f.health()
    assert f.control('CHECK_FOLLOW')['accepted']
    assert f.owner.snapshot(False)['master_locked']


def test_follow_has_own_envelope_and_foreign_nav2_callback_cannot_revoke_it():
    f = Fixture(); f.follow()
    intent = f.owner.sample(f.source, f.receipt)
    assert intent.linear_velocity_target_m_s == .5  # navigation remains .25
    assert f.owner.snapshot(False)['mode'] == 'FOLLOW'
    for v, yaw, source in ((.2, .1, f.source + .01), (float('nan'), 0., f.source), (.2, 0., 0.)):
        assert not f.owner.receive_cmd(v, yaw, source, f.source, f.receipt, 'nav2:controller_server')
    assert f.owner.sample(f.source, f.receipt).linear_velocity_target_m_s == .5
    assert f.owner.snapshot(False)['authorized']


def test_mutual_exclusion_requires_stop_then_new_explicit_grant():
    f = Fixture(); f.follow()
    old = f.owner.snapshot(False)['authority_epoch']
    assert not f.control('ACQUIRE')['accepted']
    assert not f.control('RENEW')['accepted']
    assert f.control('RENEW_FOLLOW')['accepted']
    assert f.owner.snapshot(False)['authority_epoch'] == old
    assert f.control('STOP')['accepted']
    assert f.owner.snapshot(False)['mode'] == 'NONE'
    assert f.owner.sample(f.source, f.receipt).linear_velocity_target_m_s == 0.
    assert f.control('ACQUIRE', task='guide')['accepted']
    assert f.owner.snapshot(False)['mode'] == 'GUIDE_TO'
    assert not f.command(authority_epoch=old)
    assert f.owner.receive_cmd(.2, .1, f.source + .01, f.source + .01, f.receipt)
    assert f.owner.sample(f.source + .01, f.receipt).linear_velocity_target_m_s == .2
    assert not f.control('ACQUIRE_FOLLOW', task='guide')['accepted']


def test_pregrant_frame_and_wrong_task_generation_or_association_cannot_move():
    f = Fixture(); assert f.health(); answer = f.control(); assert answer['accepted']
    assert not f.command(sequence=1)  # source was sampled before grant
    assert f.owner.sample(f.source, f.receipt).linear_velocity_target_m_s == 0.
    f.advance(); assert f.health(2)
    for kwargs in ({'task': 'old'}, {'task_epoch': 'old'}, {'authority_epoch': answer['authority_epoch'] - 1},
                   {'epoch': 'other'}, {'sequence': 1}, {'source': f.source - .001}):
        assert not f.command(**kwargs)
    assert f.command()
    assert not f.command()  # observation cannot refresh motion twice
    assert f.owner.snapshot(False)['follow_command_rejections'] == 8


@pytest.mark.parametrize('reason', ['master_lost', 'identity_uncertain', 'stale_depth',
    'wrong_extrinsic', 'pitch_history_unavailable', 'obstacle_stop'])
def test_invalid_follow_inputs_revoke_and_cannot_implicitly_recover(reason):
    f = Fixture(); f.follow(); old = f.owner.snapshot(False)['authority_epoch']
    f.advance(); assert not f.health(3, False, reason)
    snapshot = f.owner.snapshot(False)
    assert not snapshot['authorized'] and snapshot['mode'] == 'NONE'
    assert not snapshot['master_locked']
    assert f.owner.take_stop()[1] == reason
    f.advance(); assert f.health(4)
    assert not f.command(sequence=4, authority_epoch=old)
    assert not f.control('RENEW_FOLLOW')['accepted']
    assert f.owner.sample(f.source, f.receipt).linear_velocity_target_m_s == 0.
    assert f.control(task='new')['accepted']
    assert not f.command(task='new', sequence=4)  # still pregrant image
    f.advance(); assert f.health(5)
    assert f.command(task='new', sequence=5)


@pytest.mark.parametrize('clock', ['source', 'receipt'])
def test_follow_freshness_expires_in_either_clock_while_common_health_is_fresh(clock):
    f = Fixture(); f.follow()
    source = f.source + (.21 if clock == 'source' else 0.)
    receipt = f.receipt + (.21 if clock == 'receipt' else 0.)
    f.owner.update_health(True, '', receipt)
    assert f.owner.sample(source, receipt).linear_velocity_target_m_s == 0.
    snapshot = f.owner.snapshot(False)
    assert not snapshot['authorized'] and not snapshot['master_locked']
    assert f.owner.take_stop()[1] == 'follow_observation_expired'
    assert snapshot[f'follow_{clock}_age_s'] == pytest.approx(.21)


def test_camera_source_restart_revokes_old_task_and_retired_epoch_cannot_return():
    f = Fixture(); f.follow(); old = f.owner.snapshot(False)['authority_epoch']
    f.advance(); assert f.health(0, epoch='camera_restarted')
    assert not f.owner.snapshot(False)['authorized']
    assert f.owner.take_stop()[1] == 'follow_source_epoch_changed'
    assert not f.command(sequence=0, epoch='camera_restarted', authority_epoch=old)
    assert f.control(task='new')['accepted']
    f.advance(); assert f.health(1, epoch='camera_restarted')
    assert f.command(task='new', epoch='camera_restarted', sequence=1)
    assert not f.health(999, epoch='camera')
    assert f.owner.snapshot(False)['follow_source_epoch'] == 'camera_restarted'
    assert f.owner.snapshot(False)['authorized']


def test_common_health_and_independent_mcu_faults_also_revoke_follow():
    f = Fixture(); f.follow()
    f.owner.update_health(False, 'lidar_invalid', f.receipt)
    assert not f.owner.snapshot(False)['authorized']
    assert f.owner.take_stop()[1] == 'lidar_invalid'
    f.owner.update_health(True, '', f.receipt)
    assert f.control(task='new')['accepted']
    f.owner.update_mcu_status('SAFE_HOLD', 'HEARTBEAT_TIMEOUT', f.receipt)
    assert not f.owner.snapshot(False)['authorized']
    assert not f.control(task='third')['accepted']


def test_invalid_duplicate_and_stale_health_cannot_refresh_master():
    f = Fixture(); f.follow()
    old = f.owner.snapshot(False)
    assert not f.health(2)
    assert f.owner.snapshot(False)['follow_received_at_s'] == old['follow_received_at_s']
    f.advance(.21)
    assert not f.control('CHECK_FOLLOW')['accepted']
    assert not f.owner.snapshot(False)['master_locked']
    assert not f.health(3, source=f.source - .21)
    assert not f.control()['accepted']


@pytest.mark.parametrize('velocity,yaw', [(float('nan'), .5), (.5, float('inf')), (-.1, .1)])
def test_invalid_owned_follow_intent_stops_instead_of_silently_using_previous_motion(velocity, yaw):
    f = Fixture(); f.follow(); f.advance(); assert f.health(3)
    assert not f.command(sequence=3, velocity=velocity, yaw=yaw)
    assert not f.owner.snapshot(False)['authorized']
    assert f.owner.take_stop()[1] == 'invalid_follow_intent'


def test_follow_obstacle_and_braking_are_independent_required_health_facts():
    config = native.FollowAuthorityConfig(.6, .5, .3, .2, .35, .15)
    owner = native.MotionAuthority('bookstore', 'v1', follow_config=config)
    owner.update_health(True, '', 10.); owner.update_mcu_status('NORMAL', '', 10.)
    assert owner.update_follow_health(True, '', 'camera', 1, 5., 5., 10.)
    def check(sequence):
        return owner.control('task', 'agent', sequence, 'CHECK_FOLLOW', 1.2, 'bookstore', 'v1', 5., 10.)
    assert check(1)['reason'] == 'follow_obstacle_unavailable'
    assert owner.update_follow_obstacle(.4, 5., 5., 10.)
    assert check(2)['reason'] == 'follow_braking_unavailable'
    assert owner.update_follow_braking(.5, 10.)
    assert check(3)['reason'] == 'follow_obstacle_clearance'
    # Healthy vision cannot overwrite the independent obstacle decision.
    assert owner.update_follow_health(True, '', 'camera', 2, 5.01, 5.01, 10.01)
    assert not owner.control('task', 'agent', 4, 'ACQUIRE_FOLLOW', 1.2,
        'bookstore', 'v1', 5.01, 10.01)['accepted']
    assert owner.snapshot(False)['master_locked']
    assert not owner.snapshot(False)['follow_obstacle_clear']
    # Guidance readiness preserves its existing contract.
    assert owner.control('task', 'agent', 5, 'CHECK', 1.2, 'bookstore', 'v1', 5.01, 10.01)['accepted']


@pytest.mark.parametrize('channel', ['scan', 'braking'])
def test_missing_or_stale_guard_revokes_even_with_fresh_vision_and_common_health(channel):
    f = Fixture(); f.follow()
    source, receipt = f.source+.16, f.receipt+.16
    f.owner.update_health(True, '', receipt)
    assert f.owner.update_follow_health(True, '', 'camera', 3, source, source, receipt)
    if channel == 'scan':
        # Keep braking fresh while the real scan ages past its original .35s gate.
        source, receipt = f.source+.36, f.receipt+.36
        f.owner.update_health(True, '', receipt)
        assert f.owner.update_follow_health(True, '', 'camera', 4, source, source, receipt)
        assert f.owner.update_follow_braking(.5, receipt)
        expected = 'follow_obstacle_expired'
    else:
        assert f.owner.update_follow_obstacle(10., source, source, receipt)
        expected = 'follow_braking_expired'
    assert f.owner.sample(source, receipt).linear_velocity_target_m_s == 0.
    assert not f.owner.snapshot(False)['authorized']
    assert f.owner.take_stop()[1] == expected


def test_obstacle_clearance_drops_revoke_immediately_and_need_explicit_recovery():
    f = Fixture(); f.follow()
    assert f.owner.update_follow_obstacle(.4, f.source+.01, f.source+.01, f.receipt+.01)
    assert not f.owner.snapshot(False)['authorized']
    assert f.owner.take_stop()[1] == 'follow_obstacle_clearance'
    assert f.owner.update_follow_obstacle(10., f.source+.02, f.source+.02, f.receipt+.02)
    assert not f.owner.snapshot(False)['authorized']
    assert not f.control('RENEW_FOLLOW', source=f.source+.02, receipt=f.receipt+.02)['accepted']


def test_invalid_guard_is_unavailable_and_old_scan_cannot_refresh_deadline():
    f = Fixture(); f.follow()
    before = f.owner.snapshot(False)['follow_obstacle_received_at_s']
    assert not f.owner.update_follow_obstacle(10., f.source, f.source+.01, f.receipt+.01)
    assert f.owner.snapshot(False)['follow_obstacle_received_at_s'] == before
    assert not f.owner.update_follow_obstacle(float('nan'), f.source+.01, f.source+.01, f.receipt+.01)
    assert not f.owner.snapshot(False)['authorized']
    assert f.owner.take_stop()[1] == 'follow_obstacle_invalid'
    assert not f.owner.snapshot(False)['follow_obstacle_clear']


def test_mcu_reason_provenance_prevents_internal_fault_becoming_same_named_owned_stop():
    f = Fixture(); f.follow()
    # Human-readable equality does not confer the C99 external reason origin.
    f.owner.stop('HEARTBEAT_TIMEOUT', f.receipt)
    f.owner.take_stop()
    f.owner.update_mcu_status_with_provenance('SAFE_HOLD', 'HEARTBEAT_TIMEOUT', True, f.receipt)
    assert f.control(task='new')['accepted']
    assert f.owner.snapshot(False)['mcu_recovery_owned']
    f.owner.update_mcu_status_with_provenance('SAFE_HOLD', 'HEARTBEAT_TIMEOUT', False, f.receipt)
    assert not f.owner.snapshot(False)['authorized']
    assert not f.owner.snapshot(False)['mcu_recovery_owned']
    assert not f.control('CHECK')['accepted']
    assert not f.control('CHECK_FOLLOW')['accepted']
    assert not f.control(task='third')['accepted']
    # A foreign claimed origin still cannot authorize a reason this owner has
    # never emitted, or an invalid MCU state.
    f.owner.update_mcu_status_with_provenance('SAFE_HOLD', 'unknown_reason', True, f.receipt)
    assert not f.owner.snapshot(False)['mcu_recovery_owned']
    f.owner.update_mcu_status_with_provenance('INVALID', 'HEARTBEAT_TIMEOUT', True, f.receipt)
    assert not f.owner.snapshot(False)['mcu_recovery_owned']


def test_late_invalid_health_cannot_revoke_newer_frame_or_new_task():
    f = Fixture(); f.follow()
    epoch = f.owner.snapshot(False)['authority_epoch']
    before = f.owner.snapshot(False)
    for sequence, source in ((1, f.source-.05), (2, f.source), (999, f.source-.01)):
        assert not f.owner.update_follow_health(False, 'late_master_lost', 'camera', sequence,
            source, f.source, f.receipt)
        assert f.owner.snapshot(False)['authorized']
        assert f.owner.snapshot(False)['follow_sequence'] == before['follow_sequence']
    assert f.owner.sample(f.source, f.receipt).linear_velocity_target_m_s == .5
    assert f.control('STOP')['accepted']
    assert f.control(task='new')['accepted']
    newer = f.owner.snapshot(False)['authority_epoch']
    assert newer > epoch
    # A higher source sequence does not rescue an image captured pregrant.
    assert not f.owner.update_follow_health(False, 'old_task_depth_missing', 'camera', 999,
        f.source, f.source, f.receipt)
    assert f.owner.snapshot(False)['authorized']
    assert f.owner.snapshot(False)['authority_epoch'] == newer
    f.advance()
    assert not f.health(1000, False, 'fresh_master_lost')
    assert not f.owner.snapshot(False)['authorized']
    assert f.owner.take_stop()[1] == 'fresh_master_lost'


def test_current_source_failure_atomically_invalidates_follow_without_stopping_guidance():
    f = Fixture(); f.follow()
    assert f.owner.invalidate_follow('follow_worker_failed', f.receipt)
    assert not f.owner.snapshot(False)['authorized']
    assert f.owner.take_stop()[1] == 'follow_worker_failed'
    assert f.owner.sample(f.source, f.receipt).linear_velocity_target_m_s == 0.
    assert not f.owner.snapshot(False)['master_locked']
    assert f.control('ACQUIRE', task='guide')['accepted']
    assert f.owner.receive_cmd(.2, .1, f.source+.01, f.source+.01, f.receipt)
    assert not f.owner.invalidate_follow('follow_worker_closed', f.receipt)
    assert f.owner.snapshot(False)['mode'] == 'GUIDE_TO'
    assert f.owner.snapshot(False)['authorized']
    assert f.owner.sample(f.source+.01, f.receipt).linear_velocity_target_m_s == .2
    assert f.owner.take_stop() is None


def test_navigation_only_invalid_input_cannot_cross_mode_switch_and_revoke_follow():
    f = Fixture(); f.follow()
    epoch = f.owner.snapshot(False)['authority_epoch']
    assert not f.owner.invalidate_navigation('navigation_twist_frame_or_nonplanar_component', f.receipt)
    assert f.owner.snapshot(False)['authorized']
    assert f.owner.snapshot(False)['authority_epoch'] == epoch
    assert f.owner.sample(f.source, f.receipt).linear_velocity_target_m_s == .5
    assert f.control('STOP')['accepted']; f.owner.take_stop()
    assert f.control('ACQUIRE', task='guide')['accepted']
    assert f.owner.invalidate_navigation('navigation_twist_frame_or_nonplanar_component', f.receipt)
    assert not f.owner.snapshot(False)['authorized']
    assert f.owner.take_stop()[1] == 'navigation_twist_frame_or_nonplanar_component'
    assert f.control('ACQUIRE_FOLLOW', task='new_follow')['accepted']
    assert not f.owner.invalidate_navigation('old_nav_callback', f.receipt)
    assert f.owner.snapshot(False)['mode'] == 'FOLLOW'
