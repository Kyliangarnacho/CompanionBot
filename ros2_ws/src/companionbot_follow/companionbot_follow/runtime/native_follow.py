"""Canonical configuration and low-frequency solver adapter for native owners.

No task, governor, scheduler or reference lifecycle state machine lives here.
FULL mathematics remains the existing Ruckig/SciPy implementation, called once
per accepted plan; the native owner samples/executes the resulting arrays.
"""
import hashlib
import json
import math

import numpy as np

from companionbot_contracts.paths import repository_root
from companionbot_follow import _native_runtime as native

ROOT = repository_root()
FOLLOW_PATH = ROOT/'models/minisegway/stage6/config/stage6_4_2d_follow_config.json'
KF_PATH = ROOT/'models/minisegway/stage6/config/stage6_5_radial_kf_config.json'


def configuration():
    return json.loads(FOLLOW_PATH.read_text()), json.loads(KF_PATH.read_text())


def authority_configuration():
    follow, _ = configuration()
    return native.FollowAuthorityConfig(follow['governor']['max_abs_velocity_m_s'],
        follow['yaw']['max_rate_rad_s'], follow['yaw']['freshness_timeout_s'],
        follow['yaw']['freshness_timeout_s'], .35, follow['safety']['heartbeat_timeout_s'])


def controller(authority):
    follow, radial = configuration()
    return native.FollowController(authority, native.RadialKFConfig(**radial['kf']),
        native.FollowGovernorConfig(**follow['governor']), radial['state_history_retention_s'],
        follow['yaw']['gain_s_inv'], follow['yaw']['max_rate_rad_s'], follow['yaw']['deadband_rad'])


def producer_configuration(common, stream, producer):
    follow, _ = configuration()
    cfg = native.TwoPathConfig()
    cfg.producer = producer
    scheduler = native.SchedulerConfig()
    source = stream['scheduler']
    scheduler.accept_delta_velocity = source['T_accept_delta_v_m_s']
    scheduler.full_delta_velocity = source['T_full_delta_v_m_s']
    scheduler.stable_window_s = source['candidate_stable_window_s']
    scheduler.stable_range_velocity = source['candidate_stable_range_m_s']
    scheduler.initial_accepted_velocity = source['initial_accepted_velocity_m_s']
    policy = follow['follow_scheduler_gate_policy']
    scheduler.require_stable_candidate = policy['require_stable_candidate']
    scheduler.minimum_delta_enabled = policy['minimum_delta_enabled']
    scheduler.force_full_dynamic = policy['follow_force_full_dynamic']
    cfg.scheduler = scheduler
    cfg.defer_full_accept_until_observed_exit = policy['follow_defer_until_full_exit']
    cfg.feedforward_fade_s = common[1]['velocity_lifecycle']['feedforward_fade_s']
    cfg.quiet_theta_rad = math.radians(stream['full_dynamic']['terminal_theta_abs_deg'])
    cfg.quiet_theta_dot_rad_s = math.radians(stream['full_dynamic']['terminal_theta_dot_abs_deg_s'])
    return cfg


def full_solver(common, stream):
    from companionbot_follow.core.rolling_reference import RuckigFullHorizonVelocityPlanner
    from companionbot_follow.core.rolling_reference import SparseProjectionFactorizationCache
    _, _, dynamic, motion, offline, *_ = common
    fit, cfg = offline['fit'], stream['full_dynamic']
    planner = RuckigFullHorizonVelocityPlanner(
        minimum_horizon_s=cfg['minimum_horizon_s'],
        dynamic_settle_margin_s=cfg.get('dynamic_settle_margin_s',0.),
        planning_dt_s=cfg.get('planning_dt_s'), horizon_quantum_s=cfg['horizon_bucket_s'],
        projection_backend='cached_kkt', factorization_cache=SparseProjectionFactorizationCache(),
        controller_dt_s=motion['controller_dt_s'],
        max_velocity_m_s=motion['reference_limits']['max_velocity_m_s'],
        max_acceleration_m_s2=motion['reference_limits']['max_acceleration_m_s2'],
        max_jerk_m_s3=dynamic['max_jerk_m_s3'], A=np.asarray(fit['A_identified']),
        B=np.asarray(fit['B_identified']), state_scales=np.asarray(fit['state_scales']),
        input_scale_nm=fit['input_scale_nm'])
    def solve(request):
        return native.FullPlan.from_rolling(planner.plan(np.asarray(request.start_reference),
            request.start_acceleration, request.target_velocity))
    return solve


def clearance_configuration(common):
    """Derived conservative guard inputs, fixed before physical visual testing."""
    paths = {
        'footprint': ROOT/'ros2_ws/src/companionbot_bringup/config/nav2_geometry.json',
        'lidar': ROOT/'ros2_ws/src/companionbot_description/config/sim_lidar.yaml',
        'follow': FOLLOW_PATH,
    }
    values = {key:json.loads(path.read_text()) for key,path in paths.items()}
    lidar, follow = values['lidar'], values['follow']
    radius = values['footprint']['robot_radius_m'] + math.sqrt(sum(x*x for x in lidar['xyz_m'])) + 3*lidar['range_noise_std_m']
    reaction = follow['yaw']['freshness_timeout_s'] + .35 + .002
    dt = common[3]['controller_dt_s']
    vmax = common[3]['reference_limits']['max_velocity_m_s']
    amax = common[3]['reference_limits']['max_acceleration_m_s2']
    safety = follow['safety']
    decel, jerk = safety['safe_max_deceleration_m_s2'], safety['safe_max_jerk_m_s3']
    ramp = (amax+decel)/jerk
    upper = vmax+amax*ramp+.5*jerk*ramp*ramp
    decay = math.log(max(1.,2*upper/np.finfo(float).eps))/2
    steps = math.ceil((ramp+upper/decel+decay+safety['hidden_reference_handoff_time_s']+2*dt)/dt)
    return dict(radius_m=radius,reaction_time_s=reaction,
        maximum_projection_steps=steps,
        source_age_limit_s=.35,braking_receipt_limit_s=follow['safety']['heartbeat_timeout_s'],
        source_files={key:dict(path=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                      for key,path in paths.items()},
        derivation='attitude-invariant robot radius + lidar-origin triangle bound + 3 sigma simulation range noise; reaction = visual source deadline + existing scan age limit + one MCU tick',
        limitations='2D observed-plane conservative engineering guard, no hardware braking certification; physical tracking evaluated separately',
        hardware_execution_ready=False)
