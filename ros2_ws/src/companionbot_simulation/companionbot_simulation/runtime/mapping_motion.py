"""Finite mapping intent sequence through existing Pi/ReferenceBlock/MCU safety.

No pose feedback, actuator access, SLAM dependency or GT input. This is a
simulation experiment, not an external command interface or navigation stack.
"""
import json
import gc
import time
import math
from pathlib import Path
from contextlib import contextmanager

from companionbot_contracts.paths import repository_root
from companionbot_contracts.packets import PROTOCOL_VERSION, YawCommandPacket
from companionbot_control.runtime.mcu import MCUSafetyArbiter, MCUReferenceBlockReceiver
from companionbot_control.runtime.yaw_reference import MCULatestYawSource
from companionbot_control.core.safety_brake import SafetyBrakeConfig
from companionbot_follow.core.rolling_reference import MotionIntent, TargetObservation
from companionbot_follow.runtime.reference_producer import PiReferenceBlockProducer
from companionbot_simulation.runtime.packet_link import InMemoryPacketTransport, Stage6PacketLink


@contextmanager
def mapping_allocation_scope(metrics):
    """Defer cyclic GC only during this finite simulation physics loop.

    Refcount cleanup remains active. ROS object cycles are collected before
    release; the process's previous policy is restored on every exit path.
    This is host scheduling, not a change to modeled packet compute latency,
    reference deadlines, MCU safety or physical control.
    """
    enabled = gc.isenabled()
    gc.collect()
    before = [s['collections'] for s in gc.get_stats()]
    gc.disable()
    try:
        yield
    finally:
        metrics.update({'scope':'finite_mapping_physics_loop_process',
            'cyclic_gc_was_enabled':enabled,
            'collections_during_loop':[s['collections']-n for s,n in zip(gc.get_stats(),before)],
            'refcount_cleanup_active':True})
        if enabled:
            gc.enable()
        metrics['prior_gc_policy_restored'] = gc.isenabled() == enabled


class MappingIntent:
    def __init__(self, route):
        self.route = route
        self.time_s = 0.

    def values(self, time_s):
        event = self.route['commands'][0]
        for candidate in self.route['commands']:
            if candidate['time_s'] > time_s + 1e-9:
                break
            event = candidate
        return event['v_m_s'], event['yaw_rate_rad_s']

    def read(self):
        # Existing synthetic-command convention: this is explicitly an intent
        # carrier, never advertised as a detected target or a sensor reading.
        v, yaw = self.values(self.time_s)
        return TargetObservation(capture_time_s=self.time_s, x_forward_m=v, y_left_m=yaw)


class MappingLink(Stage6PacketLink):
    def on_robot_state_packet(self, packet):
        pass  # No follow governor in a preauthorized finite intent sequence.

    def after_pi_packets(self, time_s):
        self.intent.time_s = time_s
        tick = round(time_s / self.producer.dt_s)
        if tick % self.producer.block_samples == 0:
            packet = YawCommandPacket(PROTOCOL_VERSION, self.yaw_sequence, time_s,
                self.intent.values(time_s)[1])
            self.yaw_sequence += 1
            self.transport.send('mcu', 'yaw', packet.to_bytes(), time_s)


def mapping_reference(common, stage5, config, *, route_file=None):
    route_path = (Path(route_file).resolve() if route_file else
                  repository_root()/'ros2_ws/src/companionbot_bringup/config/mapping_route.json')
    route = json.loads(route_path.read_text())
    commands = route['commands']
    if (not commands or commands[0]['time_s'] != 0 or
            not all(math.isfinite(float(c[k])) for c in commands
                    for k in ('time_s', 'v_m_s', 'yaw_rate_rad_s')) or
            any(b['time_s'] <= a['time_s'] for a, b in zip(commands, commands[1:])) or
            any(abs(c['v_m_s']) > .6 or abs(c['yaw_rate_rad_s']) > .5 for c in commands) or
            commands[-1]['v_m_s'] != 0 or commands[-1]['yaw_rate_rad_s'] != 0):
        raise ValueError('invalid_finite_mapping_route')
    intent = MappingIntent(route)
    source = stage5.make_reference_source(config, common, intent,
        follower=lambda o: MotionIntent(o.capture_time_s, o.x_forward_m, o.y_left_m))
    producer = PiReferenceBlockProducer(source)
    produce = producer.produce
    def measured_produce(*args, **kwargs):
        collections = [s['collections'] for s in gc.get_stats()]
        cpu = time.thread_time()
        result = produce(*args, **kwargs)
        producer.events[-1]['thread_cpu_time_s'] = time.thread_time()-cpu
        producer.events[-1]['gc_collections_delta'] = [s['collections']-n for s,n in zip(gc.get_stats(),collections)]
        return result
    producer.produce = measured_produce
    safety_config = json.loads((repository_root()/'models/minisegway/stage6/config/stage6_4_2d_follow_config.json').read_text())
    safety, yaw = safety_config['safety'], safety_config['yaw']
    transport = InMemoryPacketTransport(safety['transport_latency_s'])
    receiver = MCUReferenceBlockReceiver(dt_s=producer.dt_s, block_samples=producer.block_samples)
    yaw_source = MCULatestYawSource(dt_s=producer.dt_s, timeout_s=yaw['freshness_timeout_s'],
        decay_rate_rad_s2=yaw['decay_rate_rad_s2'])
    arbiter = MCUSafetyArbiter(receiver, SafetyBrakeConfig(safety['safe_max_deceleration_m_s2'],
        safety['safe_max_jerk_m_s3'], safety['hidden_reference_handoff_time_s'], producer.dt_s),
        heartbeat_timeout_s=safety['heartbeat_timeout_s'], starvation_margin_s=safety['reference_starvation_margin_s'],
        nominal_pitch_rad=float(common[7]['parameters']['theta_eq_rad']), yaw_source=yaw_source,
        status_sink=lambda payload, t: transport.send('pi', 'safety_status', payload, t))
    link = MappingLink(transport=transport, producer=producer, governor=None, arbiter=arbiter,
        crash_time_s=None, hazard_time_s=None)
    link.intent, link.yaw_sequence = intent, 0
    return arbiter, link, route
