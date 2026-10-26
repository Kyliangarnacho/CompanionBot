"""Finite same-fixture physical/runtime benchmark; never a robot entry point.

The legacy implementation is loaded explicitly from tests/oracles. Selection
is diagnostic only. Functional500Hz traces and cProfile are separate from the
fair timing runs. The unchanged V1 physical controller is shared by both.
"""
import argparse
import cProfile
from contextlib import ExitStack
import gc
import hashlib
import importlib.metadata
import importlib.util
import json
import os
from pathlib import Path
import platform
import resource
import sys
import time
from unittest.mock import patch

import numpy as np

from companionbot_contracts.paths import repository_root
from companionbot_simulation.runtime import navigation_motion
from companionbot_simulation.runtime import spatial_session
from verify_navigation_safety import FaultDriver
from verify_spatial_foundation import control_gates

ROOT = repository_root()
FIXED_EPOCH = 'native_runtime_macro_cancel_resume_20261024'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_identity():
    files = {Path(__file__).resolve(), Path(__file__).with_name('verify_navigation_safety.py'),
        ROOT/'tests/oracles/navigation_packet_link_python.py', ROOT/'models/minisegway/mini_segway.xml',
        ROOT/'models/minisegway/retail_navigation/bookstore_world.lock.json'}
    import companionbot_contracts
    files.update(p for p in Path(companionbot_contracts.__file__).parent.rglob('*.py')
        if '__pycache__' not in p.parts)
    initial=json.loads((ROOT/'models/minisegway/native_runtime/results/session_20261023/source_snapshot_before.json').read_text())
    files.update(ROOT/name for name in initial['files'] if name.startswith('models/minisegway/stage')
        and Path(name).suffix in ('.json','.yaml','.xml'))
    for package in ('companionbot_contracts','companionbot_control','companionbot_follow','companionbot_simulation'):
        base = ROOT/'ros2_ws/src'/package
        files.update(p for p in base.rglob('*') if p.is_file() and
            p.suffix in ('.py','.cpp','.c','.h','.hpp') and '__pycache__' not in p.parts)
    for base in ('models/minisegway/stage3/results/config','models/minisegway/stage4/config',
                 'models/minisegway/stage5/config','models/minisegway/stage6/config',
                 'ros2_ws/src/companionbot_description/config'):
        files.update(p for p in (ROOT/base).glob('*') if p.suffix in ('.json','.yaml'))
    return {str(path.relative_to(ROOT)):sha(path) for path in sorted(files)}


def loaded_libraries():
    paths = set()
    for line in Path('/proc/self/maps').read_text().splitlines():
        path = line.split()[-1]
        if path.startswith('/') and any(token in path for token in ('companionbot','libmujoco','ruckig')):
            actual = Path(path)
            if actual.is_file(): paths.add(actual.resolve())
    return {str(path):sha(path) for path in sorted(paths)}


def distribution(values):
    values = np.asarray(values,dtype=float)
    if not len(values) or not np.isfinite(values).all() or (values<0).any():
        raise ValueError('missing_or_invalid_benchmark_timings')
    return dict(count=len(values),p50_s=float(np.quantile(values,.5)),
        p95_s=float(np.quantile(values,.95)),p99_s=float(np.quantile(values,.99)),
        max_s=float(values.max()),total_s=float(values.sum()))


class FunctionalTrace:
    """One-way diagnostic sink, explicitly excluded from performance evidence."""
    command_columns = ['time_s','p_ref','v_ref','theta_ref','theta_dot_ref','a_ref',
        'u_ff_raw','u_ff_used','v_target','yaw_target','fade_alpha','block_sequence',
        'block_sample_index','block_start_time_s','intent_source_time_s','reference_epoch','safety_state']
    state_codes = dict(NORMAL=0,BRAKING=1,LONGITUDINAL_STOP=2,SAFE_HOLD=3)

    def __init__(self,ticks):
        self.command=np.full((ticks,len(self.command_columns)),np.nan)
        self.motion=None; self.motion_columns=None
        self.command_count=self.motion_count=0
        self.reasons=[];self.phases=[]
        self.spatial_samples=self.attitude_updates=0

    def command_result(self,now,command,arbiter,backend):
        if backend=='native': epoch=arbiter._output.epoch
        else: epoch=arbiter.receiver.reference_epoch
        row=[now,*command.reference_state,command.reference_acceleration_m_s2,
            command.u_ff_raw_nm,command.u_ff_after_lifecycle_nm,
            command.linear_velocity_target_m_s,command.yaw_rate_target_rad_s,
            command.fade_alpha,command.block_id,command.block_sample_index,
            command.block_start_time_s,command.intent_source_time_s,epoch,self.state_codes[arbiter.state]]
        self.command[self.command_count]=row;self.command_count+=1
        self.reasons.append(arbiter.reason)
        self.phases.append([command.feedforward_phase,command.velocity_phase,command.planning_path])

    def observe(self,context):
        estimate=context['estimate_after'];data=context['sim'].data
        row=[context['time_s'],*context['reference_state'],context['reference_acceleration_m_s2'],
            *context['actual_wheels_nm'],*context['requested_wheels_nm'],
            estimate.position_hat_m,estimate.velocity_hat_m_s,estimate.theta_hat_rad,
            estimate.theta_dot_hat_rad_s,*estimate.wheel_relative_angle_hat_rad,
            *estimate.wheel_relative_velocity_hat_rad_s,*data.qpos,*data.qvel]
        if self.motion is None:
            self.motion_columns=['time_s','next_p_ref','next_v_ref','next_theta_ref','next_theta_dot_ref',
                'next_a_ref','actual_left','actual_right','requested_left','requested_right',
                'p_hat','v_hat','pitch_hat','pitch_rate_hat','wheel_angle_0','wheel_angle_1',
                'wheel_velocity_0','wheel_velocity_1']+['qpos_'+str(i) for i in range(len(data.qpos))]+[
                'qvel_'+str(i) for i in range(len(data.qvel))]
            self.motion=np.full((len(self.command),len(row)),np.nan)
        self.motion[self.motion_count]=row;self.motion_count+=1

    def save(self,out):
        np.savez_compressed(out/'functional_trace.npz',command=self.command,motion=self.motion)
        report=dict(command_columns=self.command_columns,motion_columns=self.motion_columns,
            command_count=self.command_count,motion_count=self.motion_count,reasons=self.reasons,
            phases=self.phases,spatial_sample_constructions=self.spatial_samples,
            attitude_updates=self.attitude_updates,gt_semantics='physical qpos/qvel only in one-way diagnostic sink',
            performance_comparison_eligible=False)
        (out/'functional_trace.json').write_text(json.dumps(report,indent=2)+'\n')
        return {k:v for k,v in report.items() if k not in ('reasons','phases')}


def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--backend',choices=['python_oracle','native'],required=True)
    cli.add_argument('--output-dir',type=Path,required=True)
    cli.add_argument('--profile',action='store_true')
    cli.add_argument('--functional-trace',action='store_true')
    cli.add_argument('--duration',type=float,default=20.)
    args=cli.parse_args()
    if not np.isfinite(args.duration) or args.duration!=20.:
        cli.error('the fixed comparison protocol requires exactly20s')
    out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    if any(out.iterdir()): raise FileExistsError('preserve_benchmark_evidence_choose_empty_output')
    before_sources=source_identity();before_libraries=loaded_libraries()
    driver=FaultDriver('cancel_during_resume')
    trace=FunctionalTrace(round(args.duration/.002)) if args.functional_trace else None
    timings={'scheduler':[],'command':[],'local_estimate':[]}
    original_build=navigation_motion.build_navigation_reference
    if args.backend=='python_oracle':
        spec=importlib.util.spec_from_file_location('motion_runtime_benchmark_oracle',
            ROOT/'tests/oracles/navigation_packet_link_python.py')
        oracle=importlib.util.module_from_spec(spec);spec.loader.exec_module(oracle)
        original_build=oracle.build_navigation_reference
    runtime_owner=dict(authority=type(driver.adapter.native_authority).__module__+'.'+
        type(driver.adapter.native_authority).__name__,
        physical_controller='unchanged frozen Python V1 sensorized estimator/controller; no --native-control override')

    def instrumented_build(*a,**kw):
        arbiter,link,metadata=original_build(*a,**kw)
        runtime_owner.update(arbiter=type(arbiter).__module__+'.'+type(arbiter).__name__,
            link=type(link).__module__+'.'+type(link).__name__,metadata=metadata)
        for obj,method,label in [(link,'before_control_tick','scheduler'),
                                (arbiter,'command','command'),(link,'mcu_local_state','local_estimate')]:
            original=getattr(obj,method)
            def measured(*inputs,_call=original,_label=label,**kwargs):
                cpu,wall=time.thread_time_ns(),time.perf_counter_ns()
                value=_call(*inputs,**kwargs)
                timings[_label].append(((time.perf_counter_ns()-wall)*1e-9,
                                       (time.thread_time_ns()-cpu)*1e-9))
                if trace and _label=='command':trace.command_result(inputs[0],value,arbiter,args.backend)
                return value
            setattr(obj,method,measured)
        return arbiter,link,metadata

    active_gc={};gc_pauses=[]
    def gc_observer(phase,info):
        generation=info['generation']
        if phase=='start':active_gc[generation]=time.perf_counter()
        elif generation in active_gc:
            gc_pauses.append(dict(generation=generation,wall_s=time.perf_counter()-active_gc.pop(generation)))
    profiler=cProfile.Profile() if args.profile else None
    with ExitStack() as stack:
        stack.enter_context(patch.object(navigation_motion,'build_navigation_reference',instrumented_build))
        if trace:
            original_observe=spatial_session.SpatialSession.observe
            original_sample=spatial_session.SpatialSample
            original_attitude=spatial_session.SpatialAttitude.update
            def observe(session,context):
                result=original_observe(session,context);trace.observe(context);return result
            def sample(*a,**kw):
                trace.spatial_samples+=1;return original_sample(*a,**kw)
            def attitude(*a,**kw):
                trace.attitude_updates+=1;return original_attitude(*a,**kw)
            stack.enter_context(patch.object(spatial_session.SpatialSession,'observe',observe))
            stack.enter_context(patch.object(spatial_session,'SpatialSample',sample))
            stack.enter_context(patch.object(spatial_session.SpatialAttitude,'update',attitude))
        gc.callbacks.append(gc_observer)
        if profiler:profiler.enable()
        before_cpu=resource.getrusage(resource.RUSAGE_SELF)
        started=time.perf_counter()
        try:
            result=spatial_session.SpatialSession(out,scenario='navigation',world='bookstore',
                epoch=FIXED_EPOCH,duration_s=args.duration,motion_adapter=driver.adapter,on_frame=driver).run()
        finally:
            elapsed=time.perf_counter()-started
            usage=resource.getrusage(resource.RUSAGE_SELF)
            if profiler:profiler.disable()
            gc.callbacks.remove(gc_observer)
    # Everything below is outside the measured window, including trace encoding.
    if profiler:profiler.dump_stats(str(out/'profile.pstats'))
    metrics={}
    for label,rows in timings.items():
        values=np.asarray(rows)
        metrics[label]=dict(count=len(rows),wall=distribution(values[:,0]),thread_cpu=distribution(values[:,1]))
        np.save(out/(label+'_timings.npy'),values)
    cpu=usage.ru_utime+usage.ru_stime-before_cpu.ru_utime-before_cpu.ru_stime
    after_sources=source_identity();after_libraries=loaded_libraries()
    configuration=json.loads((out/'configuration.json').read_text())
    authority=json.loads((out/'motion_authority.json').read_text())
    producers=[event for epoch in authority['producer_epochs'] for event in epoch['events']]
    headroom=[event['deadline_headroom_s'] for event in producers if 'deadline_headroom_s' in event]
    report=dict(benchmark_version=2,backend=args.backend,
        scope='WSL actual MuJoCo fixed cancel/resume fixture; common current native MotionAuthority, original V1 controller; no DDS/target hardware',
        classification='instrumented_diagnostic_only' if args.profile or trace else 'performance_measurement',
        instrumented_cprofile=args.profile,functional_trace_enabled=bool(trace),
        per_call_probe_enabled_both_backends=True,gc_callback_enabled_both_backends=True,
        duration_source_s=args.duration,elapsed_wall_s=elapsed,cpu_s=cpu,
        cpu_percent_one_core=100*cpu/elapsed,real_time_factor=args.duration/elapsed,
        peak_rss_mib=usage.ru_maxrss/1024,
        voluntary_context_switches=usage.ru_nvcsw-before_cpu.ru_nvcsw,
        involuntary_context_switches=usage.ru_nivcsw-before_cpu.ru_nivcsw,
        measured_window='SpatialSession construction/run: asset/model setup, physics, required logs/reports and GC restoration; excludes benchmark serialization/source/library scans',
        cpu_endpoint='rusage immediately after wall endpoint, before GC callback removal/profiling dump/artifact serialization',
        modules=metrics,control_gates=control_gates(result,args.duration),
        source_files=before_sources,source_files_unchanged=before_sources==after_sources,
        loaded_libraries_before=before_libraries,loaded_libraries_after=after_libraries,
        already_loaded_libraries_unchanged=all(after_libraries.get(k)==v for k,v in before_libraries.items()),
        runtime_owner=runtime_owner,configuration=configuration,
        actual_model_xml_sha256=sha(out/'indoor_model.xml'),
        python=sys.executable,python_version=sys.version,platform=platform.platform(),
        dependency_versions={name:importlib.metadata.version(name) for name in ('numpy','scipy','mujoco','ruckig')},
        environment={key:os.environ.get(key) for key in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS')},
        available_cpu_affinity=sorted(os.sched_getaffinity(0)),
        gc_pauses=gc_pauses,allocation_policy=authority['allocation_policy'],
        lifecycle=dict(producer_epoch_block_counts=[dict(epoch=entry['reference_epoch'],blocks=len(entry['events'])) for entry in authority['producer_epochs']],
            generated_blocks=len(producers),headroom_available=len(headroom),
            minimum_reported_headroom_s=min(headroom) if headroom else None,
            deadline_headroom_negative_count=sum(h<0 for h in headroom),
            native_watchdog_metrics=authority.get('metrics'),safety_events=authority['safety_events']),
        hardware_execution_ready=False)
    history_times=np.asarray([row['t'] for row in result['history_50hz']])
    report['retained_control_history']=dict(legacy_field_name='history_50hz',rows=len(history_times),
        observed_period_min_s=float(np.diff(history_times).min()),
        observed_period_max_s=float(np.diff(history_times).max()),
        observed_frequency_hz=(len(history_times)-1)/(history_times[-1]-history_times[0]),
        distinction='control history is500Hz in this loaded experiment; sensorized spatial DTO publication remains50Hz')
    if trace:report['functional_trace']=trace.save(out)
    report['passed']=(report['control_gates']['passed'] and report['source_files_unchanged']
        and report['already_loaded_libraries_unchanged'])
    (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    (out/'ingress.json').write_text(json.dumps(driver.events,indent=2)+'\n')
    (out/'fixture_states.json').write_text(json.dumps(driver.states,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('backend','classification','passed','elapsed_wall_s','cpu_s','real_time_factor','peak_rss_mib')}),flush=True)
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
