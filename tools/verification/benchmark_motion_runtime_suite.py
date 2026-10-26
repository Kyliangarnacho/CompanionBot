"""Fixed functional traces plus fresh-process Python/native/native/Python.

No ROS, GUI or hardware interface. This orchestrates the existing finite
Bookstore cancel/resume fixture and never acts as a production fallback.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from benchmark_motion_runtime import distribution

SCRIPT=Path(__file__).with_name('benchmark_motion_runtime.py')
ORDER=('python_oracle','native','native','python_oracle')
ATOL=2e-12  # Existing native navigation/reference parity tolerance; no refit.


def write(path,value):path.write_text(json.dumps(value,indent=2)+'\n')
def read(path):return json.loads(path.read_text())
def utc():return datetime.now(timezone.utc).isoformat()


def arrays(before,after,columns):
    if before.shape!=after.shape or before.ndim!=2 or before.shape[1]!=len(columns):
        raise ValueError('functional_trace_geometry_mismatch')
    rows={}
    for i,name in enumerate(columns):
        x,y=before[:,i],after[:,i]
        same_finite=bool(np.array_equal(np.isfinite(x),np.isfinite(y)))
        finite=np.isfinite(x)&np.isfinite(y)
        delta=np.abs(x[finite]-y[finite])
        rows[name]=dict(exact=bool(np.array_equal(x,y,equal_nan=True)),finite_mask_equal=same_finite,
            max_absolute_delta=float(delta.max()) if delta.size else 0.,
            within_existing_reference_atol=bool(same_finite and np.allclose(x,y,atol=ATOL,rtol=0,equal_nan=True)))
    return rows


def history(before,after):
    if len(before)!=len(after):return dict(exact=False,rows=[len(before),len(after)],length_equal=False)
    fields={}
    for name in sorted(before[0]):
        lhs=[row[name] for row in before];rhs=[row[name] for row in after]
        if lhs==rhs:fields[name]=dict(exact=True,max_absolute_delta=0.);continue
        try:
            x,y=np.asarray(lhs,dtype=float),np.asarray(rhs,dtype=float)
            finite=np.isfinite(x)&np.isfinite(y)
            fields[name]=dict(exact=False,finite_mask_equal=bool(np.array_equal(np.isfinite(x),np.isfinite(y))),
                max_absolute_delta=float(np.max(np.abs(x[finite]-y[finite]))) if finite.any() else 0.)
        except (ValueError,TypeError):
            changes=[dict(row=i,python=a,native=b) for i,(a,b) in enumerate(zip(lhs,rhs)) if a!=b]
            fields[name]=dict(exact=False,changed_rows=len(changes),first_differences=changes[:10])
    return dict(exact=before==after,rows=len(before),length_equal=True,fields=fields)


def producer_metadata(authority):
    return [dict(epoch=entry['reference_epoch'],sequence=event['sequence_id'],
        start_tick=event['start_control_tick'],generated_tick=round(event['generated_time_s']/.002))
        for entry in authority['producer_epochs'] for event in entry['events']]


def safety_semantics(authority):
    types={'safety_takeover','safe_hold','resume_requested','resume_armed','hard_stop_revoked'}
    keys=('reason','state','start_control_tick','reference_epoch')
    return [dict(event=row['event'],tick=round(row['time_s']/.002),
        **{key:row[key] for key in keys if key in row}) for row in authority['safety_events'] if row['event'] in types]


def transport_headroom(path):
    authority=read(path/'motion_authority.json')
    producers=[(epoch['reference_epoch'],event) for epoch in authority['producer_epochs'] for event in epoch['events']]
    sent=[event for event in authority['transport_events'] if event['event']=='packet_sent' and event.get('channel')=='reference']
    if len(producers)!=len(sent):raise ValueError('reference_send_generation_count_mismatch')
    rows=[]
    for (epoch,producer),packet in zip(producers,sent):
        if round(producer['generated_time_s']/.002)!=round(packet['time_s']/.002):
            raise ValueError('reference_send_generation_order_mismatch')
        rows.append(dict(epoch=epoch,sequence=producer['sequence_id'],start_tick=producer['start_control_tick'],
            generated_time_s=producer['generated_time_s'],available_time_s=packet['available_time_s'],
            headroom_s=producer['start_control_tick']*.002-packet['available_time_s']))
    margins=np.asarray([row['headroom_s'] for row in rows])
    result=dict(derivation='ordered actual producer events joined to reference packet_sent available_time; includes startup prefill',
        blocks=len(rows),all_block_headroom=dict(count=len(margins),minimum_s=float(margins.min()),
            maximum_s=float(margins.max()),p50_s=float(np.quantile(margins,.5)),
            p95_s=float(np.quantile(margins,.95)),p99_s=float(np.quantile(margins,.99))),
        negative_headroom_count=int(np.count_nonzero(margins<0)),
        nonzero_generation_time_minimum_headroom_s=min(row['headroom_s'] for row in rows if row['generated_time_s']>0),
        reference_packet_drops=sum(event['event']=='packet_dropped' and event.get('channel')=='reference' for event in authority['transport_events']),
        rows=rows)
    write(path/'reference_transport_timing_reaudited.json',result)
    return {key:value for key,value in result.items() if key!='rows'}


def functional(directory):
    paths=[directory/('trace_'+name) for name in ('python_oracle','native')]
    metadata=[read(p/'functional_trace.json') for p in paths]
    summaries=[read(p/'summary.json') for p in paths]
    traces=[np.load(p/'functional_trace.npz',allow_pickle=False) for p in paths]
    for key in ('command_columns','motion_columns','command_count','motion_count'):
        if metadata[0][key]!=metadata[1][key]:raise ValueError('functional_trace_metadata_mismatch:'+key)
    command=arrays(traces[0]['command'],traces[1]['command'],metadata[0]['command_columns'])
    motion=arrays(traces[0]['motion'],traces[1]['motion'],metadata[0]['motion_columns'])
    normal=(traces[0]['command'][:,-1]==0)&(traces[1]['command'][:,-1]==0)
    critical=['p_ref','v_ref','theta_ref','theta_dot_ref','a_ref','u_ff_raw','u_ff_used','v_target','yaw_target']
    integer=['block_sequence','reference_epoch','safety_state']
    normal_sample_exact=bool(np.array_equal(traces[0]['command'][normal,12],traces[1]['command'][normal,12]))
    authorities=[read(p/'motion_authority.json') for p in paths]
    histories=[read(p/'frozen_result_posthoc.json')['history_50hz'] for p in paths]
    history_times=np.asarray([row['t'] for row in histories[0]])
    clock_fields={}
    for name in ('block_start_time_s','intent_source_time_s'):
        index=metadata[0]['command_columns'].index(name)
        left,right=traces[0]['command'][:,index],traces[1]['command'][:,index]
        clock_fields[name]=dict(**command[name],
            canonical_control_tick_exact=bool(np.array_equal(np.rint(left/.002),np.rint(right/.002))),
            normal_state_float_values_exact=bool(np.array_equal(left[normal],right[normal])),
            normal_state_max_absolute_delta=float(np.max(np.abs(left[normal]-right[normal]))),
            normal_state_within_existing_reference_atol=bool(np.allclose(left[normal],right[normal],atol=ATOL,rtol=0)),
            changed_rows=int(np.count_nonzero(left!=right)),
            scope=('SAFETY DTO current time: Python uses accumulated MuJoCo time, native integer control tick*dt; NORMAL block start derives from identical start tick'
                if name=='block_start_time_s' else 'NORMAL DTO carries ReferenceBlock generated_time, Python accumulated MuJoCo time vs native integer tick*dt; SAFETY uses current time by same respective clock arithmetic; no runtime stamp modified'))
    result=dict(scope='separate instrumented500Hz diagnostic, no speedup claim',
        consumed_control_ticks=metadata[0]['command_count'],motion_ticks=metadata[0]['motion_count'],
        existing_reference_absolute_tolerance=ATOL,relative_tolerance=0,
        command_columns=command,motion_columns=motion,
        consumed_reference_within_existing_tolerance=all(command[k]['within_existing_reference_atol'] for k in critical),
        block_epoch_state_exact=all(command[k]['exact'] for k in integer),normal_block_sample_index_exact=normal_sample_exact,
        reasons_exact=metadata[0]['reasons']==metadata[1]['reasons'],
        phases_exact=metadata[0]['phases']==metadata[1]['phases'],
        producer_metadata_exact=producer_metadata(authorities[0])==producer_metadata(authorities[1]),
        common_safety_semantics_exact=safety_semantics(authorities[0])==safety_semantics(authorities[1]),
        common_safety_semantics=[safety_semantics(v) for v in authorities],
        ingress_exact=read(paths[0]/'ingress.json')==read(paths[1]/'ingress.json'),
        control_history=dict(comparison=history(*histories),legacy_field_name='history_50hz',
            physical_and_reference_values_exact=all(all(a[k]==b[k] for k in a if k not in
                ('intent_source_time_s','reference_block_start_time_s')) for a,b in zip(*histories)),
            actual_rows=len(history_times),observed_period_min_s=float(np.diff(history_times).min()),
            observed_period_max_s=float(np.diff(history_times).max()),
            observed_frequency_hz=(len(history_times)-1)/(history_times[-1]-history_times[0])),
        clock_metadata=clock_fields,
        clock_metadata_within_existing_reference_atol=all(v['within_existing_reference_atol'] for v in clock_fields.values()),
        all_trace_columns_within_existing_reference_atol=all(v['within_existing_reference_atol'] for v in command.values()),
        all_physical_motion_columns_exact=all(v['exact'] for v in motion.values()),
        constructor_counts={name:dict(spatial_samples=m['spatial_sample_constructions'],attitude_updates=m['attitude_updates'])
            for name,m in zip(('python_oracle','native'),metadata)},
        unconsumed_pending_wire_bytes='not exposed by runtime; not claimed by macro; independent61500sample full-wire parity remains separate',
        control_gates=[v['control_gates'] for v in summaries],hardware_execution_ready=False)
    result['control_and_reference_gates_passed']=all(v['passed'] for v in summaries) and all(result[k] for k in
        ('consumed_reference_within_existing_tolerance','block_epoch_state_exact','normal_block_sample_index_exact',
         'reasons_exact','producer_metadata_exact','common_safety_semantics_exact','ingress_exact'))
    result['passed']=result['control_and_reference_gates_passed'] and result['all_trace_columns_within_existing_reference_atol']
    return result


def summarize(directory):
    manifest=read(directory/'suite_manifest.json')
    trace_report=functional(directory)
    records={name:[] for name in ('python_oracle','native')}
    performance=[entry for entry in manifest['runs'] if entry['classification']=='performance_measurement']
    if [entry['backend'] for entry in performance]!=list(ORDER):raise ValueError('fixed_macro_order_changed')
    reference=None
    for entry in performance:
        if entry['state']!='COMPLETED':raise ValueError('incomplete_macro_performance_run')
        path=directory/entry['directory'];summary=read(path/'summary.json')
        if summary['classification']!='performance_measurement':raise ValueError('trace_or_profile_must_not_be_pooled')
        if reference is None:reference=summary
        for key in ('benchmark_version','duration_source_s','source_files','configuration','actual_model_xml_sha256',
                    'python','python_version','dependency_versions','environment','available_cpu_affinity',
                    'loaded_libraries_after'):
            if reference[key]!=summary[key]:raise ValueError('unfair_macro_configuration_'+key+':'+entry['directory'])
        records[entry['backend']].append((entry,summary,path))
    grouped={}
    for backend,runs in records.items():
        wall=sum(v['elapsed_wall_s'] for _,v,_ in runs);cpu=sum(v['cpu_s'] for _,v,_ in runs)
        modules={}
        for name in ('scheduler','command','local_estimate'):
            data=np.concatenate([np.load(path/(name+'_timings.npy'),allow_pickle=False) for _,_,path in runs])
            modules[name]=dict(wall=distribution(data[:,0]),thread_cpu=distribution(data[:,1]))
        run_histories=[read(path/'frozen_result_posthoc.json')['history_50hz'] for _,_,path in runs]
        grouped[backend]=dict(run_count=len(runs),elapsed_wall_total_s=wall,process_cpu_total_s=cpu,
            mean_real_time_factor=sum(v['duration_source_s'] for _,v,_ in runs)/wall,
            mean_cpu_percent_one_core=100*cpu/wall,peak_rss_mib=max(v['peak_rss_mib'] for _,v,_ in runs),
            voluntary_context_switches=sum(v['voluntary_context_switches'] for _,v,_ in runs),
            involuntary_context_switches=sum(v['involuntary_context_switches'] for _,v,_ in runs),modules=modules,
            gc_pause_count=sum(len(v['gc_pauses']) for _,v,_ in runs),
            gc_pause_max_s=max((p['wall_s'] for _,v,_ in runs for p in v['gc_pauses']),default=0.),
            retained_control_history=dict(legacy_field_name='history_50hz',rows_each_run=[len(h) for h in run_histories],
                within_backend_histories_exact=all(h==run_histories[0] for h in run_histories)),
            individual_runs=[dict(directory=e['directory'],reference_transport_timing=transport_headroom(path),**{k:v[k] for k in
                ('elapsed_wall_s','cpu_s','cpu_percent_one_core','real_time_factor','peak_rss_mib','control_gates')},
                lifecycle={**v['lifecycle'],'native_watchdog_metrics':({k:x for k,x in v['lifecycle']['native_watchdog_metrics'].items()
                    if not k.endswith('_samples_s')} if v['lifecycle']['native_watchdog_metrics'] else None)}) for e,v,path in runs])
    p,n=grouped['python_oracle'],grouped['native']
    comparison={key:dict(python=p[key],native=n[key],native_over_python=n[key]/p[key],
        reduction_percent=100*(p[key]-n[key])/p[key]) for key in
        ('elapsed_wall_total_s','process_cpu_total_s','peak_rss_mib')}
    report=dict(scope='WSL complete20s MuJoCo fixture execution; no ROS/AI/GL/hardware',
        fixed_order=list(ORDER),functional_trace=trace_report,backends=grouped,comparison=comparison,
        performance_measurement_valid=all(v['passed'] for runs in records.values() for _,v,_ in runs),
        passed=trace_report['passed'] and all(v['passed'] for runs in records.values() for _,v,_ in runs),
        limitations=['Common current native MotionAuthority and original Python V1 physical controller are shared; comparison concerns old Python vs production native scheduling/packet/MCU responsibilities.',
            'Full window includes asset/model setup, original required logging, report generation and GC restoration; no claim about pure physics CPU.',
            'Per-call clock probes and GC callbacks are enabled equally; functional500Hz traces/cProfile/tracemalloc are excluded from performance runs.',
            'Physical numeric deltas are reported without relaxing any original control/safety acceptance.',
            'No unconsumed pending wire export, DDS latency/drop metric, GIL wait or Pi/STM32 certification is claimed.'],hardware_execution_ready=False)
    write(directory/'suite_summary_reaudited.json',report)
    return report


def run(args):
    out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    if any(out.iterdir()):raise FileExistsError('preserve_macro_suite_evidence_choose_empty_output')
    entries=[dict(directory='trace_'+name,backend=name,classification='instrumented_diagnostic_only',state='PENDING')
        for name in ('python_oracle','native')]+[dict(directory=f'perf_{i:02d}_{name}',backend=name,
            classification='performance_measurement',state='PENDING') for i,name in enumerate(ORDER,1)]
    manifest=dict(state='RUNNING',created_utc=utc(),quiet_window_note=args.quiet_window_note,runs=entries,
        protocol_sha256=hashlib.sha256((SCRIPT.parents[2]/'models/minisegway/native_runtime/PHYSICS_MACRO_BENCHMARK_PLAN.md').read_bytes()).hexdigest())
    manifest['suite_script_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (out/'protocol_pre_run.md').write_bytes((SCRIPT.parents[2]/'models/minisegway/native_runtime/PHYSICS_MACRO_BENCHMARK_PLAN.md').read_bytes())
    for entry in entries:
        entry.update(state='RUNNING',started_utc=utc())
        command=[sys.executable,str(SCRIPT),'--backend',entry['backend'],'--duration','20',
            '--output-dir',str(out/entry['directory'])]
        if entry['classification']=='instrumented_diagnostic_only':command.append('--functional-trace')
        entry['command']=command;write(out/'suite_manifest.json',manifest)
        started=time.perf_counter()
        with (out/(entry['directory']+'.stdout.txt')).open('w') as stdout,\
             (out/(entry['directory']+'.stderr.txt')).open('w') as stderr:
            child=subprocess.run(command,stdout=stdout,stderr=stderr)
        entry.update(state='COMPLETED' if child.returncode==0 else 'FAILED',exit_code=child.returncode,
            ended_utc=utc(),process_wall_s=time.perf_counter()-started)
        write(out/'suite_manifest.json',manifest)
        if child.returncode:
            manifest.update(state='FAILED',ended_utc=utc());write(out/'suite_manifest.json',manifest)
            raise RuntimeError('macro_child_failed:'+entry['directory'])
    manifest.update(state='COMPLETED',ended_utc=utc());write(out/'suite_manifest.json',manifest)
    return summarize(out)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('run','summarize'))
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--quiet-window-note',default='')
    args=parser.parse_args();report=run(args) if args.command=='run' else summarize(args.output_dir.resolve())
    print(json.dumps(dict(passed=report['passed'],comparison=report['comparison'])),flush=True)
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
