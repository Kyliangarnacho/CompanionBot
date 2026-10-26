"""Run/summarize the fixed production reference microbenchmark protocol.

Diagnostic only. No robot/ROS command interface. Each fresh process runs the
same 10,000 blocks; ordering is Python/native/native/Python for each GC policy.
Performance and instrumented diagnostics are always summarized separately.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

SCRIPT = Path(__file__).with_name('benchmark_native_reference.py')
ORDER = ('python', 'native', 'native', 'python')
POLICIES = ('deferred', 'enabled')


def utc():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2)+'\n')


def distribution(values):
    values = np.asarray(values, dtype=float)
    if not len(values) or not np.isfinite(values).all() or np.any(values < 0):
        raise ValueError('missing_or_invalid_timing_samples')
    return dict(count=len(values), p50_s=float(np.quantile(values, .5)),
        p95_s=float(np.quantile(values, .95)), p99_s=float(np.quantile(values, .99)),
        max_s=float(np.max(values)), total_s=float(np.sum(values)))


def improvement(before, after):
    return dict(python=before, native=after,
        native_over_python=after/before if before else None,
        improvement_percent=100*(before-after)/before if before else None)


def summarize(directory):
    manifest = json.loads((directory/'suite_manifest.json').read_text())
    expected_protocol = [(policy, backend) for policy in POLICIES for backend in ORDER]
    if [(entry['gc_policy'], entry['backend']) for entry in manifest['runs']] != expected_protocol:
        raise ValueError('fixed_python_native_native_python_protocol_mismatch')
    grouped, comparisons = {}, []
    errors = []
    for entry in manifest['runs']:
        run = directory/entry['directory']
        if entry['state'] != 'COMPLETED':
            errors.append(dict(run=entry['directory'], error='run_not_completed'))
            continue
        result = json.loads((run/'summary.json').read_text())
        if result.get('benchmark_version') not in (2, 3):
            raise ValueError('legacy_benchmark_version_must_not_be_pooled')
        if result['backend'] != entry['backend'] or result['gc_policy'] != entry['gc_policy']:
            raise ValueError('suite_manifest_run_identity_mismatch')
        if result['backend'] == 'native' and result['implementation'] != 'native_cpp_two_path_producer':
            raise ValueError('diagnostic_lightweight_kernel_is_not_the_production_owner')
        if not result['production_comparison_eligible']:
            raise ValueError('diagnostic_only_run_cannot_establish_production_improvement')
        if result['instrumentation'] != manifest['instrumentation']:
            raise ValueError('run_instrumentation_differs_from_its_suite')
        grouping = (result['gc_policy'], result['instrumentation']['tracemalloc'], result['instrumentation']['cprofile'])
        grouped.setdefault(grouping, []).append((entry, result, run))
    for (gc_policy, allocations, profile), runs in grouped.items():
        reference_entry, reference, reference_dir = runs[0]
        inputs = np.load(reference_dir/'inputs.npy', allow_pickle=False)
        wire = (reference_dir/'reference_blocks.bin').read_bytes()
        expected_size = reference['blocks']*640
        if len(wire) != expected_size:
            raise ValueError('reference_wire_size_mismatch')
        records = {'python': [], 'native': []}
        for entry, result, run in runs:
            for key in ('benchmark_version', 'blocks', 'samples', 'seed', 'configuration_sha256', 'configuration',
                        'python', 'version', 'platform', 'benchmark_script_sha256', 'implementation_files'):
                if result[key] != reference[key]:
                    raise ValueError('unfair_comparison_different_'+key+':'+entry['directory'])
            current_inputs = np.load(run/'inputs.npy', allow_pickle=False)
            current_wire = (run/'reference_blocks.bin').read_bytes()
            if not np.array_equal(inputs, current_inputs):
                raise ValueError('unequal_input_values:'+entry['directory'])
            if current_wire != wire:
                raise ValueError('wire_parity_failed:'+entry['directory'])
            if hashlib.sha256(current_wire).hexdigest() != result['wire_sha256']:
                raise ValueError('wire_artifact_hash_changed:'+entry['directory'])
            if hashlib.sha256((run/'inputs.npy').read_bytes()).hexdigest() != result['input_sha256']:
                raise ValueError('input_artifact_hash_changed:'+entry['directory'])
            timing = np.load(run/'timings.npz', allow_pickle=False)
            if len(timing['wall_s']) != result['blocks'] or len(timing['thread_cpu_s']) != result['blocks']:
                raise ValueError('timing_window_count_mismatch:'+entry['directory'])
            records[result['backend']].append((entry, result, timing))
        if not records['python'] or not records['native']:
            raise ValueError('comparison_requires_both_backends')
        instrumented = allocations or profile
        group = dict(gc_policy=gc_policy, instrumentation=dict(tracemalloc=allocations, cprofile=profile),
            classification='instrumented_diagnostic_only' if instrumented else 'performance_measurement',
            fair_functional_configuration=True, identical_input_values=True, wire_bytes_exact=True,
            production_owner='native_cpp_two_path_producer', blocks_per_run=reference['blocks'],
            benchmark_version=reference['benchmark_version'],
            process_cpu_scope='aligned measured window before GC restoration' if reference['benchmark_version']>=3 else 'v2 rusage after GC restoration; process CPU not a strict matching-window comparison',
            seed=reference['seed'], configuration=reference['configuration'],
            configuration_sha256=reference['configuration_sha256'],
            wire_sha256=reference['wire_sha256'], runs=[entry['directory'] for entry, _, _ in runs],
            backends={}, comparison={})
        for backend, rows in records.items():
            pooled_wall = np.concatenate([timing['wall_s'] for _, _, timing in rows])
            pooled_cpu = np.concatenate([timing['thread_cpu_s'] for _, _, timing in rows])
            count = sum(result['blocks'] for _, result, _ in rows)
            elapsed = sum(result['elapsed_wall_s'] for _, result, _ in rows)
            cpu = sum(result['measured_window_cpu_s'] for _, result, _ in rows)
            pauses = [pause['wall_s'] for _, result, _ in rows for pause in result['gc_pauses']]
            group['backends'][backend] = dict(run_count=len(rows), measured_blocks=count,
                wall=distribution(pooled_wall), thread_cpu=distribution(pooled_cpu),
                process_cpu_per_block_s=cpu/count, blocks_per_elapsed_wall_second=count/elapsed,
                peak_process_rss_mib=max(result['peak_process_rss_mib'] for _, result, _ in rows),
                peak_rss_each_run_mib=[result['peak_process_rss_mib'] for _, result, _ in rows],
                gc_pause_count=len(pauses), gc_pause_max_s=max(pauses, default=0.), gc_pause_total_s=sum(pauses),
                voluntary_context_switches=sum(result['voluntary_context_switches'] for _, result, _ in rows),
                involuntary_context_switches=sum(result['involuntary_context_switches'] for _, result, _ in rows),
                native_pybind_calls=sum(result.get('native_pybind_calls_in_measured_window') or 0 for _, result, _ in rows),
                individual_runs=[dict(directory=entry['directory'], wall=result['wall'], thread_cpu=result['thread_cpu'],
                    process_cpu_s=result['measured_window_cpu_s'], peak_process_rss_mib=result['peak_process_rss_mib'],
                    gc_pause_count=len(result['gc_pauses']), elapsed_wall_s=result['elapsed_wall_s'],
                    voluntary_context_switches=result['voluntary_context_switches'],
                    involuntary_context_switches=result['involuntary_context_switches']) for entry, result, _ in rows])
        before, after = group['backends']['python'], group['backends']['native']
        for metric in ('wall', 'thread_cpu'):
            group['comparison'][metric] = {quantile: improvement(before[metric][quantile], after[metric][quantile])
                for quantile in ('p50_s', 'p95_s', 'p99_s', 'max_s')}
        for metric in ('process_cpu_per_block_s', 'peak_process_rss_mib'):
            group['comparison'][metric] = improvement(before[metric], after[metric])
        group['comparison']['throughput'] = dict(python=before['blocks_per_elapsed_wall_second'],
            native=after['blocks_per_elapsed_wall_second'],
            native_over_python=after['blocks_per_elapsed_wall_second']/before['blocks_per_elapsed_wall_second'])
        comparisons.append(group)
    complete = not errors and all(entry['state'] == 'COMPLETED' for entry in manifest['runs'])
    report = dict(scope='WSL host software microbenchmark; no running MCU/watchdog, no Pi/STM32 or physical throughput claim',
        passed=complete, fixed_order=list(ORDER), fixed_gc_policies=list(POLICIES),
        groups=comparisons, incomplete_runs=errors,
        interpretation='Positive improvement_percent means reduction; negative values are retained regressions. Quantiles are pooled from raw samples, not averaged percentiles.',
        limitations=['FULL solver mathematics and physical/ROS end-to-end behavior are outside this LIGHTWEIGHT input window',
            'Process RSS includes imports/warmup and measured window; GIL wait is not separately measured',
            'Instrumented cProfile/tracemalloc runs are separate diagnostic groups and cannot establish speedup',
            'The existing compiled Ruckig computation is reused on both sides; gains concern surrounding runtime/binding/allocation work'],
        hardware_execution_ready=False)
    write_json(directory/'suite_summary.json', report)
    return report


def run(args):
    directory = args.output_dir.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    if any(directory.iterdir()):
        raise ValueError('suite_output_directory_must_be_empty_to_preserve_evidence')
    manifest = dict(state='RUNNING', created_utc=utc(), python=sys.executable,
        fixed_blocks_per_run=10000, seed=args.seed, fixed_order=list(ORDER),
        instrumentation=dict(tracemalloc=args.allocations, cprofile=args.profile),
        quiet_window_note=args.quiet_window_note,
        runs=[])
    for policy in POLICIES:
        for index, backend in enumerate(ORDER, 1):
            manifest['runs'].append(dict(directory=f'{policy}_{index:02d}_{backend}',
                backend=backend, gc_policy=policy, state='PENDING'))
    write_json(directory/'suite_manifest.json', manifest)
    for entry in manifest['runs']:
        entry.update(state='RUNNING', started_utc=utc())
        command = [sys.executable, str(SCRIPT), '--backend', entry['backend'],
            '--output-dir', str(directory/entry['directory']), '--blocks', '10000',
            '--seed', str(args.seed), '--gc-policy', entry['gc_policy']]
        if args.allocations: command.append('--allocations')
        if args.profile: command.append('--profile')
        entry['command'] = command
        write_json(directory/'suite_manifest.json', manifest)
        start = time.perf_counter()
        with (directory/(entry['directory']+'.stdout.log')).open('w') as stdout, \
             (directory/(entry['directory']+'.stderr.log')).open('w') as stderr:
            completed = subprocess.run(command, stdout=stdout, stderr=stderr)
        entry.update(state='COMPLETED' if completed.returncode == 0 else 'FAILED',
            exit_code=completed.returncode, finished_utc=utc(), process_wall_s=time.perf_counter()-start)
        write_json(directory/'suite_manifest.json', manifest)
        if completed.returncode:
            manifest.update(state='FAILED', finished_utc=utc())
            write_json(directory/'suite_manifest.json', manifest)
            raise RuntimeError('benchmark_child_failed:'+entry['directory'])
    manifest.update(state='COMPLETED', finished_utc=utc())
    write_json(directory/'suite_manifest.json', manifest)
    return summarize(directory)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    execute = commands.add_parser('run')
    execute.add_argument('--output-dir', type=Path, required=True)
    execute.add_argument('--seed', type=int, default=20261023)
    execute.add_argument('--allocations', action='store_true')
    execute.add_argument('--profile', action='store_true')
    execute.add_argument('--quiet-window-note', default='')
    aggregate = commands.add_parser('summarize')
    aggregate.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    report = run(args) if args.command == 'run' else summarize(args.output_dir.resolve())
    print(json.dumps(dict(passed=report['passed'], groups=len(report['groups']),
        summary=str(args.output_dir.resolve()/'suite_summary.json'))), flush=True)
    if not report['passed']: raise SystemExit(1)


if __name__ == '__main__':
    main()
