"""Finite original mixed-physics DTO-allocation regression, not a robot app.

Runtime counts surround the real unchanged estimator/observer calls. Original
control and spatial acceptance functions are reused, never replaced by mocks.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
from unittest.mock import patch

from companionbot_contracts.paths import repository_root
from companionbot_simulation.runtime import spatial_session
from verify_spatial_foundation import PosthocTruth,evaluate,control_gates,source_gates,compare_history


def main():
    root=repository_root()
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--baseline-dir',type=Path,default=root/'models/minisegway/native_runtime/results/session_20261023/spatial/physical/mixed')
    args=parser.parse_args();out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    if any(out.iterdir()):raise FileExistsError('preserve_dto_allocation_evidence_choose_empty_output')
    baseline=args.baseline_dir.resolve()
    original_result=json.loads((baseline/'frozen_result_posthoc.json').read_text())
    original_samples=[json.loads(line) for line in (baseline/'sensorized_state.jsonl').read_text().splitlines()]
    configuration=json.loads((baseline/'configuration.json').read_text())
    name='ros2_ws/src/companionbot_simulation/companionbot_simulation/runtime/spatial_session.py'
    snapshot=json.loads((root/'models/minisegway/native_runtime/results/session_20261023/source_snapshot_before.json').read_text())
    old_source=subprocess.run(['git','show','HEAD:'+name],cwd=root,capture_output=True,check=True).stdout
    if hashlib.sha256(old_source).hexdigest()!=snapshot['files'][name]:
        raise RuntimeError('HEAD_is_not_the_exact_initial_spatial_session_snapshot')
    (out/'spatial_session_before.py').write_bytes(old_source)
    counts=dict(spatial_sample_constructions=0,attitude_updates=0,control_observer_calls=0)
    old_sample=spatial_session.SpatialSample;old_attitude=spatial_session.SpatialAttitude.update
    old_observe=spatial_session.SpatialSession.observe
    def sample(*a,**kw):
        counts['spatial_sample_constructions']+=1;return old_sample(*a,**kw)
    def attitude(*a,**kw):
        counts['attitude_updates']+=1;return old_attitude(*a,**kw)
    def observe(*a,**kw):
        counts['control_observer_calls']+=1;return old_observe(*a,**kw)
    truth=PosthocTruth()
    session=spatial_session.SpatialSession(out,scenario='mixed',duration_s=14.,
        epoch=configuration['epoch'],posthoc=truth)
    with patch.object(spatial_session,'SpatialSample',sample),\
         patch.object(spatial_session.SpatialAttitude,'update',attitude),\
         patch.object(spatial_session.SpatialSession,'observe',observe):
        result=session.run()
    # Offline evaluator constructors deliberately occur after counters stop.
    metrics,rows=evaluate(session,truth)
    current_configuration=json.loads((out/'configuration.json').read_text())
    actual_samples=[json.loads(line) for line in (out/'sensorized_state.jsonl').read_text().splitlines()]
    report=dict(scope='WSL original14s mixed MuJoCo passive-spatial allocation regression; no AI/ROS/GUI/hardware',
        counts=counts,raw_sensor_samples=len(actual_samples),published_sensor_dtos_exact=actual_samples==original_samples,
        full_control_history=compare_history(original_result['history_50hz'],result['history_50hz']),
        control_history_legacy_name='history_50hz',actual_control_history_rows=len(result['history_50hz']),
        existing_control_gates=control_gates(result,14.),
        existing_spatial_source_gates=source_gates(session.samples,rows,current_configuration,14.),
        existing_spatial_short_run_envelope=metrics['short_run_envelope'],
        initial_spatial_source_sha256=snapshot['files'][name],
        actual_current_spatial_source_sha256=hashlib.sha256((root/name).read_bytes()).hexdigest(),
        prechange_constructor_count='not measured in this new run; archived exact initial observe constructs one DTO per500Hz call before its50Hz return gate',
        inference='current7000observer calls /700published DTO constructions, preserving7001attitude updates including initial held stationary sample',
        hardware_execution_ready=False)
    report['passed']=(counts==dict(spatial_sample_constructions=700,attitude_updates=7001,control_observer_calls=7000)
        and report['published_sensor_dtos_exact'] and report['full_control_history']['exact']
        and report['existing_control_gates']['passed'] and report['existing_spatial_source_gates']['passed']
        and report['existing_spatial_short_run_envelope']['passed'])
    (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(passed=report['passed'],counts=counts,published_sensor_dtos_exact=report['published_sensor_dtos_exact'],
        full_control_history_exact=report['full_control_history']['exact'])),flush=True)
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
