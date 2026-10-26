"""Bounded diagnostic for actual native return-to-GIL-reacquired latency.

Never installed or selected by robot bringup. A proxy invokes the same runtime
instance through an explicit local probe ELF; all other methods delegate to it.
"""
import argparse
from collections import deque
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[2]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value):path.write_text(json.dumps(value,indent=2)+'\n')


def load_probe(path):
    files=list(path.resolve().glob('_runtime_gil_probe*.so'))
    if len(files)!=1:raise ValueError('one_explicit_local_diagnostic_probe_ELF_required')
    spec=importlib.util.spec_from_file_location('_runtime_gil_probe',files[0])
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    return module,files[0]


def fingerprints(binary):
    files=[binary,Path(__file__).resolve(),Path('/usr/include/pybind11/gil.h'),
        Path('/usr/include/pybind11/detail/internals.h'),
        ROOT/'ros2_ws/src/companionbot_simulation/native/include/companionbot_simulation/navigation_runtime.hpp',
        ROOT/'ros2_ws/install/companionbot_simulation/lib/libcompanionbot_navigation_sim.so',
        ROOT/'ros2_ws/install/companionbot_follow/lib/libcompanionbot_follow_native.so',
        ROOT/'ros2_ws/install/companionbot_control/lib/libcompanionbot_core.so',
        ROOT/'models/minisegway/native_runtime/results/session_20261023/profiling/gil_return_probe/src/step_probe.cpp']
    return {str(p.resolve()):sha(p) for p in files}


class DiagnosticRuntimeProxy:
    def __init__(self,target,probe):self._target,self._probe=target,probe
    def step(self,*args):return self._probe.step(self._target,*args)
    def __getattr__(self,name):return getattr(self._target,name)


class PythonBusyWorker:
    """One declared4ms/6ms load, not an assertion of uninterrupted GIL ownership."""
    def __init__(self):
        self.stop=threading.Event();self.worker=None
        self.rows=deque(maxlen=16384);self.total=0;self.errors=[];self.native_tid=None
    def run(self):
        self.native_tid=threading.get_native_id();counter=0
        try:
            while not self.stop.is_set():
                begin=time.perf_counter_ns();cpu=time.thread_time_ns();limit=begin+4000000
                while time.perf_counter_ns()<limit:counter=(counter+17)%65521
                self.rows.append((begin,time.perf_counter_ns(),time.thread_time_ns()-cpu))
                self.total+=1
                self.stop.wait(.006)
        except BaseException as error:self.errors.append(repr(error))
    def start(self):
        if self.worker is not None:raise RuntimeError('diagnostic_busy_worker_already_started')
        self.worker=threading.Thread(target=self.run,name='diagnostic-python-gil-contender');self.worker.start()
    def close(self):
        self.stop.set()
        if self.worker:
            self.worker.join(10.)
            if self.worker.is_alive():raise RuntimeError('diagnostic_busy_worker_shutdown_timeout')
    def save(self,out):
        np.save(out/'python_busy_intervals.npy',np.asarray(self.rows,dtype=np.uint64).reshape(-1,3))
        return dict(requested_busy_wall_s=.004,requested_event_wait_s=.006,
            actual_intervals=self.total,retained_intervals=len(self.rows),overwritten=max(0,self.total-len(self.rows)),
            native_tid=self.native_tid,errors=self.errors,thread_stopped=self.worker is None or not self.worker.is_alive(),
            interpretation='Python busy intervals can include interpreter preemption and OS scheduling; not continuous GIL-owner trace')


def distribution(values):
    values=np.asarray(values,dtype=np.float64)
    return dict(count=len(values),p50_s=float(np.quantile(values,.5)),p95_s=float(np.quantile(values,.95)),
        p99_s=float(np.quantile(values,.99)),max_s=float(values.max()),total_s=float(values.sum()))


def record_summary(records):
    if records.ndim!=2 or records.shape[1]!=10 or records.dtype!=np.uint64:
        raise ValueError('invalid_probe_record_contract')
    data=records.astype(np.int64)
    reacquire_wall=(data[:,8]-data[:,6])*1e-9
    reacquire_cpu=(data[:,9]-data[:,7])*1e-9
    return dict(records=len(records),failed_native_calls=int(np.count_nonzero(data[:,1]==0)),
        native_call_wall=distribution((data[:,6]-data[:,4])*1e-9),
        native_call_thread_cpu=distribution((data[:,7]-data[:,5])*1e-9),
        return_to_gil_reacquired_wall=distribution(reacquire_wall),
        return_to_gil_reacquired_thread_cpu=distribution(reacquire_cpu),
        cpu_greater_than_wall_rows=int(np.count_nonzero(reacquire_cpu>reacquire_wall)),
        interval_semantics='native step returned and unlocked runtime mutex -> actual gil_scoped_release destructor completed; includes OS scheduling, clock probes and interpreter restoration, not pure GIL wait')


def clock_audit(module):
    rows=[]
    for _ in range(100):
        begin=time.perf_counter_ns();wall,cpu=module.clock_sample_ns();end=time.perf_counter_ns()
        rows.append((begin,wall,end,cpu))
    return dict(python_perf_counter=time.get_clock_info('perf_counter').__dict__,
        cpp_steady_samples_bracketed_by_python_perf_counter=all(a<=b<=c for a,b,c,_ in rows),
        samples=rows)


def parity(args,module,binary,out):
    from companionbot_simulation import _native_simulation as native
    source=ROOT/'tests/test_native_navigation_runtime.py'
    spec=importlib.util.spec_from_file_location('gil_probe_existing_parity_test',source)
    test=importlib.util.module_from_spec(spec);spec.loader.exec_module(test)
    configuration=test.configuration.__wrapped__()
    original=native.NavigationRuntime
    results=[]
    for mode in ('normal','cancel_pending'):
        probe=module.StepProbe(7500);targets=[];deltas=[]
        original_assert=np.testing.assert_allclose
        def checked_assert(a,b,*inputs,**kw):
            original_assert(a,b,*inputs,**kw)
            deltas.append(float(np.max(np.abs(np.asarray(a)-np.asarray(b)))))
        def factory(*inputs,**kw):
            target=original(*inputs,**kw);targets.append(target)
            return DiagnosticRuntimeProxy(target,probe)
        with patch.object(native,'NavigationRuntime',factory),patch.object(np.testing,'assert_allclose',checked_assert):
            test.test_complete_native_scheduler_every_tick(configuration,0.,mode)
        records=probe.copy_records();assert records.flags.owndata and not records.flags.writeable
        np.save(out/(mode+'_records.npy'),records)
        results.append(dict(mode=mode,passed=True,records=len(records),original_reference_atol=2e-12,
            assert_array_calls=len(deltas),maximum_reference_delta=max(deltas),same_native_runtime_instance=len(targets)==1))
    # Native exception and capacity rejection cannot silently advance a runtime.
    _,_,_,_,target,estimate=test.runtime_pair(configuration,0.)
    probe=module.StepProbe(2)
    rejected=False
    try:probe.step(target,1,.002,0.,estimate.theta_hat_rad,0.)
    except ValueError:rejected=True
    assert rejected
    probe.step(target,0,0.,0.,estimate.theta_hat_rad,0.)
    rejected=False
    try:probe.step(target,1,.002,0.,estimate.theta_hat_rad,0.)
    except RuntimeError as error:rejected='capacity_exhausted' in str(error)
    assert rejected
    target.step(1,.002,0.,estimate.theta_hat_rad,0.)
    records=probe.copy_records();assert records[:,1].tolist()==[0,1]
    copied=probe.copy_records();assert not np.shares_memory(records,copied)
    report=dict(scope='actual existing native/Python oracle tests with only diagnostic step wrapper; no physics or performance claim',
        passed=True,cases=results,native_exception_propagated=True,capacity_rejection_did_not_advance_runtime=True,
        readonly_independently_owned_snapshot=True,clock_audit=clock_audit(module),
        pybind_internals_id=module.pybind11_internals_id,pybind_version=module.pybind11_version,
        runtime_output_size=module.runtime_output_size,probe_columns=list(module.columns),
        existing_test_source_sha256=sha(source),identities=fingerprints(binary),hardware_execution_ready=False)
    write(out/'summary.json',report);print(json.dumps(dict(passed=True,cases=results)),flush=True)


def physics(args,module,binary,out):
    import benchmark_motion_runtime as benchmark
    from companionbot_simulation.runtime import navigation_motion
    before=fingerprints(binary);probe=module.StepProbe(10000);target=[]
    original=navigation_motion.build_navigation_reference
    busy=PythonBusyWorker()
    def build(*a,**kw):
        arbiter,link,metadata=original(*a,**kw)
        if arbiter is not link or not isinstance(arbiter,navigation_motion.NativeNavigationReference):
            raise RuntimeError('diagnostic_probe_requires_actual_single_native_navigation_owner')
        target.append(arbiter.runtime)
        arbiter.runtime=DiagnosticRuntimeProxy(arbiter.runtime,probe)
        if args.contention=='python_busy_4ms':busy.start()
        return arbiter,link,metadata
    clocks=clock_audit(module);switch_interval=sys.getswitchinterval();failure=None
    try:
        with patch.object(navigation_motion,'build_navigation_reference',build),patch.object(sys,'argv',[
                str(Path(benchmark.__file__)),'--backend','native','--duration','20',
                '--functional-trace','--output-dir',str(out)]):
            benchmark.main()
    except BaseException as error:
        failure=repr(error)
    finally:
        busy.close()
    records=probe.copy_records();np.save(out/'gil_return_records.npy',records)
    worker_report=busy.save(out)
    report=dict(scope='instrumented actual20s original MuJoCo Bookstore fixed fixture; no speedup or target-hardware certification',
        contention=args.contention,passed=failure is None and len(records)==10000 and len(target)==1,
        error=failure,same_native_runtime_instance=len(target)==1,clock_audit=clocks,
        python_switch_interval_s=switch_interval,switch_interval_unchanged=switch_interval==sys.getswitchinterval(),
        probe_columns=list(module.columns),measurements=record_summary(records),worker=worker_report,
        before_identities=before,identities_unchanged=before==fingerprints(binary),
        hardware_execution_ready=False)
    report['passed']=report['passed'] and report['identities_unchanged'] and not worker_report['errors']
    write(out/'gil_diagnostic_summary.json',report)
    print(json.dumps(dict(passed=report['passed'],contention=args.contention,measurements=report['measurements'])),flush=True)
    if not report['passed']:raise SystemExit(1)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('parity','physics'))
    parser.add_argument('--module-dir',type=Path,default=ROOT/'models/minisegway/native_runtime/results/session_20261023/profiling/gil_return_probe/build')
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--contention',choices=('none','python_busy_4ms'),default='none')
    args=parser.parse_args();out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    if any(out.iterdir()):raise FileExistsError('preserve_gil_diagnostic_evidence_choose_empty_output')
    module,binary=load_probe(args.module_dir)
    if args.command=='parity':parity(args,module,binary,out)
    else:physics(args,module,binary,out)


if __name__=='__main__':main()
