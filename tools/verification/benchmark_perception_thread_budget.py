"""Finite offline-RGB CPU thread-budget ablation, never a robot runtime.

Each invocation is one fresh process and one requested budget. The original
PersonDetector/YoloDepthAdapter own separate worker threads/models. Both see the
same real, saved RGB for each paired iteration. Pair barriers make the load
repeatable; this deliberately does not model production latest-slot drops.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import struct
import sys
import threading
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def quantiles(values):
    import numpy as np
    data = np.asarray(values, dtype=np.float64)
    return dict(count=len(data), p50_s=float(np.quantile(data,.5)),
        p95_s=float(np.quantile(data,.95)), p99_s=float(np.quantile(data,.99)),
        max_s=float(data.max()), total_s=float(data.sum()))


def usage():
    value = resource.getrusage(resource.RUSAGE_SELF)
    status = {}
    for line in Path('/proc/self/status').read_text().splitlines():
        key, _, rest = line.partition(':')
        if key in ('VmRSS','VmHWM','Threads'):
            status[key] = int(rest.split()[0])
    threads = {}
    ticks = os.sysconf('SC_CLK_TCK')
    for task in Path('/proc/self/task').iterdir():
        try:
            stat = (task/'stat').read_text()
            tail = stat[stat.rfind(')')+2:].split()
            threads[task.name] = dict(name=(task/'comm').read_text().strip(),
                cpu_s=(int(tail[11])+int(tail[12]))/ticks)
        except FileNotFoundError:
            continue
    return dict(process_cpu_s=value.ru_utime+value.ru_stime,
        peak_rss_mib=value.ru_maxrss/1024, rss_mib=status.get('VmRSS',0)/1024,
        current_threads=status.get('Threads'), voluntary_switches=value.ru_nvcsw,
        involuntary_switches=value.ru_nivcsw, task_cpu=threads, task_cpu_clock_ticks_per_s=ticks)


def prepare_manifest(path, dataset, frames, protocol):
    if not 10 <= frames <= 20: raise ValueError('finite_protocol_requires_10_to_20_frames')
    candidates = sorted(dataset.glob('*heldout_split/rgb.png'))
    if len(candidates) < frames: raise ValueError('not_enough_existing_rendered_rgb_frames')
    images = []
    for item in candidates[:frames]:
        header = item.read_bytes()[:24]
        if header[:8] != b'\x89PNG\r\n\x1a\n': raise ValueError('expected_png_source')
        width,height = struct.unpack('>II',header[16:24])
        images.append(dict(path=str(item.resolve().relative_to(ROOT)),sha256=sha(item),width=width,height=height))
    cfg = json.loads(protocol.read_text())
    detector = ROOT/cfg['detector']['model']; depth = ROOT/cfg['depth']['model']
    report = dict(schema_version=1, scope='fixed_saved_true_MuJoCo_Bookstore_Gazebo_RGB_offline_replay',
        frames=frames, images=images, protocol=dict(path=str(protocol.relative_to(ROOT)),sha256=sha(protocol)),
        detector=dict(path=str(detector.relative_to(ROOT)),sha256=sha(detector),imgsz=cfg['detector']['imgsz'],
            confidence=cfg['detector']['confidence']),
        depth=dict(path=str(depth.relative_to(ROOT)),sha256=sha(depth),imgsz=cfg['depth']['imgsz']),
        requested_budgets=['default','1','2','4'],fixed_environment={'OMP_NUM_THREADS':'1'},
        warmup='one real source frame sequential model setup, then one real source frame per worker after budget; both excluded',
        scheduling='two independent model workers with paired iteration start/completion barriers; no queued stale frames',
        timestamp_policy='ColorFrame timestamp is actual cv2.imread completion in this offline replay, never original simulation exposure',
        hardware_execution_ready=False)
    path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists(): raise FileExistsError(path)
    path.write_text(json.dumps(report,indent=2)+'\n')
    return report


def run(args):
    # Imports occur only in the explicitly launched model benchmark process.
    import cv2
    import numpy as np
    import torch
    from companionbot_perception.core.camera import ColorFrame
    from companionbot_perception.core.person_detector import PersonDetector, PersonDetectorConfig
    from companionbot_perception.core.depth import YoloDepthAdapter,YoloDepthConfig
    manifest = json.loads(args.manifest.read_text())
    if os.environ.get('OMP_NUM_THREADS') != '1': raise ValueError('fixed_OMP_NUM_THREADS_must_be_1_before_process_start')
    if not 10 <= len(manifest['images']) <= 20: raise ValueError('finite_protocol_requires_10_to_20_frames')
    out = args.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    for image in manifest['images']:
        if sha(ROOT/image['path']) != image['sha256']: raise ValueError('input_changed:'+image['path'])
    for key in ('detector','depth'):
        if sha(ROOT/manifest[key]['path']) != manifest[key]['sha256']: raise ValueError('model_changed:'+key)
    if sha(ROOT/manifest['protocol']['path']) != manifest['protocol']['sha256']: raise ValueError('protocol_changed')
    source_id = 'offline-rendered-replay:'+sha(args.manifest)[:24]
    first = manifest['images'][0]
    def load(image, sequence):
        pixel = cv2.imread(str(ROOT/image['path']), cv2.IMREAD_COLOR)
        received = time.perf_counter()
        if pixel is None or pixel.shape[:2] != (image['height'],image['width']): raise ValueError('source_image_decode_mismatch')
        pixel.setflags(write=False)
        return ColorFrame(pixel,sequence,source_id,image['width'],image['height'],received)
    initial = dict(torch_intraop=torch.get_num_threads(),torch_interop=torch.get_num_interop_threads(),
        opencv_threads=cv2.getNumThreads(),torch_version=torch.__version__,torch_parallel_info=torch.__config__.parallel_info(),
        environment={key:os.environ.get(key) for key in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS','OMP_DYNAMIC','OMP_PROC_BIND')},
        cpu_count=os.cpu_count(),affinity_cpu_count=len(os.sched_getaffinity(0)))
    initial['cgroup_cpu']={str(path):path.read_text().strip() for path in
        (Path('/sys/fs/cgroup/cpu.max'),Path('/sys/fs/cgroup/cpuset.cpus.effective')) if path.exists()}
    ready = [threading.Event(),threading.Event()]
    budget = [threading.Event(),threading.Event()]
    finish = threading.Event();stop = threading.Event();barrier = threading.Barrier(3)
    frame_box = [None]
    channels = [{},{ }];errors=[];error_lock=threading.Lock()
    warm = load(first,0)
    requested = None if args.threads == 'default' else int(args.threads)
    def worker(index):
        name = ('detector','depth')[index]
        channel = channels[index]
        try:
            if index == 1 and not ready[0].wait(60.): raise TimeoutError('detector_setup_wait')
            if stop.is_set():return
            channel['native_tid'] = threading.get_native_id()
            channel['thread_before_model_init'] = torch.get_num_threads()
            cfg = manifest[name]
            adapter = (PersonDetector(PersonDetectorConfig(model_path=ROOT/cfg['path'],
                imgsz=cfg['imgsz'],confidence_threshold=cfg['confidence'],device='cpu')) if index == 0 else
                YoloDepthAdapter(YoloDepthConfig(model_path=ROOT/cfg['path'],imgsz=cfg['imgsz'],device='cpu')))
            channel['thread_after_model_init'] = torch.get_num_threads()
            before = time.perf_counter();adapter.process(warm)
            channel['setup_real_frame_wall_s'] = time.perf_counter()-before
            channel['thread_after_first_predict'] = torch.get_num_threads()
            ready[index].set()
            if index == 0:
                if not ready[1].wait(60.):raise TimeoutError('depth_setup_wait')
            else:
                if not budget[0].wait(60.):raise TimeoutError('first_budget_wait')
            if stop.is_set():return
            if requested is not None: torch.set_num_threads(requested)
            channel['thread_after_public_setter'] = torch.get_num_threads()
            budget[index].set()
            if index == 0 and not budget[1].wait(60.):raise TimeoutError('second_budget_wait')
            barrier.wait(timeout=60.)
            adapter.process(warm)  # Same real frame; unmeasured post-budget warmup.
            channel['thread_after_budget_warmup'] = torch.get_num_threads()
            channel['interop_threads'] = torch.get_num_interop_threads()
            channel['opencv_threads'] = cv2.getNumThreads()
            channel['rows'],channel['outputs'] = [],[]
            barrier.wait(timeout=60.)
            for sequence in range(len(manifest['images'])):
                barrier.wait(timeout=60.)
                frame = frame_box[0]
                thread_before = time.thread_time_ns();before = time.perf_counter_ns()
                actual_before = torch.get_num_threads()
                value = adapter.process(frame)
                wall_s = (time.perf_counter_ns()-before)*1e-9
                cpu_s = (time.thread_time_ns()-thread_before)*1e-9
                actual_after = torch.get_num_threads()
                if requested is not None and (actual_before != requested or actual_after != requested):
                    raise RuntimeError('public_thread_budget_not_honored')
                row = dict(sequence=sequence,wall_s=wall_s,thread_cpu_s=cpu_s,
                    actual_threads_before=actual_before,actual_threads_after=actual_after)
                if index == 0:
                    array = np.asarray([[*d.bbox_xyxy_px,d.confidence,d.class_id] for d in value],dtype=np.float64).reshape(-1,6)
                    row['person_count'] = len(value)
                else:
                    array = value.depth_m
                    row['finite_pixels'] = int(np.isfinite(array).sum())
                channel['outputs'].append(array)
                channel['rows'].append(row)
                barrier.wait(timeout=60.)
            finish.wait(timeout=60.)
        except BaseException as error:
            with error_lock:errors.append(dict(channel=name,error=repr(error),traceback=traceback.format_exc()))
            stop.set();barrier.abort()
            for event in ready+budget:event.set()
    workers = [threading.Thread(target=worker,args=(i,),name='benchmark-'+('detector','depth')[i]) for i in range(2)]
    pairs=[];before_usage=after_usage=None;started=ended=None
    for item in workers:item.start()
    try:
        barrier.wait(timeout=60.)  # Both original backends set up; public budget applied.
        barrier.wait(timeout=60.)  # Both real post-budget warmups completed.
        before_usage=usage();started=time.perf_counter()
        for sequence,image in enumerate(manifest['images']):
            frame_box[0]=load(image,sequence)
            cpu_before=time.process_time_ns();wall_before=time.perf_counter_ns()
            barrier.wait(timeout=60.);barrier.wait(timeout=60.)
            pairs.append(dict(sequence=sequence,paired_wall_s=(time.perf_counter_ns()-wall_before)*1e-9,
                process_cpu_s=(time.process_time_ns()-cpu_before)*1e-9,
                replay_read_complete_host_s=frame_box[0].host_receive_time_s))
        ended=time.perf_counter();after_usage=usage()
    except BaseException as error:
        stop.set();barrier.abort()
        errors.append(dict(channel='orchestrator',error=repr(error),traceback=traceback.format_exc()))
    finally:
        finish.set()
        for item in workers:item.join(timeout=5.)
    arrays={}
    for index,name in enumerate(('detector','depth')):
        values=channels[index].pop('outputs',[])
        for sequence,value in enumerate(values):arrays[name+'_'+str(sequence)]=value
    np.savez_compressed(out/'outputs.npz',**arrays)
    report=dict(schema_version=1,scope='WSL host offline saved-RGB two-worker CPU-budget ablation; no robot or safety acceptance',
        python=sys.executable,python_version=sys.version,
        requested_threads=args.threads,manifest_sha256=sha(args.manifest),input_manifest=manifest,
        initial=initial,channels={name:channels[i] for i,name in enumerate(('detector','depth'))},
        pairs=pairs,errors=errors,completed=not errors and len(pairs)==len(manifest['images']),
        process_before=before_usage,process_after=after_usage,elapsed_wall_s=None if ended is None else ended-started,
        scheduling='paired barriers for repeatable concurrent inference, not asynchronous production/drop throughput',
        thread_cpu_note='calling worker CPU excludes OpenMP helper CPU; process CPU includes every thread',
        timestamp_note='actual offline cv2.imread completion; original render exposure is not fabricated or used as a fresh timestamp',
        warmup_note='both sequential first-predict real frame and concurrent post-budget real frame are excluded from all measurement samples',
        process_peak_rss_note='includes interpreter/model import/warmup; process before/after current RSS also retained',
        hardware_execution_ready=False)
    for name,channel in report['channels'].items():
        rows=channel.get('rows',[])
        if rows:
            channel['wall']=quantiles([r['wall_s'] for r in rows]);channel['thread_cpu']=quantiles([r['thread_cpu_s'] for r in rows])
            channel['actual_thread_counts']=sorted({r['actual_threads_before'] for r in rows}|{r['actual_threads_after'] for r in rows})
    if pairs:
        report['paired_wall']=quantiles([r['paired_wall_s'] for r in pairs])
        report['pair_process_cpu']=quantiles([r['process_cpu_s'] for r in pairs])
    if before_usage and after_usage:
        report['window_process_cpu_s']=after_usage['process_cpu_s']-before_usage['process_cpu_s']
        report['window_context_switches']={key:after_usage[key]-before_usage[key] for key in ('voluntary_switches','involuntary_switches')}
        earlier,later=before_usage['task_cpu'],after_usage['task_cpu']
        report['task_cpu_delta_s']={tid:dict(name=later[tid]['name'],cpu_s=later[tid]['cpu_s']-earlier[tid]['cpu_s']) for tid in earlier.keys()&later.keys()}
        report['task_ids_appeared']=sorted(later.keys()-earlier.keys());report['task_ids_gone']=sorted(earlier.keys()-later.keys())
    report['source_sha256']={str(path.relative_to(ROOT)):sha(path) for path in [Path(__file__).resolve(),
        ROOT/'ros2_ws/src/companionbot_perception/companionbot_perception/core/person_detector.py',
        ROOT/'ros2_ws/src/companionbot_perception/companionbot_perception/core/depth.py',
        Path(torch.__file__).parent.parent/'ultralytics/utils/torch_utils.py',
        Path(torch.__file__).parent.parent/'ultralytics/engine/predictor.py']}
    (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    if not report['completed']:raise RuntimeError('thread_budget_benchmark_failed:'+repr(errors))
    print(json.dumps({k:report[k] for k in ('requested_threads','completed','paired_wall','window_process_cpu_s')}),flush=True)


def summarize(path):
    import numpy as np
    reports={name:json.loads((path/name/'summary.json').read_text()) for name in ('default','1','2','4')}
    if not all(value['completed'] for value in reports.values()):raise ValueError('incomplete_budget_ablation')
    if len({value['manifest_sha256'] for value in reports.values()}) != 1:raise ValueError('different_inputs_in_budget_ablation')
    baseline=np.load(path/'default/outputs.npz',allow_pickle=False)
    differences={}
    for name in ('1','2','4'):
        actual=np.load(path/name/'outputs.npz',allow_pickle=False)
        rows=[]
        for key in baseline.files:
            lhs,rhs=baseline[key],actual[key]
            same_shape=lhs.shape==rhs.shape
            row=dict(output=key,shape_equal=same_shape,baseline_shape=list(lhs.shape),actual_shape=list(rhs.shape))
            if same_shape:
                finite=np.isfinite(lhs)&np.isfinite(rhs)
                row['finite_mask_equal']=bool(np.array_equal(np.isfinite(lhs),np.isfinite(rhs)))
                row['exact_equal']=bool(np.array_equal(lhs,rhs,equal_nan=True))
                diff=np.abs(lhs[finite].astype(np.float64)-rhs[finite].astype(np.float64))
                row['max_absolute_difference']=float(diff.max()) if diff.size else 0.
                row['p99_absolute_difference']=float(np.quantile(diff,.99)) if diff.size else 0.
                denom=np.maximum(np.abs(lhs[finite].astype(np.float64)),np.finfo(float).tiny)
                row['max_relative_difference']=float((diff/denom).max()) if diff.size else 0.
            rows.append(row)
        differences[name]=rows
    report=dict(scope='finite same-RGB same-model same-imgsz concurrent CPU-budget ablation; WSL host only',
        budgets={name:dict(paired_wall=v['paired_wall'],pair_process_cpu=v['pair_process_cpu'],
            window_cpu_s=v['window_process_cpu_s'],rss_before_mib=v['process_before']['rss_mib'],
            peak_rss_mib=v['process_after']['peak_rss_mib'],threads_before=v['process_before']['current_threads'],
            channels={key:dict(wall=c['wall'],actual_thread_counts=c['actual_thread_counts'],thread_cpu=c['thread_cpu']) for key,c in v['channels'].items()}) for name,v in reports.items()},
        numerical_differences_from_default=differences,manifest_sha256=reports['default']['manifest_sha256'],
        limitations=['one short run per budget, not an end-to-end navigation/vision acceptance or Pi measurement',
            'paired barriers intentionally exclude latest-slot drop effects; actual asynchronous deployment needs a separate verified run',
            'no output-equivalence tolerance selected after results: exact and absolute/relative differences are reported without changing safety gates',
            'intra-op budget uses public PyTorch API after original first-predict setup; actual counts checked per measured call',
            'no inter-op/OpenCV/backend/precision/model/imgsz/safety changes in this isolated intra-op comparison'],hardware_execution_ready=False)
    base=reports['default']
    report['ratios_to_default']={name:dict(
        paired_p95=value['paired_wall']['p95_s']/base['paired_wall']['p95_s'],
        paired_p99=value['paired_wall']['p99_s']/base['paired_wall']['p99_s'],
        process_cpu=value['window_process_cpu_s']/base['window_process_cpu_s'],
        peak_rss=value['process_after']['peak_rss_mib']/base['process_after']['peak_rss_mib'])
        for name,value in reports.items()}
    (path/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'budgets':report['budgets']}),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-manifest',type=Path)
    parser.add_argument('--dataset',type=Path,default=ROOT/'models/minisegway/native_runtime/results/session_20261023/visual/gazebo_640_fast_depth_eval')
    parser.add_argument('--protocol',type=Path,default=ROOT/'models/minisegway/native_runtime/config/visual_depth_protocol_gazebo_640_fast.json')
    parser.add_argument('--frames',type=int,default=12)
    parser.add_argument('--manifest',type=Path)
    parser.add_argument('--threads',choices=('default','1','2','4'))
    parser.add_argument('--output-dir',type=Path)
    parser.add_argument('--summarize',type=Path)
    args=parser.parse_args()
    if args.prepare_manifest:
        prepare_manifest(args.prepare_manifest,args.dataset,args.frames,args.protocol);return
    if args.summarize:summarize(args.summarize);return
    if not all((args.manifest,args.threads,args.output_dir)):parser.error('run needs --manifest --threads --output-dir')
    run(args)


if __name__=='__main__':main()
