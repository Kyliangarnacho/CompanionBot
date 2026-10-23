"""Finite current-model CPU benchmark for SBC commissioning, no camera/accuracy claim.

Run the SAME script on the bought board. Host x86 numbers are not Pi estimates.
No model/export/geometry/follow thresholds are changed; all three current model
adapters are measured both sequentially and concurrently using the same fixture.
"""
from pathlib import Path
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
import platform
import resource
import time

ROOT=Path(__file__).resolve().parents[2]

def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--output-dir",type=Path,required=True)
    cli.add_argument("--samples",type=int,default=10)
    cli.add_argument("--threads",type=int,default=2)
    cli.add_argument("--detector-model",type=Path,default=ROOT/"yolo26n.pt")
    args=cli.parse_args()
    if args.samples<3 or args.threads<1: raise ValueError("samples >= 3, threads >= 1 required")
    output=args.output_dir.resolve();output.mkdir(parents=True,exist_ok=True)
    import torch
    torch.set_num_threads(args.threads);torch.set_num_interop_threads(1)
    import cv2
    import numpy as np
    from companionbot_perception.core.person_detector import PersonDetector,PersonDetectorConfig
    from companionbot_perception.core.person_reid import PersonReIdentifier,TorchReIDOSNetBackend,default_osnet_weights_path
    from companionbot_perception.core.depth import YoloDepthAdapter,YoloDepthConfig
    from companionbot_perception.core.camera import ColorFrame
    cv2.setNumThreads(1)
    original=cv2.imread(str(ROOT/"models/minisegway/stage7/stage7_1/results/person_detection_preview.jpg"))
    if original is None: raise RuntimeError("missing frozen fixture")
    image=np.zeros((720,1280,3),dtype=np.uint8);image[72:648,424:856]=cv2.resize(original,(432,576))
    frame=ColorFrame(image,1,"fixture:embedded_capacity",1280,720,time.perf_counter())
    detector=PersonDetector(PersonDetectorConfig(model_path=args.detector_model,imgsz=640,device="cpu"))
    reid=PersonReIdentifier(TorchReIDOSNetBackend(default_osnet_weights_path()))
    depth_path=ROOT/".venv/models/yolo26n-depth.pt"
    depth=YoloDepthAdapter(YoloDepthConfig(model_path=depth_path,imgsz=768,device="cpu"))
    detections=detector.process(frame)
    if not detections: raise RuntimeError("fixture person missing")
    box=detections[0].bbox_xyxy_px
    operations={"yolo26n_640":lambda:detector.process(frame),
        "osnet_x025_256x128_one_crop":lambda:reid.embed(image,box,source_id=frame.source_id,source_sequence_id=1,track_id=1),
        "yolo26n_depth_768":lambda:depth.process(frame)}
    def sample(fn):
        start=time.perf_counter();fn();return (time.perf_counter()-start)*1000
    def stats(values):return {"wall_ms":values,"p50_ms":float(np.median(values)),"p95_ms":float(np.percentile(values,95)),
                              "mean_ms":float(np.mean(values)),"max_ms":float(max(values))}
    report={"platform":platform.platform(),"machine":platform.machine(),"cpu":platform.processor(),
        "torch_version":torch.__version__,"torch_threads":args.threads,"source_resolution":[1280,720],
        "fixture_only":True,"real_camera_tested":False,"pi_performance_claim":False,
        "models_sha256":{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (args.detector_model,default_osnet_weights_path(),depth_path) if p.is_file()},"sequential":{},"concurrent":{}}
    for name,fn in operations.items():
        for _ in range(2):fn()
        report["sequential"][name]=stats([sample(fn) for _ in range(args.samples)])
        print(name,report["sequential"][name]["p50_ms"],flush=True)
    with ThreadPoolExecutor(max_workers=3) as executor:
        futures={name:executor.submit(lambda fn=fn:[sample(fn) for _ in range(args.samples)]) for name,fn in operations.items()}
        for name,future in futures.items():report["concurrent"][name]=stats(future.result())
    report["process_peak_rss_mib"]=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024
    report["available_memory_mib"]=os.sysconf("SC_AVPHYS_PAGES")*os.sysconf("SC_PAGE_SIZE")/1024**2
    (output/"capacity.json").write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)

if __name__=="__main__":main()
