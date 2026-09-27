"""Learning demo: load YOLO26n, predict one image, inspect boxes, save preview."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time
from typing import Sequence

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _class_name_items(names: object) -> list[tuple[int, str]]:
    if isinstance(names, dict):
        items = names.items()
    elif isinstance(names, (list, tuple)):
        items = enumerate(names)
    else:
        raise RuntimeError("Loaded model does not expose class-name metadata")
    return [(int(class_id), str(name)) for class_id, name in items]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, required=True,
                        help="Local image to detect; cv2.imread supplies BGR uint8")
    parser.add_argument("--model", default="yolo26n.pt",
                        help="Ultralytics model path/name (default: yolo26n.pt)")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="Ultralytics inference letterbox target (default: 640)")
    parser.add_argument("--conf", type=float, default=0.25,
                        help="Minimum prediction confidence (default: 0.25)")
    parser.add_argument("--device", default=None,
                        help="Optional Ultralytics device; default lets it select")
    parser.add_argument(
        "--output", type=Path,
        default=ROOT / "models" / "minisegway" / "stage7" / "stage7_1" / "results"
                / "person_detection_preview.jpg",
        help="Annotated preview output path",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    if args.imgsz < 1:
        parser.error("--imgsz must be positive")
    if not 0.0 <= args.conf <= 1.0:
        parser.error("--conf must be in [0, 1]")
    if not args.image.is_file():
        parser.error(f"image file does not exist: {args.image}")

    # OpenCV returns H×W×3 uint8 BGR. Ultralytics predict() accepts this
    # canonical source directly and owns its internal resize/letterbox path.
    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        parser.error(f"OpenCV could not decode image: {args.image}")
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        parser.error("image did not decode as H×W×3 uint8 BGR")
    original_height, original_width = image.shape[:2]

    try:
        from ultralytics import YOLO
    except ImportError as error:
        parser.error(f"Ultralytics is unavailable; install requirements-yolo.txt ({error})")

    model_load_start_ns = time.perf_counter_ns()
    model = YOLO(args.model)
    model_load_end_ns = time.perf_counter_ns()

    person_classes = [
        (class_id, name)
        for class_id, name in _class_name_items(model.names)
        if name.strip().casefold() == "person"
    ]
    if len(person_classes) != 1:
        raise RuntimeError(
            f"Expected exactly one 'person' class in model metadata; got {person_classes}"
        )
    person_class_id, person_class_name = person_classes[0]

    print(f"image: {args.image}")
    print(f"original image shape: {image.shape}; dtype={image.dtype}; color=BGR")
    print(f"detector model: {args.model}")
    print(f"person class from model.names: {person_class_name!r} (id={person_class_id})")
    print(f"inference settings: imgsz={args.imgsz}, conf={args.conf}, "
          f"device={args.device if args.device is not None else 'Ultralytics auto'}")
    print(f"loaded model runtime device: {getattr(model, 'device', 'unreported')}")
    print("source frame timestamp: N/A (static image has no capture metadata)")
    print(f"model load wall time: {(model_load_end_ns-model_load_start_ns)/1e6:.2f} ms")

    inference_start_ns = time.perf_counter_ns()
    predict_kwargs = {
        "source": image,
        "imgsz": args.imgsz,
        "conf": args.conf,
        "verbose": False,
        "stream": False,
    }
    if args.device is not None:
        predict_kwargs["device"] = args.device
    results = model.predict(**predict_kwargs)
    inference_end_ns = time.perf_counter_ns()
    if len(results) != 1:
        raise RuntimeError(f"Expected one Results object, got {len(results)}")

    result = results[0]
    print(f"Results type: {type(result).__module__}.{type(result).__name__}")
    print(f"Results.orig_shape: {result.orig_shape} (height, width)")
    if tuple(result.orig_shape) != (original_height, original_width):
        raise RuntimeError("Results.orig_shape does not match the original image")

    annotated = image.copy()
    person_count = 0
    if result.boxes is None:
        xyxy = np.empty((0, 4), dtype=np.float32)
        confidences = np.empty((0,), dtype=np.float32)
        class_ids = np.empty((0,), dtype=np.float32)
    else:
        # These are the public Results fields: xyxy is original-input pixel
        # space; conf is score; cls is the model class index.
        xyxy = result.boxes.xyxy.cpu().numpy()
        confidences = result.boxes.conf.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy()

    for index, (bbox, confidence, class_value) in enumerate(
        zip(xyxy, confidences, class_ids)
    ):
        class_id = int(class_value)
        class_name = result.names[class_id]
        bbox_xyxy = tuple(float(value) for value in bbox)
        print(f"detection[{index}]: class={class_name}, class_id={class_id}, "
              f"confidence={float(confidence):.4f}, xyxy_px={bbox_xyxy}")
        if class_id != person_class_id:
            continue

        x1, y1, x2, y2 = bbox_xyxy
        if (not np.isfinite(bbox).all() or x1 < 0 or y1 < 0
                or x2 > original_width or y2 > original_height):
            raise RuntimeError("Person bbox is not finite original-image pixel data")
        left = max(0, min(original_width - 1, int(round(x1))))
        top = max(0, min(original_height - 1, int(round(y1))))
        right = max(0, min(original_width - 1, int(round(x2))))
        bottom = max(0, min(original_height - 1, int(round(y2))))
        label = f"{class_name} {float(confidence):.2f}"
        cv2.rectangle(annotated, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.putText(annotated, label, (left, max(18, top - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        person_count += 1

    print(f"person detection count: {person_count}")
    print(f"inference start (host perf_counter): {inference_start_ns*1e-9:.9f} s")
    print(f"inference end (host perf_counter): {inference_end_ns*1e-9:.9f} s")
    print(f"predict call wall time: {(inference_end_ns-inference_start_ns)/1e6:.2f} ms")
    speed = getattr(result, "speed", None)
    if isinstance(speed, dict):
        for stage in ("preprocess", "inference", "postprocess"):
            value = speed.get(stage)
            print(f"Ultralytics {stage} time: "
                  f"{float(value):.3f} ms/image" if value is not None
                  else f"Ultralytics {stage} time: unavailable")
    else:
        print("Ultralytics preprocess/inference/postprocess timing: unavailable")
    print("result ready age: N/A (static image has no source timestamp)")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), annotated):
        raise RuntimeError(f"Could not save annotated preview: {args.output}")
    print(f"annotated person-only preview saved: {args.output.resolve()}")
    print("bbox semantics: xyxy, x-right/y-down, pixels in the original image")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
