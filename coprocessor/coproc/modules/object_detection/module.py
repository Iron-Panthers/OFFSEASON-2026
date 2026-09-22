"""Ball detection with YOLOv8.

Runs on the coprocessor against whatever camera it is pointed at. In simulation that is the
renderer's MJPEG endpoint; on the Rubik Pi it is the real camera. The file does not know the
difference.
"""

from __future__ import annotations

import argparse
from typing import Any, Sequence

from ...runtime.module import CoprocessorModule, Detection, ModuleContext
from . import geometry

# 2026 Fuel, from FuelSim.FUEL_RADIUS. Range is derived from apparent size, so this number is
# directly a scale factor on every distance reported.
FUEL_RADIUS_METERS = 0.075

# COCO class 32 is "sports ball". Stock YOLOv8 weights were trained on soccer and tennis balls
# rather than on Fuel under arena lighting, so recall is usable rather than good; this is a
# stand-in until real weights exist, and --classes/--model replace it without a code change.
COCO_SPORTS_BALL = 32

_BOX_COLOUR = (64, 220, 64)
_EDGE_COLOUR = (48, 160, 255)  # BGR: amber, for detections whose range is not trustworthy


class ObjectDetectionModule(CoprocessorModule):
    name = "objdetect"

    def __init__(self) -> None:
        self._model: Any = None
        self._names: dict[int, str] = {}
        self._keep: set[int] = set()

    def add_arguments(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--model",
            default="yolov8n.pt",
            help="weights to load; downloaded on first use if not a local path",
        )
        parser.add_argument(
            "--conf", type=float, default=0.25, help="confidence floor for a detection"
        )
        parser.add_argument(
            "--classes",
            default=str(COCO_SPORTS_BALL),
            help="comma-separated class ids to keep, or 'all'",
        )
        parser.add_argument(
            "--radius",
            type=float,
            default=FUEL_RADIUS_METERS,
            help="ball radius in metres; scales every reported distance",
        )
        parser.add_argument("--imgsz", type=int, default=640, help="model input size")
        parser.add_argument(
            "--device", default=None, help="torch device, e.g. cpu or 0; default is automatic"
        )

    def setup(self, context: ModuleContext) -> None:
        from ultralytics import YOLO  # deferred: pulls in Torch

        args = context.args
        self._model = YOLO(args.model)
        self._names = dict(getattr(self._model, "names", {}) or {})

        if str(args.classes).strip().lower() == "all":
            self._keep = set()
        else:
            self._keep = {int(part) for part in str(args.classes).split(",") if part.strip()}
            unknown = {c for c in self._keep if self._names and c not in self._names}
            if unknown:
                raise ValueError(
                    f"model {args.model} has no class ids {sorted(unknown)}; "
                    f"it defines 0..{max(self._names)}"
                )

        kept = (
            "all classes"
            if not self._keep
            else ", ".join(f"{c} ({self._names.get(c, '?')})" for c in sorted(self._keep))
        )
        print(f"[{self.name}] {args.model}, keeping {kept}, radius {args.radius} m", flush=True)

    def process(self, image: Any, context: ModuleContext) -> Sequence[Detection]:
        args = context.args
        results = self._model.predict(
            image,
            conf=args.conf,
            imgsz=args.imgsz,
            device=args.device,
            classes=sorted(self._keep) or None,
            verbose=False,
        )

        detections: list[Detection] = []
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                class_id = int(box.cls.item())
                if self._keep and class_id not in self._keep:
                    continue
                x0, y0, x1, y1 = (float(v) for v in box.xyxy[0].tolist())
                try:
                    detections.append(
                        geometry.locate_sphere(
                            (x0, y0, x1, y1),
                            context.intrinsics,
                            args.radius,
                            self._names.get(class_id, str(class_id)),
                            float(box.conf.item()),
                        )
                    )
                except ValueError:
                    # A degenerate box is the model's problem, not a reason to drop the frame.
                    continue

        detections.sort(key=lambda d: d.distance_meters)
        return detections

    def annotate(self, image: Any, detections: Sequence[Detection]):
        import cv2

        canvas = image.copy()
        for detection in detections:
            x0, y0, x1, y1 = (int(round(v)) for v in detection.bbox)
            colour = _EDGE_COLOUR if detection.edge else _BOX_COLOUR
            cv2.rectangle(canvas, (x0, y0), (x1, y1), colour, 2)

            label = f"{detection.distance_meters:.2f}m {detection.yaw_degrees:+.1f}deg"
            if detection.edge:
                label += " clipped"
            (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            top = max(0, y0 - text_h - 6)
            cv2.rectangle(canvas, (x0, top), (x0 + text_w + 6, top + text_h + 6), colour, -1)
            cv2.putText(
                canvas,
                label,
                (x0 + 3, top + text_h + 1),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 0),
                1,
                cv2.LINE_AA,
            )
        return canvas
