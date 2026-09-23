"""Ball detection with YOLO.

Runs on the coprocessor against whatever camera it is pointed at. In simulation that is the
renderer's MJPEG endpoint for the object-detection camera; on the Rubik Pi it is the real camera.
The file does not know the difference.
"""

from __future__ import annotations

import argparse
import math
from typing import Any, Sequence

from ...runtime.module import CoprocessorModule, Detection, ModuleContext
from . import geometry

# 2026 Fuel, from FuelSim.FUEL_RADIUS. It sets the plane the ball's centre sits on, and scales
# every size-based range, so it is wrong in two places at once if it is wrong at all.
FUEL_RADIUS_METERS = 0.075

# Weights trained on Fuel: a single-class YOLO26n, 300 epochs. Measured against rendered frames
# it finds around fifty balls in a frame of the centre pile. Stock COCO weights, which this
# replaced as the default, found zero in the same frames -- class 32 ("sports ball") was trained
# on soccer and tennis balls and does not generalise to Fuel under arena lighting at all.
DEFAULT_MODEL = "rebuilt-fuel-v1.pt"

_BOX_COLOUR = (64, 220, 64)
_EDGE_COLOUR = (48, 160, 255)  # BGR: amber, for a clipped box, whose size understates the ball
_SUSPECT_COLOUR = (64, 64, 255)  # BGR: red, for the two range estimates disagreeing


class ObjectDetectionModule(CoprocessorModule):
    name = "objdetect"

    def __init__(self) -> None:
        self._model: Any = None
        self._names: dict[int, str] = {}
        self._keep: set[int] = set()
        self._mounting: Any = None

    def add_arguments(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--model",
            default=DEFAULT_MODEL,
            help="weights to load; downloaded on first use if not a local path",
        )
        parser.add_argument(
            "--conf", type=float, default=0.25, help="confidence floor for a detection"
        )
        parser.add_argument(
            "--classes",
            default="auto",
            help=(
                "comma-separated class ids to keep, 'all', or 'auto' to keep every class a "
                "single-class model defines"
            ),
        )
        parser.add_argument(
            "--radius",
            type=float,
            default=FUEL_RADIUS_METERS,
            help="ball radius in metres; the plane its centre sits on, and a scale on size-based range",
        )
        parser.add_argument("--imgsz", type=int, default=640, help="model input size")
        parser.add_argument(
            "--device", default=None, help="torch device, e.g. cpu or 0; default is automatic"
        )
        parser.add_argument(
            "--camera-height",
            type=float,
            default=None,
            help=(
                "camera height above the floor in metres. Supplying it, with --camera-pitch, "
                "switches ranging to ground-plane projection and enables the cross-check"
            ),
        )
        parser.add_argument(
            "--camera-pitch",
            type=float,
            default=None,
            help="camera tilt in degrees, positive nose down, matching WPILib Rotation3d",
        )
        parser.add_argument(
            "--camera-roll",
            type=float,
            default=0.0,
            help="camera roll about the optical axis in degrees; zero for a square mounting",
        )
        parser.add_argument(
            "--disagreement",
            type=float,
            default=geometry.DISAGREEMENT_TOLERANCE,
            help="fractional gap between the two range estimates that flags a detection suspect",
        )

    def setup(self, context: ModuleContext) -> None:
        from ultralytics import YOLO  # deferred: pulls in Torch

        args = context.args
        self._model = YOLO(args.model)
        self._names = dict(getattr(self._model, "names", {}) or {})

        selection = str(args.classes).strip().lower()
        if selection in ("all", "auto"):
            # 'auto' and 'all' agree for the single-class weights this ships with. They differ
            # for a multi-class model, where 'auto' has no basis for choosing and says so rather
            # than guessing a class id that happens to be a ball in COCO and a robot in ours.
            if selection == "auto" and len(self._names) > 1:
                raise ValueError(
                    f"model {args.model} defines {len(self._names)} classes "
                    f"({', '.join(f'{i}={n}' for i, n in sorted(self._names.items()))}); "
                    "--classes auto cannot choose between them, so name the ids to keep"
                )
            self._keep = set()
        else:
            self._keep = {int(part) for part in selection.split(",") if part.strip()}
            unknown = {c for c in self._keep if self._names and c not in self._names}
            if unknown:
                raise ValueError(
                    f"model {args.model} has no class ids {sorted(unknown)}; "
                    f"it defines 0..{max(self._names)}"
                )

        if args.camera_height is not None and args.camera_pitch is not None:
            self._mounting = geometry.Mounting(
                height_meters=args.camera_height,
                pitch_radians=math.radians(args.camera_pitch),
                roll_radians=math.radians(args.camera_roll),
            )

        kept = (
            "all classes"
            if not self._keep
            else ", ".join(f"{c} ({self._names.get(c, '?')})" for c in sorted(self._keep))
        )
        ranging = (
            f"ground plane from {args.camera_height:.3f} m, {args.camera_pitch:+.1f} deg"
            if self._mounting is not None
            else "apparent size only (no --camera-height/--camera-pitch)"
        )
        print(
            f"[{self.name}] {args.model}, keeping {kept}, radius {args.radius} m, "
            f"ranging by {ranging}",
            flush=True,
        )

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
                            self._mounting,
                            args.disagreement,
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
            # Suspect wins over clipped: a box the two estimates disagree about is the more
            # interesting failure, because nothing else in the pipeline would have caught it.
            if detection.suspect:
                colour = _SUSPECT_COLOUR
            elif detection.edge:
                colour = _EDGE_COLOUR
            else:
                colour = _BOX_COLOUR
            cv2.rectangle(canvas, (x0, y0), (x1, y1), colour, 2)

            label = f"{detection.distance_meters:.2f}m {detection.yaw_degrees:+.1f}deg"
            if detection.suspect:
                label += f" vs {detection.size_distance_meters:.2f}m"
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
