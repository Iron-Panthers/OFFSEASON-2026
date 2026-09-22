"""Entry point: ``python -m coproc <module> [options]``.

Every module shares the stream, NetworkTables and camera options declared here and adds its own
on top. The Java supervisor in simulation builds exactly this command line.
"""

from __future__ import annotations

import argparse
import sys
from typing import Callable

from .modules.object_detection import ObjectDetectionModule
from .runtime.module import CoprocessorModule

# The registry. A new module is one entry plus its directory.
MODULES: dict[str, Callable[[], CoprocessorModule]] = {
    "objdetect": ObjectDetectionModule,
}

# Default diagonal field of view, matching VisionConstants.SIM_CAMERA_FOV_DIAGONAL_DEGREES so the
# detector and the simulated camera agree without either hardcoding a focal length.
DEFAULT_FOV_DEGREES = 70.0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m coproc",
        description="Run a coprocessor module against a camera stream.",
    )
    parser.add_argument("module", choices=sorted(MODULES), help="which module to run")
    parser.add_argument(
        "--source",
        required=True,
        help="MJPEG stream URL, e.g. http://localhost:1193/stream.mjpg",
    )
    parser.add_argument("--camera", type=int, default=0, help="camera index, used in the NT path")
    parser.add_argument(
        "--nt",
        default=None,
        help="NetworkTables server host; omit to disable publishing",
    )
    parser.add_argument("--nt-port", type=int, default=5810, help="NetworkTables port")
    parser.add_argument("--width", type=int, default=640, help="expected stream width")
    parser.add_argument("--height", type=int, default=400, help="expected stream height")
    parser.add_argument(
        "--fov",
        type=float,
        default=DEFAULT_FOV_DEGREES,
        help="camera diagonal field of view in degrees",
    )
    parser.add_argument(
        "--output-port",
        type=int,
        default=0,
        help="port to serve the annotated stream on; 0 disables it",
    )
    parser.add_argument("--jpeg-quality", type=int, default=80, help="annotated stream quality")
    return parser


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)

    # Two-stage parse: the module has to be known before its own options can be declared.
    # The pre-parser must not handle -h itself, or --help would print before the module has
    # contributed its options and would silently document only half the interface.
    pre = argparse.ArgumentParser(add_help=False)
    # No choices here either: rejecting an unknown name at this stage reports it against the
    # pre-parser, whose usage line describes an interface that does not exist.
    pre.add_argument("module", nargs="?")
    chosen = pre.parse_known_args(argv)[0].module

    parser = build_parser()
    module = MODULES[chosen]() if chosen in MODULES else None
    if module is not None:
        module.add_arguments(parser)
    # Parses for real, so a missing or misspelt module still fails with the usual message.
    args = parser.parse_args(argv)

    from .runtime.runner import run

    return run(module, args)


if __name__ == "__main__":
    sys.exit(main())
