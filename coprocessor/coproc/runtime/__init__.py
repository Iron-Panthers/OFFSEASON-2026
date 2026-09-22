"""The framework half: stream in, stream out, NetworkTables, camera geometry.

Knows nothing about any particular detector. A module supplies ``process``; everything else here
is shared.
"""

from .intrinsics import Intrinsics
from .module import CoprocessorModule, Detection

__all__ = ["Intrinsics", "CoprocessorModule", "Detection"]
