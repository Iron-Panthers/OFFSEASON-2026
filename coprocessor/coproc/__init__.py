"""Coprocessor code: runs off the robot, on the vision coprocessor.

Nothing in this package imports WPILib or knows it is in a simulation. The same files run on the
Rubik Pi against a real camera; only ``--source`` changes.
"""

__all__ = ["runtime", "modules"]
