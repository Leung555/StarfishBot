"""Research tools for the StarfishBot reconfigurable legged robot."""

from .control import CPG, MotorMapper
from .gait_planner import GaitPlanner
from .stability import signed_stability_margin

__all__ = ["CPG", "GaitPlanner", "MotorMapper", "signed_stability_margin"]
__version__ = "0.1.0"
