"""Variant registry: base name -> build(robot_type, family_dir, cli) -> RobotSpec.

Variant-specific behaviour may only live in the files of this package; the core,
channels and modules stay variant-agnostic.
"""
from . import da, dach, dasf, sf, wf

BASE_REGISTRY = {
    "SF": sf.build,
    "WF": wf.build,
    "DA": da.build,
    "DACH": dach.build,
    "DASF": dasf.build,
}
