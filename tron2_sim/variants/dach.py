"""DACH (dual arm 14 + head 2, wire order pitch -> yaw; optional 2F linkage gripper).

Model choice: robot_grasper.xml by default when present; --no-grasper or
DACH_GRASPER=0 selects robot.xml. If a family ships no grasper file the candidate
list falls back to robot.xml and DachGrasperModule disables itself.

Head ordering hazard: on DACH_TRON2A(grasper) the head actuators come after the
grasper ones, and both families declare yaw before pitch. All wire-order to index
mapping is done by joint name in JointChannel.
"""
import os

from ..modules.dach_grasper import DachGrasperModule
from ..spec import ARM_DA, HEAD_PY, IMU_STD, ChannelSpec, RobotSpec


def build(robot_type, family_dir, cli):
    use_grasper = getattr(cli, 'use_grasper', None)
    if use_grasper is None:
        use_grasper = os.getenv("DACH_GRASPER", "1").lower() not in ("0", "false", "no", "off")
    candidates = ["robot_grasper.xml", "robot.xml"] if use_grasper else ["robot.xml"]
    return RobotSpec(
        robot_type=robot_type, base="DACH", family_dir=family_dir,
        model_candidates=candidates,
        channels=[ChannelSpec("main", ARM_DA + HEAD_PY, sdk_role="main", imu=IMU_STD)],
        modules=[DachGrasperModule()],
        sdk_robot="Tron2")
