"""DA (dual arm, 14 joints): one SDK main channel.

TRON2A has a floating base, TRON2B a fixed one; the core adapts automatically.
"""
from ..spec import ARM_DA, IMU_STD, ChannelSpec, RobotSpec


def build(robot_type, family_dir, cli):
    return RobotSpec(
        robot_type=robot_type, base="DA", family_dir=family_dir,
        model_candidates=["robot.xml"],
        channels=[ChannelSpec("main", ARM_DA, sdk_role="main", imu=IMU_STD)],
        sdk_robot="Tron2")
