"""SF (sole-foot biped, 10 joints): one SDK main channel. Shared by TRON2A/B."""
from ..spec import IMU_STD, LEG_SF, ChannelSpec, RobotSpec


def build(robot_type, family_dir, cli):
    return RobotSpec(
        robot_type=robot_type, base="SF", family_dir=family_dir,
        model_candidates=["robot.xml"],
        channels=[ChannelSpec("main", LEG_SF, sdk_role="main", imu=IMU_STD)],
        sdk_robot="Tron2")
