"""DASF (26-DOF humanoid composite: SF legs + DACH arms/head, rigidly joined).

Two Centaur SDK channels:
- lower body, legs (wire 0-9): LowerBody API <-> /motor/cmd|state + /ImuData
- upper body, arms + head (wire 10-25): UpperBody API <->
  /did_upbody/motor/cmd|state + /did_upbody/ImuData

Wire order (same contract on TRON2A/B): 10 leg joints -> 14 arm joints (`_U`
infix) -> head yaw then pitch. Note this is the opposite head order from DACH.
DASF_TRON2B declares its head actuators pitch-then-yaw, which name-based
resolution absorbs.

IMU sensor names differ per family and are absorbed by the candidate lists:
TRON2A uses quat/gyro/acc and quat_U/gyro_U/acc_U; TRON2B uses base_imu_* and
upper_base_imu_*.

Note: the upper body is mounted rotated 180 degrees, so the `*_L_U` arm is
physically on the robot's right. The names are kept as-is.
"""
from ..spec import ARM_DA_U, HEAD_YP, IMU_STD, LEG_SF, ChannelSpec, RobotSpec

WIRE = LEG_SF + ARM_DA_U + HEAD_YP          # 26 joints

IMU_LOWER = [("quat", "gyro", "acc"), IMU_STD[0]]
IMU_UPPER = [("quat_U", "gyro_U", "acc_U"),
             ("upper_base_imu_quat", "upper_base_imu_gyro", "upper_base_imu_acc")]


def build(robot_type, family_dir, cli):
    return RobotSpec(
        robot_type=robot_type, base="DASF", family_dir=family_dir,
        model_candidates=["default_pose.xml", "robot.xml"],
        channels=[
            ChannelSpec("lower", WIRE[:10], sdk_role="main", imu=IMU_LOWER,
                        tau_source="actuator_force"),
            ChannelSpec("upper", WIRE[10:], sdk_role="upper", imu=IMU_UPPER,
                        tau_source="actuator_force"),
        ],
        sdk_robot="Centaur",
        publish_when_paused=True,             # keep streaming state while paused
        cam=(3.0, -15.0))
