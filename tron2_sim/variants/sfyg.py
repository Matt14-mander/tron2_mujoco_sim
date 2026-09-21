"""SFYG (sole-foot biped + 6-DoF arm + parallel gripper, 18 joints).

The controller-facing wire order is deliberately one named ``Tron2`` channel:
ten locomotion joints, six independently controlled arm joints, and two gripper
joints.  The locomotion policy still owns only the first ten entries; the
deployment process combines its output with OCS2 arm targets and the gripper
targets before publishing one complete named ``RobotCmd``.

Only TRON2A currently has an upstream SFYG MuJoCo asset.
"""

from ..modules.external_wrench import ExternalWrenchModule
from ..spec import IMU_STD, SFYG, ChannelSpec, RobotSpec

INITIAL_ARM_AND_GRIPPER = {
    "arm1_Joint": 0.0,
    "arm2_Joint": 1.5707963267948966,
    "arm3_Joint": -1.4835298641951802,
    "arm4_Joint": 0.0,
    "arm5_Joint": 0.0,
    "arm6_Joint": 0.0,
    "gripper1_Joint": 0.05,
    "gripper2_Joint": -0.05,
}


def build(robot_type, family_dir, cli):
    if family_dir != "tron2a":
        raise SystemExit(
            f"{robot_type} is not available: the pinned robot-description "
            "currently provides SFYG_TRON2A only."
        )
    modules = []
    wrench = getattr(cli, "external_wrench", None)
    if wrench is not None:
        modules.append(
            ExternalWrenchModule(
                wrench,
                body=cli.wrench_body,
                frame=cli.wrench_frame,
                profile=cli.wrench_profile,
                start=cli.wrench_start,
                duration=cli.wrench_duration,
                ramp_time=cli.wrench_ramp_time,
                frequency=cli.wrench_frequency,
            )
        )
    return RobotSpec(
        robot_type=robot_type,
        base="SFYG",
        family_dir=family_dir,
        model_candidates=["robot.xml"],
        channels=[
            ChannelSpec(
                "main",
                SFYG,
                sdk_role="main",
                imu=IMU_STD,
                tau_source="actuator_force",
            )
        ],
        modules=modules,
        initial_joint_positions=INITIAL_ARM_AND_GRIPPER,
        sdk_robot="Tron2",
        cam=(3.0, -15.0),
    )
