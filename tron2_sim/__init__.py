"""tron2_sim: composable core of the TRON2 MuJoCo simulator.

A variant is declared as joint channels (JointChannel) plus optional modules
(SimModule), mirroring the hardware's modularity (legs x torso x head x
gripper). All communication goes through the limxsdk *ForSim API.
"""
from .spec import FAMILY_DIRS


def valid_robot_types(registry):
    return [f"{b}_{f}" for b in registry for f in FAMILY_DIRS]
