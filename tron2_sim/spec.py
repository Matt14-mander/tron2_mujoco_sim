"""Variant declarations: wire-order contracts, channel/robot specs, ROBOT_TYPE parsing.

Pure data — imports neither mujoco nor limxsdk, so it is usable headless and
without a bus.

Wire order: the order of a channel's `joints` list IS the index order of that
channel's cmd/state messages. TRON2A and TRON2B share joint names and order, so
one wire order is shared per base name; the family directory (tron2a/tron2b) is
derived from the ROBOT_TYPE suffix. Actuator declaration order in the XML may
differ from wire order (e.g. DACH's head) — JointChannel resolves by joint name
and never assumes position.
"""
from dataclasses import dataclass, field

FAMILY_DIRS = {"TRON2A": "tron2a", "TRON2B": "tron2b"}

#: IMU sensor names used by the mainline models and by DASF TRON2B.
IMU_STD = [("base_imu_quat", "base_imu_gyro", "base_imu_acc")]

# ---------------- wire-order tables ----------------
LEG_SF = [
    "proximal_pitch_L_Joint", "proximal_roll_L_Joint", "proximal_yaw_L_Joint", "knee_L_Joint", "ankle_pitch_L_Joint",
    "proximal_pitch_R_Joint", "proximal_roll_R_Joint", "proximal_yaw_R_Joint", "knee_R_Joint", "ankle_pitch_R_Joint",
]
LEG_WF = [
    "proximal_pitch_L_Joint", "proximal_roll_L_Joint", "proximal_yaw_L_Joint", "knee_L_Joint", "wheel_L_Joint",
    "proximal_pitch_R_Joint", "proximal_roll_R_Joint", "proximal_yaw_R_Joint", "knee_R_Joint", "wheel_R_Joint",
]
ARM_DA = [
    "proximal_pitch_L_Joint", "proximal_roll_L_Joint", "proximal_yaw_L_Joint", "elbow_L_Joint",
    "wrist_yaw_L_Joint", "wrist_pitch_L_Joint", "wrist_roll_L_Joint",
    "proximal_pitch_R_Joint", "proximal_roll_R_Joint", "proximal_yaw_R_Joint", "elbow_R_Joint",
    "wrist_yaw_R_Joint", "wrist_pitch_R_Joint", "wrist_roll_R_Joint",
]
HEAD_PY = ["head_pitch_Joint", "head_yaw_Joint"]          # DACH wire order: pitch -> yaw
HEAD_YP = ["head_yaw_Joint", "head_pitch_Joint"]          # DASF wire order: yaw -> pitch

# DASF humanoid composite: arms carry a `_U` infix.
ARM_DA_U = [n.replace("_L_Joint", "_L_U_Joint").replace("_R_Joint", "_R_U_Joint") for n in ARM_DA]

# SFYG single-arm whole-body composite.  Keep the wire order grouped exactly as
# the deploy controller consumes it: locomotion legs, arm MPC, then gripper.
ARM_YG = [f"arm{i}_Joint" for i in range(1, 7)]
GRIPPER_YG = ["gripper1_Joint", "gripper2_Joint"]
SFYG = LEG_SF + ARM_YG + GRIPPER_YG


# ---------------- spec dataclasses ----------------
@dataclass
class ChannelSpec:
    """One joint-group channel: wire order + SDK role + MIT control-law params.

    imu: priority-ordered list of (quat, gyro, acc) sensor-name triples; resolve()
    takes the first triple whose three sensors all exist, which absorbs the
    TRON2A/TRON2B sensor naming differences. If none match, IMU publishing is
    skipped with a warning. None means this channel publishes no IMU. Topics are
    bound internally by the SDK according to sdk_role.
    """
    name: str                               # "main" | "lower" | "upper"
    joints: list[str]                       # wire order (cmd/state index order)
    sdk_role: str = "main"                  # "main" (Tron2 unqualified / Centaur LowerBody) | "upper"
    imu: list[tuple[str, str, str]] | None = None
    kp_override: list[float] | None = None   # per-joint gain override (>=0 overrides, -1 passes through)
    kd_override: list[float] | None = None
    tau_source: str = "ctrl"                # "ctrl" | "actuator_force"


@dataclass
class RobotSpec:
    robot_type: str
    base: str
    family_dir: str                         # "tron2a" | "tron2b"
    model_candidates: list[str]             # xml/ filenames by priority; first existing wins
    channels: list[ChannelSpec]
    modules: list = field(default_factory=list)     # SimModule instances (dropped if attach returns False)
    sdk_robot: str | None = "Tron2"      # "Tron2" | "Centaur" | None (no SDK instance)
    keyframe: str = "default_pose"          # applied as the initial pose when the model defines it
    initial_base_position: tuple[float, float, float] | None = None
    initial_joint_positions: dict[str, float] = field(default_factory=dict)
    publish_when_paused: bool = False
    cam: tuple[float, float] = (10.0, -20.0)  # (distance, elevation)


def resolve_robot_type(robot_type, registry):
    """Parse ROBOT_TYPE into (base, family, family_dir); exit listing valid values.

    Valid types are the cartesian product of the registry's base names and
    FAMILY_DIRS suffixes.
    """
    base, _, family = (robot_type or "").rpartition("_")
    if not robot_type or base not in registry or family not in FAMILY_DIRS:
        valid = ", ".join(f"{b}_{f}" for b in registry for f in FAMILY_DIRS)
        if not robot_type:
            print("Error: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.")
        else:
            print(f"*** unsupported robot type: {robot_type} ***")
        print(f"    supported: {valid}")
        raise SystemExit(1)
    return base, family, FAMILY_DIRS[family]
