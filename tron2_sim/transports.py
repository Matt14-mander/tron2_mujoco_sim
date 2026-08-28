"""Transport layer: SdkBus wraps the limxsdk *ForSim simulator-side API.

Threading: *ForSim subscriptions are push-based (callbacks run on native
background threads). A callback must only "build a complete new buffer, then
swap the reference in one assignment" (atomic under the GIL); it must never
mutate in place a buffer the physics thread is reading.
"""
import os
import platform
import sys
import time

_ARCH_DIRS = {"x86_64": "amd64", "AMD64": "amd64", "aarch64": "aarch64", "arm64": "aarch64"}


def _prefer_repo_limxsdk():
    """Prefer an SDK source tree shipped inside the limxsdk-lowlevel submodule.

    Some SDK distributions ship the Python package as a source tree under
    python3/<arch>/; prepending it to sys.path makes `import limxsdk` resolve
    there instead of to an older wheel in site-packages. Distributions that ship
    only wheels have no such directory, so this is a no-op and the installed
    wheel is used.
    """
    arch = _ARCH_DIRS.get(platform.machine())
    if not arch:
        return
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sdk_dir = os.path.join(repo_root, "limxsdk-lowlevel", "python3", arch)
    if os.path.isdir(os.path.join(sdk_dir, "limxsdk")) and sdk_dir not in sys.path:
        sys.path.insert(0, sdk_dir)


_prefer_repo_limxsdk()

#: The *ForSim methods each SDK robot kind actually needs. Checking the RobotType
#: enum alone is not enough: SDK builds exist that carry Tron2 but not the Centaur
#: dual-body API. Failing at startup with a clear message beats an AttributeError
#: on the first publish.
_REQUIRED_FORSIM = {
    "Tron2": ("subscribeRobotCmdForSim", "publishRobotStateForSim",
              "publishImuDataForSim"),
    "Centaur": ("subscribeLowerBodyRobotCmdForSim", "publishLowerBodyRobotStateForSim",
                "publishLowerBodyImuDataForSim", "subscribeUpperBodyRobotCmdForSim",
                "publishUpperBodyRobotStateForSim", "publishUpperBodyImuDataForSim"),
}


class SdkBus:
    """The process-wide limxsdk simulator instance, Robot(RobotType.X, is_sim=True).

    Role to API binding:
      main : Tron2 -> subscribeRobotCmdForSim / publishRobotStateForSim /
             publishImuDataForSim (/motor/cmd|state, /ImuData)
             Centaur -> the LowerBody family (same topic group)
      upper: Centaur -> the UpperBody family (/did_upbody/motor/cmd|state,
             /did_upbody/ImuData)

    RobotState published from the simulator MUST carry motor_names: the Centaur
    controller validates `cmd.motor_names` against `state.motor_names` and
    silently drops commands when the names are empty.
    """

    def __init__(self, robot_kind):
        import limxsdk
        from limxsdk import datatypes
        from limxsdk.robot.Robot import Robot
        from limxsdk.robot.RobotType import RobotType
        self.datatypes = datatypes
        self.kind = robot_kind
        sdk_path = os.path.dirname(os.path.abspath(limxsdk.__file__))
        missing = [] if hasattr(RobotType, robot_kind) else [f"RobotType.{robot_kind}"]
        missing += [m for m in _REQUIRED_FORSIM.get(robot_kind, ())
                    if not hasattr(Robot, m)]
        if missing:
            raise RuntimeError(
                f"limxsdk at {sdk_path} does not support the {robot_kind} simulator "
                f"side; missing: {', '.join(missing)}. Update limxsdk-lowlevel to a "
                f"version providing it (see the support matrix in the README), or "
                f"reinstall the wheel: uv pip install --no-deps --force-reinstall "
                f"limxsdk-lowlevel/python3/<arch>/limxsdk-*.whl")
        self.robot = Robot(getattr(RobotType, robot_kind), True)
        robot_ip = os.environ.get("ROBOT_IP", "127.0.0.1")
        if not self.robot.init(robot_ip):
            raise RuntimeError(f"limxsdk {robot_kind} sim init failed (robot_ip={robot_ip})")
        # The native publisher threads may not be up yet when init() returns;
        # publishing immediately races into "init() must succeed before publishing".
        time.sleep(0.3)
        print(f"limxsdk simulator ready: {robot_kind} "
              f"(robot_ip={robot_ip}, sdk={sdk_path})")

    # ---- inbound cmd (push callbacks) ----
    def subscribe_cmd(self, role, cb):
        if role == "main":
            if self.kind == "Centaur":
                return self.robot.subscribeLowerBodyRobotCmdForSim(cb)
            return self.robot.subscribeRobotCmdForSim(cb)
        if role == "upper":
            return self.robot.subscribeUpperBodyRobotCmdForSim(cb)
        raise ValueError(f"unknown sdk_role: {role}")

    # ---- outbound state / imu ----
    def publish_state(self, role, q, dq, tau, motor_names):
        s = self.datatypes.RobotState()
        s.stamp = time.time_ns()
        s.q = list(q)
        s.dq = list(dq)
        s.tau = list(tau)
        s.motor_names = list(motor_names)
        if role == "main":
            if self.kind == "Centaur":
                return self.robot.publishLowerBodyRobotStateForSim(s)
            return self.robot.publishRobotStateForSim(s)
        if role == "upper":
            return self.robot.publishUpperBodyRobotStateForSim(s)
        raise ValueError(f"unknown sdk_role: {role}")

    def publish_imu(self, role, quat, gyro, acc):
        m = self.datatypes.ImuData()
        m.stamp = time.time_ns()
        m.quat = list(quat)
        m.gyro = list(gyro)
        m.acc = list(acc)
        if role == "main":
            if self.kind == "Centaur":
                return self.robot.publishLowerBodyImuDataForSim(m)
            return self.robot.publishImuDataForSim(m)
        if role == "upper":
            return self.robot.publishUpperBodyImuDataForSim(m)
        raise ValueError(f"unknown sdk_role: {role}")

    # ---- 2F gripper (Tron2 only; used by the DACH module) ----
    def subscribe_gripper_cmd(self, cb):
        return self.robot.subscribeGripperCmdForSim(cb)

    def publish_gripper_state(self, q, v, vd, tau):
        g = self.datatypes.GripperState()
        g.stamp = time.time_ns()
        g.q = list(q)
        g.v = list(v)
        g.vd = list(vd)
        g.tau = list(tau)
        return self.robot.publishGripperStateForSim(g)
