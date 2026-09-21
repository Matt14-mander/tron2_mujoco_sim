"""Optional deterministic wrench excitation for sim2sim validation."""

from __future__ import annotations

import math

import mujoco
import numpy as np

from . import SimModule


class ExternalWrenchModule(SimModule):
    """Apply a commanded wrench at a body-frame origin, never as policy input.

    ``MjData.xfrc_applied`` expects a world-frame wrench at the body's center of
    mass.  The public contract here is instead ``[Fx,Fy,Fz,Mx,My,Mz]`` at the
    named body origin, optionally expressed in that body's frame.  The moment
    shift below preserves that contract exactly.
    """

    name = "external_wrench"

    def __init__(
        self,
        wrench,
        body="base_Link",
        frame="body",
        profile="step",
        start=0.0,
        duration=None,
        ramp_time=0.25,
        frequency=1.0,
    ):
        self.wrench = np.asarray(wrench, dtype=float)
        if self.wrench.shape != (6,) or not np.isfinite(self.wrench).all():
            raise ValueError("external wrench must contain six finite values")
        if frame not in ("body", "world"):
            raise ValueError("external wrench frame must be 'body' or 'world'")
        if profile not in ("step", "ramp", "sine"):
            raise ValueError("external wrench profile must be step, ramp, or sine")
        if duration is not None and duration < 0.0:
            raise ValueError("external wrench duration must be non-negative")
        if ramp_time <= 0.0 or frequency <= 0.0:
            raise ValueError("ramp time and frequency must be positive")
        self.body_name = body
        self.frame = frame
        self.profile = profile
        self.start = start
        self.duration = duration
        self.ramp_time = ramp_time
        self.frequency = frequency
        self.body_id = -1
        self.base_id = -1
        self.current_wrench_base_at_base_origin = np.zeros(6)

    def attach(self, core):
        self.body_id = mujoco.mj_name2id(
            core.model, mujoco.mjtObj.mjOBJ_BODY, self.body_name
        )
        self.base_id = mujoco.mj_name2id(
            core.model, mujoco.mjtObj.mjOBJ_BODY, "base_Link"
        )
        if self.body_id < 0:
            raise ValueError(f"wrench body '{self.body_name}' not found")
        if self.base_id < 0:
            raise ValueError("base_Link not found; cannot form ground-truth contract")
        print(
            f"External wrench enabled: body={self.body_name}, frame={self.frame}, "
            f"profile={self.profile}, wrench={self.wrench.tolist()}"
        )
        return True

    def _scale(self, time):
        elapsed = time - self.start
        if elapsed < 0.0 or (
            self.duration is not None and elapsed >= self.duration
        ):
            return 0.0
        if self.profile == "ramp":
            return min(elapsed / self.ramp_time, 1.0)
        if self.profile == "sine":
            return math.sin(2.0 * math.pi * self.frequency * elapsed)
        return 1.0

    def on_control(self, core, dt):
        scale = self._scale(float(core.data.time))
        command = self.wrench * scale
        target_rotation = core.data.xmat[self.body_id].reshape(3, 3)
        if self.frame == "body":
            force_world = target_rotation @ command[:3]
            moment_target_world = target_rotation @ command[3:]
        else:
            force_world = command[:3]
            moment_target_world = command[3:]

        target_origin = core.data.xpos[self.body_id]
        target_com = core.data.xipos[self.body_id]
        moment_com_world = moment_target_world - np.cross(
            target_com - target_origin, force_world
        )
        core.data.xfrc_applied[self.body_id, :3] += force_world
        core.data.xfrc_applied[self.body_id, 3:] += moment_com_world

        base_origin = core.data.xpos[self.base_id]
        base_rotation = core.data.xmat[self.base_id].reshape(3, 3)
        moment_base_world = moment_target_world + np.cross(
            target_origin - base_origin, force_world
        )
        self.current_wrench_base_at_base_origin[:3] = (
            base_rotation.T @ force_world
        )
        self.current_wrench_base_at_base_origin[3:] = (
            base_rotation.T @ moment_base_world
        )

    def on_manual(self, core):
        self.current_wrench_base_at_base_origin.fill(0.0)

    def on_reset(self, core):
        self.current_wrench_base_at_base_origin.fill(0.0)
