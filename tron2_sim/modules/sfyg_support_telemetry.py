"""SFYG simulator ground truth for guarded WALK/STAND experiments."""

import json
import socket

import mujoco
import numpy as np

from . import SimModule


class SfygSupportTelemetry(SimModule):
    name = "sfyg_support_telemetry"

    def __init__(self, port):
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.next_send = 0.0

    def attach(self, core):
        model = core.model
        self.ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        self.left = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ankle_pitch_L_Link")
        self.right = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ankle_pitch_R_Link")
        self.base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_Link")
        if min(self.ground, self.left, self.right, self.base) < 0:
            raise ValueError("SFYG ground, base or foot body is missing")
        self.free_joint = next(
            (j for j in range(model.njnt) if int(model.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE)
             and int(model.jnt_bodyid[j]) == self.base), None)
        if self.free_joint is None:
            raise ValueError("SFYG base free joint is missing")
        self.base_dof = int(model.jnt_dofadr[self.free_joint])
        print(f"SFYG support telemetry: 127.0.0.1:{self.port} (sim-only)")
        return True

    def _foot(self, model, geom_id):
        body = int(model.geom_bodyid[geom_id])
        while body > 0:
            if body == self.left:
                return "left"
            if body == self.right:
                return "right"
            body = int(model.body_parentid[body])
        return None

    def on_publish(self, core):
        data = core.data
        if data.time < self.next_send:
            return
        self.next_send = float(data.time) + 0.02
        forces = {"left": 0.0, "right": 0.0}
        for index in range(data.ncon):
            contact = data.contact[index]
            if contact.geom1 == self.ground:
                foot = self._foot(core.model, contact.geom2)
            elif contact.geom2 == self.ground:
                foot = self._foot(core.model, contact.geom1)
            else:
                continue
            if foot:
                force = np.zeros(6)
                mujoco.mj_contactForce(core.model, data, index, force)
                forces[foot] += max(0.0, float(force[0]))
        payload = json.dumps({
            "base_speed_xy": float(np.linalg.norm(data.qvel[self.base_dof:self.base_dof + 2])),
            "left_normal_force": forces["left"],
            "right_normal_force": forces["right"],
        }).encode("ascii")
        self.socket.sendto(payload, ("127.0.0.1", self.port))

    def on_reset(self, core):
        self.next_send = 0.0
