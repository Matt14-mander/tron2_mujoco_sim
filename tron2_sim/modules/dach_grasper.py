"""DACH 2F linkage gripper module (SDK Gripper ForSim, `/limx/2F-gripper/*`).

The gripper is a single-DOF linkage: one revolute `grasper_{L,R}_drive_Joint`
per side (actuator `..._ctrl`, radians, ctrlrange +/-2 Nm) drives both jaws,
which follow through MJCF <equality> polynomial coupling.

Wire protocol: subscribeGripperCmdForSim receives GripperCmd.opening/speed/force
(all 0-100, index 0 = left, 1 = right) — the same API and topic the real robot's
Robot.publishGripperCmd uses. publishGripperStateForSim returns GripperState.q as
the actual opening in 0-100 (v/vd/tau are zero).

Control: 0-100 <-> drive radians mapping, rate-limited target slew, and a
torque-limited position loop emulating the gripper firmware. Calibration lives in
tron2_sim/config/gripper_config.yaml (override the path with GRIPPER_CONFIG).

In manual (paused) mode the module idles and the jaws hold their current pose.
"""
from typing import ClassVar

import mujoco

from ..config import load_gripper_config
from . import SimModule


def _polyval(coef, x):
    """MuJoCo equality polycoef: c0 + c1*x + c2*x^2 + c3*x^3 + c4*x^4."""
    y = 0.0
    for c in reversed(coef):
        y = y * x + c
    return y


class DachGrasperModule(SimModule):
    name = "dach_2f_grasper"
    slider_control = False          # drive actuators keep their torque range; idle in manual mode

    SPEC: ClassVar[dict] = {
        'left':  ('grasper_L_drive_Joint', 'grasper_L_drive_Joint_ctrl',
                  ['grasper_L_jaw_left_Joint', 'grasper_L_jaw_right_Joint']),
        'right': ('grasper_R_drive_Joint', 'grasper_R_drive_Joint_ctrl',
                  ['grasper_R_jaw_left_Joint', 'grasper_R_jaw_right_Joint']),
    }

    def __init__(self):
        self.idx = {}
        self.cmd = {}
        self._sdk = None
        self._warned = set()

    def attach(self, core):
        model = core.model
        if core.sdk_bus is None:
            print(f"WARNING: {self.name}: no SDK bus, module disabled")
            return False
        for side, (jd, an, followers) in self.SPEC.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jd)
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, an)
            if jid < 0 or aid < 0:
                # Model has no grasper linkage (e.g. plain robot.xml) -> degrade
                print("INFO: model has no grasper linkage, 2F gripper disabled")
                return False
            follower_idx = []
            for fname in followers:
                fj = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, fname)
                if fj >= 0:
                    follower_idx.append((int(model.jnt_qposadr[fj]), int(model.jnt_dofadr[fj]),
                                         self._eq_polycoef(model, fj)))
            self.idx[side] = {'qpos': int(model.jnt_qposadr[jid]),
                              'qvel': int(model.jnt_dofadr[jid]),
                              'act': int(aid), 'followers': follower_idx}

        cfg = load_gripper_config()
        self.q_low = float(cfg['joint_range']['q_low'])
        self.q_high = float(cfg['joint_range']['q_high'])
        self.q_span = self.q_high - self.q_low
        ttime = float(cfg['full_travel_time_s'])
        self.v_full = self.q_span / ttime if ttime > 1e-9 else 0.0
        self.tau_min = float(cfg['tau_min_n'])
        self.tau_full = float(cfg['tau_full_n'])
        self.kp = float(cfg['gains']['kp'])
        self.kd = float(cfg['gains']['kd'])

        for side in ('left', 'right'):
            cur = float(core.data.qpos[self.idx[side]['qpos']])
            self.cmd[side] = {'q_cmd': 0.0, 'v_cmd': 0.0, 'tau_cmd': 0.0, 'q_tgt': cur}

        self._sdk = core.sdk_bus
        self._sdk.subscribe_gripper_cmd(self._on_cmd)
        print("   2F gripper (SDK): /limx/2F-gripper/cmd|state "
              "(GripperCmd/GripperState, 0-100 domain)")
        return True

    @staticmethod
    def _eq_polycoef(model, jid):
        """<equality> polycoef mapping drive -> follower joint (identity by default)."""
        try:
            for e in range(model.neq):
                if (model.eq_type[e] == mujoco.mjtEq.mjEQ_JOINT
                        and model.eq_obj1id[e] == jid):
                    return [float(x) for x in model.eq_data[e][:5]]
        except Exception as exc:
            print(f"WARNING: could not read equality polycoef for joint {jid} "
                  f"({exc}); assuming identity coupling")
        return [0.0, 1.0, 0.0, 0.0, 0.0]

    def _warn_once(self, key, message):
        """Report a recurring failure exactly once, so hot paths cannot flood output
        yet a persistent fault is never silently swallowed."""
        if key not in self._warned:
            self._warned.add(key)
            print(f"WARNING: {self.name}: {message} (further occurrences suppressed)")

    def owned_actuators(self):
        return [d['act'] for d in self.idx.values()]

    def _on_cmd(self, cmd):
        """SDK GripperCmd push callback: opening/speed/force are 0-100, [0]=left [1]=right."""
        try:
            opening = list(getattr(cmd, 'opening', None) or [])
            speed = list(getattr(cmd, 'speed', None) or [])
            force = list(getattr(cmd, 'force', None) or [])
            for i, side in enumerate(('left', 'right')):
                if i >= len(opening):
                    break
                old = self.cmd[side]
                self.cmd[side] = {
                    'q_cmd': min(100.0, max(0.0, float(opening[i]))),
                    'v_cmd': min(100.0, max(0.0, float(speed[i]))) if i < len(speed) else 0.0,
                    'tau_cmd': min(100.0, max(0.0, float(force[i]))) if i < len(force) else 0.0,
                    'q_tgt': old['q_tgt'],
                }
        except Exception as exc:
            # Runs on a native SDK thread at command rate: an exception must not
            # escape, and warning every time would flood the console.
            self._warn_once('cmd', f"malformed GripperCmd ignored ({exc})")

    def on_control(self, core, dt):
        for side, idx in self.idx.items():
            c = self.cmd[side]
            q_goal = self.q_low + (c['q_cmd'] / 100.0) * self.q_span
            v_max = (c['v_cmd'] / 100.0) * self.v_full
            step = v_max * dt
            qt = c['q_tgt']
            if qt < q_goal:
                qt = min(qt + step, q_goal)
            elif qt > q_goal:
                qt = max(qt - step, q_goal)
            c['q_tgt'] = qt

            qpos = core.data.qpos[idx['qpos']]
            qvel = core.data.qvel[idx['qvel']]
            tau_lim = self.tau_min + (c['tau_cmd'] / 100.0) * (self.tau_full - self.tau_min)
            trq = self.kp * (qt - qpos) - self.kd * qvel
            core.data.ctrl[idx['act']] = max(-tau_lim, min(tau_lim, trq))

    def on_reset(self, core):
        """After a full reset, re-seed the slew target to the post-reset drive angle
        so a stale target does not yank the jaws."""
        for side, idx in self.idx.items():
            self.cmd[side]['q_tgt'] = float(core.data.qpos[idx['qpos']])

    def on_publish(self, core):
        try:
            pct = []
            for side in ('left', 'right'):
                qpos = float(core.data.qpos[self.idx[side]['qpos']])
                p = (qpos - self.q_low) / self.q_span * 100.0 if self.q_span > 1e-9 else 0.0
                pct.append(min(100.0, max(0.0, p)))
            self._sdk.publish_gripper_state(pct, [0.0, 0.0], [0.0, 0.0], [0.0, 0.0])
        except Exception as exc:
            # Called every physics tick; warn once rather than at 1 kHz.
            self._warn_once('publish', f"gripper state publish failed ({exc})")
