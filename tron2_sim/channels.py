"""JointChannel: wire order + name-based index map + MIT control law + state/IMU publishing.

Index resolution is a two-step lookup:
  1) the `<joint>_ctrl` actuator naming convention;
  2) otherwise scan actuator_trnid for the actuator driving that joint.
Declaration order of actuators/joints in the XML is NEVER used — that is the only
correct way to handle models whose head actuators are declared out of wire order.
"""
import mujoco


class JointChannel:
    def __init__(self, spec):
        self.spec = spec
        self.n = len(spec.joints)
        self._wire_idx = {name: i for i, name in enumerate(spec.joints)}
        # Command buffer: callbacks swap the whole reference (atomic under the GIL);
        # the physics thread only reads the current reference.
        self.cmd = {'q': [0.0] * self.n, 'dq': [0.0] * self.n, 'tau': [0.0] * self.n,
                    'kp': [0.0] * self.n, 'kd': [0.0] * self.n}
        self.map = []                 # [(qpos_adr, qvel_adr, act_id)] in wire order
        self.imu_ids = None           # (quat_id, gyro_id, acc_id) or None
        self._state = ([0.0] * self.n, [0.0] * self.n, [0.0] * self.n)  # (q, dq, tau)
        self._sdk = None

    # ---------------- model resolution (pure mujoco, usable offline) ----------------
    def resolve(self, model):
        self.map = []
        for name in self.spec.joints:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"[{self.spec.name}] joint '{name}' not found in model")
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name + "_ctrl")
            if aid < 0:
                for a in range(model.nu):
                    if model.actuator_trnid[a, 0] == jid:
                        aid = a
                        break
            if aid < 0:
                raise ValueError(
                    f"[{self.spec.name}] no actuator drives joint '{name}' "
                    f"(tried actuator name '{name}_ctrl' and transmission scan)")
            self.map.append((int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid]), int(aid)))
        if self.spec.imu is not None:
            self.imu_ids = None
            for quat_n, gyro_n, acc_n in self.spec.imu:
                ids = tuple(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, s)
                            for s in (quat_n, gyro_n, acc_n))
                if all(i >= 0 for i in ids):
                    self.imu_ids = ids
                    break
            if self.imu_ids is None:
                tried = " | ".join(",".join(c) for c in self.spec.imu)
                print(f"WARNING: [{self.spec.name}] IMU sensors not found "
                      f"(tried: {tried}) - IMU publishing disabled")
        return self

    def actuators(self):
        return [act for _, _, act in self.map]

    # ---------------- bus binding ----------------
    def bind(self, sdk_bus):
        self._sdk = sdk_bus
        sdk_bus.subscribe_cmd(self.spec.sdk_role, self._on_sdk_cmd)

    def describe(self):
        role = self.spec.sdk_role
        topics = {"main": "/motor/cmd|state (+/ImuData)",
                  "upper": "/did_upbody/motor/cmd|state (+/did_upbody/ImuData)"}[role]
        return f"{self.spec.name}({self.n}) sdk:{role} {topics}"

    # ---------------- inbound cmd ----------------
    def _on_sdk_cmd(self, cmd):
        # limxsdk datatypes.RobotCmd: q/dq/tau/Kp/Kd (+motor_names)
        self._ingest(cmd.q, cmd.dq, cmd.tau, cmd.Kp, cmd.Kd,
                     list(getattr(cmd, 'motor_names', None) or []))

    def _ingest(self, q, dq, tau, kp, kd, names):
        """Fill a fresh zeroed buffer by name (when names hit at least one wire name)
        or by position, then swap the reference in one assignment."""
        if q is None:
            return
        new = {'q': [0.0] * self.n, 'dq': [0.0] * self.n, 'tau': [0.0] * self.n,
               'kp': [0.0] * self.n, 'kd': [0.0] * self.n}
        srcs = {'q': q, 'dq': dq, 'tau': tau, 'kp': kp, 'kd': kd}
        use_names = bool(names) and any(nm in self._wire_idx for nm in names)
        if use_names:
            for j, nm in enumerate(names):
                i = self._wire_idx.get(nm)
                if i is None:
                    continue
                for key, src in srcs.items():
                    if src is not None and j < len(src):
                        new[key][i] = float(src[j])
        else:
            m = min(self.n, len(q))
            for key, src in srcs.items():
                if src is None:
                    continue
                mm = min(m, len(src))
                for i in range(mm):
                    new[key][i] = float(src[i])
        self.cmd = new  # single assignment swaps the reference

    # ---------------- control law / state ----------------
    def compute_ctrl(self, data):
        """MIT law: ctrl = kp(q_cmd - q) + kd(dq_cmd - dq) + tau, with per-joint gain
        override (>=0 overrides, -1 passes the commanded gain through)."""
        c = self.cmd
        kpo, kdo = self.spec.kp_override, self.spec.kd_override
        for i, (qp, qv, act) in enumerate(self.map):
            kp = kpo[i] if (kpo is not None and i < len(kpo) and kpo[i] >= 0.0) else c['kp'][i]
            kd = kdo[i] if (kdo is not None and i < len(kdo) and kdo[i] >= 0.0) else c['kd'][i]
            data.ctrl[act] = (kp * (c['q'][i] - data.qpos[qp])
                              + kd * (c['dq'][i] - data.qvel[qv]) + c['tau'][i])

    def read_state(self, data, manual):
        q = [float(data.qpos[qp]) for qp, _, _ in self.map]
        dq = [float(data.qvel[qv]) for _, qv, _ in self.map]
        if manual:
            tau = [0.0] * self.n                       # kinematic mode applies no torque
        elif self.spec.tau_source == "actuator_force":
            tau = [float(data.actuator_force[act]) for _, _, act in self.map]
        else:
            tau = [float(data.ctrl[act]) for _, _, act in self.map]
        self._state = (q, dq, tau)
        return self._state

    def publish(self, data):
        q, dq, tau = self._state
        self._sdk.publish_state(self.spec.sdk_role, q, dq, tau, self.spec.joints)
        if self.imu_ids is not None:
            self._sdk.publish_imu(self.spec.sdk_role, *self._read_imu(data))

    def _read_imu(self, data):
        """Return (quat[4], gyro[3], acc[3]); requires cache_imu_adr to have run."""
        (a_q, a_g, a_a) = self._imu_adr
        sd = data.sensordata
        return (list(sd[a_q:a_q + 4]), list(sd[a_g:a_g + 3]), list(sd[a_a:a_a + 3]))

    def cache_imu_adr(self, model):
        """Call after resolve(): turn sensor ids into sensordata start offsets."""
        if self.imu_ids is not None:
            self._imu_adr = tuple(int(model.sensor_adr[i]) for i in self.imu_ids)
