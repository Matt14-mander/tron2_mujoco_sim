"""SimCore: the single owner of MjModel / MjData / viewer.

Threading model (dual-MjData snapshot architecture):
- The physics thread owns self.data exclusively: control law, mj_step and state
  publishing, chasing the wall clock at 1 kHz. viewer.sync() blocks ~17 ms on
  average while one mj_step costs ~0.18 ms, so sharing a single MjData would let
  rendering freeze physics and the state stream. Splitting them keeps the state
  stream at a steady 1 kHz.
- The render thread owns self.render_data. `_state_lock` only guards short
  memcpys (snapshot, reset and drag-force hand-off); it is never held across
  viewer.sync() or across state publishing.
- Viewer callbacks (Space/R) only set flags, which the physics thread consumes at
  the top of a tick, so mjData always has exactly one writer.
- A Backspace/UI reset happens on render_data and is detected by
  `render_data.time != last pushed value`. Do not test `time == 0`: the physics
  side sets time to 0 when applying the reset, so a plain zero test re-triggers
  every other frame.
- Ctrl+drag perturbations go into render_data.xfrc_applied: zero before sync,
  copy out after sync, apply on every physics step. All three are required —
  skipping the zeroing leaves a residual force after releasing, and skipping the
  copy-out makes dragging do nothing.
"""
import os
import threading
import time

import mujoco
import mujoco.viewer

from .channels import JointChannel
from .model_loader import load_mujoco_model
from .transports import SdkBus


def apply_initial_joint_positions(model, data, positions):
    """Apply variant-level scalar joint positions after the model keyframe.

    Upstream assets do not consistently provide a named keyframe.  Variants
    such as SFYG nevertheless need a safe arm pose because zero is a hard limit
    for arm2/arm3.  Resolve by name and reject incompatible joint types instead
    of relying on MJCF declaration order.
    """
    for name, value in positions.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise ValueError(f"initial-position joint '{name}' not found in model")
        # MuJoCo releases expose ``jnt_type`` as a NumPy integer while the
        # Python enum implementation has changed across bindings.  Normalize
        # both sides to plain integers so a hinge cannot be rejected merely
        # because the binding's enum comparison semantics differ.
        joint_type = int(model.jnt_type[jid])
        scalar_types = {
            int(mujoco.mjtJoint.mjJNT_HINGE),
            int(mujoco.mjtJoint.mjJNT_SLIDE),
        }
        if joint_type not in scalar_types:
            raise ValueError(f"initial-position joint '{name}' is not scalar")
        lo, hi = model.jnt_range[jid]
        if model.jnt_limited[jid] and not (float(lo) <= value <= float(hi)):
            raise ValueError(
                f"initial position {value} for '{name}' is outside [{float(lo)}, {float(hi)}]"
            )
        data.qpos[int(model.jnt_qposadr[jid])] = value

    if positions:
        mujoco.mj_forward(model, data)


class SimCore:
    def __init__(self, spec, script_dir, headless=False):
        self.spec = spec
        self.headless = headless
        self.viewer = None

        # ---- model: first existing (directory candidate x file candidate) ----
        # The directory candidates absorb both asset-repo layouts:
        # <tron2a|tron2b>/<TYPE>/xml (per family) and the flat tron2/<TYPE>/xml.
        desc_root = os.path.join(script_dir, "robot-description")
        xml_dirs = [os.path.join(desc_root, spec.family_dir, spec.robot_type, "xml"),
                    os.path.join(desc_root, "tron2", spec.robot_type, "xml")]
        self.model_path = next(
            (os.path.join(d, c) for d in xml_dirs for c in spec.model_candidates
             if os.path.exists(os.path.join(d, c))), None)
        if self.model_path is None:
            print(f"Error: none of {spec.model_candidates} exists under "
                  f"{' or '.join(xml_dirs)}. Please ensure robot-description is "
                  f"checked out and the ROBOT_TYPE is set correctly.")
            raise SystemExit(1)
        chosen = os.path.basename(self.model_path)
        if chosen != spec.model_candidates[0]:
            print(f"WARNING: {spec.robot_type} has no {spec.model_candidates[0]}, "
                  f"using {chosen}")
        if not self.model_path.startswith(xml_dirs[0]):
            print(f"WARNING: {xml_dirs[0]} not found, falling back to the flat "
                  f"layout tron2/{spec.robot_type}/xml")
        print(f"*** Model File Loaded: {self.model_path} ***")
        self.model = load_mujoco_model(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep

        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, spec.keyframe)
        if key_id < 0 and self.model.nkey > 0:
            key_id = 0
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
            mujoco.mj_forward(self.model, self.data)
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_KEY, key_id)
            print(f"Initial pose from keyframe: '{name}'")
        apply_initial_joint_positions(
            self.model, self.data, spec.initial_joint_positions
        )
        if spec.initial_joint_positions:
            print(
                "Initial joint overrides: "
                + ", ".join(spec.initial_joint_positions)
            )

        # ---- channel resolution (model only) ----
        self.channels = [JointChannel(cs).resolve(self.model) for cs in spec.channels]
        for ch in self.channels:
            ch.cache_imu_adr(self.model)

        # ---- bus ----
        self.sdk_bus = SdkBus(spec.sdk_robot) if spec.sdk_robot else None
        if self.sdk_bus is not None:
            for ch in self.channels:
                ch.bind(self.sdk_bus)
        self._awaiting_first_command = self.sdk_bus is not None and bool(self.channels)
        if self._awaiting_first_command:
            print("Waiting for the first complete RobotCmd before advancing physics")

        # ---- modules: a failed attach means the capability is unavailable ----
        self.modules = []
        for mod in spec.modules:
            try:
                ok = mod.attach(self)
            except Exception as e:
                print(f"WARNING: module {mod.name} failed to attach ({e}), disabled")
                ok = False
            if ok:
                self.modules.append(mod)

        # ---- data.ctrl ownership audit: double claims abort, unowned ones warn ----
        owners = {}
        for ch in self.channels:
            for act in ch.actuators():
                if act in owners:
                    raise ValueError(f"actuator {act} claimed by both {owners[act]} and {ch.spec.name}")
                owners[act] = f"channel:{ch.spec.name}"
        for mod in self.modules:
            for act in mod.owned_actuators():
                if act in owners:
                    raise ValueError(f"actuator {act} claimed by both {owners[act]} and module:{mod.name}")
                owners[act] = f"module:{mod.name}"
        unowned = [a for a in range(self.model.nu) if a not in owners]
        if unowned:
            names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in unowned]
            print(f"INFO: unowned actuators (ctrl stays 0): {names}")
        self._owners = owners

        # ---- manual-mode slider set: channel actuators + slider_control modules ----
        self.slider_acts = sorted(
            {act for ch in self.channels for act in ch.actuators()}
            | {act for mod in self.modules if mod.slider_control for act in mod.owned_actuators()})
        self._act_joint = {}          # act -> (jid, qpos_adr, qvel_adr) for slider kinematics
        for act in self.slider_acts:
            jid = int(self.model.actuator_trnid[act, 0])
            self._act_joint[act] = (jid, int(self.model.jnt_qposadr[jid]),
                                    int(self.model.jnt_dofadr[jid]))
        self.original_ctrlrange = self.model.actuator_ctrlrange.copy()

        # ---- run-state flags (viewer callbacks set, physics thread consumes) ----
        self.paused = False
        self.manual_mode = False
        self._mode_toggle_request = False
        self._base_reset_request = False
        self._running = True

        # ---- optional viewer: its own render_data + snapshot/reset/drag hand-off ----
        if not headless:
            self.render_data = mujoco.MjData(self.model)
            self.render_data.qpos[:] = self.data.qpos
            self.render_data.qvel[:] = self.data.qvel
            self.render_data.time = float(self.data.time)
            mujoco.mj_forward(self.model, self.render_data)
            self.viewer = mujoco.viewer.launch_passive(
                self.model, self.render_data, key_callback=self._key_callback,
                show_left_ui=True, show_right_ui=True)
            self.viewer.cam.distance, self.viewer.cam.elevation = spec.cam
            self._state_lock = threading.Lock()
            self._snap_qpos = self.data.qpos.copy()
            self._snap_qvel = self.data.qvel.copy()
            self._snap_time = float(self.data.time)
            self._pending_reset = None
            self._last_synced_time = float(self.data.time)
            self._viewer_xfrc = self.data.xfrc_applied.copy() * 0.0

        self._print_summary()

    def _print_summary(self):
        print(f"{self.spec.robot_type}: {len(self.channels)} channel(s), "
              f"{len(self.modules)} module(s), nu={self.model.nu}, headless={self.headless}")
        for ch in self.channels:
            print(f"   ├ {ch.describe()}")
        for mod in self.modules:
            print(f"   ├ module:{mod.name}")

    # ---------------- viewer callbacks (viewer thread: set flags only) ----------------
    def _key_callback(self, keycode):
        if keycode == 32:                      # Space: paused/manual <-> running/auto
            self._mode_toggle_request = True
        elif keycode == 82:                    # R: reset the floating base pose only
            self._base_reset_request = True

    # ---------------- mode toggle (consumed by the physics thread) ----------------
    def _apply_mode_toggle(self):
        entering_manual = not self.paused
        self.paused = self.manual_mode = entering_manual
        if entering_manual:
            print("PAUSED - manual control (viewer sliders)")
            # Slider actuators get their ctrlrange swapped to joint limits so the
            # slider IS the target angle; non-slider actuators (e.g. the DACH
            # grasper drive) keep their torque range.
            for act in self.slider_acts:
                jid, _, _ = self._act_joint[act]
                self.model.actuator_ctrlrange[act] = self.model.jnt_range[jid]
            if self.viewer is not None:
                with self._state_lock:
                    for act in self.slider_acts:
                        _, qpos_adr, _ = self._act_joint[act]
                        self.render_data.ctrl[act] = self.data.qpos[qpos_adr]
        else:
            print("RESUMED - auto control")
            self.model.actuator_ctrlrange[:] = self.original_ctrlrange
            self.data.ctrl[:] = 0.0
            if self.viewer is not None:
                with self._state_lock:
                    self.render_data.ctrl[:] = 0.0

    def _reset_base_pose(self):
        """R key: reset the floating-base freejoint to qpos0 and zero base velocity,
        keeping joint angles."""
        free_type = int(mujoco.mjtJoint.mjJNT_FREE)
        free = [j for j in range(self.model.njnt)
                if int(self.model.jnt_type[j]) == free_type]
        if not free:
            print("WARNING: no floating base, ignoring base reset")
            return
        qadr = self.model.jnt_qposadr[free[0]]
        vadr = self.model.jnt_dofadr[free[0]]
        self.data.qpos[qadr:qadr + 7] = self.model.qpos0[qadr:qadr + 7]
        self.data.qvel[vadr:vadr + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)
        print("Base pose reset (joint angles kept)")

    # ---------------- single step (physics thread) ----------------
    def _tick(self):
        # 1. mode toggle request
        if self._mode_toggle_request:
            self._mode_toggle_request = False
            self._apply_mode_toggle()
        # 2. adopt a full reset from the viewer (Backspace / Reset button)
        if self.viewer is not None:
            with self._state_lock:
                pending = self._pending_reset
                self._pending_reset = None
            if pending is not None:
                qpos, qvel = pending
                self.data.qpos[:] = qpos
                self.data.qvel[:] = qvel
                self.data.ctrl[:] = 0.0
                self.data.time = 0.0
                mujoco.mj_forward(self.model, self.data)
                for mod in self.modules:
                    mod.on_reset(self)
                self._resync_clock = True
                print("Reset (viewer) - physics state reset")
        # 3. R-key base reset (key repeat collapses to at most once per tick)
        if self._base_reset_request:
            self._base_reset_request = False
            self._reset_base_pose()
        # 4. control
        if self.manual_mode:
            for act in self.slider_acts:
                jid, qpos_adr, qvel_adr = self._act_joint[act]
                target = float(self.render_data.ctrl[act]) if self.viewer is not None else 0.0
                lo, hi = self.model.jnt_range[jid]
                target = max(float(lo), min(target, float(hi)))
                self.data.qpos[qpos_adr] = target
                self.data.qvel[qvel_adr] = 0.0
            for mod in self.modules:
                mod.on_manual(self)
            mujoco.mj_forward(self.model, self.data)
        elif not self.paused:
            if self._awaiting_first_command and all(
                ch.has_command for ch in self.channels
            ):
                self._awaiting_first_command = False
                self._resync_clock = True
                print("First complete RobotCmd received; physics released")
            if not self._awaiting_first_command:
                for ch in self.channels:
                    ch.compute_ctrl(self.data)
                if self.viewer is not None:
                    with self._state_lock:
                        self.data.xfrc_applied[:] = self._viewer_xfrc
                else:
                    self.data.xfrc_applied[:] = 0.0
                for mod in self.modules:
                    mod.on_control(self, self.dt)
                mujoco.mj_step(self.model, self.data)
        # 5. read state (always; publishing is gated separately)
        for ch in self.channels:
            ch.read_state(self.data, self.manual_mode)
        # 6. publish
        if (not self.paused) or self.spec.publish_when_paused:
            if self.sdk_bus is not None:
                for ch in self.channels:
                    ch.publish(self.data)
            for mod in self.modules:
                mod.on_publish(self)
        # 7. snapshot hand-off (taken between steps, so the viewer never sees a
        #    half-integrated state)
        if self.viewer is not None:
            with self._state_lock:
                self._snap_qpos[:] = self.data.qpos
                self._snap_qvel[:] = self.data.qvel
                self._snap_time = float(self.data.time)

    # ---------------- main loop ----------------
    def run(self, duration=None):
        if self.headless:
            print(f"Headless physics loop started (dt={self.dt:.4f}s"
                  + (f", duration={duration}s" if duration else "") + ")")
            try:
                self._physics_loop(duration)
            except KeyboardInterrupt:
                print("\nInterrupted")
            self._running = False
            print("Simulator stopped")
            return

        # Re-arm SIGTERM -> KeyboardInterrupt AFTER the viewer exists: GLFW/viewer
        # initialization overrides the process signal handlers. Without this, an
        # external kill/timeout leaves the daemon physics thread publishing after
        # the native singleton is destroyed, spamming
        # "init() must succeed before publishing".
        import signal

        def _sigterm(signum, frame):
            raise KeyboardInterrupt

        signal.signal(signal.SIGTERM, _sigterm)

        physics = threading.Thread(target=self._physics_loop, daemon=True)
        physics.start()
        render_period = 1.0 / 60.0
        print(f"Loop started (physics {1.0 / self.dt:.0f}Hz on its own thread, "
              f"render {1.0 / render_period:.0f}Hz)")
        print("SPACE=pause (manual sliders)/resume, R=reset base pose only, "
              "Backspace=full reset, ESC=quit")
        deadline = time.time() + duration if duration else None
        try:
            while self.viewer.is_running() and (deadline is None or time.time() < deadline):
                self._sync_viewer()
                time.sleep(render_period)
        except KeyboardInterrupt:
            print("\nInterrupted")
        self._running = False
        physics.join(timeout=1.0)
        print("Simulator stopped")

    def _physics_loop(self, duration=None):
        """Fixed-dt wall-clock chase; drop the backlog past max_catchup to avoid a
        death spiral."""
        dt = self.dt
        max_catchup = 0.05
        sim_clock = time.time()
        deadline = time.time() + duration if duration else None
        self._resync_clock = False
        while self._running and (deadline is None or time.time() < deadline):
            now = time.time()
            if self.paused or self.manual_mode:
                self._tick()
                sim_clock = now
                time.sleep(0.002)
                continue
            if now - sim_clock > max_catchup:
                sim_clock = now - max_catchup
            while sim_clock < now and self._running:
                self._tick()
                if self._resync_clock:            # re-base the clock after a viewer reset
                    self._resync_clock = False
                    sim_clock = time.time()
                    break
                sim_clock += dt
            sleep_for = sim_clock - time.time()
            if sleep_for > 0:
                time.sleep(sleep_for)

    def _sync_viewer(self):
        """Render thread: draw the snapshot, detect UI resets, hand off drag forces,
        run module render hooks.

        sync() blocking ~17 ms is fine here: this thread neither steps physics nor
        publishes state. Reset detection compares time rather than testing == 0,
        because the physics side zeroes time when applying the reset.
        """
        with self._state_lock:
            if self.render_data.time != self._last_synced_time:
                self._pending_reset = (self.render_data.qpos.copy(),
                                       self.render_data.qvel.copy())
            self.render_data.qpos[:] = self._snap_qpos
            self.render_data.qvel[:] = self._snap_qvel
            self.render_data.time = self._snap_time
            self._last_synced_time = self._snap_time

        self.render_data.xfrc_applied[:] = 0.0      # release stops the force; sync only rewrites while dragging
        user_scn = getattr(self.viewer, "user_scn", None)
        if user_scn is not None:
            user_scn.ngeom = 0                      # overlay geometry is rebuilt each frame by on_render
        for mod in self.modules:
            mod.on_render(self, self.render_data)
        mujoco.mj_forward(self.model, self.render_data)
        self.viewer.sync()                           # the blocking call, outside the lock

        with self._state_lock:                       # GUI drag force -> physics thread
            self._viewer_xfrc[:] = self.render_data.xfrc_applied
