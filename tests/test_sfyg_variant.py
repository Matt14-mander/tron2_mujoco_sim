"""Contract tests for the SFYG whole-body MuJoCo variant."""

import unittest
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np

from tron2_sim.channels import JointChannel
from tron2_sim.core import apply_initial_joint_positions, apply_variant_model_overrides
from tron2_sim.model_loader import load_mujoco_model
from tron2_sim.modules.external_wrench import ExternalWrenchModule
from tron2_sim.spec import ARM_YG, GRIPPER_YG, LEG_SF, SFYG
from tron2_sim.variants import BASE_REGISTRY

REPO = Path(__file__).resolve().parents[1]
MODEL = REPO / "robot-description" / "tron2a" / "SFYG_TRON2A" / "xml" / "robot.xml"


class SfygVariantTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not MODEL.exists():
            raise unittest.SkipTest("robot-description submodule is not initialized")
        cls.model = load_mujoco_model(MODEL)

    def setUp(self):
        self.spec = BASE_REGISTRY["SFYG"](
            "SFYG_TRON2A", "tron2a", SimpleNamespace()
        )

    def test_wire_order_is_leg_arm_gripper(self):
        self.assertEqual(SFYG, LEG_SF + ARM_YG + GRIPPER_YG)
        self.assertEqual(self.spec.channels[0].joints, SFYG)
        self.assertEqual(len(SFYG), 18)
        self.assertEqual(len(set(SFYG)), 18)

    def test_state_stream_stays_alive_in_manual_pause(self):
        self.assertTrue(self.spec.publish_when_paused)

    def test_every_wire_joint_resolves_to_a_unique_actuator(self):
        channel = JointChannel(self.spec.channels[0]).resolve(self.model)
        actuators = channel.actuators()
        self.assertEqual(len(actuators), 18)
        self.assertEqual(len(set(actuators)), 18)
        self.assertIsNotNone(channel.imu_ids)

    def test_safe_arm_pose_is_valid_and_applied_by_name(self):
        data = mujoco.MjData(self.model)
        apply_initial_joint_positions(
            self.model, data, self.spec.initial_joint_positions
        )
        for name, expected in self.spec.initial_joint_positions.items():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            actual = data.qpos[self.model.jnt_qposadr[jid]]
            self.assertAlmostEqual(float(actual), expected)

    def test_training_dynamics_overrides_are_applied(self):
        model = load_mujoco_model(MODEL)
        apply_variant_model_overrides(model, self.spec)
        self.assertAlmostEqual(model.opt.timestep, 0.005)
        for name, expected in self.spec.joint_armature.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.assertAlmostEqual(
                float(model.dof_armature[model.jnt_dofadr[jid]]), expected
            )
        for name, expected in self.spec.joint_effort_limit.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            aid = next(
                a for a in range(model.nu)
                if int(model.actuator_trnid[a, 0]) == jid
            )
            np.testing.assert_allclose(
                model.actuator_ctrlrange[aid], (-expected, expected)
            )

    def test_tron2b_fails_with_an_explicit_asset_error(self):
        with self.assertRaisesRegex(SystemExit, "SFYG_TRON2A only"):
            BASE_REGISTRY["SFYG"](
                "SFYG_TRON2B", "tron2b", SimpleNamespace()
            )

    def test_base_wrench_contract_matches_command_at_base_origin(self):
        data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, data)
        core = SimpleNamespace(model=self.model, data=data)
        command = [10.0, -5.0, 2.0, 1.0, -2.0, 3.0]
        module = ExternalWrenchModule(command, body="base_Link", frame="body")
        self.assertTrue(module.attach(core))
        module.on_control(core, self.model.opt.timestep)
        np.testing.assert_allclose(
            module.current_wrench_base_at_base_origin, command, atol=1e-12
        )


if __name__ == "__main__":
    unittest.main()
