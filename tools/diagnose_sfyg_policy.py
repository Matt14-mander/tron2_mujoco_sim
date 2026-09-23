#!/usr/bin/env python3
"""Run the SFYG ONNX controller directly against MuJoCo without limxsdk."""

from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort


REPO = Path(__file__).resolve().parents[1]
DEFAULT_DEPLOY_REPO = REPO.parent / "tron2_rl_deploy_python"
sys.path.insert(0, str(REPO))

from tron2_sim.channels import JointChannel
from tron2_sim.core import (
    apply_initial_base_position,
    apply_initial_joint_positions,
    apply_variant_model_overrides,
)
from tron2_sim.model_loader import load_mujoco_model
from tron2_sim.variants.sfyg import build as build_sfyg


class _Cli:
    external_wrench = None


def _session(path: Path) -> ort.InferenceSession:
    return ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])


def _input(session: ort.InferenceSession, vector: np.ndarray) -> dict[str, np.ndarray]:
    value = np.asarray(vector, dtype=np.float32)
    if len(session.get_inputs()[0].shape) == 2:
        value = value.reshape(1, -1)
    return {session.get_inputs()[0].name: value}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--deploy-repo", type=Path, default=DEFAULT_DEPLOY_REPO)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--report-period", type=float, default=0.5)
    parser.add_argument("--static-hold", action="store_true")
    parser.add_argument("--base-height", type=float)
    parser.add_argument("--zero-history", action="store_true")
    parser.add_argument("--command", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    args = parser.parse_args()

    deploy_repo = args.deploy_repo.resolve()
    contract_path = deploy_repo / "controllers" / "sfyg_contract.py"
    contract_spec = importlib.util.spec_from_file_location("sfyg_contract", contract_path)
    contract = importlib.util.module_from_spec(contract_spec)
    sys.modules[contract_spec.name] = contract
    contract_spec.loader.exec_module(contract)
    Ocs2Solution = contract.Ocs2Solution
    SFYGPolicyConfig = contract.SFYGPolicyConfig
    build_proprio_observation = contract.build_proprio_observation
    compose_joint_targets = contract.compose_joint_targets
    compose_policy_input = contract.compose_policy_input

    model_dir = deploy_repo / "controllers" / "model" / "SFYG_TRON2A"
    config = SFYGPolicyConfig(model_dir / "params.yaml")
    encoder = _session(model_dir / config.encoder_file)
    policy = _session(model_dir / config.policy_file)

    spec = build_sfyg("SFYG_TRON2A", "tron2a", _Cli())
    xml = REPO / "robot-description" / "tron2a" / "SFYG_TRON2A" / "xml" / "robot.xml"
    model = load_mujoco_model(xml)
    apply_variant_model_overrides(model, spec)
    data = mujoco.MjData(model)
    initial_base_position = spec.initial_base_position
    if args.base_height is not None:
        initial_base_position = (0.0, 0.0, args.base_height)
    apply_initial_base_position(model, data, initial_base_position)
    apply_initial_joint_positions(model, data, spec.initial_joint_positions)
    channel = JointChannel(spec.channels[0]).resolve(model)
    channel.cache_imu_adr(model)

    q = np.asarray([data.qpos[qp] for qp, _, _ in channel.map])
    solution = Ocs2Solution(
        time=0.0,
        arm_position=q[10:16].copy(),
        arm_velocity=np.zeros(6),
        arm_effort=np.zeros(6),
        base_command=np.zeros(3),
        wrench_prediction=np.zeros((5, 6)),
    )
    history = np.zeros(420, dtype=np.float32)
    last_action = np.zeros(10)
    targets = config.default_q.copy()
    history_ready = False
    policy_steps = max(1, round((1.0 / config.policy_frequency) / model.opt.timestep))
    report_steps = max(1, round(args.report_period / model.opt.timestep))
    total_steps = round(args.duration / model.opt.timestep)
    command = np.asarray(args.command, dtype=np.float32)

    print(
        f"dt={model.opt.timestep:.4f}s policy_every={policy_steps} physics steps "
        f"({1.0 / (policy_steps * model.opt.timestep):.1f}Hz)"
    )
    print("time,z,gravity_z,max_abs_action,max_abs_q_error")
    for step in range(total_steps + 1):
        q = np.asarray([data.qpos[qp] for qp, _, _ in channel.map])
        dq = np.asarray([data.qvel[qv] for _, qv, _ in channel.map])
        quat, gyro, _ = channel._read_imu(data)
        if not args.static_hold and step % policy_steps == 0:
            proprio = build_proprio_observation(
                config, q, dq, gyro, quat, last_action, command, data.time
            )
            if history_ready:
                history[:-42] = history[42:]
                history[-42:] = proprio
            else:
                if args.zero_history:
                    history[-42:] = proprio
                else:
                    history[:] = np.tile(proprio, config.history_length)
                history_ready = True
            latent = encoder.run(None, _input(encoder, history))[0].reshape(-1)
            policy_input = compose_policy_input(
                config, latent, proprio, solution.wrench_prediction, command
            )
            last_action = policy.run(None, _input(policy, policy_input))[0].reshape(-1)
            last_action = np.clip(last_action, -config.action_clip, config.action_clip)
            targets = compose_joint_targets(config, last_action, solution)

        for index, (qp, qv, actuator) in enumerate(channel.map):
            data.ctrl[actuator] = (
                config.kp[index] * (targets[index] - data.qpos[qp])
                - config.kd[index] * data.qvel[qv]
            )
        if step % report_steps == 0:
            gravity_z = build_proprio_observation(
                config, q, dq, gyro, quat, last_action, command, data.time
            )[5]
            print(
                f"{data.time:.3f},{data.qpos[2]:.4f},{gravity_z:.4f},"
                f"{np.max(np.abs(last_action)):.4f},"
                f"{np.max(np.abs(q[:10] - config.default_q[:10])):.4f}"
            )
        mujoco.mj_step(model, data)


if __name__ == "__main__":
    main()
