"""Evaluate an SFYG 48D BASE ONNX pair directly in MuJoCo, without SDK.

Defaults use the local BASE asset's zero arm target and native 800/40 lock.
Check the checkpoint's actual training configuration before interpreting a fail.
"""

import argparse
import csv
import importlib.util
import math
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from tron2_sim.channels import JointChannel
from tron2_sim.core import (
    apply_initial_base_position, apply_initial_joint_positions, apply_variant_model_overrides,
)
from tron2_sim.model_loader import load_mujoco_model
from tron2_sim.variants.sfyg import build as build_sfyg


def compose_base_input(latent, proprio, command):
    result = np.concatenate((latent, proprio, command)).astype(np.float32)
    if result.shape != (48,) or not np.isfinite(result).all():
        raise ValueError("BASE actor requires latent(3)+proprio(42)+command(3)")
    return result


def speed_cycle(t, peak, durations):
    """Zero, smooth acceleration, cruise, smooth braking, zero; seconds."""
    names = ('initial_stop', 'accelerate', 'cruise', 'decelerate', 'final_stop')
    elapsed = 0.
    for i, duration in enumerate(durations):
        if t < elapsed + duration:
            u = np.clip((t - elapsed) / duration, 0., 1.)
            smooth = u * u * (3. - 2. * u)
            value = (0., peak * smooth, peak, peak * (1. - smooth), 0.)[i]
            return names[i], float(value)
        elapsed += duration
    return names[-1], 0.


def session(path, nin, nout):
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    net = ort.InferenceSession(str(path), sess_options=options, providers=['CPUExecutionProvider'])
    inputs, outputs = net.get_inputs(), net.get_outputs()
    if (len(inputs) != 1 or len(outputs) != 1
            or inputs[0].shape not in ([nin], [1, nin])
            or outputs[0].shape not in ([nout], [1, nout])):
        raise ValueError(f"Unexpected model contract: {path}")
    return net


def infer(net, vector):
    if len(net.get_inputs()[0].shape) == 2:
        vector = vector[None]
    result = net.run(None, {net.get_inputs()[0].name: vector.astype(np.float32)})[0].reshape(-1)
    if not np.isfinite(result).all():
        raise ValueError("Nonfinite model output")
    return result


def native_lock(model, motor, kp, kd, effort):
    model.actuator_gainprm[motor] = 0
    model.actuator_gainprm[motor, 0] = kp
    model.actuator_biastype[motor] = mujoco.mjtBias.mjBIAS_AFFINE
    model.actuator_biasprm[motor] = 0
    model.actuator_biasprm[motor, 1:3] = [-kp, -kd]
    model.actuator_ctrllimited[motor] = False
    model.actuator_forcelimited[motor] = True
    model.actuator_forcerange[motor] = [-effort, effort]
    joint = int(model.actuator_trnid[motor, 0])
    model.jnt_actfrclimited[joint] = True
    model.jnt_actfrcrange[joint] = [-effort, effort]
    model.dof_damping[model.jnt_dofadr[joint]] = 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deploy-repo', type=Path, default=REPO.parent / 'tron2_rl_deploy_python')
    parser.add_argument('--model-dir', type=Path)
    parser.add_argument('--config', type=Path, help='18-joint SFYG mapping/scales YAML; verify against training')
    parser.add_argument('--command', type=float, nargs=3, default=(0., 0., 0.))
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--speed-cycle', action='store_true', help='0 -> peak -> 0; total duration is sum of --cycle-times')
    parser.add_argument('--peak-speed', type=float, default=.5, help='signed body-x command in m/s')
    parser.add_argument('--cycle-times', type=float, nargs=5, default=(3., 4., 5., 4., 5.),
                        metavar=('INITIAL', 'ACCEL', 'CRUISE', 'DECEL', 'STOP'))
    parser.add_argument('--log-csv', type=Path, help='new CSV path; records command and measured velocity at policy rate')
    parser.add_argument('--arm-position', type=float, nargs=6, default=(0.,)*6)
    parser.add_argument('--arm-kp', type=float, default=800.)
    parser.add_argument('--arm-kd', type=float, default=40.)
    parser.add_argument('--arm-effort', type=float, default=200.)
    parser.add_argument('--base-height', type=float, default=.85)
    parser.add_argument('--viewer', action='store_true')
    args = parser.parse_args()
    values = [*args.command, args.duration, *args.arm_position, args.peak_speed, *args.cycle_times,
              args.arm_kp, args.arm_kd, args.arm_effort, args.base_height]
    if (not np.isfinite(values).all() or min(args.duration, args.arm_kp,
            args.arm_effort, args.base_height) <= 0 or args.arm_kd < 0
            or np.any(np.abs(args.command) > [1., .5, 1.5])):
        parser.error('Invalid finite runtime settings')
    if args.speed_cycle:
        if any(args.command) or abs(args.peak_speed) > 1 or min(args.cycle_times) <= 0:
            parser.error('Speed cycle requires zero --command, positive stage times and |peak-speed| <= 1')
        args.duration = sum(args.cycle_times)
    deploy = args.deploy_repo.resolve()
    contract_spec = importlib.util.spec_from_file_location('base_test_contract', deploy / 'controllers/sfyg_contract.py')
    contract = importlib.util.module_from_spec(contract_spec)
    sys.modules[contract_spec.name] = contract
    contract_spec.loader.exec_module(contract)
    config = contract.SFYGPolicyConfig(args.config or deploy / 'controllers/model/SFYG_TRON2A/params.yaml')
    directory = args.model_dir or deploy / 'controllers/model/SFYG_BASE_TRON2A'
    encoder = session(directory / 'encoder.onnx', 420, 3)
    policy = session(directory / 'policy.onnx', 48, 10)
    spec = build_sfyg('SFYG_TRON2A', 'tron2a', argparse.Namespace())
    model = load_mujoco_model(REPO / 'robot-description/tron2a/SFYG_TRON2A/xml/robot.xml')
    apply_variant_model_overrides(model, spec)
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    data = mujoco.MjData(model)
    positions = dict(spec.initial_joint_positions)
    positions.update({f'arm{i+1}_Joint': q for i, q in enumerate(args.arm_position)})
    apply_initial_base_position(model, data, (0., 0., args.base_height))
    apply_initial_joint_positions(model, data, positions)
    channel = JointChannel(spec.channels[0]).resolve(model)
    channel.cache_imu_adr(model)
    targets = config.default_q.copy()
    targets[10:16] = args.arm_position
    for index in range(10, 18):
        native_lock(model, channel.map[index][2], args.arm_kp, args.arm_kd, args.arm_effort)
    history = np.zeros((10, 42), dtype=np.float32)
    action = np.zeros(10)
    command = np.array(args.command, dtype=np.float32)
    step_dt = 1 / config.policy_frequency
    policy_steps = round(step_dt / model.opt.timestep)
    if not np.isclose(policy_steps * model.opt.timestep, step_dt):
        raise ValueError('Physics timestep must divide the policy period')
    print(f'Model: {directory.resolve()} | encoder 420->3, BASE policy 48->10', flush=True)
    print(f'No SDK/payload/estimator/future wrench; command={command.tolist()}', flush=True)
    print(f'Arm target={args.arm_position}, native PD={args.arm_kp}/{args.arm_kd}, effort={args.arm_effort}', flush=True)
    print(f'Config scales/gait: {config.path}; verify checkpoint training settings', flush=True)
    if args.speed_cycle:
        print(f'Speed cycle: times={args.cycle_times}, peak={args.peak_speed}m/s, total={args.duration}s', flush=True)
    print('time,phase,cmd_vx,z,tilt_deg,vx_body,vy_body,max_abs_action', flush=True)
    viewer = None
    start = time.monotonic()
    max_tilt = 0.
    velocity_sum = np.zeros(3)
    velocity_count = 0
    previous_phase = None
    stop_speeds = []
    log_stream = None
    writer = None
    try:
        if args.log_csv:
            args.log_csv.parent.mkdir(parents=True, exist_ok=True)
            log_stream = args.log_csv.open('x', newline='', encoding='utf-8')
            writer = csv.writer(log_stream)
            writer.writerow(['time', 'phase', 'cmd_vx', 'cmd_vy', 'cmd_wz',
                             'z', 'tilt_deg', 'vx_body', 'vy_body', 'vz_body', 'wz_body', 'max_abs_action'])
        if args.viewer:
            from mujoco import viewer as viewer_api
            viewer = viewer_api.launch_passive(model, data)
        start = time.monotonic()
        for step in range(round(args.duration / model.opt.timestep) + 1):
            if viewer is not None and not viewer.is_running():
                raise RuntimeError('Viewer closed before test completed')
            q = np.array([data.qpos[qp] for qp, _, _ in channel.map])
            dq = np.array([data.qvel[qv] for _, qv, _ in channel.map])
            quat, gyro, _ = channel._read_imu(data)
            phase = 'constant'
            if args.speed_cycle:
                phase, vx = speed_cycle(data.time, args.peak_speed, args.cycle_times)
                command[:] = (vx, 0., 0.)
            if phase != previous_phase:
                print(f'PHASE {phase}: t={data.time:.3f}s', flush=True)
                previous_phase = phase
            if step % policy_steps == 0:
                obs = contract.build_proprio_observation(config, q, dq, gyro, quat, action, command, data.time)
                if step == 0:
                    history[:] = obs
                else:
                    history[:-1] = history[1:]
                    history[-1] = obs
                latent = infer(encoder, history.reshape(-1))
                action = np.clip(infer(policy, compose_base_input(latent, obs, command)), -config.action_clip, config.action_clip)
                targets[:10] = config.default_q[:10] + config.action_scale * action
            for index, (qp, qv, motor) in enumerate(channel.map):
                data.ctrl[motor] = (config.kp[index] * (targets[index] - data.qpos[qp])
                                    - config.kd[index] * data.qvel[qv]) if index < 10 else targets[index]
            gravity = contract.projected_gravity_from_wxyz(quat)
            tilt = math.degrees(math.acos(float(np.clip(-gravity[2], -1, 1))))
            max_tilt = max(max_tilt, tilt)
            velocity = np.zeros(6)
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY,
                                    model.body('base_Link').id, velocity, 1)
            if args.speed_cycle and data.time >= args.duration - min(2., args.cycle_times[-1]):
                stop_speeds.append(float(np.linalg.norm(velocity[3:5])))
            if writer is not None and step % policy_steps == 0:
                writer.writerow([data.time, phase, *command, data.qpos[2], tilt,
                                 *velocity[3:], velocity[2], np.max(np.abs(action))])
            if data.time >= min(2., args.duration / 2):
                velocity_sum += velocity[3:]
                velocity_count += 1
            if step % max(1, round(.5 / model.opt.timestep)) == 0:
                print(f'{data.time:.3f},{phase},{command[0]:.4f},{data.qpos[2]:.4f},{tilt:.3f},'
                      f'{velocity[3]:.4f},{velocity[4]:.4f},{np.max(np.abs(action)):.4f}', flush=True)
            if not np.isfinite(data.qpos).all() or tilt > 35 or data.qpos[2] < .45:
                raise RuntimeError(f'BASE test rejected at {data.time:.3f}s: tilt={tilt:.2f}, z={data.qpos[2]:.3f}')
            if step == round(args.duration / model.opt.timestep):
                break
            mujoco.mj_step(model, data)
            if viewer is not None and step % 20 == 0:
                viewer.sync()
                time.sleep(max(0., start + data.time - time.monotonic()))
        print(f'COMPLETED: max_tilt={max_tilt:.3f}deg, mean_body_velocity='
              f'{(velocity_sum / max(1, velocity_count)).tolist()}', flush=True)
        if stop_speeds:
            print(f'FINAL_STOP last {min(2., args.cycle_times[-1])}s: '
                  f'mean_planar_speed={np.mean(stop_speeds):.4f}m/s, '
                  f'p95_planar_speed={np.percentile(stop_speeds, 95):.4f}m/s', flush=True)
    finally:
        if log_stream is not None:
            log_stream.close()
        if viewer is not None:
            viewer.close()


if __name__ == '__main__':
    main()
