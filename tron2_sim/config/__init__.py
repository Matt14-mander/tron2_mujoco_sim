"""YAML config loading. The shipped defaults live alongside this module.

Currently one file: gripper_config.yaml (DACH 2F linkage gripper calibration).
Set GRIPPER_CONFIG to load a different file instead.
"""
import os

import yaml

_CONFIG_DIR = os.path.dirname(os.path.abspath(__file__))


def load_gripper_config():
    """DACH 2F linkage gripper: 0-100 <-> drive-joint radians, rate/torque limits,
    and the internal position-loop gains."""
    cfg = {
        'topics': {'cmd': '/limx/2F-gripper/cmd', 'state': '/limx/2F-gripper/state'},
        'joint_range': {'q_low': -0.09522, 'q_high': 0.77302},
        'full_travel_time_s': 1.0,
        'tau_min_n': 0.3,
        'tau_full_n': 2.0,
        'gains': {'kp': 2.0, 'kd': 0.05},
    }
    path = os.environ.get('GRIPPER_CONFIG', os.path.join(_CONFIG_DIR, 'gripper_config.yaml'))
    try:
        with open(path) as f:
            loaded = (yaml.safe_load(f) or {}).get('gripper') or {}
        for key, val in loaded.items():
            if isinstance(val, dict) and isinstance(cfg.get(key), dict):
                cfg[key] = {**cfg[key], **val}
            else:
                cfg[key] = val
        print(f"Gripper config loaded: {path}")
    except FileNotFoundError:
        print(f"WARNING: {path} not found, using built-in gripper calibration")
    except Exception as e:
        print(f"WARNING: failed to read {path} ({e}), using built-in gripper calibration")
    return cfg
