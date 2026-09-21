#!/usr/bin/env python3
"""TRON2 MuJoCo simulator entry point: ROBOT_TYPE -> variant spec -> SimCore.

The robot is selected by the ROBOT_TYPE environment variable
(`<BASE>_<TRON2A|TRON2B>`). Variant declarations live in tron2_sim/variants/;
all communication goes through the limxsdk *ForSim simulator-side API.
"""
import argparse
import os
import sys

# Make the repository importable no matter where this script is invoked from,
# which is why the tron2_sim imports below sit after this line.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)

from tron2_sim import spec as tspec  # noqa: E402
from tron2_sim.variants import BASE_REGISTRY  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(
        description="TRON2 MuJoCo simulator (robot selected by $ROBOT_TYPE)")
    grp = parser.add_mutually_exclusive_group()
    grp.add_argument("--grasper", dest="use_grasper", action="store_true", default=None,
                     help="DACH_*: load robot_grasper.xml with the 2F gripper (default)")
    grp.add_argument("--no-grasper", dest="use_grasper", action="store_false",
                     help="DACH_*: load robot.xml without the gripper (arms + head only)")
    parser.add_argument("--headless", action="store_true",
                        help="run without a viewer; physics and I/O pipelines are unchanged")
    parser.add_argument("--no-sdk", action="store_true",
                        help="disable SDK I/O for offline model/physics validation only")
    parser.add_argument("--duration", type=float, default=None,
                        help="exit automatically after N seconds (default: run until Ctrl+C)")
    parser.add_argument(
        "--external-wrench", type=float, nargs=6, default=None,
        metavar=("FX", "FY", "FZ", "MX", "MY", "MZ"),
        help="SFYG_* only: validation wrench at the selected body origin (N, N*m)")
    parser.add_argument("--wrench-body", default="base_Link",
                        help="body receiving --external-wrench (default: base_Link)")
    parser.add_argument("--wrench-frame", choices=("body", "world"), default="body",
                        help="frame of --external-wrench (default: body)")
    parser.add_argument("--wrench-profile", choices=("step", "ramp", "sine"), default="step")
    parser.add_argument("--wrench-start", type=float, default=0.0)
    parser.add_argument("--wrench-duration", type=float, default=None)
    parser.add_argument("--wrench-ramp-time", type=float, default=0.25)
    parser.add_argument("--wrench-frequency", type=float, default=1.0)
    cli, _ = parser.parse_known_args()
    return cli


def _sigterm_to_interrupt(signum, frame):
    """Turn SIGTERM into KeyboardInterrupt so kill/timeout takes the Ctrl+C path.

    Without this, the daemon physics thread keeps publishing after the native
    singleton is destroyed, spamming "init() must succeed before publishing".
    """
    raise KeyboardInterrupt


def main():
    import signal
    signal.signal(signal.SIGTERM, _sigterm_to_interrupt)
    cli = parse_args()
    robot_type = os.getenv("ROBOT_TYPE")
    base, _family, family_dir = tspec.resolve_robot_type(robot_type, BASE_REGISTRY)
    print(f"Robot type: {robot_type}")

    from tron2_sim.core import SimCore
    robot_spec = BASE_REGISTRY[base](robot_type, family_dir, cli)
    if cli.no_sdk:
        robot_spec.sdk_robot = None
        print("WARNING: SDK I/O disabled (--no-sdk); offline validation only")
    SimCore(robot_spec, _SCRIPT_DIR, headless=cli.headless).run(duration=cli.duration)
    # limxsdk/fastdds participants race during interpreter teardown (occasional
    # SIGSEGV after all work is done). Every thread has stopped by now, so exit hard
    # -- but os._exit skips buffered-stream flushing, which would drop all output
    # when stdout is a pipe or a file.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
