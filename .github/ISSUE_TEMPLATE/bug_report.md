---
name: Bug report
about: MuJoCo simulator / SDK bridge / joystick / doc defect
title: "[bug] <short summary>"
labels: bug
assignees: ''
---

## Affected component

<!-- e.g. simulator.py, tron2_sim/variants/*.py, README.md, .github/workflows/ci.yml -->

- File(s):
- Robot type (`$ROBOT_TYPE`): `SF_TRON2A` / `WF_TRON2A` / `DA_TRON2A` / `DACH_TRON2A` / `DASF_TRON2A` / other
- Commit / tag:
- Submodule pins (`git submodule status`):

## Environment

- OS + arch (e.g. Ubuntu 22.04 x86_64 / aarch64):
- Python (`uv run python -V`):
- MuJoCo (`uv run python -c "import mujoco; print(mujoco.__version__)"`):
- limxsdk resolved from (`uv run python -c "import limxsdk, os; print(os.path.dirname(limxsdk.__file__))"`):
- Are you running against a real robot?  **No — sim only** / Yes

  > If Yes, review `SECURITY.md`; do not attach PoCs that could
  > move a physical robot.

## Expected behavior

## Actual behavior

<!-- log lines, screenshots, numeric mismatches -->

## Minimal reproduction

```bash
# commands that reproduce it locally, ideally starting from a fresh clone
git clone --recurse-submodules https://github.com/limx-tron2/tron2-mujoco-sim.git
cd tron2-mujoco-sim
export ROBOT_TYPE=SF_TRON2A
uv run simulator.py --headless --duration 5
```

## Additional context

<!-- upstream MuJoCo quirks, controller version, joystick model, etc. -->

## Checklist

- [ ] I have searched existing issues.
- [ ] I have included the exact commit / tag and submodule status
      (`git submodule status` shows no leading `-`).
- [ ] The problem reproduces with `--headless`, ruling out the viewer.
- [ ] No other simulator or controller instance is running on the same host.
- [ ] I am **not** reporting a security issue (those go to
      `contact@limxdynamics.com` per `SECURITY.md`).
- [ ] I did not attach any control policies, calibration values,
      rosbags, or media that discloses individuals or sites.
