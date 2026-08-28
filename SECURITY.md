# Security Policy

## Scope

`tron2-mujoco-sim` is a Python MuJoCo-based simulator for the TRON2
robot family (SF, WF, DA, DACH, and DASF variants). Its entry point,
`simulator.py`, together with the `tron2_sim/` variant registry,
bridges the LimX low-level SDK simulator-side API to a MuJoCo
simulation: it accepts `RobotCmd` messages (`q`, `dq`, `tau`, `Kp`,
`Kd`) over the SDK's `*ForSim` transport and applies the resulting
joint torques inside `mujoco.MjData`, publishing `RobotState`,
`ImuData`, and (for gripper-equipped variants) gripper state back on
the same bus.

### Sim-vs-real boundary — read before deploying

The **wire format** and **command semantics** used by this simulator
are identical to those used to drive the real TRON2 robot family.
That means:

- Any process that can reach the SDK's simulator-side endpoint
  (loopback `127.0.0.1`; the `*ForSim` API does not expose a
  configurable bind address) can push `q` / `dq` / `tau` / `Kp` / `Kd`
  targets. On a real robot, the equivalent SDK API and those same
  targets move physical actuators.
- A controller that misbehaves against the simulator will misbehave
  against the physical robot the moment it is pointed at the
  hardware-side SDK endpoint instead. That switch is the entire
  safety boundary between sim and real.
- Bugs in `simulator.py` / `tron2_sim/` that cause it to accept
  malformed or out-of-range commands mask the same bugs in a
  downstream controller and can lead to unsafe real-robot behavior.
- The simulator applies the commanded PD/torque law as given and does
  not clamp torque beyond the actuator `ctrlrange` declared in the
  model. Treat it as a faithful mirror of the hardware interface, not
  as a safety net.

For those reasons this repository intentionally does **not** ship:
control policies, calibration values, or a mode that silently forwards
to a real robot. See `THIRD_PARTY_NOTICES.md` for the exclusion list.

Control-path vulnerabilities in the deployment stack itself belong to
`tron2-rl-deploy-python` / `tron2-rl-deploy-ros`, not here.

## Private-IP handling

`<robot-ip>` in this repository's Markdown / YAML command examples is
a **placeholder token**, not a real address. Nothing in this
repository (including `simulator.py`, `tron2_sim/`, CI, or
configuration) hard-codes a private IP, and CI fails the build if one
appears.

The internal-only sibling repository `tron2-rl-deploy-ros` retains a
documentation-example literal `10.192.1.2` in its source / launch
files (e.g. `Tron2HW.cpp`, `tron2_hw_node.cpp`, `tron2_hw.launch`),
kept per owner decision and declared in that repository's
`SECURITY.md`. That literal is documented there and is not mirrored
into this repository.

## Supported versions

Only the tip of the default branch and the most recent tagged release
receive security fixes. Older tags are provided as-is.

| Version         | Supported |
|------------------|-----------|
| default branch   | ✅        |
| Latest tag       | ✅        |
| Older tags       | ❌        |

## Reporting a vulnerability

**Do not** open a public issue for security reports.

Email: **contact@limxdynamics.com**
Subject prefix: `[tron2-mujoco-sim]`

Please include:

- Affected file(s) and commit / tag (and, if relevant, submodule
  pin recorded by `git submodule status`).
- A minimal reproducer or proof of concept.
- Impact assessment (e.g., "simulator accepts NaN torque without
  clamping", "malformed RobotCmd crashes the physics thread").
- Whether the issue could affect a real robot if the endpoint were
  pointed at a physical unit.
- Your preferred disclosure timeline and contact.

We aim to acknowledge reports within **3 business days** and provide a
remediation plan or an initial mitigation within **14 calendar days**.
We support coordinated disclosure; please do not publish details until
a fix or advisory is available.

## Out of scope

- Bugs in third-party parsers or runtimes (MuJoCo, PyYAML) — report
  those upstream.
- Bugs inside the git submodules (`robot-description`,
  `robot-joystick`, `limxsdk-lowlevel`) — report those to the
  respective repositories.
- Physical safety of the robot itself — report to the deployment
  repositories or to LimX product support.
- Requests to publish calibration data, control policies, or firmware —
  this repository intentionally excludes those.

## Safe harbor

Good-faith security research that follows this policy will not be
pursued legally by LimX Dynamics. Please respect user privacy, avoid
service disruption, and do not access data beyond what is necessary to
demonstrate the issue. Never point a proof-of-concept at a real
robot's control endpoint — use the local simulator target.
