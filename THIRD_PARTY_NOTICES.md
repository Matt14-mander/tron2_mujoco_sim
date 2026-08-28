# Third-Party Notices

`tron2-mujoco-sim` (TRON2 MuJoCo simulator) is distributed under the
Apache License 2.0 (see [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE)).

This file lists third-party components, git submodules, runtime
dependencies, and documentation media so downstream users can comply
with all applicable licenses and re-distribution terms.

> **Status:** items marked `⚠ TO CONFIRM` are pending sign-off from
> the SDK / hardware / product / legal owners. Do not cut a public
> release while any `⚠ TO CONFIRM` entry remains.

---

## 1. First-party sources (LimX Dynamics)

| Path | Kind | License | Notes |
|------|------|---------|-------|
| `simulator.py`, `tron2_sim/` | Python source | Apache-2.0 | MuJoCo simulator bridging LimX SDK `RobotCmd` (q / dq / tau / Kp / Kd) to `mujoco.MjData`, dispatched to per-variant specs (SF/WF/DA/DACH/DASF) in `tron2_sim/variants/`. Real-robot control interface — see [`SECURITY.md`](SECURITY.md) for the sim-vs-real boundary. |
| `README.md`, `README_zh-CN.md`, `LICENSE`, `NOTICE`, `SECURITY.md`, `CONTRIBUTING.md`, `CHANGELOG.md`, this file | Documentation | Apache-2.0 | Hand-maintained. |

---

## 2. Git submodules (not vendored — pinned by commit)

This repository declares three submodules in `.gitmodules`. Their
content is **not** copied into this tree; users initialize them with
`git submodule update --init --recursive`. Each pin below is the
commit recorded at the time this scaffolding was written; verify with
`git submodule status --recursive` before any public tag.

| Submodule | Path | Upstream URL | Pinned commit (full SHA) | License | Re-distribution allowed |
|-----------|------|--------------|--------------------------|---------|-------------------------|
| `robot-description` | `robot-description/` | `https://github.com/limxdynamics/tron2-robot-description.git` (branch `main`) | `9939c22e69d27653ec0ba8a505859a2903dd1a71` | Apache-2.0 (per sibling repo `NOTICE`) ⚠ TO CONFIRM once submodule review lands | ⚠ TO CONFIRM |
| `robot-joystick` | `robot-joystick/` | `https://github.com/limxdynamics/robot-joystick.git` (branch `main`) | `30f69a9b3cba545a23ecf3f28f4e5ae6c78479cd` | ⚠ TO CONFIRM (binary vs. source; likely ships a `robot-joystick` executable) | ⚠ TO CONFIRM |
| `limxsdk-lowlevel` | `limxsdk-lowlevel/` | `https://github.com/limxdynamics/limxsdk-lowlevel.git` (branch `master`) | `70ff83c22f5f54a07c2ddf7e32a2c6ffb1d2ebc7` | ⚠ TO CONFIRM (SDK wheels under `python3/{amd64,aarch64}/limxsdk-*.whl` — redistribution terms unknown) | ⚠ TO CONFIRM |

Reproduce the reachability probe:

```bash
git ls-remote https://github.com/limxdynamics/tron2-robot-description.git | head
git ls-remote https://github.com/limxdynamics/robot-joystick.git | head
git ls-remote https://github.com/limxdynamics/limxsdk-lowlevel.git | grep 70ff83c2
```

**Owner action required (per submodule):**

- Confirm the upstream repository is public and intended to remain so.
- Record the license identifier (SPDX) and re-distribution terms.
- Record whether the submodule ships pre-built binaries (`.whl`,
  `.so`, or a stand-alone executable) that are subject to additional
  terms.
- If a submodule is not cleared for public use, remove the entry from
  `.gitmodules` before publishing this repository.

Do not silently update submodule pins — pin bumps require the same
sign-off flow (see [`CONTRIBUTING.md`](CONTRIBUTING.md) and the
"submodule pin update" checkbox in the PR template).

The SDK submodule ships a compiled extension (`_robot.so`) and a
bundled CPython runtime inside its wheels; those binaries stay in the
submodule and this repository never commits `.so` or `.whl` files
(CI enforces this). `limxsdk`'s wheel metadata declares dependencies
this simulator does not use (`onnxruntime`, `pygame`, `scipy`,
`pandas`) and pins `numpy<1.26.4`; the
`[[tool.uv.dependency-metadata]]` block in `pyproject.toml` overrides
that metadata to declare no dependencies, so only the packages listed
in §3 are actually installed.

---

## 3. Runtime dependencies (not vendored)

Resolved from the Python package index at install time and recorded in
`uv.lock`. None of them are vendored or bundled in this repository
(see [`README.md`](README.md) for install commands).

| Dependency | Purpose | License | Where obtained |
|------------|---------|---------|-----------------|
| MuJoCo (`mujoco`) | Physics simulation, model loading, passive viewer | Apache-2.0 | https://mujoco.org — `uv add mujoco` / `pip install mujoco` |
| NumPy | Array math; pulled in transitively by `mujoco` | BSD-3-Clause | `pip install numpy` |
| PyYAML | Config parsing (`tron2_sim/config/gripper_config.yaml`) | MIT | `pip install pyyaml` |
| LimX SDK (`limxsdk`) | RobotCmd / RobotState bridge | ⚠ TO CONFIRM (shipped as a wheel under submodule `limxsdk-lowlevel/`; installed via the optional `sdk` extra, not from an index) | This repo does **not** ship the wheel; installed by the user from the submodule. |

Transitive packages of `mujoco` (`absl-py`, `etils`, `glfw`, `pyopengl`,
...) are resolved by `uv` and pinned in `uv.lock`; consult that file
for the exact set and versions.

`onnxruntime` and `pygame` are **not** dependencies of this repository;
they are consumed by the linked `tron2-rl-deploy-python` controller
(§4), not by `simulator.py` / `tron2_sim/` directly.

---

## 4. Linked (not vendored) external controller

`README.md` documents that the ONNX policy is loaded by an external
sibling repository, **`tron2-rl-deploy-python`**:

```
tron2-rl-deploy-python/controllers/model/<ROBOT_TYPE>/policy.onnx
tron2-rl-deploy-python/controllers/model/<ROBOT_TYPE>/encoder.onnx
tron2-rl-deploy-python/controllers/model/<ROBOT_TYPE>/params.yaml
```

- No `.onnx`, `.pt`, `.pth`, or `.ckpt` file is committed to
  `tron2-mujoco-sim`; the deny-list in
  [`.github/workflows/ci.yml`](.github/workflows/ci.yml) enforces this.
- The deployment repository has its own license, `THIRD_PARTY_NOTICES`,
  and model provenance. This repository only **references** it and
  does not re-distribute its contents.

---

## 5. Documentation media

| Path | Kind | Provenance | License |
|------|------|------------|---------|
| `doc/gif/SF_TRON2A.gif`, `doc/gif/WF_TRON2A.gif`, `doc/gif/DACH_TRON2A.gif`, `doc/gif/DASF_TRON2A.gif` | Simulation screen captures (per variant) | ⚠ TO CONFIRM | ⚠ TO CONFIRM |
| `doc/sfmj-ezgif.com-video-to-gif-converter.gif` | Simulation screen capture (SF, legacy) | ⚠ TO CONFIRM | ⚠ TO CONFIRM |
| `doc/wfmj-ezgif.com-video-to-gif-converter.gif` | Simulation screen capture (WF, legacy) | ⚠ TO CONFIRM | ⚠ TO CONFIRM |
| `doc/deploy.jpg` | Real-world deploy photo | ⚠ TO CONFIRM (may show individuals / office / hoist rig) | ⚠ TO CONFIRM |
| `doc/sf.GIF` | Real-world capture (SF) | ⚠ TO CONFIRM | ⚠ TO CONFIRM |
| `doc/wf.GIF` | Real-world capture (WF) | ⚠ TO CONFIRM | ⚠ TO CONFIRM |

Every image or animation added under `doc/` must be original material,
must not show individuals, office interiors, or unreleased products,
and must carry no identifying EXIF metadata. Before release, run:

```bash
exiftool doc/* doc/gif/* | grep -iE '(gps|serial|make|model|software|author|artist|copyright)'
```

and strip anything that discloses office locations, camera serials, or
individual contributors' names, unless intentionally kept:

```bash
exiftool -all= doc/*.jpg doc/*.GIF doc/*.gif doc/gif/*.gif
```

CI scans committed media for GPS, serial, author, and artist tags. See
also [`doc/gif/README.md`](doc/gif/README.md). The real-world deploy
photo/GIFs may show people or non-public hardware — obtain publication
clearance from every identifiable person before release, or replace
with a sanitized capture.

---

## 6. What this repository does **not** include

- No control policies (`.onnx`, `.pt`, `.pth`, `.ckpt`) — see
  `tron2-rl-deploy-python`.
- No SDK binaries or wheels (`.so`, `.dll`, `.dylib`, `.lib`, `.whl`)
  directly in this tree. SDK wheels live under the `limxsdk-lowlevel`
  submodule and are installed by the user, not bundled.
- No factory calibration values or per-serial calibration files.
- No motion / bag / trajectory data (`.bag`, `.mcap`).
- No firmware.
- No customer- or site-specific configuration.
- No hard-coded private network address. Command examples use the
  placeholder `<robot-ip>` or the loopback address `127.0.0.1`; CI
  fails the build if a real private IP appears.

---

## 7. Update procedure

Whenever a submodule pin, a runtime dependency, or a documentation
image is added or changed:

1. Update the corresponding row in this file.
2. Re-run the EXIF strip (§5) on any new / touched media.
3. If the change touches an `⚠ TO CONFIRM` row, block the merge on
   written sign-off from the responsible owner.
4. Bump `CHANGELOG.md` under `## [Unreleased]`.
