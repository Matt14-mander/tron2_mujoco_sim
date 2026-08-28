# Contributing to `tron2-mujoco-sim`

Thanks for helping improve the TRON2 MuJoCo simulator. This repository
holds `simulator.py` and the `tron2_sim/` package (a LimX-SDK ↔ MuJoCo
bridge) plus documentation. The runtime model / meshes and SDK
binaries live in the three git submodules — this repo pins them by
commit but does not vendor their contents.

## Table of contents

- [Ways to contribute](#ways-to-contribute)
- [Development setup](#development-setup)
- [Architecture in one paragraph](#architecture-in-one-paragraph)
- [Submodule workflow](#submodule-workflow)
- [Coding style](#coding-style)
- [Verification before opening a PR](#verification-before-opening-a-pr)
- [Commit messages](#commit-messages)
- [Pull request checklist](#pull-request-checklist)
- [Updating a submodule pin](#updating-a-submodule-pin)
- [Sign-off (DCO)](#sign-off-dco)
- [Code of conduct](#code-of-conduct)

## Ways to contribute

Welcome:

- Bug reports and fixes for the bridge: torque handling, sensor
  mapping, SDK message translation, threading.
- New variant declarations under `tron2_sim/variants/` once the
  matching assets and SDK support exist upstream.
- Improvements to the joystick / IP configuration surface.
- Documentation, verification snippets, sim integration examples.
- Demo animations for `doc/gif/` (see the rules in
  [`doc/gif/README.md`](doc/gif/README.md)).

We do **not** accept:

- Control policies, model weights, or trained artifacts (`.onnx`,
  `.pt`, `.pth`, `.ckpt`) — those live in the deployment repository.
- SDK wheels or vendor binaries checked into this tree — the SDK is
  consumed through the `limxsdk-lowlevel` submodule.
- Calibration values, firmware, rosbags / MCAP captures.
- Hard-coded private network addresses.
- Media disclosing individuals, office locations, or non-public
  products.

## Development setup

```bash
# clone with submodules
git clone --recurse-submodules https://github.com/limxdynamics/tron2_mujoco_sim.git
cd tron2-mujoco-sim

# recommended: uv-managed environment
uv sync --extra sdk
```

If you cloned without submodules, `uv sync` will fail pointing at a
missing `limxsdk-*.whl`; run `git submodule update --init --recursive`
and retry.

A plain pip setup also works:

```bash
pip install -U pip
pip install mujoco numpy pyyaml
pip install limxsdk-lowlevel/python3/amd64/limxsdk-*-py3-none-any.whl
# or, on arm64 hosts:
pip install limxsdk-lowlevel/python3/aarch64/limxsdk-*-py3-none-any.whl
```

but contributions are verified against the environment `uv.lock`
pins, so please reproduce with uv before reporting a
version-sensitive result. See [`README.md`](README.md) for the full
dependency and deployment story, including why `limxsdk` is an
optional extra rather than a plain dependency.

## Architecture in one paragraph

`simulator.py` resolves `$ROBOT_TYPE` into a `RobotSpec` through the
registry in `tron2_sim/variants/`, then hands it to `SimCore`. A spec
is a list of `ChannelSpec` (a joint group with a wire order and an SDK
role) plus optional `SimModule` instances. `JointChannel` maps wire
order to MuJoCo indices **by joint name** and runs the MIT control
law; `SdkBus` is the only transport. Variant-specific behaviour
belongs in `tron2_sim/variants/` — the core, channels and modules stay
variant-agnostic.

Two invariants are easy to break and expensive to debug:

- **Never map wire order to indices by position.** Actuator
  declaration order in the XML differs from wire order on several
  models. Resolve by name.
- **SDK callbacks run on native background threads.** Build a
  complete new buffer and swap the reference in a single assignment;
  never mutate in place a buffer the physics thread is reading.

## Submodule workflow

This repository has three submodules:

| Submodule | Purpose |
|-----------|---------|
| `robot-description` | URDF / MJCF models and meshes loaded by the simulator. |
| `robot-joystick` | Joystick binary used by the deploy stack. |
| `limxsdk-lowlevel` | LimX SDK sources and pre-built wheels. |

- **Never** commit inside a submodule from this repository; open a PR
  in the submodule's own repo, land it there, then update the pin
  here in a separate PR (see [Updating a submodule
  pin](#updating-a-submodule-pin)).
- `git submodule status --recursive` must produce a non-empty listing
  before any smoke test — CI enforces this.
- Do not add new submodules without a `THIRD_PARTY_NOTICES.md` entry
  and legal sign-off. New submodules are treated as new third-party
  dependencies.

## Coding style

- Format with `ruff format`. Lint with `ruff check`; configuration
  lives in `pyproject.toml`. CI fails on lint errors.
- Comments, docstrings and runtime messages are **English only**.
  Keep comments sparse: explain what a non-obvious block does and
  warn about real hazards; do not narrate design history.
- No decorative emoji in program output. Prefix diagnostics with
  `INFO:`, `WARNING:`, or `ERROR:`.
- Do not introduce hard-coded machine IPs, hostnames, or credentials.
  The `<robot-ip>` token used in documentation command examples is a
  **placeholder** — users substitute their own robot / simulator IP
  when running. Do not replace it with a real address, and do not
  propagate hard-coded IPs into other files.

## Verification before opening a PR

Run all of the following and paste the summary into the PR description:

```bash
# 1. Byte-compile everything this repository owns
uv run python -m py_compile simulator.py $(find tron2_sim -name '*.py')

# 2. Lint
uvx ruff check .

# 3. Submodules initialized
git submodule status --recursive

# 4. MuJoCo can import
uv run python -c "import mujoco, sys; print(mujoco.__version__)"

# 5. Every affected robot type still loads and resolves by name
ROBOT_TYPE=SF_TRON2A uv run simulator.py --headless --duration 5

# 6. No forbidden artifacts staged
git ls-files | grep -iE '(^|/)__pycache__(/|$)|\.(pyc|onnx|pt|pth|ckpt|so|dll|dylib|lib|whl|bag|mcap)$' \
  && echo FORBIDDEN || echo OK
```

CI runs equivalent steps; local pre-checks save review round-trips.
When you change the bridge, also run a controller-side loopback:
publish `RobotCmd` from a second process and confirm the joints move
and `RobotState` comes back. For Centaur-based types (`DASF_*`) the
command **must** carry `motor_names`, or the native layer silently
drops it.

## Commit messages

Follow Conventional Commits:

```
type(scope): short imperative summary

Longer explanation of why, if it is not obvious.

Signed-off-by: Your Name <you@example.com>
```

`type` ∈ `feat | fix | docs | refactor | chore | ci | test | build |
submodule`. `scope` is usually `sim`, `sdk-bridge`, `variants`,
`joystick`, or `meta`.

## Pull request checklist

- [ ] Python files byte-compile and pass `ruff check`.
- [ ] `git submodule status --recursive` reported all submodules
      initialized during testing.
- [ ] Affected robot types start headless without errors.
- [ ] Comments and messages are in English, with no decorative emoji.
- [ ] No new hard-coded IPs, hostnames, credentials, or private paths.
- [ ] No `.onnx`, `.pt`, `.so`, `.whl`, `.bag`, `.mcap`, or
      `__pycache__/` staged.
- [ ] `THIRD_PARTY_NOTICES.md` updated if a runtime dep or submodule
      pin changed.
- [ ] `CHANGELOG.md` has an entry under `## [Unreleased]`.
- [ ] DCO sign-off on every commit.

## Updating a submodule pin

1. Land the change upstream in the submodule's own repository.
2. In this repo:

   ```bash
   cd robot-description   # or robot-joystick / limxsdk-lowlevel
   git fetch
   git checkout <new-sha>
   cd ..
   git add robot-description
   ```
3. Update the row in `THIRD_PARTY_NOTICES.md` §2 with the new SHA and
   re-verify the license / re-distribution status. If the SDK wheel
   version changed, also update the filenames in the
   `[tool.uv.sources]` block of `pyproject.toml`.
4. Tick the "submodule pin update" box in the PR template and describe
   the rationale.
5. Bump `CHANGELOG.md` under `## [Unreleased]` — pin bumps are
   user-visible.

## Sign-off (DCO)

We use the [Developer Certificate of Origin](https://developercertificate.org/).
Every commit must be signed off:

```bash
git commit -s -m "your message"
```

Signing off certifies that you have the right to submit the change
under the repository's license.

## Code of conduct

Be respectful and constructive. Reports to `contact@limxdynamics.com`.
