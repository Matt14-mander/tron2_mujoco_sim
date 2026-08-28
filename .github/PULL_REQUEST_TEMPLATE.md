<!--
Thanks for contributing to tron2-mujoco-sim!
Please fill in the sections below. Delete any that are not applicable.
-->

## Summary

<!-- One paragraph: what and why. Link the issue this closes, if any. -->

## Type of change

- [ ] `fix`      — corrects a defect in `simulator.py`, `tron2_sim/`, CI, or docs
- [ ] `feat`     — new capability (new robot type, new SDK message, etc.)
- [ ] `docs`     — README, THIRD_PARTY_NOTICES, CONTRIBUTING, SECURITY
- [ ] `ci`       — GitHub Actions or verification tooling
- [ ] `chore`    — repo maintenance (deps, formatting, cleanup)
- [ ] `submodule`— submodule pin update (see checkbox below)

## Affected surfaces

- [ ] `simulator.py` / `tron2_sim/` (SDK ↔ MuJoCo bridge, variant registry)
- [ ] Documentation / media (`README.md`, `doc/`)
- [ ] CI / templates (`.github/`)
- [ ] Meta / repo-wide

## Robot types affected

<!-- e.g. SF_TRON2A, WF_TRON2A, DA_TRON2A, DACH_TRON2A, DASF_TRON2A, all, none (docs only) -->

## Submodule pin update

- [ ] This PR **updates a submodule pin**. If ticked, complete:
  - Submodule: `robot-description` / `robot-joystick` / `limxsdk-lowlevel`
  - Old SHA → new SHA:
  - Reason for the bump:
  - Upstream PR / release notes:
  - `THIRD_PARTY_NOTICES.md` §2 row updated: yes / no
  - License / re-distribution status re-verified: yes / no
  - Owner sign-off recorded (issue link or reviewer):

## Verification

Paste the output (or a summary) of the local verification steps from
`CONTRIBUTING.md#verification-before-opening-a-pr`:

```text
uv run python -m py_compile simulator.py $(find tron2_sim -name '*.py')
uvx ruff check .
git submodule status
ROBOT_TYPE=<TYPE> uv run simulator.py --headless --duration 5
forbidden-artifacts scan: ...
```

- [ ] Byte-compiles and passes `ruff check`.
- [ ] Affected robot types start headless without errors.
- [ ] Controller-side loopback exercised, if the bridge changed.
      (Centaur types require `motor_names` on the command.)

## Provenance & sensitivity

- [ ] No control policies (`.onnx`, `.pt`, `.pth`, `.ckpt`) are added.
- [ ] No SDK binaries (`.so`, `.dll`, `.dylib`, `.lib`) or wheels
      (`.whl`) are added directly to this tree.
- [ ] No calibration values, firmware, rosbags (`.bag`, `.mcap`), or
      customer data are added.
- [ ] No new hard-coded IPs, hostnames, or credentials are introduced.
      The `<robot-ip>` token that appears in `README.md` / docs is a
      placeholder for users to substitute at run time; no source-side
      private IP remains in this repository.
- [ ] Any new / modified media under `doc/` has been EXIF-stripped and
      cleared for people / office / non-public product visibility.
- [ ] `THIRD_PARTY_NOTICES.md` is up to date.

## Sim-vs-real boundary

- [ ] This PR does **not** introduce a code path that silently
      forwards simulator commands to a physical robot, and does not
      widen the default SDK bind address beyond `127.0.0.1`.
      (If it does, describe the safety controls added.)

## Checklist

- [ ] Comments and runtime messages are English, no decorative emoji.
- [ ] `CHANGELOG.md` has an entry under `## [Unreleased]`.
- [ ] All commits are DCO-signed (`git commit -s`).
- [ ] CI is expected to pass.

## Related issues

<!-- Fixes #123 / Refs #456 -->

## Notes for reviewers

<!-- Anything non-obvious: trade-offs considered, follow-ups deferred. -->
