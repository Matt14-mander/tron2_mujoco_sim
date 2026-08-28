---
name: Feature request
about: Suggest an improvement to the MuJoCo simulator, SDK bridge, or docs
title: "[feat] <short summary>"
labels: enhancement
assignees: ''
---

## Problem

<!-- What is missing or awkward in the current simulator? -->

## Proposed change

<!-- Files or interfaces you propose adding / editing.
     If it touches simulator.py or tron2_sim/, describe the SDK message flow. -->

## Alternatives considered

## Scope

- Robot types affected: `SF_TRON2A` / `WF_TRON2A` / `DA_TRON2A` / `DACH_TRON2A` / `DASF_TRON2A` / other
- Backwards compatibility (SDK message layout, CLI flags, env vars):
- Submodule pin bumps required (which submodule, why):
- Does this need new assets in `robot-description`?
- Does this need SDK support that does not exist yet? If so, which API?
- Does it change the sim-vs-real boundary described in `SECURITY.md`?

<!--
Note: robot types whose channels have no limxsdk equivalent are out of scope
for this repository. See the support matrix in the README.
-->

## Checklist

- [ ] This request does **not** require shipping control policies
      (`.onnx`, `.pt`), SDK binaries (`.so`, `.dll`), calibration
      values, firmware, or model weights.
- [ ] This request does **not** introduce a mode that silently
      forwards simulator commands to a real robot.
- [ ] I have skimmed `CONTRIBUTING.md` for the submodule workflow
      and coding style.
