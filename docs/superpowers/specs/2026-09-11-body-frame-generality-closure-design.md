# Foundation generality closure: body-backed frames (not Phase B)

**Date:** 2026-09-11  
**Status:** implementation spec (user-approved `/goal`)  
**Baseline `main`:** `d3b4488e3e2a7f019278f843946634ee00dd3342` (PR #13 merged)  
**Preserved:** `GENERALITY_FREEZE_SHA` `20cb4270f89e81e7256ebb72bfda44c9251fbdf9`  
**Preserved artifacts:** `external_ur5e_matrix.json`, `external_holdout_first_score.json` (never rewrite)

This pass closes the body-backed EE gap exposed by the historical Panda first score. It is **not** Phase B. Do not implement GRASP, PUSH, PLACE, vision, VLA, planner, tactile, or locomotion.

## Frozen

Do not edit: kernel authority, governor/session, metal, consume/replay, `RealityOs::decide`, SIM ≠ METAL.

The historical Panda first-score JSON is immutable evidence. New results go to new files.

## Root cause to prove first

Panda YAML declares `end_effectors: [{name: tool0, body: hand}]` with no site. Current mapping writes **unknown** translation/rotation for body-only frames, so `ModelFrame::pose()` is missing and semantic FK returns `Unsupported` before the arm is evaluated.

Prove that with a generic test, then fix it. Do not special-case product names.

## Frame references

```text
SITE_FRAME     → model site pose on its parent body
BODY_FRAME     → identity on the declared body, provenance BUNDLE_DECLARED_BODY_FRAME (not Assumed)
BODY_OFFSET    → explicit xyz/quat with provenance
unknown        → stay unknown / MODEL_FEATURE_UNSUPPORTED
```

Semantic EE name (`tool0`) is not the physical frame id. Prefer `frame = ee:{name}`, `parent_body = <body>`.

## After the generic fix

1. Synthetic tests (body / site / offset / missing) with no model-name conditions.
2. Body-frame FK oracle vs MuJoCo body pose (≥100 samples).
3. UR5e remains green.
4. Rerun official Panda XML unchanged → `external_holdout_postfix_score.json`.
5. Discover tendon/equality metadata. Do not implement tendon control. Grasping stays unproven (`COUPLED_GRIPPER_NOT_QUALIFIED`).
6. Precise refuse taxonomy (do not collapse everything to `UNREACHABLE`).
7. Extra fingers / tendon actuator must not block arm-only REACH; hold unrelated resources.
8. `GENERALITY_V2_FREEZE_SHA`, then untouched official Menagerie `kuka_iiwa_14` (BSD-3-Clause) first score → `external_holdout_v2_first_score.json`.

## Stop

STOP after the V2 hold-out. No GRASP. Phase B only if an untouched second external arm passes the same REACH semantics.
