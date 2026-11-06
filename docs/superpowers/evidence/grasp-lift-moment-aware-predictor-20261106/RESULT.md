# Moment-aware post-close hold predictor — 2026-11-06 (window-59)

**Classification:** `ROBOTICS_CAPABILITY_ADVANCE; OFFLINE_SIMULATION_PREDICTOR; TWO_FROZEN_DEVELOPMENT_CONTROLS; NOT_ADMITTED`

## Capability delta

Beyond frozen SO-101 / encoder-noise reruns: a **post-close-only** hold predictor that uses contact-wrench direction and actuator saturation, not just scalar μ×normal.

| Trial | Scalar μ×normal | Predictor | Observed |
|---|---|---|---|
| `21006:21` | would **accept** (ratio ≈1.20) | **refuse_lift** (`saturated_horizontal_pinch_negligible_vertical_share`) | action_failed; first-lift does not support weight |
| `21010:25` | would refuse (ratio ≈0.42) | **refuse_lift** (`scalar_proxy_below_weight`) | unexpected_refusal |

`catches_scalar_false_accept`: **true** on this frozen pair.

## Method

Refuse if:

1. `mu_lower * Σ normal < object_weight`, or
2. `actuator_utilization ≥ 0.98` and `|Fz|/Σ|F| < 0.15` (saturated horizontal pinch).

Inputs: first bilateral `post_close` `grasp_contact_audits` from `grasp-lift-wrench-trace-20261103`. No full-lift replay in the predictor path.

## Reproduce

```text
py -3.10 -B docs/superpowers/evidence/grasp-lift-moment-aware-predictor-20261106/predict_hold_from_post_close.py
py -3.10 -B docs/superpowers/evidence/grasp-lift-moment-aware-predictor-20261106/verify_predictor.py
```

Independent verification: **PASS** (0 failed checks).

## Authority / limits

`NOT_ADMITTED`; metal/shop_release/viewport_admits false; does not authorize hardware or change runtime gates. Two development controls only — not held-out validation; simulation contact evidence only.
