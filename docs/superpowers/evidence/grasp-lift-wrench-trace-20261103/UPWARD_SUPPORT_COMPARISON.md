# Upward Contact Support Comparison

This offline comparison uses the committed freeze and its two unchanged trial inputs. It compares the earlier scalar proxy (`lower simulated friction × summed post-close normal force`) with the signed world-z resultant of object-finger forces at `lift_waypoint_1`. Object weight uses the frozen trial mass and the checked-in model gravity, `[0, 0, -9.81] m/s²`.

| Trial | Scalar proxy / weight | First-lift finger z-force / weight | Outcome |
|---|---:|---:|---|
| 21010:25 | 0.595 / 1.419 N (0.419×) | 0.562 / 1.419 N (0.396×) | Runtime divergence |
| 21006:21 | 2.628 / 2.192 N (1.199×) | 1.295 / 2.192 N (0.591×) | Hold not verified; 7.362 mm object-follow residual |

For trial `21006:21`, the scalar proxy predicts more upward capacity than the object's weight, but the first-lift finger contact resultant is 0.897 N below weight. The first-lift contacts also produce a net moment about the object COM of approximately `[-0.000699, 0.000634, 0.011807] N·m`. The `grip_l` and `grip_r` actuator forces at that phase are `-1.950` and `-1.897 N`; at post-close both are at the existing 1.950 N magnitude cap. The actual outcome remains a hold-verifier failure. This is evidence that the scalar proxy is insufficient to accept this simulated lift, not evidence of hardware capacity. Table support is excluded from the finger-contact resultant.

Trial `21010:25` is directionally consistent with the scalar screen's rejection: its proxy is below object weight and its first-lift finger z-force is 0.562 N for a 1.419 N object. This case does not establish predictive performance.

**Decision:** keep the force cap, hold verifier, and refusal gates unchanged. Do not use the scalar proxy alone to authorize a lift. Prioritize an offline direction- and moment-aware predictor that also accounts for actuator realization, then evaluate it on independent cases before another full-lift replay.

Only two development trials were analyzed. Contact values are simulator-derived, the post-close phase has no sample timestamp, and no held-out, physical, or commercial claim follows. Reproduction: run `compare_upward_support.ps1`; its output is bound to `FREEZE.json` and `RESULT.json` in `UPWARD_SUPPORT_COMPARISON.json`.
