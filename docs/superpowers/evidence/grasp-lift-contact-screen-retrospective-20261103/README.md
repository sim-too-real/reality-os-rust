# Retrospective contact-capacity screen

This package freezes an exploratory calculation on two primary grasp-lift
controls. It checks whether measured post-close finger normal loads, combined
with the lower simulated friction interval, could have rejected an infeasible
scalar hold before lift.

The historical post-close traces match the exact current trial digests and
object mass/friction, but their run manifests mark the source worktrees dirty.
The result is therefore retrospective diagnostic evidence, not a validated
predictor. The method, limits, inputs, source hashes, and closed authority
flags are recorded in `FREEZE.json`; findings are in `RESULT.md` and
`RESULTS.json`. Run `python verify_retrospective.py` to recompute the results.
