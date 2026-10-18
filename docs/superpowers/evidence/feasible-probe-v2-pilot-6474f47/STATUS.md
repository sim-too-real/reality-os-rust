# Suite-v2 Pilot Status

This run is diagnostic evidence only. The report records
`candidate_worktree_dirty: true`, so it does not satisfy the clean-candidate
freeze required before a public baseline. Do not use it as a public baseline or
as a competitive claim.

The MuJoCo-required campaign accounted for all 60 scheduled trials and passed
the hard safety gate, but it did not demonstrate push availability: both
development arms completed 0/12 expected-success push tasks. The force-aware
estimator also produced no friction narrowing in this campaign. The detailed
report and 60 raw trial records are retained beside this note so the failures
remain reproducible.

Next: finish the contact-paced probe and baseline-window fixes, freeze them in
a clean commit, then rerun the predeclared pilot before deciding whether the
public baseline gate is met.
