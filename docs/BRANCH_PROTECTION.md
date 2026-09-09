# Branch protection — operational P0 / P1

Repository code cannot enable GitHub branch protection. A workflow file is
CI, not merge policy.

## Required GitHub operation (humans / org admins)

On `sim-too-real/reality-os-rust`, protect `main`:

1. **Prevent direct pushes to `main`.**
2. **Require a pull request** before merge.
3. **Require the `authority / verify` check** (workflow `.github/workflows/authority.yml`)
   to pass:
   - `cargo fmt --all -- --check`
   - `cargo clippy --workspace --all-targets -- -D warnings`
   - `cargo test --workspace --all-targets`
4. **Require the branch to be up to date** with `main` before merge, where the
   org allows that setting.
5. Restrict who can dismiss required reviews / bypass protection. Do not allow
   skipping `authority / verify` on `main`.

These settings live in GitHub **Settings → Branches → Branch protection rules**
(or Rulesets). They are **P0/P1 operational** work. This repository does not
pretend that committing a YAML file is branch protection.

## What the workflow is

The authority workflow must remain exactly those three commands. Adding
optional jobs does not replace the required check.
