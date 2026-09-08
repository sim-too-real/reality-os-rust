# First-principles used by the last-gate

Crate: `realityos-physics`. These are identities and conventional constants.
They are **not** MEASURED plant parameters. Unknown μ / Kt / I are caller screens.

| Law | Formula | Gate use |
|-----|---------|----------|
| Standard g | gₙ = 9.80665 m/s² | Coulomb decel ceiling |
| Stop distance | s = v² / (2 a) | `stop_distance` domain; cat-0 analog |
| Stop time | t = v / a | freshness vs stop |
| Newton | F = m a, τ = I α | force/torque envelopes |
| KE | ½ m v², ½ I ω² | `energy_envelope` |
| Power | P = τ ω = F v | mechanical power screen |
| Contact energy | ½ m v_close² | PFL SIM screen |
| Motor | τ = η N Kₜ I | `motor_torque` |
| Joule | P = I² R | thermal derate input |
| Back-EMF | V = I R + Kₑ ω | terminal voltage screen |
| Thermal derate | τ(T) = τ_max · max(0, 1 − k (T − T_r)) | SIM derate |
| Nyquist | f_N = f_s / 2 | 1 kHz dispose ⇒ 500 Hz plant band |
| Period | Δt = 1 / f | heartbeat stale if age > k Δt |
| Joint margin | min(q−q_min, q_max−q) | workspace |

Tests assert identities (e.g. v² = v0² + 2 a s after a stop), not dyno data.

PFL body-region newtons in `pfl_contact` are **configured SIM screens**, not ISO 10218 certificates.
