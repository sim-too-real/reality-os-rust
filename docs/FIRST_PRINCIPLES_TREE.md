# First-principles tree

Roots are identities. Branches are gate checks. Leaves are robot/env instances.
Nothing here is MEASURED metal.

```
conservation / kinematics / information
├── Newton
│   ├── F = m a                         force envelope
│   ├── τ = I α                         torque envelope
│   └── P = F v = τ ω                   power screen
├── Energy
│   ├── ½ m v²                          translational KE
│   ├── ½ I ω²                          rotational KE
│   └── ½ m v_close²                    PFL contact energy screen
├── Kinematics
│   ├── s = v² / (2 a)                  stop-distance domain
│   ├── t = v / a                       stop time vs heartbeat
│   └── a ≤ μ g                         Coulomb decel (μ is a screen)
├── Motor (steady SIM)
│   ├── τ = η N Kₜ I
│   ├── P_j = I² R
│   └── V = I R + Kₑ ω
├── Sampling / information
│   ├── Δt = 1/f
│   ├── f_N = f_s / 2                   1 kHz ⇒ 500 Hz plant band
│   ├── stale ⇔ age > k Δt              heartbeat / sensor
│   └── see ⇔ pixels compiled           no gifted pose
└── Honesty
    ├── SIM ≠ metal
    ├── LLM proposes, never writes
    └── empty boxes stay empty (fieldbus, dual-channel e-stop)

embodiment (robot-agnostic)
├── uniaxial / arm6 / wheeled / unitree_h1 (from TheWorld h1.xml ranges)
└── DoF, q limits, τ_max, dq_max  →  WorldView + envelope

environment (agnostic)
├── earth gₙ=9.80665  μ=0.6
├── moon g=1.62 vacuum
├── ice μ=0.05
└── high_g 20 m/s² screen

vision
├── RGB frame + SHA-256
├── pinhole unproject
└── missing camera REFUSE; black frame PROBE

rates (SIM deadlines)
├── vision 30 / screen 50 / telemetry 100 / gov-legacy 120 / control 500 / dispose 1000 Hz
└── deadline miss → not a metal claim; PassiveFallback when hold/NaN

agent (Grok)
├── admit: strip metal/MEASURED keys
├── verbs allowlist
├── offline proposer (CI)
└── live: POST https://api.x.ai/v1/chat/completions  (feature live-grok, not 1 kHz)

governor / session / plant
├── identity complete vs complete_online
├── CertifiedCommand only write
└── HardwareDriverPort  →  named hole: EtherCAT
```
