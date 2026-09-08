# Public surface

Customer-facing, grouped the way Python `theworld.packages` is grouped.

## CLI (`ros-governor`)

| Command | Meaning |
|---------|---------|
| `status` | Honesty stamps |
| `decide` | Kernel only — no plant write |
| `dispatch` | SIM session bind + Governor write |
| `estop` | Software e-stop analog |
| `edge` | Sealed decide+gate (null motion) |
| `topics` | ROS 2 adapter map |

## Sealed packages (`realityos-session::packages`)

- `RealityOsPackage::decide`
- `GovernorPackage::gate_only` (admission; `_NullPlant` does not move)
- `SafetyEdge::decide_and_gate`

Stamps always: `metal=false`, `online=false`, `learned_actuator_authority=false`, `invent_authority=false`.

## ROS 2 (adapter)

Topics listed in `realityos_ros2::TOPICS`. Boot veto is **true**. `HARDWARE_WRITES_ENABLED=false`. Do not port `governor_node.py` as the production governor.

## Not public

Campaign plants, walk improvers, Foundry FEA (`theworld.native`), studio HUD, ONNX.
