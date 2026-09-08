# How this stack connects to a real robot

Honest map. Empty boxes stay empty.

```
 planner / VLA / language
        │  proposal only
        ▼
 RealityOs.decide  (first-principles screens: τ, s=v²/2a, ½mv², PFL)
        │  CertifiedCommand  acknowledged=false
        ▼
 RuntimeSession.bind_and_dispatch
        │  identity bind (never overwrite foreign hashes)
        ▼
 RuntimeGovernor.write_driver
        │  e-stop, heartbeat, envelope, journal
        ▼
 execute_certified_command
        │  certified_write_scope
        ▼
 HardwareBackedPlant.act
        │
        ▼
 HardwareDriverPort.write_action     ◄── you implement this for a robot
        │
        ▼
 CommandEgress / fieldbus / servo    ◄── NAMED HOLE today
        │
        ▼
 metal robot                         ◄── NAMED HOLE (SIM ≠ metal)
```

Sensors travel the other way:

```
 joint_states / WrenchStamped  →  codec (pure)  →  SensorPacket hash
        →  RuntimeSession.ingest_sensor  →  Governor freshness gate
```

## What we handle now

| Link | Status |
|------|--------|
| Decide / certify / five-word verdicts | Present |
| First-principles stop / energy / motor screens | Present (SIM formulas) |
| Identity + ONLINE refuse `SIM_*` | Present |
| Certified-only `plant.act` | Present |
| HardwareDriverPort + HardwareBackedPlant | Present |
| SimulatedHardwarePort harness | Present (not metal) |
| RefuseCommandEgress (writes off) | Present |
| ROS 2 codecs + veto topics | Present (no rclrs) |
| Event store / debug correlation | Present |
| EtherCAT / CANopen / CiA 402 | **Named hole** |
| Independent hardware e-stop channel | **Named hole** (software latch only) |
| MEASURED dyno / ONLINE metal | **Named hole** |

## What a vendor port must do

1. Implement `HardwareDriverPort` (identity, sensor, write, hw e-stop).
2. Do **not** call `write_action` from a ROS callback or VLA.
3. Attach the port to `HardwareBackedPlant` → `RuntimeSession.start`.
4. Keep `CommandEgress` behind `execute_certified_command`.
5. Leave `HonestyStamp.metal == false` until MEASURED rails exist.

`ros-governor chain` prints this map. `ros-governor debug` runs a harness loop and dumps the data layer.
