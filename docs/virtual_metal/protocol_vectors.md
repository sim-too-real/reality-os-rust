# Protocol 2.0 vectors vs Virtual Metal

Source: ROBOTIS e-Manual Protocol 2.0 (https://docs.robotis.com/docs/dxl/protocol/protocol2/) and XL330-M288 (https://docs.robotis.com/docs/dxl/model_reference/x_series/xl_series/xl330-m288).

## Matches (in-repo `encode_*` / `crc16` / Virtual Metal)

| Packet | Bytes |
| --- | --- |
| PING id=1 | `FF FF FD 00 01 03 00 01 19 4E` |
| PING broadcast 254 | `FF FF FD 00 FE 03 00 01 31 42` |
| PING status model 1030 fw 38 | `FF FF FD 00 01 07 00 55 00 06 04 26 65 5D` |
| WRITE Goal Position 512 | `FF FF FD 00 01 09 00 03 74 00 00 02 00 00 CA 89` |
| READ Present Position 4 | `FF FF FD 00 01 07 00 02 84 00 04 00 1D 15` |
| REBOOT id=1 | `FF FF FD 00 01 03 00 08 2F 4E` |
| empty STATUS | `FF FF FD 00 01 04 00 55 00 A1 0C` |

Length = parameters + 3 (instruction packets). CRC is over header through parameters.

## Disagreements / ambiguities (not silently picked)

1. **Byte stuffing vs CRC order.** Current e-Manual "Packet Process" says stuff, update length, then CRC on stuffed bytes. In-repo `crates/metal/src/protocol.rs` (and the Python PTY stand-in) compute CRC on the unstuffed body then stuff, including CRC bytes. Virtual Metal reuses the in-repo codec. Official examples above contain no `FF FF FD` in the body, so they match either rule. A stuffed body is a possible SDK/manual split; do not treat SDK C as firmware truth.

2. **Input Voltage Hardware Error bit.** Shutdown table: bit 0 (`0x01`). Min/Max Voltage text: `0x10`. Virtual Metal uses bit 0; physical READ of register 70 under brownout is required.

3. **Velocity P Gain initial value.** XL330-M288 e-Manual RAM table: 180. Previously in-repo `FACTORY_VELOCITY_P_GAIN = 100` (XL430-class). Classification `CONFIRMED_STALE_CODE`; production and Virtual Metal now use 180. See `docs/virtual_metal/constants_audit.json`.

4. **Official Dynamixel SDK** is an independent implementation reference, not firmware. Not executed in this Windows environment.

The Python PTY echo (`crates/metal/tests/xl330_responder.py`) is not Virtual Metal V1 and is not metal evidence.
