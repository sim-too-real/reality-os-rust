//! How this stack attaches to a real robot. Named holes stay empty.

use realityos_kernel::Layer;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LinkState {
    Present,
    NamedHole,
    Refused,
    OptionalAbsent,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectionLayer {
    pub index: u8,
    pub name: &'static str,
    pub layer: Layer,
    pub state: LinkState,
    pub note: String,
}

impl ConnectionLayer {
    fn hole(index: u8, name: &'static str, layer: Layer, note: impl Into<String>) -> Self {
        Self {
            index,
            name,
            layer,
            state: LinkState::NamedHole,
            note: note.into(),
        }
    }

    fn present(index: u8, name: &'static str, layer: Layer, note: impl Into<String>) -> Self {
        Self {
            index,
            name,
            layer,
            state: LinkState::Present,
            note: note.into(),
        }
    }
}

/// Static map of every link from first principles to metal.
/// Holes are explicit: this software does not invent EtherCAT or MEASURED.
pub fn architecture_map() -> Vec<ConnectionLayer> {
    vec![
        ConnectionLayer::hole(
            0,
            "metal_robot",
            Layer::Fieldbus,
            "Physical robot / dyno. Not in this crate. SIM ≠ metal.",
        ),
        ConnectionLayer::hole(
            1,
            "servo_drive",
            Layer::Fieldbus,
            "CiA 402 / vendor servo. Implement behind HardwareDriverPort.",
        ),
        ConnectionLayer::hole(
            2,
            "fieldbus",
            Layer::Fieldbus,
            "EtherCAT / CANopen / USB. FieldbusLink::named_hole until a port is attached.",
        ),
        ConnectionLayer::present(
            3,
            "hardware_driver_port",
            Layer::Plant,
            "Trait: probe_identity / read_sensor / write_action / hw_estop. Port never certifies.",
        ),
        ConnectionLayer::present(
            4,
            "command_egress",
            Layer::Plant,
            "Default RefuseCommandEgress (writes off). RecordingCommandEgress for SIM tests.",
        ),
        ConnectionLayer::present(
            5,
            "hardware_backed_plant",
            Layer::Plant,
            "Plant adapter. ONLINE act requires certified_write_scope. metal clamped false on harness.",
        ),
        ConnectionLayer::present(
            6,
            "execute_certified_command",
            Layer::Plant,
            "Sole sanctioned plant.act. Replay/TTL/ack/cert ALLOW|MODIFY.",
        ),
        ConnectionLayer::present(
            7,
            "runtime_governor",
            Layer::Governor,
            "Identity, e-stop, heartbeat, envelope, journal continuity.",
        ),
        ConnectionLayer::present(
            8,
            "runtime_session",
            Layer::Session,
            "bind_and_dispatch. Foreign hashes refused. HOLD blocks all modes.",
        ),
        ConnectionLayer::present(
            9,
            "reality_os_decide",
            Layer::RealityOs,
            "see → identify → certify → CertifiedCommand. VLA is a candidate.",
        ),
        ConnectionLayer::present(
            10,
            "first_principles_physics",
            Layer::Physics,
            "s=v²/2a, ½mv², τ=ηNKtI, Nyquist, Coulomb a≤μg. Screens, not MEASURED.",
        ),
        ConnectionLayer::present(
            11,
            "data_event_store",
            Layer::Kernel,
            "Append-only debug events with correlation id. Not a second write path.",
        ),
        ConnectionLayer::present(
            12,
            "ros2_adapter",
            Layer::Adapter,
            "Codecs + veto topics. rclrs not linked. hardware_writes_enabled=false.",
        ),
        ConnectionLayer::present(
            13,
            "planner_vla",
            Layer::RealityOs,
            "Proposal only. Learned sources cannot hold governor_ack.",
        ),
    ]
}

pub fn writable(layers: &[ConnectionLayer]) -> bool {
    !layers
        .iter()
        .any(|l| matches!(l.index, 3..=9) && l.state != LinkState::Present)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn map_has_no_silent_gaps() {
        let m = architecture_map();
        assert_eq!(m.len(), 14);
        assert_eq!(m[0].state, LinkState::NamedHole);
        assert_eq!(m[2].state, LinkState::NamedHole);
        assert!(m.iter().any(|l| l.name == "runtime_governor"));
        // Software path 3–9 is present; metal/fieldbus remain holes.
        assert!(m
            .iter()
            .filter(|l| (3..=9).contains(&l.index))
            .all(|l| l.state == LinkState::Present));
    }
}
