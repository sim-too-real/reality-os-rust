//! Fieldbus link. EtherCAT/CANopen are named holes until a vendor port exists.

use serde::{Deserialize, Serialize};

use crate::error::{PlantError, PlantResult};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FieldbusKind {
    None,
    EtherCat,
    CanOpen,
    UsbSerial,
}

impl FieldbusKind {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::EtherCat => "ethercat",
            Self::CanOpen => "canopen",
            Self::UsbSerial => "usb_serial",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LinkState {
    Present,
    NamedHole,
    Refused,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FieldbusPhase {
    Detached,
    Probing,
    EnableRequested,
    NamedHole,
    Fault,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FieldbusLink {
    pub kind: FieldbusKind,
    pub attached: bool,
    pub note: String,
    pub metal: bool,
    pub phase: FieldbusPhase,
}

impl FieldbusLink {
    /// Honest default: no fieldbus in this software crate.
    pub fn named_hole() -> Self {
        Self {
            kind: FieldbusKind::None,
            attached: false,
            note: "EtherCAT/CANopen/USB drive I/O is a named hole — implement HardwareDriverPort; do not invent metal".into(),
            metal: false,
            phase: FieldbusPhase::NamedHole,
        }
    }

    pub fn state(&self) -> LinkState {
        if self.attached {
            LinkState::Present
        } else if self.phase == FieldbusPhase::Fault {
            LinkState::Refused
        } else {
            LinkState::NamedHole
        }
    }

    pub fn probe(&mut self, kind: FieldbusKind) -> LinkState {
        self.kind = kind;
        self.phase = FieldbusPhase::Probing;
        self.attached = false;
        self.metal = false;
        self.note = format!("{} probed; no vendor adapter — named hole", kind.as_str());
        self.phase = FieldbusPhase::NamedHole;
        LinkState::NamedHole
    }

    /// Software enable request. Still cannot attach without a vendor port + island.
    pub fn request_enable(&mut self, island_enabled: bool) -> PlantResult<LinkState> {
        if !island_enabled {
            self.phase = FieldbusPhase::Fault;
            return Err(PlantError::refused("fieldbus_enable_requires_software_island"));
        }
        self.phase = FieldbusPhase::EnableRequested;
        self.attached = false;
        self.metal = false;
        Err(PlantError::FieldbusNotAttached)
    }

    pub fn transmit(&self, _frame: &[u8]) -> PlantResult<()> {
        if !self.attached {
            return Err(PlantError::FieldbusNotAttached);
        }
        Err(PlantError::NamedHole("fieldbus_tx_not_implemented"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn probe_and_enable_cannot_attach() {
        let mut bus = FieldbusLink::named_hole();
        assert_eq!(bus.probe(FieldbusKind::EtherCat), LinkState::NamedHole);
        assert!(!bus.attached);
        assert!(!bus.metal);
        assert!(bus.request_enable(false).is_err());
        assert_eq!(bus.state(), LinkState::Refused);
        let mut bus = FieldbusLink::named_hole();
        assert!(bus.request_enable(true).is_err());
        assert!(!bus.attached);
        assert!(bus.transmit(&[1, 2]).is_err());
    }
}
