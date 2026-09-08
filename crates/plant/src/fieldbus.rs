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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FieldbusLink {
    pub kind: FieldbusKind,
    pub attached: bool,
    pub note: String,
    pub metal: bool,
}

impl FieldbusLink {
    /// Honest default: no fieldbus in this software crate.
    pub fn named_hole() -> Self {
        Self {
            kind: FieldbusKind::None,
            attached: false,
            note: "EtherCAT/CANopen/USB drive I/O is a named hole — implement HardwareDriverPort; do not invent metal".into(),
            metal: false,
        }
    }

    pub fn state(&self) -> LinkState {
        if self.attached {
            LinkState::Present
        } else {
            LinkState::NamedHole
        }
    }

    pub fn transmit(&self, _frame: &[u8]) -> PlantResult<()> {
        if !self.attached {
            return Err(PlantError::FieldbusNotAttached);
        }
        Err(PlantError::NamedHole("fieldbus_tx_not_implemented"))
    }
}
