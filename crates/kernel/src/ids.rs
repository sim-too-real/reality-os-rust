//! Newtypes for identity fields. Empty strings are invalid except where noted.

use crate::error::{KernelError, KernelResult};
use serde::{Deserialize, Serialize};
use std::fmt;

macro_rules! nonempty_id {
    ($name:ident, $field:expr) => {
        #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
        #[serde(transparent)]
        pub struct $name(String);

        impl $name {
            pub fn new(raw: impl Into<String>) -> KernelResult<Self> {
                let s = raw.into();
                if s.trim().is_empty() {
                    return Err(KernelError::InvalidId {
                        field: $field,
                        reason: "empty".into(),
                    });
                }
                Ok(Self(s))
            }

            #[inline]
            pub fn as_str(&self) -> &str {
                &self.0
            }

            #[inline]
            pub fn starts_with_sim(&self) -> bool {
                self.0.starts_with("SIM_")
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str(&self.0)
            }
        }

        impl AsRef<str> for $name {
            fn as_ref(&self) -> &str {
                &self.0
            }
        }
    };
}

nonempty_id!(ReleaseHash, "release_hash");
nonempty_id!(DesignContentHash, "design_content_hash");
nonempty_id!(CommandId, "command_id");
nonempty_id!(FirmwareId, "firmware_id");
nonempty_id!(SerialOrAsBuilt, "serial_or_as_built");
nonempty_id!(CalibrationId, "calibration_id");
nonempty_id!(SessionId, "session_id");

impl CalibrationId {
    pub fn is_sim_placeholder(&self) -> bool {
        self.as_str() == "SIM_CAL" || self.starts_with_sim()
    }
}

impl SerialOrAsBuilt {
    pub fn is_sim_placeholder(&self) -> bool {
        self.starts_with_sim()
    }
}

impl FirmwareId {
    pub fn is_sim_placeholder(&self) -> bool {
        self.starts_with_sim()
    }
}
