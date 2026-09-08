//! Mandatory identity for any actuation session.

use realityos_kernel::{
    CalibrationId, DesignContentHash, FirmwareId, KernelResult, ReleaseHash, SerialOrAsBuilt,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RuntimeIdentity {
    pub release_hash: ReleaseHash,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub design_content_hash: Option<DesignContentHash>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub serial_or_as_built: Option<SerialOrAsBuilt>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub firmware_id: Option<FirmwareId>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub calibration_id: Option<CalibrationId>,
}

impl RuntimeIdentity {
    pub fn sim(release_hash: impl Into<String>) -> KernelResult<Self> {
        Ok(Self {
            release_hash: ReleaseHash::new(release_hash)?,
            design_content_hash: None,
            serial_or_as_built: Some(SerialOrAsBuilt::new("SIM_SERIAL")?),
            firmware_id: Some(FirmwareId::new("SIM_FW")?),
            calibration_id: Some(CalibrationId::new("SIM_CAL")?),
        })
    }

    /// SIM completeness: release + calibration (SIM_CAL counts).
    pub fn complete(&self) -> bool {
        self.calibration_id.is_some()
    }

    /// ONLINE completeness: non-SIM serial/firmware/cal + design hash.
    pub fn complete_online(&self) -> bool {
        if !self.complete() {
            return false;
        }
        let Some(design) = &self.design_content_hash else {
            return false;
        };
        if design.as_str().trim().is_empty() {
            return false;
        }
        match &self.serial_or_as_built {
            Some(s) if !s.is_sim_placeholder() => {}
            _ => return false,
        }
        match &self.firmware_id {
            Some(f) if !f.is_sim_placeholder() => {}
            _ => return false,
        }
        match &self.calibration_id {
            Some(c) if !c.is_sim_placeholder() => {}
            _ => return false,
        }
        true
    }

    pub fn calibration_id_str(&self) -> &str {
        self.calibration_id
            .as_ref()
            .map_or("", CalibrationId::as_str)
    }

    pub fn serial_str(&self) -> &str {
        self.serial_or_as_built
            .as_ref()
            .map_or("", SerialOrAsBuilt::as_str)
    }

    pub fn firmware_str(&self) -> &str {
        self.firmware_id.as_ref().map_or("", FirmwareId::as_str)
    }

    pub fn design_str(&self) -> &str {
        self.design_content_hash
            .as_ref()
            .map_or("", DesignContentHash::as_str)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use realityos_kernel::{CalibrationId, DesignContentHash, FirmwareId, SerialOrAsBuilt};

    #[test]
    fn complete_online_refuses_sim_placeholders() {
        let thin = RuntimeIdentity::sim("rel1").unwrap();
        assert!(thin.complete());
        assert!(!thin.complete_online());

        let full = RuntimeIdentity {
            release_hash: realityos_kernel::ReleaseHash::new("rel1").unwrap(),
            design_content_hash: Some(DesignContentHash::new("des1").unwrap()),
            serial_or_as_built: Some(SerialOrAsBuilt::new("SN-1").unwrap()),
            firmware_id: Some(FirmwareId::new("FW-1").unwrap()),
            calibration_id: Some(CalibrationId::new("cal-1").unwrap()),
        };
        assert!(full.complete_online());
    }
}
