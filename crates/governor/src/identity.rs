//! Mandatory identity for any actuation session.
//!
//! ONLINE signing uses [`ValidatedRuntimeIdentity`] only. That type can be
//! constructed solely after an exact match of configured expected identity
//! against [`realityos_plant::HardwareDriverPort::probe_identity`].

use realityos_kernel::{
    CalibrationId, DesignContentHash, FirmwareId, KernelResult, ReleaseHash, SerialOrAsBuilt,
};
use realityos_plant::HardwareIdentity;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Domain separator + encoding version. Length-prefixed fields; injective.
pub const INSTANCE_SCHEMA: &str = "realityos.runtime_instance/2";

/// Caller/operator configuration. Never signed until hardware verifies it.
pub type ExpectedRuntimeIdentity = RuntimeIdentity;

/// Driver-reported attachment. Never copied over expected fields.
pub type MeasuredHardwareIdentity = HardwareIdentity;

/// Canonical binding of execution-relevant runtime identity.
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

    /// Digest over canonical instance bytes. ONLINE signing must call this
    /// only through [`ValidatedRuntimeIdentity`].
    pub fn instance_hash(&self, actuator_ids: &[String]) -> String {
        hex::encode(Sha256::digest(canonical_instance_bytes(self, actuator_ids)))
    }
}

/// Identity that may authorize and sign an `OnlineWrite`.
///
/// Fields are the **expected** values after an exact probe match. Probed
/// values are never written back into this type.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ValidatedRuntimeIdentity {
    identity: RuntimeIdentity,
}

impl ValidatedRuntimeIdentity {
    /// Fail closed on any execution-relevant mismatch. Does not overwrite
    /// `expected` with `measured`.
    pub fn bind(
        expected: ExpectedRuntimeIdentity,
        measured: &MeasuredHardwareIdentity,
        authorized_actuators: &[String],
    ) -> Result<Self, String> {
        if !expected.complete_online() {
            return Err("incomplete_online_runtime_identity".into());
        }
        match_expected_to_measured(&expected, measured, authorized_actuators)?;
        Ok(Self { identity: expected })
    }

    pub fn as_runtime(&self) -> &RuntimeIdentity {
        &self.identity
    }

    pub fn instance_hash(&self, actuator_ids: &[String]) -> String {
        self.identity.instance_hash(actuator_ids)
    }
}

/// Length-prefixed little-endian encoding. Mapping from fields → bytes is injective.
pub fn canonical_instance_bytes(id: &RuntimeIdentity, actuator_ids: &[String]) -> Vec<u8> {
    let mut acts: Vec<&str> = actuator_ids.iter().map(String::as_str).collect();
    acts.sort_unstable();
    acts.dedup();
    let mut out = Vec::new();
    write_lp(&mut out, INSTANCE_SCHEMA.as_bytes());
    write_lp(&mut out, id.release_hash.as_str().as_bytes());
    write_lp(&mut out, id.design_str().as_bytes());
    write_lp(&mut out, id.serial_str().as_bytes());
    write_lp(&mut out, id.firmware_str().as_bytes());
    write_lp(&mut out, id.calibration_id_str().as_bytes());
    out.extend_from_slice(&(u32::try_from(acts.len()).unwrap_or(u32::MAX)).to_le_bytes());
    for a in acts {
        write_lp(&mut out, a.as_bytes());
    }
    out
}

fn write_lp(out: &mut Vec<u8>, bytes: &[u8]) {
    let len = u32::try_from(bytes.len()).unwrap_or(u32::MAX);
    out.extend_from_slice(&len.to_le_bytes());
    out.extend_from_slice(bytes);
}

/// Exact match of execution-relevant fields. Mismatch is configuration/device
/// inconsistency — fail closed.
pub fn match_expected_to_measured(
    expected: &RuntimeIdentity,
    measured: &HardwareIdentity,
    authorized_actuators: &[String],
) -> Result<(), String> {
    if !measured.connected {
        return Err("online_hardware_disconnected".into());
    }
    if measured.is_placeholder() {
        return Err("online_hardware_placeholder_or_sim".into());
    }
    if measured.serial.trim().is_empty()
        || measured.firmware_id.trim().is_empty()
        || measured.calibration_id.trim().is_empty()
        || measured.design_content_hash.trim().is_empty()
    {
        return Err("online_hardware_identity_missing".into());
    }
    if expected.serial_str() != measured.serial {
        return Err(format!(
            "hardware_serial_mismatch:expected={} actual={}",
            expected.serial_str(),
            measured.serial
        ));
    }
    if expected.firmware_str() != measured.firmware_id {
        return Err(format!(
            "hardware_firmware_mismatch:expected={} actual={}",
            expected.firmware_str(),
            measured.firmware_id
        ));
    }
    if expected.calibration_id_str() != measured.calibration_id {
        return Err(format!(
            "hardware_calibration_mismatch:expected={} actual={}",
            expected.calibration_id_str(),
            measured.calibration_id
        ));
    }
    if expected.design_str() != measured.design_content_hash {
        return Err(format!(
            "hardware_design_mismatch:expected={} actual={}",
            expected.design_str(),
            measured.design_content_hash
        ));
    }
    if !measured.actuator_ids.is_empty() {
        let mut want: Vec<&str> = authorized_actuators.iter().map(String::as_str).collect();
        let mut got: Vec<&str> = measured.actuator_ids.iter().map(String::as_str).collect();
        want.sort_unstable();
        want.dedup();
        got.sort_unstable();
        got.dedup();
        if want != got {
            return Err("hardware_actuator_topology_mismatch".into());
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use realityos_kernel::{CalibrationId, DesignContentHash, FirmwareId, SerialOrAsBuilt};

    fn full() -> RuntimeIdentity {
        RuntimeIdentity {
            release_hash: ReleaseHash::new("rel1").unwrap(),
            design_content_hash: Some(DesignContentHash::new("des1").unwrap()),
            serial_or_as_built: Some(SerialOrAsBuilt::new("SN-1").unwrap()),
            firmware_id: Some(FirmwareId::new("FW-1").unwrap()),
            calibration_id: Some(CalibrationId::new("cal-1").unwrap()),
        }
    }

    fn measured_ok() -> HardwareIdentity {
        HardwareIdentity {
            serial: "SN-1".into(),
            firmware_id: "FW-1".into(),
            calibration_id: "cal-1".into(),
            design_content_hash: "des1".into(),
            connected: true,
            metal: false,
            evidence_status: "TEST_ATTACHED".into(),
            actuator_ids: vec![],
        }
    }

    #[test]
    fn complete_online_refuses_sim_placeholders() {
        let thin = RuntimeIdentity::sim("rel1").unwrap();
        assert!(thin.complete());
        assert!(!thin.complete_online());

        assert!(full().complete_online());
    }

    #[test]
    fn instance_hash_covers_serial_firmware_calibration_actuators() {
        let a = full();
        let acts = vec!["j0".into(), "j1".into()];
        let h = a.instance_hash(&acts);
        let mut b = a.clone();
        b.serial_or_as_built = Some(SerialOrAsBuilt::new("SN-2").unwrap());
        assert_ne!(h, b.instance_hash(&acts));
        b = a.clone();
        b.firmware_id = Some(FirmwareId::new("FW-2").unwrap());
        assert_ne!(h, b.instance_hash(&acts));
        b = a.clone();
        b.calibration_id = Some(CalibrationId::new("cal-2").unwrap());
        assert_ne!(h, b.instance_hash(&acts));
        assert_ne!(h, a.instance_hash(&["j0".into()]));
        let mut shuffled = vec!["j1".into(), "j0".into()];
        shuffled.reverse();
        assert_eq!(h, a.instance_hash(&shuffled));
    }

    #[test]
    fn delimiter_like_values_cannot_collide() {
        let acts: Vec<String> = vec![];
        let mut a = full();
        let mut b = full();
        a.serial_or_as_built = Some(SerialOrAsBuilt::new("a").unwrap());
        a.firmware_id = Some(FirmwareId::new("b|c").unwrap());
        b.serial_or_as_built = Some(SerialOrAsBuilt::new("a|b").unwrap());
        b.firmware_id = Some(FirmwareId::new("c").unwrap());
        assert_ne!(
            canonical_instance_bytes(&a, &acts),
            canonical_instance_bytes(&b, &acts)
        );
        assert_ne!(a.instance_hash(&acts), b.instance_hash(&acts));

        a.serial_or_as_built = Some(SerialOrAsBuilt::new("x\0design=y").unwrap());
        a.firmware_id = Some(FirmwareId::new("z").unwrap());
        b.serial_or_as_built = Some(SerialOrAsBuilt::new("x").unwrap());
        b.firmware_id = Some(FirmwareId::new("y\0serial=z").unwrap());
        assert_ne!(
            canonical_instance_bytes(&a, &acts),
            canonical_instance_bytes(&b, &acts)
        );

        a.serial_or_as_built = Some(SerialOrAsBuilt::new("p\x1fq").unwrap());
        a.firmware_id = Some(FirmwareId::new("r").unwrap());
        b.serial_or_as_built = Some(SerialOrAsBuilt::new("p").unwrap());
        b.firmware_id = Some(FirmwareId::new("q\x1fr").unwrap());
        assert_ne!(
            canonical_instance_bytes(&a, &acts),
            canonical_instance_bytes(&b, &acts)
        );
    }

    #[test]
    fn schema_is_in_canonical_bytes() {
        let bytes = canonical_instance_bytes(&full(), &["j0".into()]);
        assert!(bytes
            .windows(INSTANCE_SCHEMA.len())
            .any(|w| w == INSTANCE_SCHEMA.as_bytes()));
    }

    #[test]
    fn bind_refuses_serial_firmware_calibration_design_mismatch() {
        let exp = full();
        let mut m = measured_ok();
        m.serial = "SN-B".into();
        assert!(ValidatedRuntimeIdentity::bind(exp.clone(), &m, &[]).is_err());
        m = measured_ok();
        m.firmware_id = "FW-B".into();
        assert!(ValidatedRuntimeIdentity::bind(exp.clone(), &m, &[]).is_err());
        m = measured_ok();
        m.calibration_id = "cal-B".into();
        assert!(ValidatedRuntimeIdentity::bind(exp.clone(), &m, &[]).is_err());
        m = measured_ok();
        m.design_content_hash = "des-B".into();
        assert!(ValidatedRuntimeIdentity::bind(exp.clone(), &m, &[]).is_err());
    }

    #[test]
    fn bind_refuses_disconnected_placeholder_and_missing() {
        let exp = full();
        let mut m = measured_ok();
        m.connected = false;
        assert!(match_expected_to_measured(&exp, &m, &[])
            .unwrap_err()
            .contains("disconnected"));
        m = measured_ok();
        m.serial = "SIM_SERIAL".into();
        assert!(match_expected_to_measured(&exp, &m, &[])
            .unwrap_err()
            .contains("placeholder"));
        m = measured_ok();
        m.serial.clear();
        assert!(match_expected_to_measured(&exp, &m, &[])
            .unwrap_err()
            .contains("missing"));
    }

    #[test]
    fn bind_keeps_expected_fields_not_measured() {
        let exp = full();
        let v = ValidatedRuntimeIdentity::bind(exp.clone(), &measured_ok(), &[]).unwrap();
        assert_eq!(v.as_runtime(), &exp);
        assert_eq!(v.as_runtime().serial_str(), "SN-1");
    }

    #[test]
    fn bind_checks_actuator_topology_when_reported() {
        let exp = full();
        let mut m = measured_ok();
        m.actuator_ids = vec!["j1".into()];
        assert!(ValidatedRuntimeIdentity::bind(exp.clone(), &m, &["j0".into()]).is_err());
        assert!(ValidatedRuntimeIdentity::bind(exp, &m, &["j1".into()]).is_ok());
    }
}

#[cfg(test)]
mod canonical_props {
    use super::*;
    use proptest::prelude::*;
    use realityos_kernel::{
        CalibrationId, DesignContentHash, FirmwareId, ReleaseHash, SerialOrAsBuilt,
    };

    fn id(serial: &str, firmware: &str) -> RuntimeIdentity {
        RuntimeIdentity {
            release_hash: ReleaseHash::new("rel").unwrap(),
            design_content_hash: Some(DesignContentHash::new("des").unwrap()),
            serial_or_as_built: Some(SerialOrAsBuilt::new(serial).unwrap()),
            firmware_id: Some(FirmwareId::new(firmware).unwrap()),
            calibration_id: Some(CalibrationId::new("cal").unwrap()),
        }
    }

    proptest! {
        #[test]
        fn adjacent_fields_are_injective(
            a in "[A-Za-z0-9_|\\x00\\x1f]{1,12}",
            b in "[A-Za-z0-9_|\\x00\\x1f]{1,12}",
        ) {
            let left = id(&a, &b);
            let right = id(&format!("{a}{b}"), "x");
            if a != format!("{a}{b}") || b != "x" {
                prop_assert_ne!(
                    canonical_instance_bytes(&left, &[]),
                    canonical_instance_bytes(&right, &[])
                );
            }
            let swapped = id(&b, &a);
            if a != b {
                prop_assert_ne!(
                    canonical_instance_bytes(&left, &[]),
                    canonical_instance_bytes(&swapped, &[])
                );
            }
        }
    }
}
