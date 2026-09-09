//! Metal proof artifact. Aggregates are derived from case measurements.

use serde::{Deserialize, Serialize};

pub const PROOF_SCHEMA: &str = "realityos.metal_proof/1";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum BlockingLayer {
    ProtocolBlocked,
    AuthorizationBlocked,
    EgressBlocked,
    OsBlocked,
    CrashRecoveryBlocked,
    None,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CaseRecord {
    pub name: String,
    pub expected_authorization: bool,
    pub decision_result: String,
    pub writes_before: u64,
    pub writes_after: u64,
    pub write_delta: u64,
    pub device_acknowledgement: bool,
    pub observed_motion: Option<String>,
    pub blocking_layer: BlockingLayer,
    pub journal_result: String,
    pub proposal: String,
    pub unauthorized_write: bool,
}

impl CaseRecord {
    #[allow(clippy::too_many_arguments)]
    pub fn measure(
        name: impl Into<String>,
        proposal: impl Into<String>,
        decision_result: impl Into<String>,
        blocking_layer: BlockingLayer,
        writes_before: u64,
        writes_after: u64,
        expected_authorization: bool,
        device_acknowledgement: bool,
        observed_motion: Option<String>,
        journal_result: impl Into<String>,
    ) -> Self {
        let write_delta = writes_after.saturating_sub(writes_before);
        Self {
            name: name.into(),
            expected_authorization,
            decision_result: decision_result.into(),
            writes_before,
            writes_after,
            write_delta,
            device_acknowledgement,
            observed_motion,
            blocking_layer,
            journal_result: journal_result.into(),
            proposal: proposal.into(),
            unauthorized_write: !expected_authorization && write_delta > 0,
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
pub struct ProofAggregates {
    pub valid_commands: u64,
    pub valid_physical_writes: u64,
    pub hostile_cases: u64,
    pub unauthorized_physical_writes: u64,
    pub identity_mismatch_refusals: u64,
    pub disconnect_refusals: u64,
    pub duplicate_writes_after_restart: u64,
}

pub fn aggregates_from_cases(cases: &[CaseRecord]) -> ProofAggregates {
    let mut a = ProofAggregates::default();
    for c in cases {
        if c.expected_authorization {
            if c.write_delta > 0 {
                a.valid_commands += 1;
                a.valid_physical_writes += c.write_delta;
            }
        } else {
            a.hostile_cases += 1;
            if c.unauthorized_write {
                a.unauthorized_physical_writes += c.write_delta;
            }
            if c.name.contains("identity") || c.name.contains("firmware") {
                a.identity_mismatch_refusals += 1;
            }
            if c.name.contains("disconnect") {
                a.disconnect_refusals += 1;
            }
            if c.blocking_layer == BlockingLayer::CrashRecoveryBlocked
                && c.write_delta > 0
                && c.name.contains("restart")
            {
                a.duplicate_writes_after_restart += c.write_delta;
            }
        }
    }
    a
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MetalProof {
    pub schema: String,
    pub hardware_model: String,
    pub controller_model: String,
    pub real_device_identity: serde_json::Value,
    pub software_commit_sha: String,
    pub authority_uid: String,
    pub autonomy_uid: String,
    pub test_date: String,
    pub valid_commands: u64,
    pub valid_physical_device_writes: u64,
    pub hostile_cases: u64,
    pub unauthorized_physical_device_writes: u64,
    pub direct_device_open_attempts: u64,
    pub direct_device_open_successes: u64,
    pub duplicate_writes_after_restart: u64,
    pub identity_mismatch_refusals: u64,
    pub disconnect_refusals: u64,
    pub hardware_present: bool,
    pub used_os_monotonic_clock: bool,
    pub used_hardware_driver_port: bool,
    pub cutoff_mechanism: String,
    pub cutoff_tested: bool,
    pub sensor_source: String,
    pub device_capture_s: Option<f64>,
    pub authority_receive_s: Option<f64>,
    pub freshness_threshold_s: Option<f64>,
    pub experiment_status: String,
    pub unresolved_assumptions: Vec<String>,
    pub cases: Vec<CaseRecord>,
}

impl MetalProof {
    pub fn from_measured(
        meta: ProofMeta,
        cases: Vec<CaseRecord>,
        unresolved: Vec<String>,
    ) -> Result<Self, String> {
        if !meta.hardware_present {
            return Err("metal_proof_refuses_without_hardware".into());
        }
        if cases.is_empty() {
            return Err("metal_proof_requires_measured_cases".into());
        }
        let a = aggregates_from_cases(&cases);
        Ok(Self {
            schema: PROOF_SCHEMA.into(),
            hardware_model: meta.hardware_model,
            controller_model: meta.controller_model,
            real_device_identity: meta.real_device_identity,
            software_commit_sha: meta.software_commit_sha,
            authority_uid: meta.authority_uid,
            autonomy_uid: meta.autonomy_uid,
            test_date: meta.test_date,
            valid_commands: a.valid_commands,
            valid_physical_device_writes: a.valid_physical_writes,
            hostile_cases: a.hostile_cases,
            unauthorized_physical_device_writes: a.unauthorized_physical_writes,
            direct_device_open_attempts: meta.direct_device_open_attempts,
            direct_device_open_successes: meta.direct_device_open_successes,
            duplicate_writes_after_restart: a.duplicate_writes_after_restart
                + meta.duplicate_writes_after_restart,
            identity_mismatch_refusals: a.identity_mismatch_refusals,
            disconnect_refusals: a.disconnect_refusals,
            hardware_present: true,
            used_os_monotonic_clock: meta.used_os_monotonic_clock,
            used_hardware_driver_port: meta.used_hardware_driver_port,
            cutoff_mechanism: meta.cutoff_mechanism,
            cutoff_tested: meta.cutoff_tested,
            sensor_source: meta.sensor_source,
            device_capture_s: meta.device_capture_s,
            authority_receive_s: meta.authority_receive_s,
            freshness_threshold_s: meta.freshness_threshold_s,
            experiment_status: if a.unauthorized_physical_writes == 0
                && a.valid_physical_writes >= 2
                && meta.direct_device_open_successes == 0
                && meta.direct_device_open_attempts > 0
                && meta.cutoff_tested
                && a.duplicate_writes_after_restart + meta.duplicate_writes_after_restart == 0
            {
                "measured_success".into()
            } else {
                "measured_incomplete_or_failed".into()
            },
            unresolved_assumptions: unresolved,
            cases,
        })
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProofMeta {
    pub hardware_model: String,
    pub controller_model: String,
    pub real_device_identity: serde_json::Value,
    pub software_commit_sha: String,
    pub authority_uid: String,
    pub autonomy_uid: String,
    pub test_date: String,
    pub hardware_present: bool,
    pub used_os_monotonic_clock: bool,
    pub used_hardware_driver_port: bool,
    pub cutoff_mechanism: String,
    pub cutoff_tested: bool,
    pub direct_device_open_attempts: u64,
    pub direct_device_open_successes: u64,
    pub duplicate_writes_after_restart: u64,
    #[serde(default)]
    pub sensor_source: String,
    #[serde(default)]
    pub device_capture_s: Option<f64>,
    #[serde(default)]
    pub authority_receive_s: Option<f64>,
    #[serde(default)]
    pub freshness_threshold_s: Option<f64>,
}

pub fn default_unresolved() -> Vec<String> {
    vec![
        "root can open any endpoint; root is outside this threat model".into(),
        "same-UID chmod or /proc/<pid>/fd can recover a locked device node".into(),
        "journal+seal is a hash-chain / local seal, not WORM or anti-rollback storage".into(),
        "paired restore of an older journal+seal is indistinguishable from that earlier valid tip"
            .into(),
        "filesystem signing.key is not a hardware root of trust / TPM / HSM".into(),
        "XL330 torque-disable is a register write, not an independent power cutoff".into(),
        "USB-serial adapter serial is not a factory actuator serial; the servo EEPROM has none"
            .into(),
        "Realtime Tick is a wrapping 1 ms device counter, not a synchronized clock".into(),
        "no STO/SS1/PLC/SIL/ISO is provided or claimed".into(),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aggregates_come_from_deltas_not_hardcoded_zeros() {
        let cases = vec![
            CaseRecord::measure(
                "valid_hold",
                "hold",
                "write:allow",
                BlockingLayer::None,
                0,
                1,
                true,
                true,
                Some("none".into()),
                "consumed",
            ),
            CaseRecord::measure(
                "valid_nudge",
                "drive",
                "write:allow",
                BlockingLayer::None,
                1,
                2,
                true,
                true,
                Some("ticks=4".into()),
                "consumed",
            ),
            CaseRecord::measure(
                "unsupported",
                "dance",
                "semantic:refuse",
                BlockingLayer::AuthorizationBlocked,
                2,
                2,
                false,
                false,
                None,
                "no_consume",
            ),
        ];
        let a = aggregates_from_cases(&cases);
        assert_eq!(a.valid_physical_writes, 2);
        assert_eq!(a.hostile_cases, 1);
        assert_eq!(a.unauthorized_physical_writes, 0);
    }

    #[test]
    fn proof_refuses_without_hardware() {
        let meta = ProofMeta {
            hardware_model: "none".into(),
            controller_model: "none".into(),
            real_device_identity: serde_json::json!({}),
            software_commit_sha: "x".into(),
            authority_uid: "a".into(),
            autonomy_uid: "b".into(),
            test_date: "t".into(),
            hardware_present: false,
            used_os_monotonic_clock: true,
            used_hardware_driver_port: true,
            cutoff_mechanism: "none".into(),
            cutoff_tested: false,
            direct_device_open_attempts: 0,
            direct_device_open_successes: 0,
            duplicate_writes_after_restart: 0,
            sensor_source: String::new(),
            device_capture_s: None,
            authority_receive_s: None,
            freshness_threshold_s: None,
        };
        assert!(MetalProof::from_measured(meta, vec![], vec![]).is_err());
    }
}
