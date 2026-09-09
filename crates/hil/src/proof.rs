//! HIL proof artifact. Aggregates are derived from case measurements.

use serde::{Deserialize, Serialize};

pub const PROOF_SCHEMA: &str = "realityos.hil_proof/2";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum BlockingLayer {
    CompileTimeBlocked,
    ProtocolBlocked,
    AuthorizationBlocked,
    EgressBlocked,
    OsBlocked,
    CrashRecoveryBlocked,
    None,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct CaseRecord {
    pub name: String,
    pub proposal: String,
    pub semantic_verdict: String,
    pub authority_transition: String,
    pub blocking_layer: BlockingLayer,
    pub writes_before: u64,
    pub writes_after: u64,
    pub write_delta: u64,
    pub expected_authorized: bool,
    pub executed: bool,
    pub driver_write_count: u64,
    pub journal_state: String,
    pub outcome: String,
    pub unauthorized_write: bool,
}

impl CaseRecord {
    #[allow(clippy::too_many_arguments)]
    pub fn measure(
        name: impl Into<String>,
        proposal: impl Into<String>,
        semantic_verdict: impl Into<String>,
        authority_transition: impl Into<String>,
        blocking_layer: BlockingLayer,
        writes_before: u64,
        writes_after: u64,
        expected_authorized: bool,
        executed: bool,
        journal_state: impl Into<String>,
        outcome: impl Into<String>,
    ) -> Self {
        let write_delta = writes_after.saturating_sub(writes_before);
        Self {
            name: name.into(),
            proposal: proposal.into(),
            semantic_verdict: semantic_verdict.into(),
            authority_transition: authority_transition.into(),
            blocking_layer,
            writes_before,
            writes_after,
            write_delta,
            expected_authorized,
            executed,
            driver_write_count: writes_after,
            journal_state: journal_state.into(),
            outcome: outcome.into(),
            unauthorized_write: !expected_authorized && write_delta > 0,
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ProofAggregates {
    pub hostile_cases: u64,
    pub refused_before_authorization: u64,
    pub refused_before_driver_egress: u64,
    pub unauthorized_driver_writes: u64,
    pub valid_commands: u64,
    pub valid_driver_writes: u64,
    pub duplicate_writes_after_restart: u64,
}

pub fn aggregates_from_cases(cases: &[CaseRecord]) -> ProofAggregates {
    let mut a = ProofAggregates::default();
    for c in cases {
        if c.expected_authorized {
            if c.executed {
                a.valid_commands += 1;
                a.valid_driver_writes += c.write_delta;
            }
        } else {
            a.hostile_cases += 1;
            if c.unauthorized_write {
                a.unauthorized_driver_writes += c.write_delta;
            }
            match c.blocking_layer {
                BlockingLayer::ProtocolBlocked
                | BlockingLayer::AuthorizationBlocked
                | BlockingLayer::CompileTimeBlocked => {
                    a.refused_before_authorization += 1;
                }
                BlockingLayer::EgressBlocked => {
                    a.refused_before_driver_egress += 1;
                }
                BlockingLayer::CrashRecoveryBlocked => {
                    if c.write_delta > 0
                        && c.name.contains("crash_")
                        && c.name != "crash_before_prepare"
                    {
                        a.duplicate_writes_after_restart += c.write_delta;
                    }
                }
                BlockingLayer::OsBlocked | BlockingLayer::None => {}
            }
        }
    }
    a
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProofReport {
    pub schema: String,
    pub hostile_cases: u64,
    pub refused_before_authorization: u64,
    pub refused_before_driver_egress: u64,
    pub unauthorized_driver_writes: u64,
    pub valid_commands: u64,
    pub valid_driver_writes: u64,
    pub duplicate_writes_after_restart: u64,
    pub direct_device_open_attempts: u64,
    pub direct_device_open_succeeded: u64,
    pub journal_continuity_failures_detected: u64,
    pub unresolved_trust_assumptions: Vec<String>,
    pub cases: Vec<CaseRecord>,
}

#[derive(Debug, Clone, Default)]
pub struct ProofExtras {
    pub direct_device_open_attempts: u64,
    pub direct_device_open_succeeded: u64,
    pub journal_continuity_failures_detected: u64,
    pub unresolved_trust_assumptions: Vec<String>,
}

impl ProofReport {
    pub fn default_assumptions() -> Vec<String> {
        vec![
            "same-UID chmod or /proc/<pid>/fd can recover a mode-000 log".into(),
            "root can open any endpoint; root is outside this threat model".into(),
            "journal+seal is a hash-chain / local seal, not authenticated persistence, anti-rollback secure storage, or WORM".into(),
            "paired restore of an older journal+seal is indistinguishable from that earlier valid tip".into(),
            "independent STO/SS1 / safety PLC is a named hole; this is semantic execution authority only".into(),
            "HIL sensor samples may be synthetic; production freshness uses authority_receive_monotonic".into(),
            "multi-user OS identities are a deployment/HIL requirement, not a unit-test guarantee".into(),
        ]
    }

    /// Construction requires measured cases. Aggregates are recomputed, then checked.
    pub fn from_measured_cases(
        cases: Vec<CaseRecord>,
        extras: ProofExtras,
    ) -> Result<Self, String> {
        if cases.is_empty() {
            return Err("proof_requires_case_measurements".into());
        }
        let a = aggregates_from_cases(&cases);
        let report = Self {
            schema: PROOF_SCHEMA.into(),
            hostile_cases: a.hostile_cases,
            refused_before_authorization: a.refused_before_authorization,
            refused_before_driver_egress: a.refused_before_driver_egress,
            unauthorized_driver_writes: a.unauthorized_driver_writes,
            valid_commands: a.valid_commands,
            valid_driver_writes: a.valid_driver_writes,
            duplicate_writes_after_restart: a.duplicate_writes_after_restart,
            direct_device_open_attempts: extras.direct_device_open_attempts,
            direct_device_open_succeeded: extras.direct_device_open_succeeded,
            journal_continuity_failures_detected: extras.journal_continuity_failures_detected,
            unresolved_trust_assumptions: extras.unresolved_trust_assumptions,
            cases,
        };
        verify_proof_consistency(&report)?;
        Ok(report)
    }
}

pub fn verify_proof_consistency(report: &ProofReport) -> Result<(), String> {
    let a = aggregates_from_cases(&report.cases);
    let checks = [
        ("hostile_cases", report.hostile_cases, a.hostile_cases),
        (
            "refused_before_authorization",
            report.refused_before_authorization,
            a.refused_before_authorization,
        ),
        (
            "refused_before_driver_egress",
            report.refused_before_driver_egress,
            a.refused_before_driver_egress,
        ),
        (
            "unauthorized_driver_writes",
            report.unauthorized_driver_writes,
            a.unauthorized_driver_writes,
        ),
        ("valid_commands", report.valid_commands, a.valid_commands),
        (
            "valid_driver_writes",
            report.valid_driver_writes,
            a.valid_driver_writes,
        ),
        (
            "duplicate_writes_after_restart",
            report.duplicate_writes_after_restart,
            a.duplicate_writes_after_restart,
        ),
    ];
    for (name, reported, recomputed) in checks {
        if reported != recomputed {
            return Err(format!(
                "proof_aggregate_mismatch:{name}:reported={reported} recomputed={recomputed}"
            ));
        }
    }
    for c in &report.cases {
        let expect_unauth = !c.expected_authorized && c.write_delta > 0;
        if c.unauthorized_write != expect_unauth {
            return Err(format!("proof_case_unauthorized_mismatch:{}", c.name));
        }
        if c.write_delta != c.writes_after.saturating_sub(c.writes_before) {
            return Err(format!("proof_case_delta_mismatch:{}", c.name));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aggregates_are_derived_not_assigned() {
        let cases = vec![
            CaseRecord::measure(
                "valid",
                "hold",
                "allow",
                "write",
                BlockingLayer::None,
                0,
                1,
                true,
                true,
                "consumed",
                "ok",
            ),
            CaseRecord::measure(
                "hostile",
                "dance",
                "refuse",
                "semantic",
                BlockingLayer::AuthorizationBlocked,
                1,
                1,
                false,
                false,
                "no_consume",
                "refused",
            ),
            CaseRecord::measure(
                "leak",
                "bad",
                "allow",
                "write",
                BlockingLayer::EgressBlocked,
                1,
                3,
                false,
                true,
                "consumed",
                "ok",
            ),
        ];
        let extras = ProofExtras {
            unresolved_trust_assumptions: ProofReport::default_assumptions(),
            ..ProofExtras::default()
        };
        let report = ProofReport::from_measured_cases(cases, extras).unwrap();
        assert_eq!(report.valid_driver_writes, 1);
        assert_eq!(report.unauthorized_driver_writes, 2);
        assert!(!report.cases[1].unauthorized_write);
        assert!(report.cases[2].unauthorized_write);
        verify_proof_consistency(&report).unwrap();
    }

    #[test]
    fn empty_cases_cannot_build_a_proof() {
        assert!(ProofReport::from_measured_cases(vec![], ProofExtras::default()).is_err());
    }
}
