//! Episode evidence from measured simulation state. Never metal proof.

use crate::honesty::{
    refuse_physical_proof_origin, VerificationClass, EVIDENCE_SCHEMA, REPORT_SCHEMA,
    SIMULATION_ONLY,
};
use crate::normalize::RobotManifest;
use crate::observation::VerifierTruth;
use crate::scenario::ResolvedScenario;
use crate::verifier::{RuntimeViolation, VerifierStats, ViolationKind};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::fs;
use std::path::Path;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EpisodeEvidence {
    pub schema: String,
    pub verification_class: String,
    pub metal: bool,
    pub metal_verified: bool,
    pub functional_safety_certified: bool,
    pub evidence_status: String,
    pub robot_id: String,
    pub robot_hash: String,
    pub source_model: String,
    pub mujoco_version: String,
    pub software_version: String,
    pub scenario: String,
    pub family: String,
    pub resolved_parameters: std::collections::BTreeMap<String, f64>,
    pub seed: u64,
    pub policy_id: String,
    pub task: String,
    pub authority_decisions: Vec<crate::authority::AuthorityRecord>,
    pub commands_executed: u32,
    pub observations: u32,
    pub initial_state: Value,
    pub final_state: Value,
    pub task_success: bool,
    pub not_applicable: bool,
    pub authority_refusals: u32,
    pub violations: Vec<RuntimeViolation>,
    pub contacts_of_interest: Vec<Value>,
    pub max_joint_speed: f64,
    pub max_effort: f64,
    pub max_contact_force: f64,
    pub minimum_obstacle_distance: Option<f64>,
    pub termination_reason: String,
    pub simulation_duration_s: f64,
    pub wall_clock_duration_s: f64,
    pub ctrl_writes: u64,
}

impl EpisodeEvidence {
    pub fn to_json_value(&self) -> Value {
        let v = serde_json::to_value(self).unwrap_or(Value::Null);
        let _ = refuse_physical_proof_origin(&v);
        v
    }

    pub fn write(&self, path: impl AsRef<Path>) -> Result<(), String> {
        let v = self.to_json_value();
        refuse_physical_proof_origin(&v)?;
        fs::write(
            path,
            serde_json::to_string_pretty(&v).map_err(|e| e.to_string())?,
        )
        .map_err(|e| e.to_string())
    }
}

fn finite_clearance(d: f64) -> Option<f64> {
    if d.is_finite() && d < 1.0e100 {
        Some(d)
    } else {
        None
    }
}

#[allow(clippy::too_many_arguments)]
pub fn assemble_episode(
    manifest: &RobotManifest,
    resolved: &ResolvedScenario,
    policy_id: &str,
    initial: &VerifierTruth,
    final_state: &VerifierTruth,
    decisions: Vec<crate::authority::AuthorityRecord>,
    violations: Vec<RuntimeViolation>,
    stats: &VerifierStats,
    task_success: bool,
    not_applicable: bool,
    termination: &str,
    sim_s: f64,
    wall_s: f64,
    writes: u64,
    observations: u32,
) -> EpisodeEvidence {
    let authority_refusals = decisions.iter().filter(|d| !d.executed).count() as u32;
    EpisodeEvidence {
        schema: EVIDENCE_SCHEMA.into(),
        verification_class: VerificationClass::SimulationVerified.as_str().into(),
        metal: false,
        metal_verified: false,
        functional_safety_certified: false,
        evidence_status: SIMULATION_ONLY.into(),
        robot_id: manifest.robot_id.clone(),
        robot_hash: manifest.model_hash.clone(),
        source_model: manifest.source_format.clone(),
        mujoco_version: manifest.mujoco_version.clone(),
        software_version: env!("CARGO_PKG_VERSION").into(),
        scenario: resolved.spec_id.clone(),
        family: resolved.family.clone(),
        resolved_parameters: resolved.resolved.clone(),
        seed: resolved.seed,
        policy_id: policy_id.into(),
        task: resolved.task.id(),
        authority_decisions: decisions,
        commands_executed: writes as u32,
        observations,
        initial_state: serde_json::to_value(initial).unwrap_or(Value::Null),
        final_state: serde_json::to_value(final_state).unwrap_or(Value::Null),
        task_success,
        not_applicable,
        authority_refusals,
        contacts_of_interest: final_state
            .contacts
            .iter()
            .map(|c| serde_json::to_value(c).unwrap_or(Value::Null))
            .collect(),
        violations,
        max_joint_speed: stats.max_joint_speed,
        max_effort: stats.max_effort,
        max_contact_force: stats.max_contact_force,
        minimum_obstacle_distance: finite_clearance(stats.min_obstacle_distance),
        termination_reason: termination.into(),
        simulation_duration_s: sim_s,
        wall_clock_duration_s: wall_s,
        ctrl_writes: writes,
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct FamilySlice {
    pub robot: String,
    pub family: String,
    pub policy: String,
    pub episodes: u64,
    pub not_applicable: u64,
    pub task_success_rate: f64,
    pub authority_refusal_rate: f64,
    pub safe_completion_rate: f64,
    pub physical_invariant_violation_rate: f64,
    pub crash_rate: f64,
    pub nan_divergence_rate: f64,
    pub worst_joint_velocity: f64,
    pub worst_effort: f64,
    pub worst_contact: f64,
    pub minimum_clearance: Option<f64>,
    pub worst_seed: Option<u64>,
    pub failed_seeds: Vec<u64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VerificationReport {
    pub schema: String,
    pub verification_class: String,
    pub metal: bool,
    pub metal_verified: bool,
    pub functional_safety_certified: bool,
    pub evidence_status: String,
    pub mujoco_version: String,
    pub software_version: String,
    pub slices: Vec<FamilySlice>,
    pub total_episodes: u64,
    pub failed_episode_seeds: Vec<(String, String, u64)>,
}

pub fn aggregate(episodes: &[EpisodeEvidence]) -> VerificationReport {
    let mut slices: std::collections::BTreeMap<(String, String, String), Vec<&EpisodeEvidence>> =
        std::collections::BTreeMap::new();
    for e in episodes {
        slices
            .entry((e.robot_id.clone(), e.family.clone(), e.policy_id.clone()))
            .or_default()
            .push(e);
    }
    let mut out_slices = Vec::new();
    let mut failed = Vec::new();
    for ((robot, family, policy), group) in slices {
        let applicable: Vec<_> = group.iter().filter(|e| !e.not_applicable).collect();
        let an = applicable.len() as f64;
        let rate = |n: f64| if an == 0.0 { 0.0 } else { n / an };
        let success = rate(applicable.iter().filter(|e| e.task_success).count() as f64);
        let refuse = rate(
            applicable
                .iter()
                .filter(|e| e.authority_refusals > 0)
                .count() as f64,
        );
        let phys = rate(
            applicable
                .iter()
                .filter(|e| {
                    e.violations
                        .iter()
                        .any(|v| v.kind == ViolationKind::PhysicalInvariantViolation)
                })
                .count() as f64,
        );
        let crash = rate(
            applicable
                .iter()
                .filter(|e| {
                    e.termination_reason.contains("POLICY_CRASH")
                        || e.termination_reason.contains("crash")
                })
                .count() as f64,
        );
        let nan = rate(
            applicable
                .iter()
                .filter(|e| {
                    e.violations
                        .iter()
                        .any(|v| v.code == "NAN_STATE" || v.code == "DIVERGENT_SIMULATION")
                })
                .count() as f64,
        );
        let safe = rate(
            applicable
                .iter()
                .filter(|e| {
                    !e.violations
                        .iter()
                        .any(|v| v.kind == ViolationKind::PhysicalInvariantViolation)
                        && e.termination_reason != "NAN_STATE"
                })
                .count() as f64,
        );
        let mut failed_seeds = Vec::new();
        for e in &applicable {
            let failed_ep = !e.task_success
                && !matches!(
                    e.family.as_str(),
                    "COMMAND_REPLAY"
                        | "DUPLICATE_COMMAND"
                        | "WRONG_ROBOT_IDENTITY"
                        | "WRONG_TASK_AUTHORITY"
                        | "POLICY_CRASH"
                        | "AUTHORITY_RESTART"
                        | "STALE_OBSERVATION"
                )
                || e.violations.iter().any(|v| {
                    v.kind == ViolationKind::PhysicalInvariantViolation && v.code == "NAN_STATE"
                });
            // Authority-negative families succeed when they refuse and do not actuate.
            let auth_neg_ok = matches!(
                e.family.as_str(),
                "COMMAND_REPLAY"
                    | "DUPLICATE_COMMAND"
                    | "WRONG_ROBOT_IDENTITY"
                    | "WRONG_TASK_AUTHORITY"
                    | "POLICY_CRASH"
                    | "AUTHORITY_RESTART"
                    | "STALE_OBSERVATION"
            ) && e.ctrl_writes == 0
                && e.authority_refusals > 0;
            if failed_ep && !auth_neg_ok && !e.task_success {
                failed_seeds.push(e.seed);
                failed.push((robot.clone(), family.clone(), e.seed));
            }
        }
        let worst = applicable.iter().max_by(|a, b| {
            a.max_joint_speed
                .partial_cmp(&b.max_joint_speed)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        out_slices.push(FamilySlice {
            robot,
            family,
            policy,
            episodes: group.len() as u64,
            not_applicable: group.iter().filter(|e| e.not_applicable).count() as u64,
            task_success_rate: success,
            authority_refusal_rate: refuse,
            safe_completion_rate: safe,
            physical_invariant_violation_rate: phys,
            crash_rate: crash,
            nan_divergence_rate: nan,
            worst_joint_velocity: worst.map(|e| e.max_joint_speed).unwrap_or(0.0),
            worst_effort: worst.map(|e| e.max_effort).unwrap_or(0.0),
            worst_contact: worst.map(|e| e.max_contact_force).unwrap_or(0.0),
            minimum_clearance: applicable
                .iter()
                .filter_map(|e| e.minimum_obstacle_distance)
                .fold(None, |acc, d| Some(acc.map(|a: f64| a.min(d)).unwrap_or(d))),
            worst_seed: worst.map(|e| e.seed),
            failed_seeds,
        });
    }
    VerificationReport {
        schema: REPORT_SCHEMA.into(),
        verification_class: VerificationClass::SimulationVerified.as_str().into(),
        metal: false,
        metal_verified: false,
        functional_safety_certified: false,
        evidence_status: SIMULATION_ONLY.into(),
        mujoco_version: episodes
            .first()
            .map(|e| e.mujoco_version.clone())
            .unwrap_or_default(),
        software_version: env!("CARGO_PKG_VERSION").into(),
        slices: out_slices,
        total_episodes: episodes.len() as u64,
        failed_episode_seeds: failed,
    }
}

pub fn render_markdown(report: &VerificationReport) -> String {
    let mut md = String::from("# SIMULATION VERIFICATION REPORT\n\n");
    md.push_str("**SIMULATION VERIFIED** — this is not **METAL VERIFIED** and not **FUNCTIONAL-SAFETY CERTIFIED**.\n\n");
    md.push_str(&format!(
        "schema: `{}`  \nevidence_status: `{}`  \nmetal: false  \nMuJoCo: {}  \nepisodes: {}\n\n",
        report.schema, report.evidence_status, report.mujoco_version, report.total_episodes
    ));
    md.push_str("| robot | family | policy | n | na | success | refuse | safe | phys | crash | nan | worst | failed seeds |\n");
    md.push_str("|---|---|---|---|---|---|---|---|---|---|---|---|---|\n");
    for s in &report.slices {
        md.push_str(&format!(
            "| {} | {} | {} | {} | {} | {:.3} | {:.3} | {:.3} | {:.3} | {:.3} | {:.3} | {} | {:?} |\n",
            s.robot,
            s.family,
            s.policy,
            s.episodes,
            s.not_applicable,
            s.task_success_rate,
            s.authority_refusal_rate,
            s.safe_completion_rate,
            s.physical_invariant_violation_rate,
            s.crash_rate,
            s.nan_divergence_rate,
            s.worst_seed
                .map(|x| x.to_string())
                .unwrap_or_else(|| "-".into()),
            s.failed_seeds
        ));
    }
    md.push_str("\nFailures are listed by seed. Averages do not hide them.\n");
    md
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn evidence_cannot_be_metal_proof() {
        let e = EpisodeEvidence {
            schema: EVIDENCE_SCHEMA.into(),
            verification_class: "SIMULATION_VERIFIED".into(),
            metal: false,
            metal_verified: false,
            functional_safety_certified: false,
            evidence_status: SIMULATION_ONLY.into(),
            robot_id: "a".into(),
            robot_hash: "h".into(),
            source_model: "mjcf".into(),
            mujoco_version: "3".into(),
            software_version: "0".into(),
            scenario: "s".into(),
            family: "JOINT_TRACKING".into(),
            resolved_parameters: Default::default(),
            seed: 1,
            policy_id: "pd".into(),
            task: "t".into(),
            authority_decisions: vec![],
            commands_executed: 0,
            observations: 0,
            initial_state: Value::Null,
            final_state: Value::Null,
            task_success: false,
            not_applicable: false,
            authority_refusals: 0,
            violations: vec![],
            contacts_of_interest: vec![],
            max_joint_speed: 0.0,
            max_effort: 0.0,
            max_contact_force: 0.0,
            minimum_obstacle_distance: Some(1.0),
            termination_reason: "end".into(),
            simulation_duration_s: 0.1,
            wall_clock_duration_s: 0.1,
            ctrl_writes: 0,
        };
        let v = e.to_json_value();
        assert!(refuse_physical_proof_origin(&v).is_ok());
        assert_ne!(v["schema"], crate::honesty::METAL_PROOF_SCHEMA);
        let mut evil = v.clone();
        evil["schema"] = Value::from(crate::honesty::METAL_PROOF_SCHEMA);
        assert!(refuse_physical_proof_origin(&evil).is_err());
    }
}
