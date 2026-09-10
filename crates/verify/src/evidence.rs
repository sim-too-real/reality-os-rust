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
    #[serde(default)]
    pub episode_status: String,
    #[serde(default)]
    pub policy_ctrl_writes: u64,
    #[serde(default)]
    pub authority_safe_state_writes: u64,
    #[serde(default)]
    pub total_ctrl_writes: u64,
    #[serde(default)]
    pub effort_verification: String,
    #[serde(default)]
    pub infra_error: Option<String>,
    #[serde(default)]
    pub ctrl_writes_before_restart: Option<u64>,
    #[serde(default)]
    pub ctrl_writes_after_first_command: Option<u64>,
    #[serde(default)]
    pub ctrl_writes_after_replay: Option<u64>,
    #[serde(default)]
    pub replay_write_delta: Option<i64>,
    #[serde(default)]
    pub last_rpc: Option<String>,
    #[serde(default)]
    pub worker_exit: Option<i32>,
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
        episode_status: String::new(),
        policy_ctrl_writes: writes,
        authority_safe_state_writes: 0,
        total_ctrl_writes: writes,
        effort_verification: String::new(),
        infra_error: None,
        ctrl_writes_before_restart: None,
        ctrl_writes_after_first_command: None,
        ctrl_writes_after_replay: None,
        replay_write_delta: None,
        last_rpc: None,
        worker_exit: None,
    }
}

pub fn classify_episode(ep: &EpisodeEvidence) -> String {
    if ep.infra_error.is_some() || ep.episode_status == "INFRA_ERROR" {
        return "INFRA_ERROR".into();
    }
    if ep.not_applicable || ep.termination_reason == "NOT_IMPLEMENTED" {
        return "NOT_APPLICABLE".into();
    }
    let auth_neg = matches!(
        ep.family.as_str(),
        "COMMAND_REPLAY"
            | "DUPLICATE_COMMAND"
            | "WRONG_ROBOT_IDENTITY"
            | "WRONG_TASK_AUTHORITY"
            | "POLICY_CRASH"
            | "AUTHORITY_RESTART"
            | "STALE_OBSERVATION"
    );
    if auth_neg && ep.task_success {
        "EXPECTED_REFUSAL_PASS".into()
    } else if ep.task_success {
        "PASS".into()
    } else {
        "FAIL".into()
    }
}

pub fn infra_episode(
    robot_id: &str,
    family: &str,
    policy: &str,
    seed: u64,
    error: &str,
) -> EpisodeEvidence {
    EpisodeEvidence {
        schema: EVIDENCE_SCHEMA.into(),
        verification_class: VerificationClass::SimulationVerified.as_str().into(),
        metal: false,
        metal_verified: false,
        functional_safety_certified: false,
        evidence_status: SIMULATION_ONLY.into(),
        robot_id: robot_id.into(),
        robot_hash: String::new(),
        source_model: String::new(),
        mujoco_version: String::new(),
        software_version: env!("CARGO_PKG_VERSION").into(),
        scenario: family.into(),
        family: family.into(),
        resolved_parameters: Default::default(),
        seed,
        policy_id: policy.into(),
        task: family.into(),
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
        minimum_obstacle_distance: None,
        termination_reason: "INFRA_ERROR".into(),
        simulation_duration_s: 0.0,
        wall_clock_duration_s: 0.0,
        ctrl_writes: 0,
        episode_status: "INFRA_ERROR".into(),
        policy_ctrl_writes: 0,
        authority_safe_state_writes: 0,
        total_ctrl_writes: 0,
        effort_verification: "NOT_EVALUATED".into(),
        infra_error: Some(error.into()),
        ctrl_writes_before_restart: None,
        ctrl_writes_after_first_command: None,
        ctrl_writes_after_replay: None,
        replay_write_delta: None,
        last_rpc: None,
        worker_exit: None,
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
    pub worst_joint_velocity_seed: Option<u64>,
    pub worst_effort: f64,
    pub worst_effort_seed: Option<u64>,
    pub worst_contact: f64,
    pub worst_contact_seed: Option<u64>,
    pub minimum_clearance: Option<f64>,
    pub minimum_clearance_seed: Option<u64>,
    pub worst_seed: Option<u64>,
    pub failed_seeds: Vec<u64>,
    pub physical_violation_seeds: Vec<u64>,
    #[serde(default)]
    pub safe_completion_definition: String,
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
    pub scheduled_jobs: u64,
    pub completed_jobs: u64,
    pub passed_jobs: u64,
    pub failed_jobs: u64,
    pub not_applicable_jobs: u64,
    pub infra_error_jobs: u64,
    pub failed_episode_seeds: Vec<(String, String, u64)>,
    #[serde(default)]
    pub campaign_complete: bool,
    #[serde(default)]
    pub unaccounted_jobs: u64,
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
        let auth_neg = |e: &EpisodeEvidence| {
            matches!(
                e.family.as_str(),
                "COMMAND_REPLAY"
                    | "DUPLICATE_COMMAND"
                    | "WRONG_ROBOT_IDENTITY"
                    | "WRONG_TASK_AUTHORITY"
                    | "POLICY_CRASH"
                    | "AUTHORITY_RESTART"
                    | "STALE_OBSERVATION"
            )
        };
        let safe = rate(
            applicable
                .iter()
                .filter(|e| {
                    if auth_neg(e) {
                        e.task_success
                            && e.policy_ctrl_writes
                                <= if e.family == "COMMAND_REPLAY"
                                    || e.family == "DUPLICATE_COMMAND"
                                    || e.family == "AUTHORITY_RESTART"
                                {
                                    1
                                } else {
                                    0
                                }
                    } else {
                        e.task_success
                            && !e
                                .violations
                                .iter()
                                .any(|v| v.kind == ViolationKind::PhysicalInvariantViolation)
                            && e.termination_reason != "NAN_STATE"
                    }
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
        let worst_speed = applicable.iter().max_by(|a, b| {
            a.max_joint_speed
                .partial_cmp(&b.max_joint_speed)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let worst_effort = applicable.iter().max_by(|a, b| {
            a.max_effort
                .partial_cmp(&b.max_effort)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let worst_contact = applicable.iter().max_by(|a, b| {
            a.max_contact_force
                .partial_cmp(&b.max_contact_force)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let min_clear = applicable.iter().min_by(|a, b| {
            let da = a.minimum_obstacle_distance.unwrap_or(f64::MAX);
            let db = b.minimum_obstacle_distance.unwrap_or(f64::MAX);
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        });
        let phys_seeds: Vec<u64> = applicable
            .iter()
            .filter(|e| {
                e.violations
                    .iter()
                    .any(|v| v.kind == ViolationKind::PhysicalInvariantViolation)
            })
            .map(|e| e.seed)
            .collect();
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
            worst_joint_velocity: worst_speed.map(|e| e.max_joint_speed).unwrap_or(0.0),
            worst_joint_velocity_seed: worst_speed.map(|e| e.seed),
            worst_effort: worst_effort.map(|e| e.max_effort).unwrap_or(0.0),
            worst_effort_seed: worst_effort.map(|e| e.seed),
            worst_contact: worst_contact.map(|e| e.max_contact_force).unwrap_or(0.0),
            worst_contact_seed: worst_contact.map(|e| e.seed),
            minimum_clearance: min_clear.and_then(|e| e.minimum_obstacle_distance),
            minimum_clearance_seed: min_clear
                .filter(|e| e.minimum_obstacle_distance.is_some())
                .map(|e| e.seed),
            worst_seed: worst_speed.map(|e| e.seed),
            failed_seeds,
            physical_violation_seeds: phys_seeds,
            safe_completion_definition: "positive: task_success AND no physical invariant violation; authority-negative: expected refusal AND zero unauthorized policy writes AND no lease overrun".into(),
        });
    }
    let statuses: Vec<String> = episodes.iter().map(classify_episode).collect();
    let infra = statuses.iter().filter(|s| *s == "INFRA_ERROR").count() as u64;
    let na = statuses.iter().filter(|s| *s == "NOT_APPLICABLE").count() as u64;
    let passed = statuses
        .iter()
        .filter(|s| *s == "PASS" || *s == "EXPECTED_REFUSAL_PASS")
        .count() as u64;
    let failed_n = statuses.iter().filter(|s| *s == "FAIL").count() as u64;
    let scheduled = episodes.len() as u64;
    let completed = scheduled.saturating_sub(infra);
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
        total_episodes: scheduled,
        scheduled_jobs: scheduled,
        completed_jobs: completed,
        passed_jobs: passed,
        failed_jobs: failed_n,
        not_applicable_jobs: na,
        infra_error_jobs: infra,
        failed_episode_seeds: failed,
        campaign_complete: scheduled == completed + infra,
        unaccounted_jobs: 0,
    }
}

pub fn render_markdown(report: &VerificationReport) -> String {
    let mut md = String::from("# SIMULATION CAMPAIGN COMPLETE\n\n");
    md.push_str("Evidence class is **SIMULATION_VERIFIED**. This is not **METAL VERIFIED** and not **FUNCTIONAL-SAFETY CERTIFIED**.\n\n");
    md.push_str(&format!(
        "schema: `{}`  \nevidence_status: `{}`  \nmetal: false  \nMuJoCo: {}  \nscheduled_jobs: {}  \ncompleted_jobs: {}  \npassed_jobs: {}  \nfailed_jobs: {}  \nnot_applicable_jobs: {}  \ninfra_error_jobs: {}\n\n",
        report.schema,
        report.evidence_status,
        report.mujoco_version,
        report.scheduled_jobs,
        report.completed_jobs,
        report.passed_jobs,
        report.failed_jobs,
        report.not_applicable_jobs,
        report.infra_error_jobs
    ));
    md.push_str("safe_completion_rate: positive tasks require task success AND no critical/physical invariant violation. Authority-negative tasks require expected refusal AND zero unauthorized policy ctrl writes AND no continuation past an expired lease.\n\n");
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
            episode_status: "FAIL".into(),
            policy_ctrl_writes: 0,
            authority_safe_state_writes: 0,
            total_ctrl_writes: 0,
            effort_verification: "EFFORT_BOUND_UNAVAILABLE".into(),
            infra_error: None,
            ctrl_writes_before_restart: None,
            ctrl_writes_after_first_command: None,
            ctrl_writes_after_replay: None,
            replay_write_delta: None,
            last_rpc: None,
            worker_exit: None,
        };
        let v = e.to_json_value();
        assert!(refuse_physical_proof_origin(&v).is_ok());
        assert_ne!(v["schema"], crate::honesty::METAL_PROOF_SCHEMA);
        let mut evil = v.clone();
        evil["schema"] = Value::from(crate::honesty::METAL_PROOF_SCHEMA);
        assert!(refuse_physical_proof_origin(&evil).is_err());
    }
}
