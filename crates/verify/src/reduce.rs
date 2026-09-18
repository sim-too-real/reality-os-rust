//! Counterexample minimization: shrink a failing resolved scenario.

use crate::evidence::EpisodeEvidence;
use crate::scenario::{ResolvedScenario, ScenarioSpec};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MinimalCounterexample {
    pub original_seed: u64,
    pub family: String,
    pub robot_id: String,
    pub reduced: std::collections::BTreeMap<String, f64>,
    pub fewer_objects: usize,
    pub earlier_fail_s: Option<f64>,
    pub narrative: String,
    pub reproducible_from: String,
}

pub fn minimize(
    spec: &ScenarioSpec,
    evidence: &EpisodeEvidence,
    reeval: impl Fn(&ResolvedScenario) -> Option<EpisodeEvidence>,
) -> MinimalCounterexample {
    let mut resolved = spec.resolve(evidence.seed);
    let original_n = resolved.objects.len();
    if resolved.objects.len() > 1 {
        let mut shrunk = resolved.clone();
        shrunk.objects.truncate(1);
        if let Some(e) = reeval(&shrunk) {
            if still_fails(&e) {
                resolved = shrunk;
                let _ = e;
            }
        }
    }
    let keys: Vec<String> = resolved.resolved.keys().cloned().collect();
    for k in keys {
        let Some(v) = resolved.resolved.get(&k).copied() else {
            continue;
        };
        for scale in [0.5, 0.25, 0.1] {
            let mut trial = resolved.clone();
            trial.resolved.insert(k.clone(), v * scale);
            apply_resolved_to_task(&mut trial);
            if let Some(e) = reeval(&trial) {
                if still_fails(&e) {
                    resolved = trial;
                    break;
                }
            }
        }
    }
    let earlier = evidence
        .violations
        .iter()
        .map(|v| v.time_s)
        .fold(None, |acc: Option<f64>, t| {
            Some(acc.map(|a| a.min(t)).unwrap_or(t))
        });
    let narrative = format!(
        "scenario {} failed; reduced to {:?}; first violation {:?}; ctrl_writes={}; authority_refusals={}. Reproducible from robot hash {} + scenario {} + seed {}.",
        evidence.scenario,
        resolved.resolved,
        evidence.violations.first().map(|v| &v.code),
        evidence.ctrl_writes,
        evidence.authority_refusals,
        evidence.robot_hash,
        evidence.family,
        evidence.seed
    );
    MinimalCounterexample {
        original_seed: evidence.seed,
        family: evidence.family.clone(),
        robot_id: evidence.robot_id.clone(),
        reduced: resolved.resolved,
        fewer_objects: original_n.saturating_sub(resolved.objects.len()),
        earlier_fail_s: earlier,
        narrative,
        reproducible_from: format!(
            "robot_hash={} scenario={} seed={}",
            evidence.robot_hash, evidence.family, evidence.seed
        ),
    }
}

fn apply_resolved_to_task(resolved: &mut ResolvedScenario) {
    if let crate::task::TaskSpec::JointTrack { target, .. } = &mut resolved.task {
        for (i, t) in target.iter_mut().enumerate() {
            if let Some(v) = resolved.resolved.get(&format!("q{i}")) {
                *t = *v;
            }
        }
    }
    if let crate::task::TaskSpec::Reach { target, .. } = &mut resolved.task {
        if let Some(x) = resolved.resolved.get("target.x") {
            target[0] = *x;
        }
        if let Some(y) = resolved.resolved.get("target.y") {
            target[1] = *y;
        }
        if let Some(z) = resolved.resolved.get("target.z") {
            target[2] = *z;
        }
    }
}

/// Shrink a failed manipulation episode over pose/mass/friction/push/grasp offset.
pub fn minimize_manipulation(
    seed: u64,
    robot_id: &str,
    model_hash: &str,
    params: &std::collections::BTreeMap<String, f64>,
    still_fails: impl Fn(&std::collections::BTreeMap<String, f64>) -> bool,
) -> MinimalCounterexample {
    let mut reduced = params.clone();
    for key in [
        "object.x",
        "object.y",
        "object.z",
        "mass",
        "friction",
        "push.distance",
        "grasp.offset",
    ] {
        if let Some(v) = reduced.get(key).copied() {
            for scale in [0.5, 0.25, 0.1] {
                let mut trial = reduced.clone();
                trial.insert(key.into(), v * scale);
                if still_fails(&trial) {
                    reduced = trial;
                    break;
                }
            }
        }
    }
    MinimalCounterexample {
        original_seed: seed,
        family: "MANIPULATION".into(),
        robot_id: robot_id.into(),
        reduced: reduced.clone(),
        fewer_objects: 0,
        earlier_fail_s: None,
        narrative: format!("manipulation seed {seed} reduced to {reduced:?}; robot={robot_id}"),
        reproducible_from: format!("robot_hash={model_hash} seed={seed}"),
    }
}

fn still_fails(e: &EpisodeEvidence) -> bool {
    !e.task_success
        || e.violations.iter().any(|v| v.code == "NAN_STATE")
        || (e.family.contains("WRONG") && e.ctrl_writes > 0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::honesty::{EVIDENCE_SCHEMA, SIMULATION_ONLY};
    use crate::task::TaskSpec;

    #[test]
    fn reducer_records_seed_and_resolved() {
        let spec = crate::families::family_spec("JOINT_TRACKING", 1, "ee");
        let ev = EpisodeEvidence {
            schema: EVIDENCE_SCHEMA.into(),
            verification_class: "SIMULATION_VERIFIED".into(),
            metal: false,
            metal_verified: false,
            functional_safety_certified: false,
            evidence_status: SIMULATION_ONLY.into(),
            robot_id: "planar_arm".into(),
            robot_hash: "abc".into(),
            source_model: "mjcf".into(),
            mujoco_version: "3".into(),
            software_version: "0".into(),
            scenario: "joint_tracking".into(),
            family: "JOINT_TRACKING".into(),
            resolved_parameters: spec.resolve(18472).resolved.clone(),
            seed: 18472,
            policy_id: "pd".into(),
            task: TaskSpec::Hold { duration_s: 0.1 }.id(),
            authority_decisions: vec![],
            commands_executed: 0,
            observations: 1,
            initial_state: serde_json::Value::Null,
            final_state: serde_json::Value::Null,
            task_success: false,
            not_applicable: false,
            authority_refusals: 1,
            violations: vec![],
            contacts_of_interest: vec![],
            max_joint_speed: 0.0,
            max_effort: 0.0,
            max_contact_force: 0.0,
            minimum_obstacle_distance: Some(1.0),
            termination_reason: "end".into(),
            simulation_duration_s: 0.1,
            wall_clock_duration_s: 0.0,
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
        let m = minimize(&spec, &ev, |_| None);
        assert_eq!(m.original_seed, 18472);
        assert!(m.reproducible_from.contains("18472"));
    }

    #[test]
    fn manipulation_reducer_keeps_seed() {
        let mut params = std::collections::BTreeMap::new();
        params.insert("mass".into(), 1.0);
        params.insert("friction".into(), 0.8);
        params.insert("push.distance".into(), 0.1);
        let m = minimize_manipulation(9, "arm", "hash", &params, |p| {
            p.get("mass").copied().unwrap_or(0.0) >= 0.1
        });
        assert_eq!(m.original_seed, 9);
        assert!(m.reduced.get("mass").copied().unwrap() <= 1.0);
        assert!(m.reproducible_from.contains("hash"));
    }
}
