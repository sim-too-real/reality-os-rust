//! Foundation REACH evidence report. SIMULATION_ONLY. Never metal.

use crate::bundle::RobotBundle;
use crate::honesty::SIMULATION_ONLY;
use crate::reach_foundation::{run_foundation_reach, FoundationReachReport};
use crate::mujoco_exec::checkin_worker;
use crate::load_and_normalize;
use crate::semantics_map::embodiment_from_manifest;
use realityos_semantics::capability::{derive_capabilities, CapabilityGraph};
use serde::{Deserialize, Serialize};
use std::path::Path;

const SKILL_ID: &str = "REACH";
const ADAPTER_ID: &str = "chain_ik_position_pd";
const ADAPTATION: &str = "CONFIGURED";

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FoundationRobotRecord {
    pub robot_id: String,
    pub role: String,
    pub model_hash: String,
    pub capability_graph: CapabilityGraph,
    pub reach: FoundationReachReport,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FoundationReport {
    pub software_sha: String,
    pub evidence_status: String,
    pub metal: bool,
    pub skill: String,
    pub adapter_id: String,
    pub adaptation: String,
    pub robots: Vec<FoundationRobotRecord>,
    pub scenario_count: u64,
    pub successes: u64,
    pub refusals: u64,
    pub probes: u64,
    pub ctrl_writes: u64,
    pub replay_deltas: Vec<i64>,
    pub not_implemented: Vec<String>,
}

pub fn software_sha() -> String {
    std::env::var("GITHUB_SHA").unwrap_or_else(|_| "local".into())
}

pub fn not_implemented_list() -> Vec<String> {
    vec![
        "Advanced vision / RGB-D policy".into(),
        "VLA".into(),
        "tactile".into(),
        "F/T servoing".into(),
        "behavior planner".into(),
        "PROBE motion skills (LOOK_AT, CHANGE_VIEWPOINT) beyond refuse/probe status".into(),
        "GRASP/PUSH/PLACE qualification".into(),
        "locomotion".into(),
        "world-estimator fusion".into(),
        "human tracking".into(),
        "first-principles merge".into(),
        "Sim2Real training".into(),
        "100 robots".into(),
        "metal transfer".into(),
        "safety certification".into(),
        "global controllability claims".into(),
        "world / sensors crates".into(),
        "USD as a supported ingest path".into(),
    ]
}

fn capability_graph_for(bundle: &RobotBundle) -> Result<CapabilityGraph, String> {
    let (inst, manifest) = load_and_normalize(bundle, &[], 0)?;
    let model = embodiment_from_manifest(bundle, &manifest);
    let caps = derive_capabilities(&model, None);
    checkin_worker(inst);
    Ok(caps)
}

fn record_robot(
    bundle: &RobotBundle,
    role: &str,
    target: [f64; 3],
    radius: f64,
    replay: bool,
) -> Result<FoundationRobotRecord, String> {
    let caps = capability_graph_for(bundle)?;
    let reach = run_foundation_reach(bundle, target, radius, 10.0, 0.25, None, false, replay)?;
    Ok(FoundationRobotRecord {
        robot_id: reach.robot_id.clone(),
        role: role.into(),
        model_hash: reach.model_hash.clone(),
        capability_graph: caps,
        reach,
    })
}

pub fn run_campaign() -> Result<FoundationReport, String> {
    let planar =
        RobotBundle::load(crate::corpus::robot_dir("planar_arm")).map_err(|e| e.to_string())?;
    let spatial =
        RobotBundle::load(crate::corpus::robot_dir("spatial_arm4")).map_err(|e| e.to_string())?;
    let held_out =
        RobotBundle::load(crate::held_out::held_out_bundle()).map_err(|e| e.to_string())?;

    let robots = vec![
        record_robot(&planar, "development", [0.22, 0.0, 0.12], 0.20, true)?,
        record_robot(&spatial, "development", [0.22, 0.0, 0.12], 0.20, false)?,
        record_robot(&held_out, "held_out_first_score", [0.20, 0.0, 0.12], 0.25, false)?,
    ];

    let scenario_count = robots.len() as u64;
    let mut successes = 0_u64;
    let mut refusals = 0_u64;
    let mut probes = 0_u64;
    let mut ctrl_writes = 0_u64;
    let mut replay_deltas = Vec::new();

    for r in &robots {
        let reach = &r.reach;
        if reach.skill_refuse.is_none() && reach.task_success {
            successes += 1;
        }
        match reach.skill_refuse.as_deref() {
            Some("REFUSE") => refusals += 1,
            Some("PROBE") => probes += 1,
            _ => {}
        }
        ctrl_writes += reach.ctrl_writes;
        if let Some(delta) = reach.replay_write_delta {
            replay_deltas.push(delta);
        }
    }

    Ok(FoundationReport {
        software_sha: software_sha(),
        evidence_status: SIMULATION_ONLY.into(),
        metal: false,
        skill: SKILL_ID.into(),
        adapter_id: ADAPTER_ID.into(),
        adaptation: ADAPTATION.into(),
        robots,
        scenario_count,
        successes,
        refusals,
        probes,
        ctrl_writes,
        replay_deltas,
        not_implemented: not_implemented_list(),
    })
}

pub fn render_markdown(report: &FoundationReport) -> String {
    let mut md = String::new();
    md.push_str("# Foundation REACH Report\n\n");
    md.push_str(&format!("software_sha: `{}`\n\n", report.software_sha));
    md.push_str(&format!("evidence_status: {}\n\n", report.evidence_status));
    md.push_str("metal: false\n\n");
    md.push_str(&format!("skill: {}  adapter: {}  adaptation: {}\n\n", report.skill, report.adapter_id, report.adaptation));
    md.push_str(&format!(
        "scenarios={} successes={} refusals={} probes={} ctrl_writes={}\n\n",
        report.scenario_count, report.successes, report.refusals, report.probes, report.ctrl_writes
    ));
    if !report.replay_deltas.is_empty() {
        md.push_str(&format!("replay_deltas: {:?}\n\n", report.replay_deltas));
    }
    md.push_str("## Robots\n\n");
    for r in &report.robots {
        md.push_str(&format!(
            "- **{}** ({}) hash=`{}` success={} refuse={:?} writes={}\n",
            r.robot_id,
            r.role,
            r.model_hash,
            r.reach.task_success,
            r.reach.skill_refuse,
            r.reach.ctrl_writes
        ));
    }
    md.push_str("\n## NOT IMPLEMENTED\n\n");
    for item in &report.not_implemented {
        md.push_str(&format!("- {item}\n"));
    }
    md
}

pub fn write_report(report: &FoundationReport, out_dir: &Path) -> Result<(), String> {
    std::fs::create_dir_all(out_dir).map_err(|e| e.to_string())?;
    let json = serde_json::to_string_pretty(report).map_err(|e| e.to_string())?;
    std::fs::write(out_dir.join("foundation_report.json"), json).map_err(|e| e.to_string())?;
    std::fs::write(
        out_dir.join("foundation_report.md"),
        render_markdown(report),
    )
    .map_err(|e| e.to_string())?;
    Ok(())
}
