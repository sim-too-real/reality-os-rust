//! Manipulation episode runner. Skills propose; Reality OS authorizes; verifier judges.

use crate::authority::{AuthorityOutcome, SimAuthority};
use crate::bundle::RobotBundle;
use crate::driver::{SharedMujoco, SharedSimPort};
use crate::honesty::SIMULATION_ONLY;
use crate::manipulation_scenarios::{
    arm_is_planar, grasp_scenario, push_scenario, release_scenario, ManipulationScenario, NegKind,
    Polarity,
};
use crate::manipulation_verify::{
    body_xyz, opening_from_truth, verify_grasp, verify_push, verify_release, PrivilegedVerdict,
};
use crate::mujoco_exec::checkin_worker;
use crate::normalize::RobotManifest;
use crate::observation::{policy_observation, PolicyObservation, VerifierTruth, VisionMode};
use crate::policy::ActionProposal;
use crate::resource_discover::discover_resources;
use crate::resource_qualify::qualify_on;
use crate::runner::load_and_normalize;
use crate::semantics_map::embodiment_from_manifest;
use crate::task::TaskSpec;
use realityos_plant::HardwareBackedPlant;
use realityos_semantics::adapter::{
    lower_actuator_commands, lower_named_targets, ChainIkPositionPdAdapter, CompiledCtrl,
};
use realityos_semantics::capability::{apply_resource_qualification, derive_capabilities};
use realityos_semantics::embodiment::EmbodimentModel;
use realityos_semantics::failure::ManipulationFailure;
use realityos_semantics::grasp::{compile_grasp, GraspCandidate};
use realityos_semantics::gripper_state::{derive_gripper_state, GripperStateEvidence};
use realityos_semantics::interaction::{insert_interaction_frame, InteractionFrameKind};
use realityos_semantics::object::{GeometryClass, GraspOccupancy, ObjectGeometry, ObjectState};
use realityos_semantics::observation::{JointStateSample, ObservationFrame};
use realityos_semantics::plan::{SkillPlan, SkillStep};
use realityos_semantics::provenance::{Provenance, Provenanced};
use realityos_semantics::push::{compile_push, PushCandidate};
use realityos_semantics::reach::compile_reach;
use realityos_semantics::release::compile_release;
use realityos_semantics::resource::{ControlledResource, QualificationStatus, ResourceTopology};
use realityos_semantics::skill::SkillRefuse;
use realityos_semantics::transform::{Se3, TransformEdge, TransformGraph};
use realityos_semantics::world::WorldState;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

const HORIZON_S: f64 = 1.0;
const CONTROL_HZ: f64 = 40.0;
const REACH_ATTEMPTS: u32 = 5;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ManipulationEpisode {
    pub software_sha: String,
    pub robot_id: String,
    pub model_hash: String,
    pub object_definitions: Vec<Value>,
    pub world_seed: u64,
    pub skill_contract: String,
    pub semantic_resource: Option<String>,
    pub resource_topology: Option<String>,
    pub joint_state: Value,
    pub object_evidence: Value,
    pub commands: Vec<Value>,
    pub authority_decisions: Vec<String>,
    pub contacts: Vec<Value>,
    pub support_relations: Vec<Value>,
    pub task_result: String,
    pub failure_taxonomy: Option<String>,
    pub evidence_used: Vec<String>,
    pub physical_violations: Vec<String>,
    pub ctrl_writes: u64,
    pub unauthorized_writes: u64,
    pub replay_write_delta: Option<i64>,
    pub metal: bool,
    pub evidence_status: String,
    pub perception: String,
    pub simulation_only: bool,
    pub expected_refusal: bool,
    #[serde(default)]
    pub earliest_failure_stage: String,
    #[serde(default)]
    pub first_stage_entered: String,
    #[serde(default)]
    pub last_stage_completed: String,
    #[serde(default)]
    pub earliest_pipeline_failed: String,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ManipulationMetrics {
    pub release_success: u64,
    pub release_episodes: u64,
    pub grasp_acquisition_success: u64,
    pub grasp_verified_hold_success: u64,
    pub grasp_episodes: u64,
    pub grasp_slip: u64,
    pub push_contact_establishment: u64,
    pub push_task_success: u64,
    pub push_episodes: u64,
    pub safe_completion: u64,
    pub expected_refusals: u64,
    pub unexpected_physical_violations: u64,
    pub authority_violations: u64,
    pub unauthorized_writes: u64,
}

impl ManipulationMetrics {
    pub fn absorb(&mut self, e: &ManipulationEpisode) {
        match e.skill_contract.as_str() {
            "skill.release" => {
                self.release_episodes += 1;
                if e.task_result == "success" {
                    self.release_success += 1;
                }
            }
            "skill.grasp" => {
                self.grasp_episodes += 1;
                if e.task_result == "success" {
                    self.grasp_acquisition_success += 1;
                    if e.evidence_used.iter().any(|x| x == "object_follows_ee") {
                        self.grasp_verified_hold_success += 1;
                    }
                }
                if e.failure_taxonomy.as_deref() == Some("SLIP") {
                    self.grasp_slip += 1;
                }
            }
            "skill.push" => {
                self.push_episodes += 1;
                if e.evidence_used
                    .iter()
                    .any(|x| x == "controlled_contact_established")
                {
                    self.push_contact_establishment += 1;
                }
                if e.task_result == "success" {
                    self.push_task_success += 1;
                }
            }
            _ => {}
        }
        if e.task_result != "authority_violation" && e.unauthorized_writes == 0 {
            self.safe_completion += 1;
        }
        if e.expected_refusal && e.task_result != "success" && e.unauthorized_writes == 0 {
            self.expected_refusals += 1;
        }
        if e.task_result == "authority_violation" {
            self.authority_violations += 1;
        }
        self.unauthorized_writes += e.unauthorized_writes;
        if !e.physical_violations.is_empty() && !e.expected_refusal {
            self.unexpected_physical_violations += 1;
        }
    }
}

pub fn software_sha() -> String {
    std::env::var("MANIPULATION_SOFTWARE_SHA")
        .or_else(|_| std::env::var("GENERALITY_V2_FREEZE_SHA"))
        .unwrap_or_else(|_| "local".into())
}

pub fn run_release_matrix(
    bundle: &RobotBundle,
    n: usize,
    sha: &str,
) -> Result<(Vec<ManipulationEpisode>, ManipulationMetrics), String> {
    let (mut inst, manifest) = load_and_normalize(bundle, &[], 0)?;
    let inspect = inst.inspect.clone();
    let model = embodiment_from_manifest(bundle, &manifest);
    let discovered = discover_resources(bundle, &manifest, &inspect);
    let mut qualified_res = Vec::new();
    let mut quals = Vec::new();
    for r in &discovered {
        match qualify_on(&mut inst, &manifest, r) {
            Ok(q) => {
                let mut rr = r.clone();
                rr.qualification = if q.qualified {
                    QualificationStatus::Qualified
                } else {
                    QualificationStatus::Failed
                };
                quals.push(q);
                qualified_res.push(rr);
            }
            Err(e) => return Err(e),
        }
    }
    checkin_worker(inst);
    let _ = quals;
    let mut model = model;
    model.resources = qualified_res.clone();
    let mut eps = Vec::new();
    let mut metrics = ManipulationMetrics::default();
    for i in 0..n {
        let sc = release_scenario(1000 + i as u64, i);
        let ep = run_release_episode(bundle, &model, &qualified_res, &sc, sha)?;
        metrics.absorb(&ep);
        if i == 0 || (i + 1) % 10 == 0 || i + 1 == n {
            eprintln!(
                "phase_b RELEASE {} {}/{} result={} writes={}",
                bundle.manifest.robot_id,
                i + 1,
                n,
                ep.task_result,
                ep.ctrl_writes
            );
        }
        eps.push(ep);
    }
    Ok((eps, metrics))
}

pub fn run_grasp_matrix(
    bundle: &RobotBundle,
    n: usize,
    sha: &str,
) -> Result<(Vec<ManipulationEpisode>, ManipulationMetrics), String> {
    let (probe, man_probe) = load_and_normalize(bundle, &[], 0)?;
    let planar = manifest_arm_is_planar(&man_probe);
    let discovered = discover_resources(bundle, &man_probe, &probe.inspect);
    checkin_worker(probe);
    let (inst, manifest) = load_and_normalize(bundle, &template_objects(planar), 0)?;
    let mut model = embodiment_from_manifest(bundle, &manifest);
    let mut qualified_res = Vec::new();
    for r in &discovered {
        let (q, rr) = crate::resource_qualify::qualify_resource(bundle, r)?;
        let _ = q;
        qualified_res.push(rr);
    }
    model.resources = qualified_res.clone();
    let mut eps = Vec::new();
    let mut metrics = ManipulationMetrics::default();
    let mut loaded = Some((inst, manifest));
    for i in 0..n {
        let sc = grasp_scenario(2000 + i as u64, planar, i);
        let (ep, inst, man) = run_skill_episode(
            bundle,
            &model,
            &qualified_res,
            &sc,
            sha,
            "GRASP",
            loaded.take(),
        )?;
        loaded = Some((inst, man));
        metrics.absorb(&ep);
        if i == 0 || (i + 1) % 10 == 0 || i + 1 == n {
            eprintln!(
                "phase_b GRASP {} {}/{} planar={planar} result={} fail={:?} writes={}",
                bundle.manifest.robot_id,
                i + 1,
                n,
                ep.task_result,
                ep.failure_taxonomy,
                ep.ctrl_writes
            );
        }
        eps.push(ep);
    }
    if let Some((inst, _)) = loaded {
        checkin_worker(inst);
    }
    Ok((eps, metrics))
}

pub fn run_push_matrix(
    bundle: &RobotBundle,
    n: usize,
    sha: &str,
) -> Result<(Vec<ManipulationEpisode>, ManipulationMetrics), String> {
    run_push_matrix_from(bundle, n, sha, 0)
}

pub fn run_push_matrix_from(
    bundle: &RobotBundle,
    n: usize,
    sha: &str,
    idx_offset: usize,
) -> Result<(Vec<ManipulationEpisode>, ManipulationMetrics), String> {
    let (probe, man_probe) = load_and_normalize(bundle, &[], 0)?;
    let planar = manifest_arm_is_planar(&man_probe);
    checkin_worker(probe);
    let (inst, manifest) = load_and_normalize(bundle, &template_objects(planar), 0)?;
    let model = embodiment_from_manifest(bundle, &manifest);
    let mut eps = Vec::new();
    let mut metrics = ManipulationMetrics::default();
    let mut loaded = Some((inst, manifest));
    for i in 0..n {
        let sc = push_scenario(3000 + i as u64, planar, idx_offset + i);
        let (ep, inst, man) =
            run_skill_episode(bundle, &model, &[], &sc, sha, "PUSH", loaded.take())?;
        loaded = Some((inst, man));
        metrics.absorb(&ep);
        if i == 0 || (i + 1) % 10 == 0 || i + 1 == n {
            eprintln!(
                "phase_b PUSH {} {}/{} planar={planar} result={} fail={:?} writes={}",
                bundle.manifest.robot_id,
                i + 1,
                n,
                ep.task_result,
                ep.failure_taxonomy,
                ep.ctrl_writes
            );
        }
        eps.push(ep);
    }
    if let Some((inst, _)) = loaded {
        checkin_worker(inst);
    }
    Ok((eps, metrics))
}

fn template_objects(planar: bool) -> Vec<Value> {
    let (table_pos, table_size, table_mass, obj_z) = if planar {
        ([0.24, 0.0, 0.105], [0.40, 0.22, 0.01], 10.0, 0.145)
    } else {
        ([0.45, 0.0, 0.40], [0.25, 0.25, 0.02], 20.0, 0.445)
    };
    vec![
        json!({
            "name": "table",
            "type": "box",
            "pos": table_pos,
            "size": table_size,
            "mass": table_mass,
            "movable": false
        }),
        json!({
            "name": "obj0",
            "type": "box",
            "pos": [0.22, 0.0, obj_z],
            "size": [0.025, 0.025, 0.025],
            "mass": 0.05,
            "friction": 0.8,
            "movable": true
        }),
        json!({
            "name": "obstacle",
            "type": "box",
            "pos": [8.0, 8.0, -1.0],
            "size": [0.03, 0.03, 0.03],
            "mass": 1.0,
            "movable": false
        }),
    ]
}

fn apply_scenario_objects(
    inst: &mut crate::mujoco_exec::MujocoInstance,
    sc: &ManipulationScenario,
) -> Result<(), String> {
    const TEMPLATE: &[&str] = &["table", "obj0", "obstacle"];
    for name in TEMPLATE {
        if !sc.objects.iter().any(|o| o["name"] == *name) {
            inst.configure_body(name, None, None, None, None, true)
                .map_err(|e| e.to_string())?;
        }
    }
    for obj in &sc.objects {
        let Some(name) = obj["name"].as_str() else {
            continue;
        };
        let pos = obj["pos"].as_array().and_then(|a| {
            if a.len() < 3 {
                return None;
            }
            Some([a[0].as_f64()?, a[1].as_f64()?, a[2].as_f64()?])
        });
        let size = obj["size"]
            .as_array()
            .map(|a| a.iter().filter_map(|v| v.as_f64()).collect::<Vec<_>>());
        inst.configure_body(
            name,
            pos,
            obj["mass"].as_f64(),
            obj["friction"].as_f64(),
            size,
            false,
        )
        .map_err(|e| e.to_string())?;
    }
    Ok(())
}

fn run_release_episode(
    bundle: &RobotBundle,
    model: &EmbodimentModel,
    resources: &[ControlledResource],
    sc: &ManipulationScenario,
    sha: &str,
) -> Result<ManipulationEpisode, String> {
    let (mut inst, manifest) = load_and_normalize(bundle, &[], sc.seed)?;
    let mut resource = resources.first().cloned();
    if matches!(
        sc.neg,
        Some(NegKind::GripperUnavailable) | Some(NegKind::UnsupportedCoupling)
    ) {
        if let Some(r) = resource.as_mut() {
            r.topology = ResourceTopology::UnsupportedResourceTopology;
            r.unsupported_detail = Some("injected_negative".into());
            r.qualification = QualificationStatus::Failed;
        }
    }
    let Some(resource) = resource else {
        checkin_worker(inst);
        return Ok(refused_episode(
            bundle,
            model,
            sc,
            sha,
            "skill.release",
            Some("RESOURCE_UNSUPPORTED"),
            true,
        ));
    };
    let initial = truth0(&mut inst)?;
    let now = 10.0;
    let freshness = if sc.stale { 0.0 } else { 0.4 };
    let opening = opening_from_truth(&initial, &resource, &manifest);
    let grip = if sc.stale {
        GripperStateEvidence::unknown(0.0)
    } else {
        derive_gripper_state(&resource, opening, Some(0.0), Some(false), Some(false), now)
    };
    let caps = apply_resource_qualification(derive_capabilities(model, None), true, false, true);
    let obs = obs_frame(
        model,
        &initial.qpos,
        &model.calibration_epoch,
        now,
        freshness,
    );
    let compiled = compile_release(
        model,
        &caps,
        &resource,
        &grip,
        &obs,
        &model.model_hash,
        now,
        freshness,
        0.5,
        sc.required_opening,
    );
    match compiled {
        Err(e) => {
            checkin_worker(inst);
            Ok(refused_episode(
                bundle,
                model,
                sc,
                sha,
                "skill.release",
                Some(fail_from_refuse(e)),
                sc.expected_refusal.is_some() || sc.stale,
            ))
        }
        Ok(plan) => {
            let (ep, inst) = execute_plan(
                bundle, inst, manifest, model, sc, sha, plan, &resource, &initial, now,
            )?;
            checkin_worker(inst);
            Ok(ep)
        }
    }
}

fn run_skill_episode(
    bundle: &RobotBundle,
    model: &EmbodimentModel,
    resources: &[ControlledResource],
    sc: &ManipulationScenario,
    sha: &str,
    skill: &str,
    loaded: Option<(crate::mujoco_exec::MujocoInstance, RobotManifest)>,
) -> Result<
    (
        ManipulationEpisode,
        crate::mujoco_exec::MujocoInstance,
        RobotManifest,
    ),
    String,
> {
    let (mut inst, manifest) = if let Some((mut inst, manifest)) = loaded {
        if let Err(e) =
            reset_episode_pose(&mut inst).and_then(|_| apply_scenario_objects(&mut inst, sc))
        {
            checkin_worker(inst);
            return Err(e);
        }
        (inst, manifest)
    } else {
        let (mut inst, manifest) = load_and_normalize(bundle, &sc.objects, sc.seed)?;
        if let Err(e) = reset_episode_pose(&mut inst) {
            checkin_worker(inst);
            return Err(e);
        }
        (inst, manifest)
    };
    let mut resource = resources.first().cloned();
    if matches!(
        sc.neg,
        Some(NegKind::GripperUnavailable) | Some(NegKind::UnsupportedCoupling)
    ) {
        if let Some(r) = resource.as_mut() {
            r.topology = ResourceTopology::UnsupportedResourceTopology;
            r.unsupported_detail = Some("injected_negative".into());
            r.qualification = QualificationStatus::Failed;
        }
    }
    if matches!(sc.neg, Some(NegKind::ForceBoundUnavailable)) {
        if let Some(r) = resource.as_mut() {
            r.force_bound = Provenanced::unknown("FORCE_BOUND_UNAVAILABLE", 0.0);
        }
    }
    let mut initial = truth0(&mut inst)?;
    if !matches!(
        sc.neg,
        Some(NegKind::Unreachable) | Some(NegKind::EmptyClose)
    ) {
        if let Some(ee) = ee_workspace(&initial, bundle) {
            let tips = resource.as_ref().and_then(|r| {
                finger_contact_point(&initial, &inst.inspect, &r.finger_bodies, Some(ee))
            });
            place_object_in_workspace(&mut inst, sc, model, &semantic_ee(bundle), ee, tips)?;
            initial = truth0(&mut inst)?;
        }
    }
    if sc.move_object_after_obs {
        if let Some(pos) = body_xyz(&initial, &sc.object_id) {
            let moved = [pos[0] + 0.18, pos[1] + 0.14, pos[2]];
            let _ = inst.set_body_pos(&sc.object_id, moved);
        }
    }
    let now = 10.0;
    let freshness = if sc.stale { 0.0 } else { 0.5 };
    let epoch = model.calibration_epoch.clone();
    let mut transforms = graph_from_truth(model, &initial, &epoch, now);
    let obj_pose = body_se3(&initial, &sc.object_id).unwrap_or_else(Se3::identity);
    let _ = insert_interaction_frame(
        &mut transforms,
        "world",
        InteractionFrameKind::Object,
        &sc.object_id,
        obj_pose,
        &epoch,
        now,
        "perfect_perception",
    );
    let ee_now = ee_workspace(&initial, bundle);
    let finger_mid = resource.as_ref().and_then(|r| {
        finger_contact_point(&initial, &inst.inspect, &r.finger_bodies, ee_now)
            .or_else(|| finger_midpoint(&initial, &r.finger_bodies))
    });
    let (approach, grasp_or_contact) = candidate_poses(sc, obj_pose, ee_now, finger_mid, skill);
    let _ = insert_interaction_frame(
        &mut transforms,
        "world",
        InteractionFrameKind::Approach,
        &sc.object_id,
        approach,
        &epoch,
        now,
        "perfect_perception",
    );
    let kind = if skill == "PUSH" {
        InteractionFrameKind::PushContact
    } else {
        InteractionFrameKind::Grasp
    };
    let _ = insert_interaction_frame(
        &mut transforms,
        "world",
        kind,
        &sc.object_id,
        grasp_or_contact,
        &epoch,
        now,
        "perfect_perception",
    );
    let object = object_state(sc, obj_pose, now, freshness);
    let caps = apply_resource_qualification(
        derive_capabilities(model, None),
        resource.as_ref().is_some_and(|r| {
            r.qualification == QualificationStatus::Qualified
                || (r.is_supported() && r.command_range.value.is_some())
        }),
        resource.is_some(),
        true,
    );
    let obs = obs_frame(model, &initial.qpos, &epoch, now, freshness);
    let ee = semantic_ee(bundle);
    let compiled = if skill == "PUSH" {
        compile_push(
            model,
            &caps,
            &object,
            &PushCandidate {
                object_id: sc.object_id.clone(),
                contact: grasp_or_contact,
                approach,
                direction: sc.push_dir,
                distance_m: sc.push_dist,
                target_region: None,
                reference_frame: "world".into(),
            },
            &obs,
            &transforms,
            &ee,
            &model.model_hash,
            now,
            freshness,
        )
    } else {
        let Some(resource) = resource.as_ref() else {
            return Ok((
                refused_episode(
                    bundle,
                    model,
                    sc,
                    sha,
                    "skill.grasp",
                    Some("RESOURCE_UNSUPPORTED"),
                    true,
                ),
                inst,
                manifest,
            ));
        };
        let opening = opening_from_truth(&initial, resource, &manifest);
        let grip =
            derive_gripper_state(resource, opening, Some(0.0), Some(false), Some(false), now);
        compile_grasp(
            model,
            &caps,
            resource,
            &object,
            &GraspCandidate {
                object_id: sc.object_id.clone(),
                approach,
                grasp: grasp_or_contact,
                required_opening: sc.required_opening,
                closing_axis: [0.0, 1.0, 0.0],
                verify_displacement: if sc.planar {
                    [0.03, 0.0, 0.0]
                } else {
                    [0.0, 0.0, 0.03]
                },
                reference_frame: "world".into(),
            },
            &grip,
            &obs,
            &transforms,
            &ee,
            &model.model_hash,
            now,
            freshness,
            0.5,
        )
    };
    match compiled {
        Err(e) => Ok((
            refused_episode(
                bundle,
                model,
                sc,
                sha,
                if skill == "PUSH" {
                    "skill.push"
                } else {
                    "skill.grasp"
                },
                Some(fail_from_refuse(e)),
                sc.expected_refusal.is_some() || sc.stale,
            ),
            inst,
            manifest,
        )),
        Ok(plan) => {
            let res = resource.unwrap_or_else(|| ControlledResource {
                id: "none".into(),
                kind: realityos_semantics::resource::ResourceKind::ContactEndEffector,
                topology: ResourceTopology::UnsupportedResourceTopology,
                actuator_inputs: vec![],
                affected_joints: vec![],
                finger_bodies: vec![],
                coupling: realityos_semantics::resource::CouplingModel::none(),
                command_coordinate: String::new(),
                opening_range: Provenanced::unknown("none", 0.0),
                command_range: Provenanced::unknown("none", 0.0),
                closing_direction: realityos_semantics::resource::ClosingDirection::TowardMin,
                force_bound: Provenanced::unknown("none", 0.0),
                qualification: QualificationStatus::NotApplicable,
                unsupported_detail: None,
            });
            let (ep, inst) = execute_plan(
                bundle,
                inst,
                manifest.clone(),
                model,
                sc,
                sha,
                plan,
                &res,
                &initial,
                now,
            )?;
            Ok((ep, inst, manifest))
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn execute_plan(
    bundle: &RobotBundle,
    inst: crate::mujoco_exec::MujocoInstance,
    manifest: RobotManifest,
    model: &EmbodimentModel,
    sc: &ManipulationScenario,
    sha: &str,
    plan: SkillPlan,
    resource: &ControlledResource,
    initial: &VerifierTruth,
    now0: f64,
) -> Result<(ManipulationEpisode, crate::mujoco_exec::MujocoInstance), String> {
    if sc.crash_controller {
        return Ok((
            refused_episode(
                bundle,
                model,
                sc,
                sha,
                &plan.contract_id,
                Some("CONTROLLER_FAILURE"),
                true,
            ),
            inst,
        ));
    }
    let shared = Arc::new(SharedMujoco {
        inst: Mutex::new(inst),
        probe: Default::default(),
        robot_id: manifest.robot_id.clone(),
        model_hash: manifest.model_hash.clone(),
    });
    let port = SharedSimPort::new(shared.clone());
    let max_a = manifest.tau_max().into_iter().fold(1.0, f64::max);
    let plant =
        HardwareBackedPlant::new(port, &manifest.robot_id, manifest.nu.max(1) as usize, max_a);
    let journal = std::env::temp_dir().join(format!(
        "realityos-manip-{}-{}-{}.jsonl",
        manifest.robot_id,
        std::process::id(),
        sc.seed
    ));
    let _ = std::fs::remove_file(&journal);
    let mut auth = SimAuthority::open_with_journal(plant, &manifest, now0, Some(journal))?;
    auth.freshness_s = 1.5;
    auth.command_lifetime_s = 0.45;
    let mut decisions = Vec::new();
    let mut commands = Vec::new();
    let mut now = now0;
    let mut truth = initial.clone();
    let before = initial.clone();
    let mut unauthorized = 0u64;
    let episode_id = format!("manip-{}-{}", plan.contract_id, sc.seed);

    for (si, step) in plan.steps.iter().enumerate() {
        match step {
            SkillStep::Reach {
                end_effector,
                target,
                success_radius,
            } => {
                let xyz = target.xyz().unwrap_or([0.0, 0.0, 0.0]);
                let world = WorldState::empty(&model.calibration_epoch, now)
                    .with_target_in_frame(
                        end_effector,
                        "world",
                        xyz,
                        now + 0.5,
                        &model.calibration_epoch,
                        now,
                        Provenance::UserDeclared,
                    )
                    .with_success_radius(*success_radius)
                    .with_perfect_perception();
                let obs = obs_frame(model, &truth.qpos, &model.calibration_epoch, now, 0.5);
                let g = graph_from_truth(model, &truth, &model.calibration_epoch, now);
                let caps = derive_capabilities(model, None);
                match compile_reach(
                    model,
                    &caps,
                    &world,
                    &obs,
                    &g,
                    &model.model_hash,
                    now,
                    0.5,
                    &ChainIkPositionPdAdapter,
                ) {
                    Err(e) => {
                        let writes = shared.probe.snapshot().policy_ctrl_writes;
                        drop(auth);
                        let inst = unwrap_shared(shared)?;
                        let mut ep = refused_episode(
                            bundle,
                            model,
                            sc,
                            sha,
                            &plan.contract_id,
                            Some(fail_from_refuse(e)),
                            sc.polarity != Polarity::Positive,
                        );
                        ep.ctrl_writes = writes;
                        ep.authority_decisions = decisions;
                        return Ok((ep, inst));
                    }
                    Ok(ctrl) => {
                        let rec = match write_ctrl(
                            &mut auth,
                            &manifest,
                            model,
                            &truth,
                            &ctrl,
                            &episode_id,
                            si,
                            now,
                            "reach",
                            xyz,
                        ) {
                            Ok(rec) => rec,
                            Err(e) => {
                                return refuse_in_flight(
                                    bundle,
                                    model,
                                    sc,
                                    sha,
                                    &plan.contract_id,
                                    e,
                                    shared,
                                    auth,
                                    decisions,
                                    sc.polarity != Polarity::Positive,
                                );
                            }
                        };
                        decisions.push(format!(
                            "{:?}:{}:{}",
                            rec.outcome, rec.status, rec.physical_reason
                        ));
                        commands.push(json!({"step": si, "kind": "reach", "target": xyz}));
                        maybe_replay_or_restart(
                            &mut auth,
                            &shared,
                            &manifest,
                            sc,
                            &episode_id,
                            si,
                            now,
                            &mut decisions,
                            &mut unauthorized,
                        )?;
                        let (t, n) = step_sim(&shared, &manifest, &mut auth, now)?;
                        truth = t;
                        now = n;
                        for attempt in 1..REACH_ATTEMPTS {
                            let ee = privileged_ee(bundle);
                            if let Some(p) =
                                truth.named_pos.get(&ee).or_else(|| truth.xpos.get(&ee))
                            {
                                if p.len() >= 3 {
                                    let d = ((p[0] - xyz[0]).powi(2)
                                        + (p[1] - xyz[1]).powi(2)
                                        + (p[2] - xyz[2]).powi(2))
                                    .sqrt();
                                    if d <= *success_radius {
                                        break;
                                    }
                                }
                            }
                            let obs =
                                obs_frame(model, &truth.qpos, &model.calibration_epoch, now, 0.5);
                            let g = graph_from_truth(model, &truth, &model.calibration_epoch, now);
                            if let Ok(ctrl) = compile_reach(
                                model,
                                &caps,
                                &world,
                                &obs,
                                &g,
                                &model.model_hash,
                                now,
                                0.5,
                                &ChainIkPositionPdAdapter,
                            ) {
                                let rec = match write_ctrl(
                                    &mut auth,
                                    &manifest,
                                    model,
                                    &truth,
                                    &ctrl,
                                    &episode_id,
                                    si * 10 + attempt as usize,
                                    now,
                                    "reach",
                                    xyz,
                                ) {
                                    Ok(rec) => rec,
                                    Err(e) => {
                                        return refuse_in_flight(
                                            bundle,
                                            model,
                                            sc,
                                            sha,
                                            &plan.contract_id,
                                            e,
                                            shared,
                                            auth,
                                            decisions,
                                            sc.polarity != Polarity::Positive,
                                        );
                                    }
                                };
                                decisions
                                    .push(format!("reach-retry:{:?}:{}", rec.outcome, rec.status));
                                let (t, n) = step_sim(&shared, &manifest, &mut auth, now)?;
                                truth = t;
                                now = n;
                            }
                        }
                    }
                }
            }
            SkillStep::ResourceCommand {
                commands: cmds,
                opening_01,
                expires_at_s,
                ..
            } => {
                let current = observed_hold_values(model, &manifest, &truth.qpos, &truth.ctrl);
                let lowered = match lower_actuator_commands(model, cmds, &current) {
                    Ok(v) => v,
                    Err(e) => {
                        return refuse_in_flight(
                            bundle,
                            model,
                            sc,
                            sha,
                            &plan.contract_id,
                            e,
                            shared,
                            auth,
                            decisions,
                            sc.polarity != Polarity::Positive,
                        );
                    }
                };
                let action = match named_to_ctrl(&manifest, &lowered, &truth.ctrl) {
                    Ok(a) => a,
                    Err(e) => {
                        return refuse_in_flight(
                            bundle,
                            model,
                            sc,
                            sha,
                            &plan.contract_id,
                            e,
                            shared,
                            auth,
                            decisions,
                            sc.polarity != Polarity::Positive,
                        );
                    }
                };
                let obs = pol_obs(&manifest, &episode_id, si, &truth, now);
                let proposal = ActionProposal {
                    robot_id: obs.robot_id.clone(),
                    model_hash: obs.model_hash.clone(),
                    episode_id: obs.episode_id.clone(),
                    observation_id: format!("obs-{si}"),
                    observation_timestamp: now,
                    task_id: plan.contract_id.clone(),
                    action,
                    control_mode: "position".into(),
                    requested_horizon_s: (*expires_at_s - now).max(0.1),
                    confidence: Some(1.0),
                    policy_id: "resource_lowerer".into(),
                    policy_version: "1".into(),
                };
                let rec = auth.decide_and_maybe_write(
                    &proposal,
                    &manifest,
                    &TaskSpec::Hold { duration_s: 0.2 },
                    &obs,
                    now,
                    None,
                    Some("drive"),
                );
                decisions.push(format!(
                    "{:?}:{}:{}",
                    rec.outcome, rec.status, rec.physical_reason
                ));
                commands.push(json!({"step": si, "kind": "resource", "opening": opening_01}));
                if rec.outcome != AuthorityOutcome::Allowed && sc.replay {
                    unauthorized = 0;
                }
                let (t, n) = step_sim(&shared, &manifest, &mut auth, now)?;
                truth = t;
                now = n;
                if *opening_01 < 0.05 {
                    for _ in 0..2 {
                        let (t2, n2) = step_sim(&shared, &manifest, &mut auth, now)?;
                        truth = t2;
                        now = n2;
                    }
                }
                maybe_replay_or_restart(
                    &mut auth,
                    &shared,
                    &manifest,
                    sc,
                    &episode_id,
                    si,
                    now,
                    &mut decisions,
                    &mut unauthorized,
                )?;
            }
            SkillStep::VerifyMotion(vm) => {
                let ee = privileged_ee(bundle);
                if let Some(p) = truth.named_pos.get(&ee).or_else(|| truth.xpos.get(&ee)) {
                    if p.len() >= 3 {
                        let t = [
                            p[0] + vm.displacement_xyz[0],
                            p[1] + vm.displacement_xyz[1],
                            p[2] + vm.displacement_xyz[2],
                        ];
                        let world = WorldState::empty(&model.calibration_epoch, now)
                            .with_target_in_frame(
                                semantic_ee(bundle),
                                "world",
                                t,
                                now + 0.5,
                                &model.calibration_epoch,
                                now,
                                Provenance::UserDeclared,
                            )
                            .with_success_radius(vm.bound_m);
                        let obs = obs_frame(model, &truth.qpos, &model.calibration_epoch, now, 0.5);
                        let g = graph_from_truth(model, &truth, &model.calibration_epoch, now);
                        let caps = derive_capabilities(model, None);
                        if let Ok(ctrl) = compile_reach(
                            model,
                            &caps,
                            &world,
                            &obs,
                            &g,
                            &model.model_hash,
                            now,
                            0.5,
                            &ChainIkPositionPdAdapter,
                        ) {
                            let rec = match write_ctrl(
                                &mut auth,
                                &manifest,
                                model,
                                &truth,
                                &ctrl,
                                &episode_id,
                                si,
                                now,
                                "reach",
                                t,
                            ) {
                                Ok(rec) => rec,
                                Err(e) => {
                                    return refuse_in_flight(
                                        bundle,
                                        model,
                                        sc,
                                        sha,
                                        &plan.contract_id,
                                        e,
                                        shared,
                                        auth,
                                        decisions,
                                        sc.polarity != Polarity::Positive,
                                    );
                                }
                            };
                            decisions.push(format!("verify:{:?}:{}", rec.outcome, rec.status));
                            let (t2, n2) = step_sim(&shared, &manifest, &mut auth, now)?;
                            truth = t2;
                            now = n2;
                        }
                    }
                }
            }
            SkillStep::OpenThenRetract | SkillStep::Stop => {}
        }
    }

    let ctrl_writes = shared.probe.snapshot().policy_ctrl_writes;
    drop(auth);
    let inst = unwrap_shared(shared)?;

    let opening = opening_from_truth(&truth, resource, &manifest);
    let tables = ["table", "floor"];
    let unexpected = sc
        .objects
        .iter()
        .filter_map(|o| o.get("name").and_then(|v| v.as_str()))
        .filter(|n| *n == "obstacle")
        .map(|s| s.to_string())
        .collect::<Vec<_>>();
    let verdict = match plan.contract_id.as_str() {
        "skill.release" => verify_release(&truth, resource, opening, None, None, None, now),
        "skill.push" => verify_push(
            &before,
            &truth,
            &sc.object_id,
            sc.push_dir,
            sc.push_dist,
            &unexpected,
            sc.immovable,
        ),
        _ => {
            let ee = privileged_ee(bundle);
            let sem = semantic_ee(bundle);
            verify_grasp(
                &before,
                &truth,
                resource,
                &sc.object_id,
                &tables,
                &unexpected,
                opening,
                now,
                &[&ee, &sem],
            )
        }
    };
    Ok((
        finish_episode(
            bundle,
            model,
            sc,
            sha,
            &plan,
            resource,
            &truth,
            verdict,
            decisions,
            commands,
            ctrl_writes,
            unauthorized,
            None,
        ),
        inst,
    ))
}

fn finish_episode(
    bundle: &RobotBundle,
    model: &EmbodimentModel,
    sc: &ManipulationScenario,
    sha: &str,
    plan: &SkillPlan,
    resource: &ControlledResource,
    truth: &VerifierTruth,
    verdict: PrivilegedVerdict,
    decisions: Vec<String>,
    commands: Vec<Value>,
    ctrl_writes: u64,
    unauthorized: u64,
    replay_delta: Option<i64>,
) -> ManipulationEpisode {
    let contacts = truth
        .contacts
        .iter()
        .map(|c| {
            json!({
                "a": c.body1, "b": c.body2, "fn": c.normal_force, "ft": c.tangential_force,
                "available": c.force_available
            })
        })
        .collect();
    let mut ep = ManipulationEpisode {
        software_sha: sha.into(),
        robot_id: bundle.manifest.robot_id.clone(),
        model_hash: model.model_hash.clone(),
        object_definitions: sc.objects.clone(),
        world_seed: sc.seed,
        skill_contract: plan.contract_id.clone(),
        semantic_resource: Some(resource.id.clone()),
        resource_topology: Some(format!("{:?}", resource.topology)),
        joint_state: json!(truth.qpos),
        object_evidence: json!({
            "id": sc.object_id,
            "perception": "PERFECT_PERCEPTION",
            "pose": body_xyz(truth, &sc.object_id)
        }),
        commands,
        authority_decisions: decisions,
        contacts,
        support_relations: verdict
            .support
            .iter()
            .map(|s| json!({"object": s.object_id, "surface": s.surface_id, "kind": s.kind}))
            .collect(),
        task_result: if verdict.success {
            "success".into()
        } else {
            "fail".into()
        },
        failure_taxonomy: verdict.failure,
        evidence_used: verdict.evidence_used,
        physical_violations: verdict.notes.clone(),
        ctrl_writes,
        unauthorized_writes: unauthorized,
        replay_write_delta: replay_delta,
        metal: false,
        evidence_status: SIMULATION_ONLY.into(),
        perception: "PERFECT_PERCEPTION".into(),
        simulation_only: true,
        expected_refusal: sc.polarity == Polarity::Negative || sc.expected_refusal.is_some(),
        earliest_failure_stage: String::new(),
        first_stage_entered: String::new(),
        last_stage_completed: String::new(),
        earliest_pipeline_failed: String::new(),
    };
    ep.earliest_failure_stage = crate::failure_diagnosis::classify_push_stage(
        &ep.task_result,
        ep.failure_taxonomy.as_deref().unwrap_or(""),
        &ep.evidence_used,
        ep.contacts.iter().any(|c| {
            let a = c.get("a").and_then(|v| v.as_str()).unwrap_or("");
            let b = c.get("b").and_then(|v| v.as_str()).unwrap_or("");
            a.contains(&sc.object_id) || b.contains(&sc.object_id)
        }),
    )
    .as_str()
    .into();
    if ep.skill_contract != "skill.push" {
        ep.earliest_failure_stage = if ep.task_result == "success" {
            "SUCCESS".into()
        } else if ep.task_result == "refuse" {
            "EXPECTED_REFUSAL".into()
        } else {
            ep.failure_taxonomy
                .clone()
                .unwrap_or_else(|| "UNKNOWN".into())
        };
    }
    let pev = crate::push_pipeline::evidence_from_episode_fields(
        &ep.task_result,
        ep.failure_taxonomy.as_deref().unwrap_or(""),
        &ep.evidence_used,
        !ep.contacts.is_empty(),
        ep.expected_refusal,
    );
    let tr = crate::push_pipeline::classify_push_pipeline(&pev);
    ep.first_stage_entered = tr
        .first_stage_entered
        .map(|s| s.as_str().into())
        .unwrap_or_default();
    ep.last_stage_completed = tr
        .last_stage_completed
        .map(|s| s.as_str().into())
        .unwrap_or_default();
    ep.earliest_pipeline_failed = tr
        .earliest_failed_stage
        .map(|s| s.as_str().into())
        .unwrap_or_default();
    ep
}

#[allow(clippy::too_many_arguments)]
fn refuse_in_flight(
    bundle: &RobotBundle,
    model: &EmbodimentModel,
    sc: &ManipulationScenario,
    sha: &str,
    contract: &str,
    e: SkillRefuse,
    shared: Arc<SharedMujoco>,
    auth: SimAuthority<SharedSimPort>,
    decisions: Vec<String>,
    expected: bool,
) -> Result<(ManipulationEpisode, crate::mujoco_exec::MujocoInstance), String> {
    let writes = shared.probe.snapshot().policy_ctrl_writes;
    drop(auth);
    let inst = unwrap_shared(shared)?;
    let mut ep = refused_episode(
        bundle,
        model,
        sc,
        sha,
        contract,
        Some(fail_from_refuse(e)),
        expected,
    );
    ep.ctrl_writes = writes;
    ep.authority_decisions = decisions;
    Ok((ep, inst))
}

fn refused_episode(
    bundle: &RobotBundle,
    model: &EmbodimentModel,
    sc: &ManipulationScenario,
    sha: &str,
    contract: &str,
    failure: Option<&str>,
    expected: bool,
) -> ManipulationEpisode {
    ManipulationEpisode {
        software_sha: sha.into(),
        robot_id: bundle.manifest.robot_id.clone(),
        model_hash: model.model_hash.clone(),
        object_definitions: sc.objects.clone(),
        world_seed: sc.seed,
        skill_contract: contract.into(),
        semantic_resource: None,
        resource_topology: None,
        joint_state: json!([]),
        object_evidence: json!({"perception":"PERFECT_PERCEPTION"}),
        commands: vec![],
        authority_decisions: vec![],
        contacts: vec![],
        support_relations: vec![],
        task_result: "refuse".into(),
        failure_taxonomy: failure.map(|s| s.into()),
        evidence_used: vec![],
        physical_violations: vec![],
        ctrl_writes: 0,
        unauthorized_writes: 0,
        replay_write_delta: None,
        metal: false,
        evidence_status: SIMULATION_ONLY.into(),
        perception: "PERFECT_PERCEPTION".into(),
        simulation_only: true,
        expected_refusal: expected,
        earliest_failure_stage: if expected {
            "EXPECTED_REFUSAL".into()
        } else {
            failure.unwrap_or("UNKNOWN").into()
        },
        first_stage_entered: "TARGET_AVAILABLE".into(),
        last_stage_completed: String::new(),
        earliest_pipeline_failed: String::new(),
    }
}

fn write_ctrl(
    auth: &mut SimAuthority<crate::driver::SharedSimPort>,
    manifest: &RobotManifest,
    model: &EmbodimentModel,
    truth: &VerifierTruth,
    ctrl: &CompiledCtrl,
    episode_id: &str,
    si: usize,
    now: f64,
    verb: &str,
    target: [f64; 3],
) -> Result<crate::authority::AuthorityRecord, SkillRefuse> {
    let obs = pol_obs(manifest, episode_id, si, truth, now);
    let current = observed_hold_values(model, manifest, &truth.qpos, &truth.ctrl);
    let lowered = lower_named_targets(model, &ctrl.targets, &current)?;
    let action = named_to_ctrl(manifest, &lowered, &truth.ctrl)?;
    let proposal = ActionProposal {
        robot_id: obs.robot_id.clone(),
        model_hash: obs.model_hash.clone(),
        episode_id: obs.episode_id.clone(),
        observation_id: format!("obs-{si}"),
        observation_timestamp: now,
        task_id: obs.task_id.clone(),
        action,
        control_mode: ctrl.control_mode.clone(),
        requested_horizon_s: HORIZON_S,
        confidence: Some(1.0),
        policy_id: ctrl.adapter_id.clone(),
        policy_version: ctrl.adapter_version.clone(),
    };
    Ok(auth.decide_and_maybe_write(
        &proposal,
        manifest,
        &TaskSpec::Reach {
            end_effector: "ee".into(),
            target,
            radius: 0.05,
        },
        &obs,
        now,
        None,
        Some(verb),
    ))
}

fn step_sim(
    shared: &Arc<SharedMujoco>,
    manifest: &RobotManifest,
    auth: &mut SimAuthority<crate::driver::SharedSimPort>,
    now: f64,
) -> Result<(VerifierTruth, f64), String> {
    let dt = manifest.timestep.max(1e-4);
    let sub = ((1.0 / CONTROL_HZ) / dt).round().max(1.0) as u32;
    let cycles = ((HORIZON_S * CONTROL_HZ).round() as u32).clamp(12, 24);
    let mut truth = VerifierTruth::default();
    for i in 0..cycles {
        let t = now + f64::from(i) * (1.0 / CONTROL_HZ);
        let _ = auth.session.governor.heartbeat(t);
        let _ = auth.session.governor.watchdog_tick(t);
        let stepped = {
            let mut g = shared.inst.lock().map_err(|e| e.to_string())?;
            g.step(sub).map_err(|e| e.to_string())?
        };
        truth = VerifierTruth::from_mujoco_state(stepped.get("state").unwrap_or(&stepped));
    }
    Ok((truth, now + f64::from(cycles) * (1.0 / CONTROL_HZ)))
}

fn truth0(inst: &mut crate::mujoco_exec::MujocoInstance) -> Result<VerifierTruth, String> {
    inst.step(0).map_err(|e| e.to_string()).and_then(|st| {
        st.get("state")
            .ok_or_else(|| "missing state".into())
            .map(VerifierTruth::from_mujoco_state)
    })
}

fn unwrap_shared(shared: Arc<SharedMujoco>) -> Result<crate::mujoco_exec::MujocoInstance, String> {
    match Arc::try_unwrap(shared) {
        Ok(owned) => owned
            .inst
            .into_inner()
            .map_err(|_| "mujoco mutex poisoned".into()),
        Err(_) => Err("shared mujoco still referenced".into()),
    }
}

fn manifest_arm_is_planar(manifest: &RobotManifest) -> bool {
    let hinges: Vec<&[f64; 3]> = manifest
        .joints
        .iter()
        .filter(|j| j.joint_type == "hinge")
        .filter_map(|j| j.axis.as_ref())
        .collect();
    // Local z-axes are common on spatial arms (Panda). More than three
    // hinges is a spatial serial chain, not a planar 3R fixture.
    hinges.len() <= 3 && arm_is_planar(hinges)
}

fn semantic_ee(bundle: &RobotBundle) -> String {
    bundle
        .manifest
        .end_effectors
        .first()
        .map(|e| e.name.clone())
        .unwrap_or_else(|| "ee".into())
}

fn privileged_ee(bundle: &RobotBundle) -> String {
    bundle
        .manifest
        .end_effectors
        .first()
        .and_then(|e| e.site.clone().or_else(|| e.body.clone()))
        .unwrap_or_else(|| "ee".into())
}

fn graph_from_truth(
    model: &EmbodimentModel,
    truth: &VerifierTruth,
    epoch: &str,
    now_s: f64,
) -> TransformGraph {
    let mut g = TransformGraph::new(epoch);
    for (name, pos) in &truth.xpos {
        if name == "world" || pos.len() < 3 {
            continue;
        }
        let Some(quat) = truth.xquat.get(name).filter(|q| q.len() >= 4) else {
            continue;
        };
        let Ok(pose) = Se3::try_new(
            [pos[0], pos[1], pos[2]],
            [quat[0], quat[1], quat[2], quat[3]],
        ) else {
            continue;
        };
        let _ = g.insert(TransformEdge::from_se3(
            "world",
            name,
            pose,
            epoch,
            now_s,
            "verify.privileged_fk",
        ));
    }
    if g.lookup("world", "base").is_none() {
        if let Some(base) = model.bodies.iter().find(|b| b.parent.is_none()) {
            if let Some(pose) = base.local_pose.value {
                let _ = g.insert(TransformEdge::from_se3(
                    "world",
                    &base.name,
                    pose,
                    epoch,
                    now_s,
                    "verify.model_base",
                ));
            }
        }
    }
    g
}

fn obs_frame(
    model: &EmbodimentModel,
    qpos: &[f64],
    epoch: &str,
    now_s: f64,
    freshness_s: f64,
) -> ObservationFrame {
    ObservationFrame {
        frame_id: "manip-obs".into(),
        transform_epoch: epoch.into(),
        observations: vec![],
        joint_state: model
            .joints
            .iter()
            .filter_map(|j| {
                let adr = j.qpos_adr? as usize;
                let q = *qpos.get(adr)?;
                Some(JointStateSample::new(
                    &j.name,
                    q,
                    None,
                    now_s,
                    now_s,
                    format!("enc.{}", j.name),
                    "sim_cal",
                    epoch,
                ))
            })
            .collect(),
        as_of_s: now_s - if freshness_s <= 0.0 { 10.0 } else { 0.0 },
    }
}

pub(crate) fn observed_hold_values(
    model: &EmbodimentModel,
    manifest: &RobotManifest,
    qpos: &[f64],
    ctrl: &[f64],
) -> HashMap<String, f64> {
    let mut out = HashMap::new();
    for joint in &model.joints {
        if let Some(adr) = joint.qpos_adr {
            if let Some(q) = qpos.get(adr as usize).copied().filter(|v| v.is_finite()) {
                out.insert(joint.name.clone(), q);
            }
        }
    }
    for (i, a) in manifest.actuators.iter().enumerate() {
        if let Some(v) = ctrl.get(i).copied().filter(|v| v.is_finite()) {
            out.insert(a.name.clone(), v);
        }
    }
    out
}

fn pol_obs(
    manifest: &RobotManifest,
    episode_id: &str,
    si: usize,
    truth: &VerifierTruth,
    now: f64,
) -> PolicyObservation {
    let mut obs = policy_observation(
        manifest,
        episode_id,
        &format!("obs-{si}"),
        &TaskSpec::Hold { duration_s: 0.1 },
        VisionMode::PerfectPerception,
        truth,
        0.0,
        false,
    );
    obs.timestamp_s = now;
    obs
}

fn object_state(sc: &ManipulationScenario, pose: Se3, now: f64, freshness: f64) -> ObjectState {
    ObjectState {
        object_id: sc.object_id.clone(),
        pose: Provenanced {
            value: Some(pose),
            provenance: Provenance::UserDeclared,
            source: "perfect_perception".into(),
            as_of_s: now,
            uncertainty: None,
        },
        reference_frame: "world".into(),
        linear_velocity: Provenanced::unknown("v", now),
        angular_velocity: Provenanced::unknown("w", now),
        geometry: ObjectGeometry {
            class: GeometryClass::Box,
            bounds: Provenanced::declared([0.03, 0.03, 0.03], "scenario", now),
        },
        mass: Provenanced::unknown("mass", now),
        support_relation: Some("table".into()),
        contact_relations: vec![],
        grasp_state: GraspOccupancy::Free,
        provenance: Provenance::UserDeclared,
        timestamp_s: now,
        expires_at_s: if sc.stale { now - 0.1 } else { now + freshness },
    }
}

fn reset_episode_pose(inst: &mut crate::mujoco_exec::MujocoInstance) -> Result<(), String> {
    inst.reset_keyframe(0)
        .map(|_| ())
        .map_err(|e| e.to_string())
}

fn json_xyz(v: &Value) -> Option<[f64; 3]> {
    let a = v.as_array()?;
    if a.len() < 3 {
        return None;
    }
    Some([a[0].as_f64()?, a[1].as_f64()?, a[2].as_f64()?])
}

fn rotate_by_quat(q_wxyz: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [w, x, y, z] = q_wxyz;
    let cx = y * v[2] - z * v[1];
    let cy = z * v[0] - x * v[2];
    let cz = x * v[1] - y * v[0];
    let t = [2.0 * cx, 2.0 * cy, 2.0 * cz];
    [
        v[0] + w * t[0] + (y * t[2] - z * t[1]),
        v[1] + w * t[1] + (z * t[0] - x * t[2]),
        v[2] + w * t[2] + (x * t[1] - y * t[0]),
    ]
}

fn finger_contact_point(
    truth: &VerifierTruth,
    inspect: &Value,
    fingers: &[String],
    ee: Option<[f64; 3]>,
) -> Option<[f64; 3]> {
    let geoms = inspect.get("geoms").and_then(|v| v.as_array())?;
    let mut tips = Vec::new();
    for f in fingers {
        let Some(body_p) = body_xyz(truth, f) else {
            continue;
        };
        let body_q = truth
            .xquat
            .get(f)
            .filter(|q| q.len() >= 4)
            .map(|q| [q[0], q[1], q[2], q[3]])
            .unwrap_or([1.0, 0.0, 0.0, 0.0]);
        let mut best: Option<([f64; 3], f64)> = None;
        for g in geoms {
            if g["body"] != *f {
                continue;
            }
            if g["group"].as_i64() == Some(2) && g["contype"].as_i64() == Some(0) {
                continue;
            }
            let Some(local) = json_xyz(&g["pos"]) else {
                continue;
            };
            let w = rotate_by_quat(body_q, local);
            let p = [body_p[0] + w[0], body_p[1] + w[1], body_p[2] + w[2]];
            let dist = match ee {
                Some(e) => {
                    let d = [p[0] - e[0], p[1] - e[1], p[2] - e[2]];
                    d[0] * d[0] + d[1] * d[1] + d[2] * d[2]
                }
                None => 0.0,
            };
            if best.map(|(_, prev)| dist >= prev).unwrap_or(true) {
                best = Some((p, dist));
            }
        }
        if let Some((p, _)) = best {
            tips.push(p);
        } else {
            tips.push(body_p);
        }
    }
    if tips.is_empty() {
        return None;
    }
    let n = tips.len() as f64;
    Some([
        tips.iter().map(|p| p[0]).sum::<f64>() / n,
        tips.iter().map(|p| p[1]).sum::<f64>() / n,
        tips.iter().map(|p| p[2]).sum::<f64>() / n,
    ])
}

fn finger_midpoint(truth: &VerifierTruth, fingers: &[String]) -> Option<[f64; 3]> {
    let mut acc = [0.0; 3];
    let mut n = 0.0;
    for f in fingers {
        if let Some(p) = body_xyz(truth, f) {
            acc[0] += p[0];
            acc[1] += p[1];
            acc[2] += p[2];
            n += 1.0;
        }
    }
    if n < 1.0 {
        None
    } else {
        Some([acc[0] / n, acc[1] / n, acc[2] / n])
    }
}

fn clamp_offset(o: [f64; 3], max_m: f64) -> [f64; 3] {
    let mag = (o[0] * o[0] + o[1] * o[1] + o[2] * o[2]).sqrt();
    if mag <= max_m || mag < 1e-9 {
        o
    } else {
        let s = max_m / mag;
        [o[0] * s, o[1] * s, o[2] * s]
    }
}

fn candidate_poses(
    sc: &ManipulationScenario,
    obj: Se3,
    ee: Option<[f64; 3]>,
    finger_mid: Option<[f64; 3]>,
    skill: &str,
) -> (Se3, Se3) {
    if skill == "PUSH" {
        let n = (sc.push_dir[0] * sc.push_dir[0]
            + sc.push_dir[1] * sc.push_dir[1]
            + sc.push_dir[2] * sc.push_dir[2])
            .sqrt()
            .max(1e-9);
        let dir = [sc.push_dir[0] / n, sc.push_dir[1] / n, sc.push_dir[2] / n];
        let half = sc
            .objects
            .iter()
            .find(|o| o["name"] == sc.object_id)
            .and_then(|o| o["size"].as_array())
            .and_then(|a| a.first().and_then(|v| v.as_f64()))
            .unwrap_or(0.025);
        let face = half + 0.015;
        let contact = Se3::try_new(
            [
                obj.xyz[0] - dir[0] * face,
                obj.xyz[1] - dir[1] * face,
                obj.xyz[2] - dir[2] * face,
            ],
            obj.quat_wxyz,
        )
        .unwrap_or(obj);
        let approach = Se3::try_new(
            [
                contact.xyz[0] - dir[0] * 0.04,
                contact.xyz[1] - dir[1] * 0.04,
                contact.xyz[2] - dir[2] * 0.04,
            ],
            obj.quat_wxyz,
        )
        .unwrap_or(contact);
        return (approach, contact);
    }
    let offset = match (ee, finger_mid) {
        (Some(e), Some(f)) => clamp_offset([e[0] - f[0], e[1] - f[1], e[2] - f[2]], 0.08),
        _ if sc.planar => [0.01, 0.0, 0.0],
        _ => [0.0, 0.0, 0.04],
    };
    let grasp = Se3::try_new(
        [
            obj.xyz[0] + offset[0],
            obj.xyz[1] + offset[1],
            obj.xyz[2] + offset[2],
        ],
        obj.quat_wxyz,
    )
    .unwrap_or(obj);
    let along = if sc.planar {
        [-0.05, 0.0, 0.0]
    } else {
        [0.0, 0.0, 0.06]
    };
    let approach = Se3::try_new(
        [
            grasp.xyz[0] + along[0],
            grasp.xyz[1] + along[1],
            grasp.xyz[2] + along[2],
        ],
        obj.quat_wxyz,
    )
    .unwrap_or(grasp);
    (approach, grasp)
}

fn body_se3(truth: &VerifierTruth, name: &str) -> Option<Se3> {
    let p = body_xyz(truth, name)?;
    let q = truth
        .xquat
        .get(name)
        .filter(|q| q.len() >= 4)
        .map(|q| [q[0], q[1], q[2], q[3]])
        .unwrap_or([1.0, 0.0, 0.0, 0.0]);
    Se3::try_new(p, q).ok()
}

fn fail_from_refuse(e: SkillRefuse) -> &'static str {
    ManipulationFailure::from_refuse(e).as_str()
}

pub(crate) fn named_to_ctrl(
    manifest: &RobotManifest,
    named: &[(String, f64)],
    current: &[f64],
) -> Result<Vec<f64>, SkillRefuse> {
    let nu = manifest.nu.max(0) as usize;
    if current.len() != nu {
        return Err(SkillRefuse::InvalidCommand);
    }
    let mut action = vec![None; nu];
    let mut seen = std::collections::HashSet::new();
    for (name, v) in named {
        if !v.is_finite() {
            return Err(SkillRefuse::InvalidCommand);
        }
        let Some(i) = manifest.actuators.iter().position(|a| a.name == *name) else {
            return Err(SkillRefuse::MissingActuator);
        };
        if !seen.insert(i) {
            return Err(SkillRefuse::InvalidCommand);
        }
        if i >= action.len() {
            return Err(SkillRefuse::InvalidCommand);
        }
        let [lo, hi] = manifest.actuators[i].ctrlrange;
        action[i] = Some(v.clamp(lo.min(hi), lo.max(hi)));
    }
    action
        .into_iter()
        .collect::<Option<Vec<f64>>>()
        .ok_or(SkillRefuse::MissingActuator)
}

fn ee_workspace(truth: &VerifierTruth, bundle: &RobotBundle) -> Option<[f64; 3]> {
    let ee = privileged_ee(bundle);
    let sem = semantic_ee(bundle);
    for k in [ee.as_str(), sem.as_str()] {
        if let Some(p) = truth.named_pos.get(k).or_else(|| truth.xpos.get(k)) {
            if p.len() >= 3 {
                return Some([p[0], p[1], p[2]]);
            }
        }
    }
    None
}

fn place_object_in_workspace(
    inst: &mut crate::mujoco_exec::MujocoInstance,
    sc: &ManipulationScenario,
    model: &EmbodimentModel,
    ee_name: &str,
    ee: [f64; 3],
    finger_tip: Option<[f64; 3]>,
) -> Result<(), String> {
    let anchor = finger_tip.unwrap_or(ee);
    let half = sc
        .objects
        .iter()
        .find(|o| o["name"] == sc.object_id)
        .and_then(|o| o["size"].as_array())
        .and_then(|a| a.first().and_then(|v| v.as_f64()))
        .unwrap_or(0.025);
    let pos = if sc.skill == "PUSH" {
        use rand::{Rng, SeedableRng};
        let mut rng = rand::rngs::StdRng::seed_from_u64(sc.seed ^ 0x00A1_1CE5);
        let n_chain = model.ee_joint_chain(ee_name).map(|c| c.len()).unwrap_or(0);
        let mut units = Vec::new();
        for _ in 0..64 {
            for _ in 0..n_chain {
                units.push(rng.gen::<f64>());
            }
        }
        let cloud = realityos_semantics::workspace::reachable_ee_xyz(model, ee_name, &units);
        if cloud.is_empty() {
            if sc.planar {
                [ee[0] - 0.01, ee[1], ee[2]]
            } else {
                let z = (anchor[2] - 0.002).max(half + 0.02);
                [anchor[0], anchor[1], z]
            }
        } else {
            let contact = cloud[rng.gen_range(0..cloud.len())];
            realityos_semantics::workspace::push_object_xyz(contact, sc.push_dir, half, 0.015)
        }
    } else if sc.planar {
        [ee[0] - 0.01, ee[1], ee[2]]
    } else {
        let z = (anchor[2] - 0.002).max(half + 0.02);
        [anchor[0], anchor[1], z]
    };
    if !sc.planar {
        let table_z = (pos[2] - half - 0.02).max(0.05);
        let _ = inst.configure_body(
            "table",
            Some([anchor[0], anchor[1], table_z]),
            None,
            None,
            None,
            false,
        );
    }
    let _ = inst.set_body_pos(&sc.object_id, pos);
    let _ = inst.step(15);
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn maybe_replay_or_restart(
    auth: &mut SimAuthority<crate::driver::SharedSimPort>,
    shared: &Arc<SharedMujoco>,
    manifest: &RobotManifest,
    sc: &ManipulationScenario,
    episode_id: &str,
    si: usize,
    now: f64,
    decisions: &mut Vec<String>,
    unauthorized: &mut u64,
) -> Result<(), String> {
    if !sc.replay && !sc.restart_replay {
        return Ok(());
    }
    let command_id = format!("{episode_id}:obs-{si}");
    let truth = {
        let mut g = shared.inst.lock().map_err(|e| e.to_string())?;
        let st = g.step(0).map_err(|e| e.to_string())?;
        VerifierTruth::from_mujoco_state(st.get("state").unwrap_or(&st))
    };
    let obs = pol_obs(manifest, episode_id, si, &truth, now);
    let proposal = ActionProposal {
        robot_id: obs.robot_id.clone(),
        model_hash: obs.model_hash.clone(),
        episode_id: obs.episode_id.clone(),
        observation_id: format!("replay-obs-{si}"),
        observation_timestamp: now,
        task_id: obs.task_id.clone(),
        action: vec![0.0; manifest.nu.max(1) as usize],
        control_mode: "position".into(),
        requested_horizon_s: 0.2,
        confidence: Some(1.0),
        policy_id: "replay".into(),
        policy_version: "1".into(),
    };
    if sc.restart_replay {
        let port = SharedSimPort::new(shared.clone());
        let max_a = manifest.tau_max().into_iter().fold(1.0, f64::max);
        let plant =
            HardwareBackedPlant::new(port, &manifest.robot_id, manifest.nu.max(1) as usize, max_a);
        auth.restart(plant, manifest, now)?;
    }
    let before_w = shared.probe.snapshot().policy_ctrl_writes;
    let rec = auth.decide_and_maybe_write(
        &proposal,
        manifest,
        &TaskSpec::Hold { duration_s: 0.2 },
        &obs,
        now,
        Some(command_id),
        Some("drive"),
    );
    decisions.push(format!("replay:{:?}:{}", rec.outcome, rec.status));
    let after_w = shared.probe.snapshot().policy_ctrl_writes;
    if after_w > before_w {
        *unauthorized += after_w - before_w;
    }
    Ok(())
}

pub fn run_phase_b_development(
    sha: &str,
    out_dir: &std::path::Path,
    full: bool,
) -> Result<Value, String> {
    let (n_rel, n_grasp, n_push) = if full { (100, 300, 300) } else { (8, 16, 16) };
    let only = std::env::var("REALITYOS_PHASE_B_ROBOT").unwrap_or_default();
    let mut extra = json!({"full": full, "sha": sha, "only": only});
    if only.is_empty() || only == "arm_gripper" {
        let arm = crate::bundle::RobotBundle::load(crate::corpus::robot_dir("arm_gripper"))
            .map_err(|e| e.to_string())?;
        let (rel_a, rel_am) = run_release_matrix(&arm, n_rel, sha)?;
        let (gr_a, gr_am) = run_grasp_matrix(&arm, n_grasp, sha)?;
        let (pu_a, pu_am) = run_push_matrix(&arm, n_push, sha)?;
        write_phase_b_evidence(
            out_dir,
            "manipulation_arm_gripper.json",
            &rel_a
                .iter()
                .chain(gr_a.iter())
                .chain(pu_a.iter())
                .cloned()
                .collect::<Vec<_>>(),
            &{
                let mut m = rel_am.clone();
                m.absorb_metrics(&gr_am);
                m.absorb_metrics(&pu_am);
                m
            },
            json!({"robot":"arm_gripper"}),
        )?;
        extra["arm_gripper"] = json!({"release": rel_am, "grasp": gr_am, "push": pu_am});
    }

    if (only.is_empty() || only == "panda")
        && crate::menagerie::holdout_bundle_dir()
            .join("robot.yaml")
            .exists()
    {
        match crate::menagerie::ensure_holdout_model() {
            Ok(_) => {
                let panda =
                    crate::bundle::RobotBundle::load(crate::menagerie::holdout_bundle_dir())
                        .map_err(|e| e.to_string())?;
                let skill = std::env::var("REALITYOS_PHASE_B_SKILL").unwrap_or_default();
                let mut rel_p = Vec::new();
                let mut rel_pm = ManipulationMetrics::default();
                let mut gr_p = Vec::new();
                let mut gr_pm = ManipulationMetrics::default();
                let mut pu_p = Vec::new();
                let mut pu_pm = ManipulationMetrics::default();
                if skill.is_empty() || skill == "release" {
                    let out = run_release_matrix(&panda, n_rel, sha)?;
                    rel_p = out.0;
                    rel_pm = out.1;
                    write_phase_b_evidence(
                        out_dir,
                        "manipulation_panda_release.json",
                        &rel_p,
                        &rel_pm,
                        json!({"robot":"menagerie_panda","skill":"release"}),
                    )?;
                }
                if skill.is_empty() || skill == "grasp" {
                    let out = run_grasp_matrix(&panda, n_grasp, sha)?;
                    gr_p = out.0;
                    gr_pm = out.1;
                    write_phase_b_evidence(
                        out_dir,
                        "manipulation_panda_grasp.json",
                        &gr_p,
                        &gr_pm,
                        json!({"robot":"menagerie_panda","skill":"grasp"}),
                    )?;
                }
                if skill.is_empty() || skill == "push" {
                    let out = run_push_matrix(&panda, n_push, sha)?;
                    pu_p = out.0;
                    pu_pm = out.1;
                    write_phase_b_evidence(
                        out_dir,
                        "manipulation_panda_push.json",
                        &pu_p,
                        &pu_pm,
                        json!({"robot":"menagerie_panda","skill":"push"}),
                    )?;
                }
                if skill.is_empty() {
                    write_phase_b_evidence(
                        out_dir,
                        "manipulation_panda.json",
                        &rel_p
                            .iter()
                            .chain(gr_p.iter())
                            .chain(pu_p.iter())
                            .cloned()
                            .collect::<Vec<_>>(),
                        &{
                            let mut m = rel_pm.clone();
                            m.absorb_metrics(&gr_pm);
                            m.absorb_metrics(&pu_pm);
                            m
                        },
                        json!({"robot":"menagerie_panda","topology":"TENDON_DRIVEN_GRIPPER"}),
                    )?;
                }
                extra["panda"] = json!({"release": rel_pm, "grasp": gr_pm, "push": pu_pm});
            }
            Err(e) => extra["panda_error"] = json!(e),
        }
    }

    if (only.is_empty() || only == "ur5e") && crate::menagerie::development_bundle_ready() {
        match crate::bundle::RobotBundle::load(crate::menagerie::development_bundle_dir()) {
            Ok(ur) => match run_push_matrix(&ur, n_push, sha) {
                Ok((pu_u, pu_um)) => {
                    write_phase_b_evidence(
                        out_dir,
                        "manipulation_ur5e_push.json",
                        &pu_u,
                        &pu_um,
                        json!({"robot":"menagerie_ur5e","grasp":"NOT_APPLICABLE"}),
                    )?;
                    extra["ur5e_push"] = json!(pu_um);
                }
                Err(e) => extra["ur5e_error"] = json!(e),
            },
            Err(e) => extra["ur5e_error"] = json!(e.to_string()),
        }
    }

    if only.is_empty() || only == "iiwa14" || only == "kuka" {
        match crate::menagerie::ensure_v2_holdout_model() {
            Ok(_) => {
                match crate::bundle::RobotBundle::load(crate::menagerie::v2_holdout_bundle_dir()) {
                    Ok(kuka) => match run_push_matrix(&kuka, n_push, sha) {
                        Ok((pu_k, pu_km)) => {
                            write_phase_b_evidence(
                                out_dir,
                                "manipulation_iiwa14_push.json",
                                &pu_k,
                                &pu_km,
                                json!({"robot":"menagerie_iiwa14","grasp":"NOT_APPLICABLE"}),
                            )?;
                            extra["iiwa14_push"] = json!(pu_km);
                        }
                        Err(e) => extra["iiwa14_error"] = json!(e),
                    },
                    Err(e) => extra["iiwa14_error"] = json!(e.to_string()),
                }
            }
            Err(e) => extra["iiwa14_error"] = json!(e),
        }
    }
    Ok(extra)
}

impl ManipulationMetrics {
    fn absorb_metrics(&mut self, other: &Self) {
        self.release_success += other.release_success;
        self.release_episodes += other.release_episodes;
        self.grasp_acquisition_success += other.grasp_acquisition_success;
        self.grasp_verified_hold_success += other.grasp_verified_hold_success;
        self.grasp_episodes += other.grasp_episodes;
        self.grasp_slip += other.grasp_slip;
        self.push_contact_establishment += other.push_contact_establishment;
        self.push_task_success += other.push_task_success;
        self.push_episodes += other.push_episodes;
        self.safe_completion += other.safe_completion;
        self.expected_refusals += other.expected_refusals;
        self.unexpected_physical_violations += other.unexpected_physical_violations;
        self.authority_violations += other.authority_violations;
        self.unauthorized_writes += other.unauthorized_writes;
    }
}

pub fn write_phase_b_evidence(
    out_dir: &std::path::Path,
    name: &str,
    episodes: &[ManipulationEpisode],
    metrics: &ManipulationMetrics,
    extras: Value,
) -> Result<(), String> {
    std::fs::create_dir_all(out_dir).map_err(|e| e.to_string())?;
    let doc = json!({
        "metal": false,
        "evidence_status": SIMULATION_ONLY,
        "perception": "PERFECT_PERCEPTION",
        "simulation_only": true,
        "metrics": metrics,
        "n_episodes": episodes.len(),
        "episodes": episodes,
        "extra": extras,
    });
    std::fs::write(
        out_dir.join(name),
        serde_json::to_string_pretty(&doc).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bundle::RobotBundle;
    use crate::corpus;
    use crate::manipulation_scenarios::required_negative_kinds;
    use crate::mujoco_exec::ensure_mujoco_or_skip;
    use crate::resource_discover::discover_resources;
    use crate::runner::load_and_normalize;
    use std::path::Path;

    #[test]
    fn named_to_ctrl_refuses_zero_fill_on_dim_mismatch() {
        let man = RobotManifest {
            robot_id: "t".into(),
            nq: 1,
            nv: 1,
            nu: 1,
            nbody: 1,
            njoint: 1,
            nactuator: 1,
            nsensor: 0,
            ncamera: 0,
            timestep: 0.002,
            joints: vec![],
            actuators: vec![crate::normalize::ActuatorRecord {
                name: "act1".into(),
                transmission_target: "j0".into(),
                control_dimensions: 1,
                ctrlrange: [-1.0, 1.0],
                ctrllimited: true,
                force_range: None,
                actuator_type: "position".into(),
                transmission_kind: "joint".into(),
            }],
            sensors: vec![],
            cameras: vec![],
            bodies: vec![],
            sites: vec![],
            site_records: vec![],
            derived: crate::normalize::DerivedInterface::default(),
            model_hash: "h".into(),
            source_hash: "s".into(),
            mujoco_version: "3".into(),
            source_format: "mjcf".into(),
            lost_features: vec![],
            support_bodies: vec![],
            collision_groups: Default::default(),
            metal: false,
            evidence_status: crate::honesty::SIMULATION_ONLY.into(),
        };
        let err = named_to_ctrl(&man, &[("act1".into(), 0.1)], &[]).unwrap_err();
        assert_eq!(err, SkillRefuse::InvalidCommand);
    }

    #[test]
    fn observed_hold_values_omits_non_finite() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("planar_arm")).unwrap();
        let (i, man) = load_and_normalize(&b, &[], 0).unwrap();
        let model = crate::semantics_map::embodiment_from_manifest(&b, &man);
        let nq = man.nq.max(0) as usize;
        let nu = man.nu.max(0) as usize;
        let mut qpos = vec![0.1; nq];
        let mut ctrl = vec![0.2; nu];
        if !qpos.is_empty() {
            qpos[0] = f64::NAN;
        }
        if !ctrl.is_empty() {
            ctrl[0] = f64::NAN;
            if nu > 1 {
                ctrl[1] = 0.5;
            }
        }
        let out = observed_hold_values(&model, &man, &qpos, &ctrl);
        for v in out.values() {
            assert!(v.is_finite(), "non-finite leaked into hold map: {out:?}");
        }
        if nu > 1 {
            let name = &man.actuators[1].name;
            assert_eq!(out.get(name).copied(), Some(0.5));
        }
        crate::mujoco_exec::checkin_worker(i);
    }

    #[test]
    fn grasp_generator_covers_required_negatives() {
        let mut seen = std::collections::BTreeSet::new();
        for i in 0..300 {
            if let Some(n) = grasp_scenario(2000 + i as u64, true, i).neg {
                seen.insert(format!("{n:?}"));
            }
        }
        for k in required_negative_kinds() {
            assert!(
                seen.contains(&format!("{k:?}")),
                "missing required negative {k:?} in 300 grasp scenarios; have {seen:?}"
            );
        }
    }

    #[test]
    fn arm_gripper_hinges_are_planar_slides_are_not() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("arm_gripper")).unwrap();
        let (inst, man) = load_and_normalize(&b, &[], 0).unwrap();
        crate::mujoco_exec::checkin_worker(inst);
        assert!(
            manifest_arm_is_planar(&man),
            "three parallel hinges stay planar; slide fingers must not hide that"
        );
        let all = crate::manipulation_scenarios::is_planar_model(
            &man.joints.iter().filter_map(|j| j.axis).collect::<Vec<_>>(),
        );
        assert!(
            !all,
            "regression guard: including slide fingers must not look planar"
        );
    }

    #[test]
    fn arm_gripper_discovers_direct_fingers_not_arm_joints() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("arm_gripper")).unwrap();
        let (inst, man) = load_and_normalize(&b, &[], 0).unwrap();
        let found = discover_resources(&b, &man, &inst.inspect);
        crate::mujoco_exec::checkin_worker(inst);
        assert!(!found.is_empty());
        let r = &found[0];
        assert_eq!(r.topology, ResourceTopology::DirectJointGripper);
        assert!(!r.affected_joints.iter().any(|j| j.starts_with("joint")));
        assert!(r.actuator_inputs.iter().any(|a| a.contains("grip")));
    }

    #[test]
    fn phase_b_smoke_arm_gripper() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("arm_gripper")).unwrap();
        let sha = software_sha();
        let n = if std::env::var("REALITYOS_PHASE_B_FULL").ok().as_deref() == Some("1") {
            100
        } else {
            2
        };
        let (rel, rel_m) = run_release_matrix(&b, n.max(2), &sha).expect("release");
        assert!(rel.iter().all(|e| e.simulation_only && !e.metal));
        assert_eq!(rel_m.unauthorized_writes, 0);
        let g_n = if n > 2 { 300 } else { 3 };
        let (gr, gr_m) = run_grasp_matrix(&b, g_n, &sha).expect("grasp");
        assert!(gr
            .iter()
            .all(|e| e.unauthorized_writes == 0 || e.expected_refusal));
        assert_eq!(gr_m.unauthorized_writes, 0);
        let p_n = if n > 2 { 300 } else { 3 };
        let (pu, pu_m) = run_push_matrix(&b, p_n, &sha).expect("push");
        assert_eq!(pu_m.unauthorized_writes, 0);
        assert!(pu.iter().all(|e| e.evidence_status == SIMULATION_ONLY));
        let (inst, man) = load_and_normalize(&b, &[], 0).unwrap();
        let discovered = discover_resources(&b, &man, &inst.inspect);
        crate::mujoco_exec::checkin_worker(inst);
        let mut qualified = Vec::new();
        for r in &discovered {
            qualified.push(crate::resource_qualify::qualify_resource(&b, r).unwrap().1);
        }
        let model = {
            let mut m = embodiment_from_manifest(&b, &man);
            m.resources = qualified.clone();
            m
        };
        let mut pos_g = Vec::new();
        for i in [15, 16, 20] {
            let sc = grasp_scenario(2014 + i as u64, true, i);
            let (ep, inst, _) =
                run_skill_episode(&b, &model, &qualified, &sc, &sha, "GRASP", None).unwrap();
            crate::mujoco_exec::checkin_worker(inst);
            pos_g.push(ep);
        }
        let mut pos_p = Vec::new();
        for i in [15, 16, 20] {
            let sc = push_scenario(3014 + i as u64, true, i);
            let (ep, inst, _) =
                run_skill_episode(&b, &model, &[], &sc, &sha, "PUSH", None).unwrap();
            crate::mujoco_exec::checkin_worker(inst);
            pos_p.push(ep);
        }
        eprintln!(
            "phase_b_smoke release={:?} grasp={:?} push={:?} rel_results={:?} gr_results={:?} pu_results={:?} pos_g={:?} pos_p={:?}",
            rel_m,
            gr_m,
            pu_m,
            rel.iter().map(|e| (&e.task_result, &e.failure_taxonomy, e.ctrl_writes)).collect::<Vec<_>>(),
            gr.iter().map(|e| (&e.task_result, &e.failure_taxonomy, e.ctrl_writes)).collect::<Vec<_>>(),
            pu.iter().map(|e| (&e.task_result, &e.failure_taxonomy, e.ctrl_writes)).collect::<Vec<_>>(),
            pos_g.iter().map(|e| (&e.task_result, &e.failure_taxonomy, e.ctrl_writes, &e.evidence_used, &e.authority_decisions, &e.commands)).collect::<Vec<_>>(),
            pos_p.iter().map(|e| (&e.task_result, &e.failure_taxonomy, e.ctrl_writes, &e.evidence_used, &e.authority_decisions)).collect::<Vec<_>>()
        );
    }

    #[test]
    fn phase_b_development_matrices() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let full = std::env::var("REALITYOS_PHASE_B_FULL").ok().as_deref() == Some("1");
        let evidence =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../docs/superpowers/evidence");
        let sha = software_sha();
        let extra = run_phase_b_development(&sha, &evidence, full).expect("phase b development");
        eprintln!("phase_b_development {extra}");
        assert_eq!(extra["sha"], sha);
    }

    #[test]
    fn panda_tendon_resource_if_present() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let dir = crate::menagerie::holdout_bundle_dir();
        if !dir.join("robot.yaml").exists() {
            return;
        }
        if crate::menagerie::ensure_holdout_model().is_err() {
            return;
        }
        let b = RobotBundle::load(&dir).unwrap();
        let (inst, man) = load_and_normalize(&b, &[], 0).unwrap();
        let found = discover_resources(&b, &man, &inst.inspect);
        crate::mujoco_exec::checkin_worker(inst);
        assert!(!found.is_empty(), "panda yaml declares a gripper");
        let r = &found[0];
        assert_eq!(r.topology, ResourceTopology::TendonDrivenGripper);
        assert!(r.coupling.tendon.is_some());
        assert!(!r.coupling.equalities.is_empty());
        assert_eq!(r.actuator_inputs.len(), 1);
    }

    #[test]
    fn push_pipeline_funnel_keeps_unauthorized_writes_zero() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let sha = software_sha();
        let arm = RobotBundle::load(corpus::robot_dir("arm_gripper")).unwrap();
        let (eps, _) = run_push_matrix_from(&arm, 16, &sha, 9).expect("push matrix");
        let mut funnel = crate::push_pipeline::PushFunnel::default();
        for ep in &eps {
            assert_eq!(
                ep.unauthorized_writes, 0,
                "PUSH must not mint unauthorized writes"
            );
            let ev = crate::push_pipeline::evidence_from_episode_fields(
                &ep.task_result,
                ep.failure_taxonomy.as_deref().unwrap_or(""),
                &ep.evidence_used,
                !ep.contacts.is_empty(),
                ep.expected_refusal,
            );
            funnel.absorb(&ev, ep.unauthorized_writes);
            assert!(
                !ep.first_stage_entered.is_empty() || ep.expected_refusal,
                "pipeline stages must be labeled"
            );
        }
        eprintln!(
            "push_funnel n={} contact={} p_task_given_contact={:.3} p_contact_given_approach={:.3} unauthorized={}",
            funnel.n,
            funnel.n_contact,
            funnel.p_task_given_contact(),
            funnel.p_contact_given_approach(),
            funnel.n_unauthorized_writes
        );
        assert_eq!(funnel.n_unauthorized_writes, 0);
        if let Ok(path) = std::env::var("REALITYOS_PUSH_FUNNEL_OUT") {
            let recs: Vec<serde_json::Value> = eps
                .iter()
                .map(|ep| {
                    serde_json::json!({
                        "episode_id": format!("{}:{}:{}", ep.robot_id, ep.world_seed, ep.skill_contract),
                        "skill": ep.skill_contract,
                        "task_result": ep.task_result,
                        "failure_taxonomy": ep.failure_taxonomy,
                        "first_stage_entered": ep.first_stage_entered,
                        "last_stage_completed": ep.last_stage_completed,
                        "earliest_pipeline_failed": ep.earliest_pipeline_failed,
                        "unauthorized_writes": ep.unauthorized_writes,
                        "evidence_used": ep.evidence_used,
                    })
                })
                .collect();
            let report = serde_json::json!({
                "condition": "after_workspace_aware_sampling",
                "baseline_arm_gripper_idx_offset_9_n16": {
                    "n_contact": 0,
                    "p_task_success_given_contact": 0.0,
                    "note": "pre-sampler: randomized positives refused UNREACHABLE before contact"
                },
                "n": funnel.n,
                "n_contact": funnel.n_contact,
                "n_approach": funnel.n_approach,
                "p_task_success_given_contact": funnel.p_task_given_contact(),
                "p_contact_given_approach": funnel.p_contact_given_approach(),
                "unauthorized_writes": funnel.n_unauthorized_writes,
                "episodes": recs,
            });
            std::fs::write(&path, serde_json::to_string_pretty(&report).unwrap())
                .unwrap_or_else(|e| panic!("write {path}: {e}"));
        }
    }

    #[test]
    fn push_holdout_funnel_frozen_no_retune() {
        if std::env::var("REALITYOS_PUSH_HOLDOUT_OUT").is_err() {
            return;
        }
        if !ensure_mujoco_or_skip() {
            return;
        }
        let dir = crate::menagerie::manipulation_holdout_bundle_dir();
        if !dir.join("robot.yaml").exists() {
            let path = std::env::var("REALITYOS_PUSH_HOLDOUT_OUT").unwrap();
            std::fs::write(
                &path,
                serde_json::json!({
                    "holdout_bundle_present": false,
                    "episodes": [],
                    "tuned_on_holdout": false,
                })
                .to_string(),
            )
            .unwrap();
            return;
        }
        if crate::menagerie::ensure_manipulation_holdout_model().is_err() {
            return;
        }
        let sha = software_sha();
        let b = RobotBundle::load(&dir).unwrap();
        let (eps, _) = run_push_matrix_from(&b, 12, &sha, 9).expect("holdout push");
        let mut funnel = crate::push_pipeline::PushFunnel::default();
        let recs: Vec<serde_json::Value> = eps
            .iter()
            .map(|ep| {
                let ev = crate::push_pipeline::evidence_from_episode_fields(
                    &ep.task_result,
                    ep.failure_taxonomy.as_deref().unwrap_or(""),
                    &ep.evidence_used,
                    !ep.contacts.is_empty(),
                    ep.expected_refusal,
                );
                funnel.absorb(&ev, ep.unauthorized_writes);
                serde_json::json!({
                    "episode_id": format!("{}:{}:{}", ep.robot_id, ep.world_seed, ep.skill_contract),
                    "skill": ep.skill_contract,
                    "task_result": ep.task_result,
                    "failure_taxonomy": ep.failure_taxonomy,
                    "first_stage_entered": ep.first_stage_entered,
                    "last_stage_completed": ep.last_stage_completed,
                    "earliest_pipeline_failed": ep.earliest_pipeline_failed,
                    "unauthorized_writes": ep.unauthorized_writes,
                })
            })
            .collect();
        let path = std::env::var("REALITYOS_PUSH_HOLDOUT_OUT").unwrap();
        let report = serde_json::json!({
            "tuned_on_holdout": false,
            "n": funnel.n,
            "n_contact": funnel.n_contact,
            "p_task_success_given_contact": funnel.p_task_given_contact(),
            "unauthorized_writes": funnel.n_unauthorized_writes,
            "episodes": recs,
        });
        std::fs::write(&path, serde_json::to_string_pretty(&report).unwrap()).unwrap();
        assert_eq!(funnel.n_unauthorized_writes, 0);
    }
}
