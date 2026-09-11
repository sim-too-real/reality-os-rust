//! Manipulation episode runner. Skills propose; Reality OS authorizes; verifier judges.

use crate::authority::{AuthorityOutcome, SimAuthority};
use crate::bundle::RobotBundle;
use crate::driver::{SharedMujoco, SharedSimPort};
use crate::honesty::SIMULATION_ONLY;
use crate::manipulation_scenarios::{
    grasp_scenario, is_planar_model, push_scenario, release_scenario, ManipulationScenario, NegKind,
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
use realityos_semantics::object::{
    GeometryClass, GraspOccupancy, ObjectGeometry, ObjectState,
};
use realityos_semantics::observation::{JointStateSample, ObservationFrame};
use realityos_semantics::plan::{SkillPlan, SkillStep};
use realityos_semantics::provenance::{Provenance, Provenanced};
use realityos_semantics::push::{compile_push, PushCandidate};
use realityos_semantics::reach::compile_reach;
use realityos_semantics::release::compile_release;
use realityos_semantics::resource::{
    ControlledResource, QualificationStatus, ResourceTopology,
};
use realityos_semantics::skill::SkillRefuse;
use realityos_semantics::transform::{Se3, TransformEdge, TransformGraph};
use realityos_semantics::world::WorldState;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

const HORIZON_S: f64 = 1.0;
const CONTROL_HZ: f64 = 40.0;
const REACH_ATTEMPTS: u32 = 2;

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
        eps.push(ep);
    }
    Ok((eps, metrics))
}

pub fn run_grasp_matrix(
    bundle: &RobotBundle,
    n: usize,
    sha: &str,
) -> Result<(Vec<ManipulationEpisode>, ManipulationMetrics), String> {
    let (inst, manifest) = load_and_normalize(bundle, &[], 0)?;
    let inspect = inst.inspect.clone();
    let mut model = embodiment_from_manifest(bundle, &manifest);
    let planar = is_planar_model(
        &manifest
            .joints
            .iter()
            .filter_map(|j| j.axis)
            .collect::<Vec<_>>(),
    );
    let discovered = discover_resources(bundle, &manifest, &inspect);
    checkin_worker(inst);
    let mut qualified_res = Vec::new();
    for r in &discovered {
        let (q, rr) = crate::resource_qualify::qualify_resource(bundle, r)?;
        let _ = q;
        qualified_res.push(rr);
    }
    model.resources = qualified_res.clone();
    let mut eps = Vec::new();
    let mut metrics = ManipulationMetrics::default();
    for i in 0..n {
        let sc = grasp_scenario(2000 + i as u64, planar, i);
        let ep = run_skill_episode(bundle, &model, &qualified_res, &sc, sha, "GRASP")?;
        metrics.absorb(&ep);
        eps.push(ep);
    }
    Ok((eps, metrics))
}

pub fn run_push_matrix(
    bundle: &RobotBundle,
    n: usize,
    sha: &str,
) -> Result<(Vec<ManipulationEpisode>, ManipulationMetrics), String> {
    let (inst, manifest) = load_and_normalize(bundle, &[], 0)?;
    let model = embodiment_from_manifest(bundle, &manifest);
    let planar = is_planar_model(
        &manifest
            .joints
            .iter()
            .filter_map(|j| j.axis)
            .collect::<Vec<_>>(),
    );
    checkin_worker(inst);
    let mut eps = Vec::new();
    let mut metrics = ManipulationMetrics::default();
    for i in 0..n {
        let sc = push_scenario(3000 + i as u64, planar, i);
        let ep = run_skill_episode(bundle, &model, &[], &sc, sha, "PUSH")?;
        metrics.absorb(&ep);
        eps.push(ep);
    }
    Ok((eps, metrics))
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
    if matches!(sc.neg, Some(NegKind::GripperUnavailable) | Some(NegKind::UnsupportedCoupling))
    {
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
    let obs = obs_frame(model, &initial.qpos, &model.calibration_epoch, now, freshness);
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
        Ok(plan) => execute_plan(
            bundle, inst, manifest, model, sc, sha, plan, &resource, &initial, now,
        ),
    }
}

fn run_skill_episode(
    bundle: &RobotBundle,
    model: &EmbodimentModel,
    resources: &[ControlledResource],
    sc: &ManipulationScenario,
    sha: &str,
    skill: &str,
) -> Result<ManipulationEpisode, String> {
    let (mut inst, manifest) = load_and_normalize(bundle, &sc.objects, sc.seed)?;
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
            place_object_in_workspace(&mut inst, sc, ee)?;
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
    let (approach, grasp_or_contact) = candidate_poses(sc, obj_pose);
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
        resource
            .as_ref()
            .is_some_and(|r| r.qualification == QualificationStatus::Qualified),
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
            checkin_worker(inst);
            return Ok(refused_episode(
                bundle,
                model,
                sc,
                sha,
                "skill.grasp",
                Some("RESOURCE_UNSUPPORTED"),
                true,
            ));
        };
        let opening = opening_from_truth(&initial, resource, &manifest);
        let grip = derive_gripper_state(resource, opening, Some(0.0), Some(false), Some(false), now);
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
        Err(e) => {
            checkin_worker(inst);
            Ok(refused_episode(
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
            ))
        }
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
            execute_plan(
                bundle, inst, manifest, model, sc, sha, plan, &res, &initial, now,
            )
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
) -> Result<ManipulationEpisode, String> {
    if sc.crash_controller {
        crate::mujoco_exec::checkin_worker(inst);
        return Ok(refused_episode(
            bundle,
            model,
            sc,
            sha,
            &plan.contract_id,
            Some("CONTROLLER_FAILURE"),
            true,
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
    let plant = HardwareBackedPlant::new(port, &manifest.robot_id, manifest.nu.max(1) as usize, max_a);
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
                        checkin_worker(inst);
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
                        return Ok(ep);
                    }
                    Ok(ctrl) => {
                        let rec = write_ctrl(
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
                        )?;
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
                            if let Some(p) = truth
                                .named_pos
                                .get(&ee)
                                .or_else(|| truth.xpos.get(&ee))
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
                            let obs = obs_frame(
                                model,
                                &truth.qpos,
                                &model.calibration_epoch,
                                now,
                                0.5,
                            );
                            let g = graph_from_truth(
                                model,
                                &truth,
                                &model.calibration_epoch,
                                now,
                            );
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
                                let rec = write_ctrl(
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
                                )?;
                                decisions.push(format!(
                                    "reach-retry:{:?}:{}",
                                    rec.outcome, rec.status
                                ));
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
                let mut current = current_map(model, &truth);
                for a in &model.actuators {
                    if let Some(i) = manifest.actuators.iter().position(|x| x.name == a.name) {
                        if let Some(v) = truth.ctrl.get(i) {
                            current.insert(a.name.clone(), *v);
                        }
                    }
                }
                let lowered = lower_actuator_commands(model, cmds, &current)
                    .map_err(|e| format!("{e:?}"))?;
                let action = named_to_ctrl(&manifest, &lowered, &truth.ctrl);
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
                    let (t2, n2) = step_sim(&shared, &manifest, &mut auth, now)?;
                    truth = t2;
                    now = n2;
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
                                &semantic_ee(bundle),
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
                            let rec = write_ctrl(
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
                            )?;
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
    checkin_worker(inst);

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
    Ok(finish_episode(
        bundle, model, sc, sha, &plan, resource, &truth, verdict, decisions, commands, ctrl_writes,
        unauthorized, None,
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
    ManipulationEpisode {
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
    }
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
) -> Result<crate::authority::AuthorityRecord, String> {
    let obs = pol_obs(manifest, episode_id, si, truth, now);
    let current = current_map(model, truth);
    let lowered = lower_named_targets(model, &ctrl.targets, &current).map_err(|e| format!("{e:?}"))?;
    let action = named_to_ctrl(manifest, &lowered, &truth.ctrl);
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
        let Ok(pose) = Se3::try_new([pos[0], pos[1], pos[2]], [quat[0], quat[1], quat[2], quat[3]])
        else {
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

fn current_map(model: &EmbodimentModel, truth: &VerifierTruth) -> HashMap<String, f64> {
    let mut out = HashMap::new();
    for joint in &model.joints {
        if let Some(adr) = joint.qpos_adr {
            if let Some(q) = truth.qpos.get(adr as usize) {
                out.insert(joint.name.clone(), *q);
            }
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

fn candidate_poses(sc: &ManipulationScenario, obj: Se3) -> (Se3, Se3) {
    if sc.planar {
        let approach = Se3::try_new(
            [obj.xyz[0] - 0.03, obj.xyz[1], obj.xyz[2]],
            obj.quat_wxyz,
        )
        .unwrap_or(obj);
        (approach, obj)
    } else {
        let approach = Se3::try_new(
            [obj.xyz[0], obj.xyz[1], obj.xyz[2] + 0.08],
            obj.quat_wxyz,
        )
        .unwrap_or(obj);
        let grasp = Se3::try_new(
            [obj.xyz[0], obj.xyz[1], obj.xyz[2] + 0.02],
            obj.quat_wxyz,
        )
        .unwrap_or(obj);
        (approach, grasp)
    }
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

fn named_to_ctrl(
    manifest: &RobotManifest,
    named: &[(String, f64)],
    current: &[f64],
) -> Vec<f64> {
    let mut action = if current.len() == manifest.nu.max(0) as usize {
        current.to_vec()
    } else {
        vec![0.0; manifest.nu.max(0) as usize]
    };
    for (name, v) in named {
        if let Some(i) = manifest.actuators.iter().position(|a| a.name == *name) {
            if i < action.len() {
                let [lo, hi] = manifest.actuators[i].ctrlrange;
                action[i] = v.clamp(lo.min(hi), lo.max(hi));
            }
        }
    }
    for (i, a) in manifest.actuators.iter().enumerate() {
        if i < action.len() {
            let [lo, hi] = a.ctrlrange;
            action[i] = action[i].clamp(lo.min(hi), lo.max(hi));
        }
    }
    action
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
    ee: [f64; 3],
) -> Result<(), String> {
    let pos = if sc.planar {
        [ee[0] - 0.01, ee[1], ee[2]]
    } else {
        [
            (ee[0] * 0.25 + 0.42 * 0.75).clamp(0.30, 0.55),
            ee[1].clamp(-0.12, 0.12),
            0.445,
        ]
    };
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
    decisions.push(format!(
        "replay:{:?}:{}",
        rec.outcome, rec.status
    ));
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
        && crate::menagerie::holdout_bundle_dir().join("robot.yaml").exists()
    {
        match crate::menagerie::ensure_holdout_model() {
            Ok(_) => {
                let panda = crate::bundle::RobotBundle::load(crate::menagerie::holdout_bundle_dir())
                    .map_err(|e| e.to_string())?;
                let (rel_p, rel_pm) = run_release_matrix(&panda, n_rel, sha)?;
                let (gr_p, gr_pm) = run_grasp_matrix(&panda, n_grasp, sha)?;
                let (pu_p, pu_pm) = run_push_matrix(&panda, n_push, sha)?;
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
        assert!(gr.iter().all(|e| e.unauthorized_writes == 0 || e.expected_refusal));
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
            pos_g.push(run_skill_episode(&b, &model, &qualified, &sc, &sha, "GRASP").unwrap());
        }
        let mut pos_p = Vec::new();
        for i in [15, 16, 20] {
            let sc = push_scenario(3014 + i as u64, true, i);
            pos_p.push(run_skill_episode(&b, &model, &[], &sc, &sha, "PUSH").unwrap());
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
}
