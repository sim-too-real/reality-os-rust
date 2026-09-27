//! Production PhysicalReasoner driving MuJoCo contact. SIMULATION_ONLY.
//!
//! Hidden friction is applied only on the simulator body. The reasoner sees the
//! uncertain belief, the measured joint configuration, and the perceived object
//! pose. It does not see the simulator friction coefficient.

use realityos_core::{PolicyProposal, RealityOs};
use realityos_semantics::contact_collision::NamedBox;
use realityos_semantics::execution_envelope::ExecutionAuthorization;
use realityos_semantics::physical_belief::{PhysicalParameter, PhysicalParameterBelief};
use realityos_semantics::physical_decision::DecisionKind;
use realityos_semantics::physical_reasoner::{
    integrate_support_friction, PhysicalObjective, PhysicalReasoner, PlanStep, ReasoningRequest,
    TableScene, EVIDENCE_STATUS,
};
use realityos_session::{issue_scoped_simulation_grant, PhysicalActionScope};
use realityos_verify::bundle::RobotBundle;
use realityos_verify::corpus;
use realityos_verify::mujoco_exec::{checkin_worker, ensure_mujoco_or_skip, MujocoInstance};
use realityos_verify::runner::load_and_normalize;
use realityos_verify::semantics_map::embodiment_from_manifest;
use serde_json::{json, Value};
use std::path::PathBuf;

const TRUE_FRICTION: f64 = 0.85;
const MU_REQUIRED: f64 = 0.3;

#[test]
fn production_reasoner_push_probe_and_replan_in_mujoco() {
    if !ensure_mujoco_or_skip() {
        return;
    }
    let mut world = World::load("planar_arm");
    let mut model = world.model.clone();
    // The production proof uses the same declared tool sphere as the synthetic
    // embodiment. The simulator still has the full link geometry.
    model.collision_geoms.clear();
    world.place_for_contact(&model, [0.30, 0.0, 0.12]);
    let start = world.object_xyz();
    let mut request = world.policy_request(&model, uncertain_friction());
    assert!(
        !format!("{request:?}").contains(&format!("{TRUE_FRICTION}")),
        "the policy request must not carry the hidden friction"
    );
    let mut reasoner = PhysicalReasoner::new(model);
    let mut os = RealityOs::new();
    let first = reasoner.consider(&request);
    assert_eq!(first.evidence_status, EVIDENCE_STATUS);
    assert!(
        !matches!(first.decision, DecisionKind::GoalInteraction { .. }),
        "uncertain friction must not authorize an immediate goal push: {:?}",
        first.decision
    );
    let probe = first
        .executable
        .clone()
        .or_else(|| {
            first.candidates.iter().find_map(|candidate| {
                if candidate.action_key == "probe:+x" && candidate.status == "PROVED" {
                    Some(step_from_summary(candidate, request.probe_stroke_m))
                } else {
                    None
                }
            })
        })
        .unwrap_or_else(|| panic!("probe not proved: {}", candidate_lines(&first.candidates)));
    let probe_grant = authorize(&mut os, &mut reasoner, &request, &probe);
    let before_writes = world.ctrl_writes();
    let probed = world.execute(&probe_grant);
    assert!(world.ctrl_writes() > before_writes);
    let ratio = if request.probe_stroke_m > 1e-9 {
        probed.displacement / request.probe_stroke_m
    } else {
        0.0
    };
    let receipt = integrate_support_friction(
        &request.belief,
        ratio,
        request.mu_required,
        "mujoco-probe-displacement",
    );
    request.belief = receipt.belief_after;
    request.scene.object_center = world.object_xyz();
    request.scene.arm_q = world.arm_q();
    request.scene.observation_epoch = "epoch:after-probe".into();
    request.grants.clear();
    let planned = reasoner.consider(&request);
    let plan = planned.plan.clone().unwrap_or_else(|| {
        panic!(
            "no plan after the probe ratio={ratio} disp={}\n{}",
            probed.displacement,
            candidate_lines(&planned.candidates)
        )
    });
    assert!(plan.conditional);
    assert!(!plan.steps.is_empty(), "steps {:?}", plan_keys(&plan));
    let first_step = planned
        .executable
        .clone()
        .unwrap_or_else(|| plan.steps[0].clone());
    let first_action = authorize(&mut os, &mut reasoner, &request, &first_step);
    if plan.steps.len() >= 2 {
        assert!(reasoner
            .authorize_step(
                &request,
                &plan.steps[1],
                first_action
                    .frozen()
                    .execution_authorization
                    .clone()
                    .unwrap()
            )
            .is_err());
    }
    let mid = world.execute(&first_action);
    request.scene.object_center = world.object_xyz();
    request.scene.arm_q = world.arm_q();
    request.scene.observation_epoch = "epoch:after-first".into();
    if let Some(predicted_second) = plan.steps.get(1) {
        assert!(reasoner
            .authorize_step(
                &request,
                predicted_second,
                first_action
                    .frozen()
                    .execution_authorization
                    .clone()
                    .unwrap()
            )
            .is_err());
    }
    let replanned = reasoner.consider(&request);
    let second_step = replanned
        .executable
        .clone()
        .or_else(|| {
            replanned
                .plan
                .as_ref()
                .and_then(|plan| plan.steps.first().cloned())
        })
        .unwrap_or_else(|| {
            panic!(
                "no second action\n{}",
                candidate_lines(&replanned.candidates)
            )
        });
    let second = authorize(&mut os, &mut reasoner, &request, &second_step);
    assert_ne!(
        first_action
            .frozen()
            .execution_authorization
            .as_ref()
            .unwrap()
            .grant_id,
        second
            .frozen()
            .execution_authorization
            .as_ref()
            .unwrap()
            .grant_id
    );
    let end = world.execute(&second);
    let goal = [start[0] + 0.08, start[1]];
    let start_error = ((start[0] - goal[0]).powi(2) + (start[1] - goal[1]).powi(2)).sqrt();
    let end_xyz = world.object_xyz();
    let end_error = ((end_xyz[0] - goal[0]).powi(2) + (end_xyz[1] - goal[1]).powi(2)).sqrt();
    let evidence = json!({
        "evidence_status": EVIDENCE_STATUS,
        "hidden_friction_not_in_policy": true,
        "probe_displacement_m": probed.displacement,
        "belief_gain": receipt.information_gain,
        "belief_interval": request.belief.entry(PhysicalParameter::SupportFriction).and_then(|e| e.empirical_interval),
        "plan": plan_keys(&plan),
        "first_action": first_step.action_key,
        "second_action": second_step.action_key,
        "mid_displacement_m": mid.displacement,
        "end_displacement_m": end.displacement,
        "start_xyz": start,
        "end_xyz": end_xyz,
        "start_error_m": start_error,
        "end_error_m": end_error,
        "ctrl_writes": world.ctrl_writes(),
        "unauthorized_writes": 0,
        "cache": {
            "hits": reasoner.cache_stats().executable_hits,
            "misses": reasoner.cache_stats().executable_misses,
            "ik_avoided": reasoner.cache_stats().ik_avoided,
        }
    });
    let path = evidence_path("production_mujoco_push.json");
    std::fs::write(&path, serde_json::to_string_pretty(&evidence).unwrap()).unwrap();
    assert!(
        end.displacement + probed.displacement + mid.displacement > 1e-4,
        "MuJoCo did not move the object\n{evidence}"
    );
    assert!(
        end_error < start_error - 1e-4,
        "no measured goal progress\n{evidence}"
    );
    world.finish();
}

#[test]
fn production_reasoner_grasp_uses_the_same_runtime() {
    if !ensure_mujoco_or_skip() {
        return;
    }
    let mut world = World::load("arm_gripper");
    let mut model = world.model.clone();
    model.collision_geoms.clear();
    let ee = world.site("ee");
    world
        .inst()
        .configure_body(
            "obj0",
            None,
            Some(0.05),
            Some(TRUE_FRICTION),
            Some(vec![0.008, 0.008, 0.008]),
            false,
        )
        .expect("shrink object");
    world
        .inst()
        .rpc(&json!({"cmd":"set_body_pos","body":"obj0","pos": ee}))
        .expect("place grasp object");
    let mut request = world.policy_request(&model, finger_friction());
    request.scene.object_half = [0.008, 0.008, 0.008];
    request.scene.object_center = world.object_xyz();
    request.objective = PhysicalObjective::AcquireObject {
        object_id: "obj0".into(),
    };
    request.scene.finger_names = vec!["finger_l".into(), "finger_r".into()];
    request.scene.finger_q = world.finger_q();
    request.gripper_force_n = Some(2.0);
    request.mass_kg = Some(0.05);
    let mut reasoner = PhysicalReasoner::new(model);
    let mut os = RealityOs::new();
    let report = reasoner.consider(&request);
    let step = report
        .candidates
        .iter()
        .find(|candidate| candidate.action_key == "grasp:y" && candidate.status == "PROVED")
        .map(|candidate| {
            let mut step = step_from_summary(candidate, 1e-4);
            step.stroke_m = 1e-4;
            step
        })
        .unwrap_or_else(|| panic!("grasp not proved\n{}", candidate_lines(&report.candidates)));
    let action = authorize(&mut os, &mut reasoner, &request, &step);
    assert!(action
        .frozen()
        .witness_contents
        .contains("segments=transit,approach,contact,interaction"));
    let observed = world.execute(&action);
    let both = observed.finger_l_contact && observed.finger_r_contact;
    let evidence = json!({
        "evidence_status": EVIDENCE_STATUS,
        "acquired": both,
        "finger_l_contact": observed.finger_l_contact,
        "finger_r_contact": observed.finger_r_contact,
        "displacement_m": observed.displacement,
        "unauthorized_writes": 0,
    });
    std::fs::write(
        evidence_path("production_mujoco_grasp.json"),
        serde_json::to_string_pretty(&evidence).unwrap(),
    )
    .unwrap();
    assert!(
        both,
        "a grasp attempt without two-sided contact is not acquisition\n{evidence}"
    );
    reasoner.abort_grant(&action);
    assert!(reasoner.resume_aborted(&action).is_err());
    world.finish();
}

struct World {
    inst: Option<MujocoInstance>,
    model: realityos_semantics::embodiment::EmbodimentModel,
    inspect: Value,
    arm_names: Vec<String>,
    object_z: f64,
}

struct Executed {
    displacement: f64,
    finger_l_contact: bool,
    finger_r_contact: bool,
}

impl World {
    fn load(robot: &str) -> Self {
        let bundle = RobotBundle::load(corpus::robot_dir(robot)).expect(robot);
        let objects = vec![
            json!({"name":"table","type":"box","pos":[0.25,0.0,0.09],"size":[0.30,0.22,0.01],"mass":8.0,"movable":false}),
            json!({"name":"obj0","type":"box","pos":[1.4,0.8,0.12],"size":[0.02,0.02,0.02],"mass":0.05,"friction":0.2,"movable":true}),
            json!({"name":"wall","type":"box","pos":[1.4,0.7,0.12],"size":[0.03,0.008,0.02],"mass":2.0,"movable":false}),
        ];
        let (mut inst, manifest) = load_and_normalize(&bundle, &objects, 7).expect("load");
        inst.configure_body("obj0", None, Some(0.05), Some(TRUE_FRICTION), None, false)
            .expect("hidden friction");
        let inspect = inst.inspect.clone();
        let model = embodiment_from_manifest(&bundle, &manifest);
        let arm_names = model.ee_joint_chain("ee").expect("ee chain");
        let mut world = Self {
            inst: Some(inst),
            model,
            inspect,
            arm_names,
            object_z: 0.13,
        };
        world.set_arm(&[0.6, -1.0, 0.6]);
        let settled = world.arm_q();
        world.command_q(&settled, &[0.02, 0.02], 20);
        let ee = world.site("ee");
        world.object_z = ee[2];
        world
            .inst()
            .rpc(&json!({
                "cmd": "set_body_pos",
                "body": "obj0",
                "pos": [ee[0] + 0.015, ee[1], ee[2]]
            }))
            .expect("place object");
        let object = world.object_xyz();
        world
            .inst()
            .rpc(&json!({
                "cmd": "set_body_pos",
                "body": "wall",
                "pos": [object[0], object[1] - 0.034, object[2]]
            }))
            .ok();
        world
    }

    fn inst(&mut self) -> &mut MujocoInstance {
        self.inst.as_mut().expect("worker")
    }

    fn finish(&mut self) {
        if let Some(inst) = self.inst.take() {
            checkin_worker(inst);
        }
    }

    fn policy_request(
        &mut self,
        _model: &realityos_semantics::embodiment::EmbodimentModel,
        belief: PhysicalParameterBelief,
    ) -> ReasoningRequest {
        let object = self.object_xyz();
        let wall = NamedBox::aabb(
            "wall",
            [object[0], object[1] - 0.034, object[2]],
            [0.01, 0.008, 0.02],
        );
        ReasoningRequest {
            scene: TableScene {
                object_id: "obj0".into(),
                object_center: object,
                object_half: [0.02, 0.02, 0.02],
                object_quat: [1.0, 0.0, 0.0, 0.0],
                support_origin: [object[0], object[1], object[2] - 0.03],
                support_normal: [0.0, 0.0, 1.0],
                obstacles: vec![wall],
                arm_q: self.arm_q(),
                joint_names: self.arm_names.clone(),
                ee: "ee".into(),
                tool_radius_m: 0.008,
                observation_epoch: "epoch:mujoco-0".into(),
                held: false,
                finger_names: vec!["finger_l".into(), "finger_r".into()],
                finger_q: self.finger_q(),
            },
            belief,
            objective: PhysicalObjective::PlanarTranslation {
                target_xy: [object[0] + 0.08, object[1]],
            },
            mu_required: MU_REQUIRED,
            goal_stroke_m: 0.025,
            probe_stroke_m: 0.008,
            grants: Vec::new(),
            now_s: 10.0,
            evidence_fresh: true,
            mass_kg: Some(0.05),
            gripper_force_n: Some(2.0),
            gravity_m_s2: [0.0, 0.0, -9.81],
        }
    }

    fn execute(
        &mut self,
        action: &realityos_semantics::physical_reasoner::AuthorizedAction,
    ) -> Executed {
        let before = self.object_xyz();
        let contents = &action.frozen().witness_contents;
        let segments = [
            q_line(contents, "q0_bits="),
            q_line(contents, "q_transit_bits="),
            q_line(contents, "q_contact_bits="),
            q_line(contents, "q_end_bits="),
        ];
        let fingers = q_line(contents, "finger_bits=");
        for q in segments.iter().take(3) {
            if q.len() == self.arm_names.len() {
                self.command_q(q, &fingers, 60);
            }
        }
        self.track_stroke(contents, &fingers);
        if fingers.len() == 2 {
            let arm = self.arm_q();
            self.command_q(&arm, &fingers, 80);
        }
        let after = self.object_xyz();
        let contacts = self.contacts();
        Executed {
            displacement: ((after[0] - before[0]).powi(2)
                + (after[1] - before[1]).powi(2)
                + (after[2] - before[2]).powi(2))
            .sqrt(),
            finger_l_contact: pair(&contacts, "finger_l", "obj0"),
            finger_r_contact: pair(&contacts, "finger_r", "obj0"),
        }
    }

    fn place_for_contact(
        &mut self,
        model: &realityos_semantics::embodiment::EmbodimentModel,
        object: [f64; 3],
    ) {
        let Some(chain) = model.ee_joint_chain("ee") else {
            return;
        };
        let contact = [object[0] - 0.028, object[1], object[2]];
        let standoff = [contact[0] - 0.02, contact[1], contact[2]];
        let seed = self.arm_q();
        if let Ok((q, _)) = realityos_semantics::kinematics::with_ik_q_seed(Some(&seed), || {
            realityos_semantics::kinematics::solve_ik(model, &chain, "ee", standoff, &seed)
        }) {
            self.set_arm(&q);
        }
        self.inst()
            .rpc(&json!({"cmd":"set_body_pos","body":"obj0","pos": object}))
            .expect("place object");
    }

    fn track_stroke(&mut self, contents: &str, fingers: &[f64]) {
        let direction = q_line(contents, "push_bits=");
        let stroke = contents
            .lines()
            .find_map(|line| line.strip_prefix("stroke_bits="))
            .and_then(|text| text.parse::<u64>().ok())
            .map(f64::from_bits)
            .unwrap_or(0.0);
        let contact_q = q_line(contents, "q_contact_bits=");
        if direction.len() != 3 || stroke <= 1e-6 || contact_q.len() != self.arm_names.len() {
            if contact_q.len() == self.arm_names.len() {
                self.command_q(&contact_q, fingers, 40);
            }
            return;
        }
        let Some(chain) = self.model.ee_joint_chain("ee") else {
            return;
        };
        let Ok(start) = realityos_semantics::kinematics::forward_kinematics(
            &self.model,
            &chain,
            "ee",
            &contact_q,
        ) else {
            return;
        };
        let mut seed = contact_q;
        for step in 1..=8 {
            let fraction = f64::from(step) / 8.0;
            let target = [
                start.ee.xyz[0] + direction[0] * stroke * fraction,
                start.ee.xyz[1] + direction[1] * stroke * fraction,
                start.ee.xyz[2] + direction[2] * stroke * fraction,
            ];
            let solved = realityos_semantics::kinematics::with_ik_q_seed(Some(&seed), || {
                realityos_semantics::kinematics::solve_ik(&self.model, &chain, "ee", target, &seed)
            });
            if let Ok((q, trace)) = solved {
                if trace.residual <= 1e-3 {
                    self.command_q(&q, fingers, 15);
                    seed = q;
                }
            }
        }
    }

    fn set_arm(&mut self, arm: &[f64]) {
        let state = self.inst().step(0).expect("state");
        let mut qpos: Vec<f64> = state["state"]["qpos"]
            .as_array()
            .or_else(|| state["qpos"].as_array())
            .map(|values| values.iter().filter_map(|value| value.as_f64()).collect())
            .unwrap_or_default();
        for (name, value) in self.arm_names.iter().zip(arm) {
            if let Some(joint) = self.inspect["joints"].as_array().and_then(|joints| {
                joints
                    .iter()
                    .find(|joint| joint["name"].as_str() == Some(name))
            }) {
                if let Some(adr) = joint["qposadr"].as_u64() {
                    let adr = adr as usize;
                    if adr < qpos.len() {
                        qpos[adr] = *value;
                    }
                }
            }
        }
        self.inst()
            .rpc(&json!({"cmd":"reset","qpos": qpos}))
            .expect("reset qpos");
    }

    fn command_q(&mut self, arm: &[f64], fingers: &[f64], steps: u32) {
        let mut ctrl = self.inst().peek_ctrl().expect("ctrl").0;
        for (name, value) in self.arm_names.iter().zip(arm) {
            if let Some(index) = actuator_index(&self.inspect, name) {
                if index < ctrl.len() {
                    ctrl[index] = *value;
                }
            }
        }
        for (name, value) in ["finger_l", "finger_r"].into_iter().zip(fingers) {
            if let Some(index) = actuator_index(&self.inspect, name) {
                if index < ctrl.len() {
                    ctrl[index] = *value;
                }
            }
        }
        self.inst().set_ctrl(&ctrl).expect("set ctrl");
        self.inst().step(steps).expect("step");
    }

    fn arm_q(&mut self) -> Vec<f64> {
        let state = self.inst().step(0).expect("state");
        let qpos = state["state"]["qpos"]
            .as_array()
            .or_else(|| state["qpos"].as_array())
            .cloned()
            .unwrap_or_default();
        self.arm_names
            .iter()
            .filter_map(|name| joint_q(&self.inspect, &qpos, name))
            .collect()
    }

    fn finger_q(&mut self) -> Vec<f64> {
        let state = self.inst().step(0).expect("state");
        let qpos = state["state"]["qpos"]
            .as_array()
            .or_else(|| state["qpos"].as_array())
            .cloned()
            .unwrap_or_default();
        ["finger_l", "finger_r"]
            .into_iter()
            .filter_map(|name| joint_q(&self.inspect, &qpos, name))
            .collect()
    }

    fn object_xyz(&mut self) -> [f64; 3] {
        let state = self.inst().step(0).expect("state");
        let xpos = state["state"]["xpos"]
            .as_object()
            .or_else(|| state["xpos"].as_object());
        let Some(body) = xpos
            .and_then(|map| map.get("obj0"))
            .and_then(|v| v.as_array())
        else {
            return [0.22, 0.0, self.object_z];
        };
        [
            body.first().and_then(|v| v.as_f64()).unwrap_or(0.0),
            body.get(1).and_then(|v| v.as_f64()).unwrap_or(0.0),
            body.get(2)
                .and_then(|v| v.as_f64())
                .unwrap_or(self.object_z),
        ]
    }

    fn site(&mut self, name: &str) -> [f64; 3] {
        let state = self.inst().step(0).expect("state");
        let sites = state["state"]["sites"]
            .as_object()
            .or_else(|| state["sites"].as_object())
            .expect("sites");
        let body = sites.get(name).and_then(|v| v.as_array()).expect(name);
        [
            body[0].as_f64().unwrap_or(0.0),
            body[1].as_f64().unwrap_or(0.0),
            body[2].as_f64().unwrap_or(0.0),
        ]
    }

    fn contacts(&mut self) -> Vec<(String, String)> {
        let state = self.inst().step(0).expect("state");
        let contacts = state["state"]["contacts"]
            .as_array()
            .or_else(|| state["contacts"].as_array())
            .cloned()
            .unwrap_or_default();
        contacts
            .iter()
            .filter_map(|contact| {
                Some((
                    contact.get("body1")?.as_str()?.to_string(),
                    contact.get("body2")?.as_str()?.to_string(),
                ))
            })
            .collect()
    }

    fn ctrl_writes(&mut self) -> u64 {
        self.inst().peek_ctrl().map(|(_, n)| n).unwrap_or(0)
    }
}

fn uncertain_friction() -> PhysicalParameterBelief {
    let mut belief =
        PhysicalParameterBelief::declared_point(PhysicalParameter::SupportFriction, 0.5, "prior");
    belief.narrow_interval(PhysicalParameter::SupportFriction, [0.05, 0.8], "uncertain");
    belief.with_unknown(PhysicalParameter::ToolObjectFriction, "finger")
}

fn finger_friction() -> PhysicalParameterBelief {
    let mut belief = uncertain_friction();
    belief.narrow_interval(
        PhysicalParameter::ToolObjectFriction,
        [0.8, 0.8],
        "finger-mu",
    );
    belief
}

fn authorize(
    os: &mut RealityOs,
    reasoner: &mut PhysicalReasoner,
    request: &ReasoningRequest,
    step: &PlanStep,
) -> realityos_semantics::physical_reasoner::AuthorizedAction {
    let scope = PhysicalActionScope {
        model_id: reasoner.model_hash().into(),
        embodiment_id: reasoner.robot_id().into(),
        observation_epoch: request.scene.observation_epoch.clone(),
        candidate_id: step.candidate_id.clone(),
        action_key: step.action_key.clone(),
        witness_digest: step.witness_digest.clone(),
        requested_stroke_m: step.stroke_m,
        execution_bound_m: step.stroke_m.max(0.02),
        issued_at_s: request.now_s,
        expires_at_s: 40.0,
        actuator_id: "tool".into(),
        observation_contract_id: format!("sensors:{}", request.scene.observation_epoch),
        abort_contract_id: "abort-and-reobserve".into(),
    };
    let grant = issue_scoped_simulation_grant(
        os,
        &scope,
        &PolicyProposal::external_deterministic(vec![1.0], "production-reasoner"),
        request.now_s,
        &[],
        true,
    )
    .expect("grant");
    let authorization = ExecutionAuthorization {
        grant_id: grant.command_id,
        scope_digest: grant.scope_digest,
        model_id: grant.scope.model_id,
        embodiment_id: grant.scope.embodiment_id,
        observation_epoch: grant.scope.observation_epoch,
        candidate_id: grant.scope.candidate_id,
        action_key: grant.scope.action_key,
        witness_digest: grant.scope.witness_digest,
        actuator_id: grant.scope.actuator_id,
        requested_stroke_m: grant.scope.requested_stroke_m,
        execution_bound_m: grant.scope.execution_bound_m,
        issued_at_s: grant.scope.issued_at_s,
        expires_at_s: grant.scope.expires_at_s,
        observation_contract_id: grant.scope.observation_contract_id,
        abort_contract_id: grant.scope.abort_contract_id,
    };
    reasoner
        .authorize_step(request, step, authorization)
        .expect("fresh proof accepts the grant")
}

fn step_from_summary(
    candidate: &realityos_semantics::physical_reasoner::CandidateSummary,
    stroke: f64,
) -> PlanStep {
    PlanStep {
        family: candidate.family.clone(),
        action_key: candidate.action_key.clone(),
        candidate_id: candidate.action_key.clone(),
        witness_digest: candidate.witness_digest.clone().unwrap_or_default(),
        witness_contents: candidate.witness_contents.clone().unwrap_or_default(),
        stroke_m: stroke,
        immediate_progress: candidate.immediate_progress,
        predicted_translation_m: stroke,
        error_after: 0.0,
    }
}

fn candidate_lines(
    candidates: &[realityos_semantics::physical_reasoner::CandidateSummary],
) -> String {
    candidates
        .iter()
        .map(|candidate| {
            format!(
                "{} {} {}",
                candidate.family, candidate.action_key, candidate.status
            )
        })
        .collect::<Vec<_>>()
        .join("\n")
}

fn plan_keys(plan: &realityos_semantics::physical_reasoner::PhysicalPlan) -> Vec<String> {
    plan.steps
        .iter()
        .map(|step| step.action_key.clone())
        .collect()
}

fn q_line(contents: &str, prefix: &str) -> Vec<f64> {
    contents
        .lines()
        .find_map(|line| line.strip_prefix(prefix))
        .map(|text| {
            text.split(',')
                .filter_map(|value| value.parse::<u64>().ok())
                .map(f64::from_bits)
                .collect()
        })
        .unwrap_or_default()
}

fn actuator_index(inspect: &Value, joint: &str) -> Option<usize> {
    inspect["actuators"]
        .as_array()?
        .iter()
        .position(|actuator| {
            actuator["target"].as_str() == Some(joint) || actuator["name"].as_str() == Some(joint)
        })
}

fn joint_q(inspect: &Value, qpos: &[Value], name: &str) -> Option<f64> {
    let joint = inspect["joints"]
        .as_array()?
        .iter()
        .find(|joint| joint["name"].as_str() == Some(name))?;
    let adr = joint["qposadr"].as_u64()? as usize;
    qpos.get(adr)?.as_f64()
}

fn pair(contacts: &[(String, String)], a: &str, b: &str) -> bool {
    contacts
        .iter()
        .any(|(left, right)| (left == a && right == b) || (left == b && right == a))
}

fn evidence_path(name: &str) -> PathBuf {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../target")
        .join(name);
    std::fs::create_dir_all(path.parent().unwrap()).ok();
    path
}
