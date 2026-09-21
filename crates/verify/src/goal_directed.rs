//! Goal-directed receding-horizon planar loop. MuJoCo is a falsifier only.

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::honesty::SIMULATION_ONLY;
use crate::manipulation::ManipulationEpisode;
use crate::push_mechanics::FrozenMechanicsPrediction;
use realityos_semantics::goal_loop::{CausalActionRecord, GoalLoopOutcome, LoopDecision};
use realityos_semantics::physical_interaction::PhysicalInteractionCandidate;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClosedLoopTrace {
    pub metal: bool,
    pub evidence_status: String,
    pub embodiment: String,
    pub start_xy: [f64; 2],
    pub start_yaw: f64,
    pub goal_xy: Option<[f64; 2]>,
    pub goal_yaw: Option<f64>,
    pub actions: Vec<Value>,
    pub final_outcome: String,
    pub unauthorized_writes: u64,
    pub authority_violations: u64,
    pub selected_faces: Vec<String>,
    pub open_loop_reached: Option<bool>,
    pub closed_loop_reached: Option<bool>,
}

impl ClosedLoopTrace {
    pub fn new(embodiment: impl Into<String>) -> Self {
        Self {
            metal: false,
            evidence_status: SIMULATION_ONLY.into(),
            embodiment: embodiment.into(),
            start_xy: [0.0, 0.0],
            start_yaw: 0.0,
            goal_xy: None,
            goal_yaw: None,
            actions: Vec::new(),
            final_outcome: String::new(),
            unauthorized_writes: 0,
            authority_violations: 0,
            selected_faces: Vec::new(),
            open_loop_reached: None,
            closed_loop_reached: None,
        }
    }
}

pub fn record_action(
    trace: &mut ClosedLoopTrace,
    rec: &CausalActionRecord,
    cand: Option<&PhysicalInteractionCandidate>,
    ep: Option<&ManipulationEpisode>,
    frozen: Option<&FrozenMechanicsPrediction>,
) {
    if let Some(f) = cand.map(|c| c.face_id.clone()) {
        trace.selected_faces.push(f);
    }
    if let Some(ep) = ep {
        trace.unauthorized_writes += ep.unauthorized_writes;
        if ep.task_result == "authority_violation" {
            trace.authority_violations += 1;
        }
    }
    trace.actions.push(serde_json::json!({
        "object_xy": rec.object_xy,
        "object_yaw": rec.object_yaw,
        "candidate_count": rec.candidate_count,
        "rejection_reasons": rec.rejection_reasons,
        "selected_id": rec.selected_id,
        "selected_face": rec.selected_face,
        "selection_rationale": rec.selection_rationale,
        "predicted_contact_mode": rec.predicted_contact_mode,
        "predicted_twist": rec.predicted_twist,
        "predicted_goal_progress": rec.predicted_goal_progress,
        "model_validity": rec.model_validity,
        "authority_decision": rec.authority_decision,
        "goal_error_before": rec.goal_error_before,
        "goal_error_after": rec.goal_error_after,
        "prediction_residual": rec.prediction_residual,
        "first_divergence": rec.first_divergence,
        "decision": rec.decision,
        "outcome": rec.outcome,
        "contact_switch": rec.contact_switch,
        "unauthorized_writes": rec.unauthorized_writes,
        "privileged_force_in_predictor": frozen.map(|f| f.contains_privileged_force()).unwrap_or(false),
        "evidence_status": ep.map(|e| e.evidence_status.clone()).unwrap_or_else(|| SIMULATION_ONLY.into()),
        "ctrl_writes": ep.map(|e| e.ctrl_writes).unwrap_or(0),
        "failure_taxonomy": ep.and_then(|e| e.failure_taxonomy.clone()),
        "had_feasible_contact_maneuver": ep.map(|e| e.had_feasible_contact_maneuver),
        "selected_rank_why": ep.map(|e| e.selected_rank_why.clone()),
    }));
    trace.final_outcome = format!("{:?}", rec.outcome);
    if rec.outcome == GoalLoopOutcome::GoalReached {
        trace.closed_loop_reached = Some(true);
    }
    if rec.decision == LoopDecision::Halt {
        trace.closed_loop_reached = Some(rec.outcome == GoalLoopOutcome::GoalReached);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bundle::RobotBundle;
    use crate::corpus;
    use crate::manipulation::{
        chain_q_from_qpos, local_ee_poses, run_skill_episode_ex, sample_push_units,
        template_objects, with_episode_qpos, with_injected_push_maneuver,
    };
    use crate::manipulation_scenarios::{ManipulationScenario, Polarity};
    use crate::manipulation_verify::body_xyz;
    use crate::mujoco_exec::{checkin_worker, ensure_mujoco_or_skip};
    use crate::observation::VerifierTruth;
    use crate::runner::load_and_normalize;
    use crate::semantics_map::embodiment_from_manifest;
    use realityos_physics::{PressureDistribution, SupportFrictionModel};
    use realityos_semantics::command_domain::named_joint_limit_margin;
    use realityos_semantics::contact::declared_manipulation_contact_bodies;
    use realityos_semantics::contact_jacobian::contact_jacobian_witness;
    use realityos_semantics::contact_maneuver::{
        object_center_for_sample, select_fixed_world_push_seeded_with_funnel, tool_offset_in_ee,
        BoxObject, ContactInfeasible, ContactManeuver, ContactManeuverSpec, ContactSelectFunnel,
        SampledEePose, SupportPlane,
    };
    use realityos_semantics::effect_feasibility::PlanarPushInitiation;
    use realityos_semantics::effort::{any_link_com_known, chain_physical_effort_signed};
    use realityos_semantics::geometry::PrimitiveShape;
    use realityos_semantics::goal_loop::{
        receding_horizon_step, record_after_with_goal, GoalLoopOutcome, LoopDecision, LoopState,
        WorldObservation,
    };
    use realityos_semantics::kinematics::{forward_kinematics, ik_residual_is_precise, solve_ik};
    use realityos_semantics::maneuver_witness::execution_block_reason;
    use realityos_semantics::pair_friction::PairFriction;
    use realityos_semantics::physical_interaction::{
        evaluate_all, evaluate_candidate, generate_planar_push_candidates,
        initiation_from_candidate, select_interaction, EvaluationContext, SelectionOutcome,
    };
    use realityos_semantics::physical_quantity::PhysicalEffort;
    use realityos_semantics::planar_goal::{
        evaluate_goal_error, yaw_from_quat_wxyz, InteractionFamily, PlanarObjectGoal,
        SafetyConstraints,
    };
    use realityos_semantics::provenance::Provenanced;
    use realityos_semantics::self_load::{gravity_self_load, self_load_provenanced};
    use realityos_semantics::transform::{rotate_by_quat, Se3};
    use realityos_semantics::workspace::reachable_ee_poses;
    use serde_json::json;
    use std::path::PathBuf;

    fn scratch_dir() -> PathBuf {
        PathBuf::from(r"C:\Users\moram\AppData\Local\Temp\grok-goal-45646d0dc2cc\implementer")
    }

    fn write_scratch(name: &str, body: &str) {
        let _ = std::fs::create_dir_all(scratch_dir());
        let _ = std::fs::write(scratch_dir().join(name), body);
    }

    fn trans_goal(target: [f64; 2], tol: f64, attempts: u32) -> PlanarObjectGoal {
        PlanarObjectGoal {
            object_id: "obj0".into(),
            world_id: "sim".into(),
            model_id: "sim".into(),
            target_xy: Some(target),
            target_xy_region: None,
            target_yaw: None,
            target_yaw_interval: None,
            translation_tolerance_m: tol,
            orientation_tolerance_rad: 0.2,
            freshness_s: 1.0,
            allowed_interaction_family: InteractionFamily::PlanarPush,
            safety: SafetyConstraints::default(),
            max_bounded_attempts: attempts,
        }
    }

    fn pose(xy: [f64; 2], z: f64) -> Se3 {
        Se3 {
            xyz: [xy[0], xy[1], z],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        }
    }

    fn mechanics_template(mass: f64, mu: f64, tau: f64) -> PlanarPushInitiation {
        let n = mass * 9.80665;
        let f_max = mu * n;
        PlanarPushInitiation {
            mass_kg: Provenanced::declared(mass, "scenario.mass", 0.0),
            object_com_world: Provenanced::declared([0.0, 0.0, 0.03], "scenario.com", 0.0),
            gravity_m_s2: Provenanced::declared([0.0, 0.0, -9.80665], "scenario.g", 0.0),
            support_normal: Provenanced::declared([0.0, 0.0, 1.0], "scenario.n", 0.0),
            object_support_friction: PairFriction::coulomb(
                "object",
                "support",
                Provenanced::declared(mu, "scenario.mu_s", 0.0),
            ),
            tool_object_friction: PairFriction::coulomb(
                "tool",
                "object",
                Provenanced::declared(0.8, "scenario.mu_t", 0.0),
            ),
            contact_point_world: Provenanced::unknown("c", 0.0),
            contact_normal_world: Provenanced::unknown("n", 0.0),
            push_direction_world: Provenanced::unknown("d", 0.0),
            contact_force_direction_world: Provenanced::unknown("f", 0.0),
            pusher_velocity_world: Provenanced::unknown("v", 0.0),
            joint_names: vec!["j0".into(), "j1".into(), "j2".into()],
            translational_jacobian_3xn: vec![
                vec![1.0, 0.0, 0.0],
                vec![0.0, 1.0, 0.0],
                vec![0.0, 0.0, 1.0],
            ],
            jacobian_residual: Some(0.0),
            joint_effort_abs: vec![
                Provenanced::declared(tau, "t", 0.0),
                Provenanced::declared(tau, "t", 0.0),
                Provenanced::declared(tau, "t", 0.0),
            ],
            joint_effort_min: vec![
                Provenanced::declared(-tau, "t", 0.0),
                Provenanced::declared(-tau, "t", 0.0),
                Provenanced::declared(-tau, "t", 0.0),
            ],
            joint_effort_max: vec![
                Provenanced::declared(tau, "t", 0.0),
                Provenanced::declared(tau, "t", 0.0),
                Provenanced::declared(tau, "t", 0.0),
            ],
            self_load_torque_nm: vec![
                Provenanced::declared(0.0, "s", 0.0),
                Provenanced::declared(0.0, "s", 0.0),
                Provenanced::declared(0.0, "s", 0.0),
            ],
            link_com_known: true,
            object_supported: true,
            approximately_planar: true,
            quasi_static: true,
            single_intended_contact: true,
            no_significant_impact: true,
            object_characteristic_length_m: Some(0.04),
            support_friction_model: SupportFrictionModel::Ellipsoidal {
                f_max,
                tau_max: f_max * (2.0 / 3.0) * 0.04,
                pressure: PressureDistribution::DeclaredUniform,
            },
            object_yaw_rad: Provenanced::declared(0.0, "yaw", 0.0),
            stale_object_evidence: false,
            intended_contact_lost: false,
            authority_ok: true,
        }
    }

    fn eval_at(
        xy: [f64; 2],
        goal: &PlanarObjectGoal,
        authority_ok: bool,
    ) -> Vec<realityos_semantics::physical_interaction::PhysicalInteractionCandidate> {
        let mut cands = generate_planar_push_candidates(
            "obj0",
            pose(xy, 0.03),
            [0.04, 0.03, 0.03],
            [0.0, 0.0, 1.0],
            0.01,
            0.02,
        );
        let mut mech = mechanics_template(0.05, 0.2, 20.0);
        mech.authority_ok = authority_ok;
        let ctx = EvaluationContext {
            goal: goal.clone(),
            object_xy: xy,
            object_yaw: 0.0,
            object_com_world: [xy[0], xy[1], 0.03],
            mechanics_template: Some(mech),
            authority_ok,
            robot_provided: false,
            robot_reachable: None,
            collision_admissible: None,
            executable_witness: None,
            robot_reject_reason: None,
        };
        evaluate_all(&mut cands, &ctx);
        cands
    }

    #[test]
    fn analytic_selection_and_loop_twice() {
        let mut log = String::new();
        for pass in 1..=2 {
            let g = trans_goal([0.20, 0.0], 0.01, 6);
            let xy = [0.0, 0.0];
            let cands = eval_at(xy, &g, true);
            let sel = select_interaction(&cands, &[]);
            let SelectionOutcome::Selected { index, reason } = sel else {
                panic!("pass {pass}: expected selection, got {sel:?}");
            };
            assert_eq!(
                cands[index].goal_progress,
                Some(realityos_semantics::planar_goal::GoalProgressClass::StrictProgress)
            );
            assert!(reason.contains("STRICT_PROGRESS"));
            let frozen = {
                let init = initiation_from_candidate(
                    &mechanics_template(0.05, 0.2, 20.0),
                    &cands[index],
                    [xy[0], xy[1], 0.03],
                    0.0,
                    true,
                );
                FrozenMechanicsPrediction::freeze(
                    cands[index].witness.clone().expect("witness"),
                    &init,
                )
            };
            assert!(!frozen.contains_privileged_force());
            let obs = WorldObservation {
                object_id: "obj0".into(),
                xy,
                yaw: 0.0,
                robot_q: vec![0.0],
                freshness_ok: true,
                intended_contact_face: None,
                authority_ok: true,
                observed_at_s: 1.0,
            };
            let step = receding_horizon_step(&obs, &g, &cands, LoopState::default(), None);
            assert_eq!(step.record.unauthorized_writes, 0);
            let after = WorldObservation {
                object_id: "obj0".into(),
                xy: [0.03, 0.0],
                yaw: 0.0,
                robot_q: vec![0.1],
                freshness_ok: true,
                intended_contact_face: step.record.selected_face.clone(),
                authority_ok: true,
                observed_at_s: 2.0,
            };
            let rec = record_after_with_goal(step, &after, &g);
            assert!(rec.state.last_predicted_pose.is_none());
            log.push_str(&format!(
                "pass={pass} selected={} reason={} outcome={:?} residual={:?}\n",
                rec.record.selected_face.as_deref().unwrap_or("-"),
                rec.record.selection_rationale,
                rec.record.outcome,
                rec.record.prediction_residual
            ));
        }
        write_scratch("selection-counterfactual.log", &log);
        write_scratch("closed-loop-analytic.log", &log);
    }

    fn object_xy_yaw(ep: &ManipulationEpisode, truth: &VerifierTruth) -> ([f64; 2], f64) {
        let xy = body_xyz(truth, "obj0")
            .map(|p| [p[0], p[1]])
            .or_else(|| {
                ep.object_evidence
                    .get("pose")
                    .and_then(|p| p.as_array())
                    .and_then(|a| Some([a.first()?.as_f64()?, a.get(1)?.as_f64()?]))
            })
            .unwrap_or([0.0, 0.0]);
        let yaw = truth
            .xquat
            .get("obj0")
            .and_then(|q| {
                if q.len() >= 4 {
                    Some(yaw_from_quat_wxyz([q[0], q[1], q[2], q[3]]))
                } else {
                    None
                }
            })
            .unwrap_or(0.0);
        (xy, yaw)
    }

    fn push_scenario(
        obj: [f64; 3],
        size: f64,
        mass: f64,
        friction: f64,
        push_dir: [f64; 3],
        push_dist: f64,
        seed: u64,
    ) -> ManipulationScenario {
        let table_z = (obj[2] - size - 0.01).max(0.02);
        ManipulationScenario {
            seed,
            polarity: Polarity::Positive,
            skill: "PUSH".into(),
            objects: vec![
                json!({"name":"table","type":"box","pos":[obj[0], obj[1], table_z],"size":[0.18,0.18,0.01],"mass":10.0,"movable":false}),
                json!({
                    "name":"obj0","type":"box","pos":obj,"size":[size,size,size],
                    "mass":mass,"friction":friction,"movable":true
                }),
                json!({"name":"obstacle","type":"box","pos":[8.0,8.0,-1.0],"size":[0.03,0.03,0.03],"mass":1.0,"movable":false}),
            ],
            planar: true,
            object_id: "obj0".into(),
            expected_refusal: None,
            neg: None,
            stale: false,
            move_object_after_obs: false,
            replay: false,
            restart_replay: false,
            crash_controller: false,
            immovable: false,
            push_dir,
            push_dist,
            required_opening: 0.8,
            world_construction:
                realityos_semantics::contact_maneuver::WorldConstructionMode::FixedWorld,
        }
    }

    fn truth_of(inst: &mut crate::mujoco_exec::MujocoInstance) -> Result<VerifierTruth, String> {
        let st = inst.step(0).map_err(|e| e.to_string())?;
        Ok(VerifierTruth::from_mujoco_state(
            st.get("state").unwrap_or(&st),
        ))
    }

    fn flags_from_infeasible(e: ContactInfeasible) -> (Option<bool>, Option<bool>, Option<bool>) {
        match e {
            ContactInfeasible::NoIkSolution
            | ContactInfeasible::ContactPoseUnreachableFromApproach
            | ContactInfeasible::NoFeasibleContactPose => (Some(false), None, None),
            ContactInfeasible::CollisionInadmissible
            | ContactInfeasible::ApproachCollidesBeforeContact
            | ContactInfeasible::SupportPlaneBlocksEe => (Some(true), Some(false), None),
            ContactInfeasible::NoExecutableContactManeuver
            | ContactInfeasible::InsufficientJointMargin
            | ContactInfeasible::OrientationInfeasible
            | ContactInfeasible::InsufficientRemainingStroke
            | ContactInfeasible::WrongContactGeometry => (Some(true), Some(true), Some(false)),
        }
    }

    fn build_ee_cloud(
        model: &realityos_semantics::embodiment::EmbodimentModel,
        ee: &str,
        qpos: &[f64],
        seed: u64,
    ) -> Vec<SampledEePose> {
        let units = sample_push_units(model, ee, seed, 96);
        let mut cloud = reachable_ee_poses(model, ee, &units);
        if let Some(q) = chain_q_from_qpos(model, ee, qpos) {
            if let Some(chain) = model.ee_joint_chain(ee) {
                if let Ok(fk) = forward_kinematics(model, &chain, ee, &q) {
                    cloud.insert(
                        0,
                        SampledEePose {
                            xyz: fk.ee.xyz,
                            quat_wxyz: fk.ee.quat_wxyz,
                            q: q.clone(),
                            joint_names: chain.clone(),
                        },
                    );
                }
                cloud.extend(local_ee_poses(model, ee, &chain, &q, seed, 32));
            }
        }
        cloud
    }

    fn fill_mechanics_from_q(
        model: &realityos_semantics::embodiment::EmbodimentModel,
        ee: &str,
        qpos: &[f64],
        contact_world: [f64; 3],
        yaw: f64,
        mass: f64,
        mu: f64,
    ) -> PlanarPushInitiation {
        let mut p = mechanics_template(mass, mu, 20.0);
        p.object_yaw_rad = Provenanced::declared(yaw, "obs.yaw", 0.0);
        let Some(q) = chain_q_from_qpos(model, ee, qpos) else {
            return p;
        };
        let Some(chain) = model.ee_joint_chain(ee) else {
            return p;
        };
        let Ok(fk) = forward_kinematics(model, &chain, ee, &q) else {
            return p;
        };
        let contact_in_ee = tool_offset_in_ee(fk.ee.quat_wxyz, contact_world, fk.ee.xyz);
        let Ok(jac) = contact_jacobian_witness(model, &chain, ee, &q, contact_in_ee, 1e-6) else {
            return p;
        };
        p.joint_names = jac.joint_names.clone();
        p.translational_jacobian_3xn = jac.analytic_3xn.clone();
        p.jacobian_residual = Some(jac.residual);
        p.link_com_known = any_link_com_known(model);
        match chain_physical_effort_signed(model, &p.joint_names) {
            Ok(signed) => {
                p.joint_effort_min = signed
                    .iter()
                    .map(|s| Provenanced::declared(s.tau_min_nm, "joint.effort_min", 0.0))
                    .collect();
                p.joint_effort_max = signed
                    .iter()
                    .map(|s| Provenanced::declared(s.tau_max_nm, "joint.effort_max", 0.0))
                    .collect();
                p.joint_effort_abs = signed
                    .iter()
                    .map(|s| {
                        Provenanced::declared(
                            s.tau_min_nm.abs().max(s.tau_max_nm.abs()),
                            "joint.effort",
                            0.0,
                        )
                    })
                    .collect();
            }
            Err(PhysicalEffort::Unknown { reason }) => {
                p.joint_effort_abs = p
                    .joint_names
                    .iter()
                    .map(|_| Provenanced::unknown(reason.clone(), 0.0))
                    .collect();
            }
            Err(_) => {}
        }
        let mut q_by_joint = std::collections::BTreeMap::new();
        for (name, qi) in chain.iter().zip(q.iter()) {
            q_by_joint.insert(name.clone(), *qi);
        }
        if let Ok(tau) = gravity_self_load(model, &p.joint_names, &q_by_joint, [0.0, 0.0, -9.80665])
        {
            p.self_load_torque_nm = self_load_provenanced(&tau, "self_load.gravity");
        }
        p
    }

    fn prove_face_funnel(
        model: &realityos_semantics::embodiment::EmbodimentModel,
        ee: &str,
        cloud: &[SampledEePose],
        object: BoxObject,
        support: SupportPlane,
        push: [f64; 3],
        stroke: f64,
        ee_xyz: [f64; 3],
        tool_off: [f64; 3],
    ) -> (
        Result<ContactManeuver, ContactInfeasible>,
        ContactSelectFunnel,
    ) {
        let mut spec = ContactManeuverSpec::table_push(push, stroke, ee_xyz, tool_off);
        spec.object_id = "obj0".into();
        spec.intended_tool_bodies =
            declared_manipulation_contact_bodies(model, model.resources.first(), ee);
        if spec.intended_tool_bodies.is_empty() {
            spec.intended_tool_bodies.push("tool".into());
        }
        let intended = spec.intended_tool_bodies.clone();
        for b in &model.bodies {
            if intended.iter().any(|n| n == &b.name) {
                continue;
            }
            if b.parent.is_none() {
                continue;
            }
            spec.robot_body_volumes
                .push(realityos_semantics::contact_collision::AttachedSphere {
                    body: b.name.clone(),
                    radius: spec.ee_radius,
                    offset: [0.0, 0.0, 0.0],
                });
        }
        let (res, funnel) =
            select_fixed_world_push_seeded_with_funnel(model, ee, cloud, object, support, &spec);
        (
            match res {
                Ok((m, _)) => {
                    if m.executable
                        .as_ref()
                        .is_some_and(|w| execution_block_reason(w).is_none())
                    {
                        Ok(m)
                    } else {
                        Err(ContactInfeasible::NoExecutableContactManeuver)
                    }
                }
                Err(e) => Err(e),
            },
            funnel,
        )
    }

    fn prove_face(
        model: &realityos_semantics::embodiment::EmbodimentModel,
        ee: &str,
        cloud: &[SampledEePose],
        object: BoxObject,
        support: SupportPlane,
        push: [f64; 3],
        stroke: f64,
        ee_xyz: [f64; 3],
        tool_off: [f64; 3],
    ) -> Result<ContactManeuver, ContactInfeasible> {
        prove_face_funnel(
            model, ee, cloud, object, support, push, stroke, ee_xyz, tool_off,
        )
        .0
    }

    fn midreach_aligned_seed(
        model: &realityos_semantics::embodiment::EmbodimentModel,
        ee: &str,
        desired: [f64; 3],
        z: f64,
        tool_off: [f64; 3],
    ) -> Option<SampledEePose> {
        let chain = model.ee_joint_chain(ee)?;
        let n = chain.len();
        if n == 0 {
            return None;
        }
        let target = [desired[0] * 0.30, desired[1] * 0.30, z];
        let mut guesses = vec![vec![0.0; n]];
        let mut g1 = vec![0.0; n];
        if n > 1 {
            g1[1] = -0.9;
        }
        if n > 0 {
            g1[0] = 0.45;
        }
        if n > 2 {
            g1[2] = 0.45;
        }
        guesses.push(g1);
        let mut g2 = vec![0.0; n];
        if n > 1 {
            g2[1] = 0.9;
        }
        if n > 0 {
            g2[0] = -0.45;
        }
        if n > 2 {
            g2[2] = -0.45;
        }
        guesses.push(g2);
        let mut g3 = vec![0.0; n];
        if n > 0 {
            g3[0] = 0.7;
        }
        if n > 1 {
            g3[1] = -1.3;
        }
        if n > 2 {
            g3[2] = 0.6;
        }
        guesses.push(g3);
        let mut best: Option<(SampledEePose, f64)> = None;
        for seed in guesses {
            let Ok((q, tr)) = solve_ik(model, &chain, ee, target, &seed) else {
                continue;
            };
            if !ik_residual_is_precise(tr.residual) {
                continue;
            }
            let Ok(fk) = forward_kinematics(model, &chain, ee, &q) else {
                continue;
            };
            let s = SampledEePose {
                xyz: fk.ee.xyz,
                quat_wxyz: fk.ee.quat_wxyz,
                q,
                joint_names: chain.clone(),
            };
            let align = planar_tool_axis(s.quat_wxyz, tool_off)
                .map(|ax| ax[0] * desired[0] + ax[1] * desired[1])
                .unwrap_or(-1.0);
            let margin = named_joint_limit_margin(&s.q, &s.joint_names, &model.joints);
            if margin < 0.04 || align < 0.85 {
                continue;
            }
            let better = match &best {
                None => true,
                Some((_, a)) => align > *a,
            };
            if better {
                best = Some((s, align));
            }
        }
        best.map(|(s, _)| s)
    }

    fn ee_from_truth(truth: &VerifierTruth, bundle: &RobotBundle) -> Option<[f64; 3]> {
        let names: Vec<String> = bundle
            .manifest
            .end_effectors
            .iter()
            .map(|e| e.name.clone())
            .collect();
        for k in names.iter().map(String::as_str) {
            if let Some(p) = truth.named_pos.get(k).or_else(|| truth.xpos.get(k)) {
                if p.len() >= 3 {
                    return Some([p[0], p[1], p[2]]);
                }
            }
        }
        None
    }

    fn ee_quat_from_truth(truth: &VerifierTruth, ee_name: &str) -> [f64; 4] {
        truth
            .site_xquat
            .get(ee_name)
            .or_else(|| truth.xquat.get(ee_name))
            .and_then(|q| {
                if q.len() >= 4 {
                    Some([q[0], q[1], q[2], q[3]])
                } else {
                    None
                }
            })
            .unwrap_or([1.0, 0.0, 0.0, 0.0])
    }

    fn geom_local_points(shape: &PrimitiveShape) -> Vec<[f64; 3]> {
        match *shape {
            PrimitiveShape::Box { half_extents } => {
                let h = half_extents;
                let mut v = Vec::with_capacity(8);
                for sx in [-1.0, 1.0] {
                    for sy in [-1.0, 1.0] {
                        for sz in [-1.0, 1.0] {
                            v.push([sx * h[0], sy * h[1], sz * h[2]]);
                        }
                    }
                }
                v
            }
            PrimitiveShape::Sphere { radius } => vec![
                [radius, 0.0, 0.0],
                [-radius, 0.0, 0.0],
                [0.0, radius, 0.0],
                [0.0, -radius, 0.0],
                [0.0, 0.0, radius],
                [0.0, 0.0, -radius],
            ],
            PrimitiveShape::Capsule {
                radius,
                half_length,
            }
            | PrimitiveShape::Cylinder {
                radius,
                half_length,
            } => vec![
                [0.0, 0.0, half_length],
                [0.0, 0.0, -half_length],
                [radius, 0.0, 0.0],
                [-radius, 0.0, 0.0],
            ],
            PrimitiveShape::Plane { .. } | PrimitiveShape::Unsupported { .. } => {
                vec![[0.0, 0.0, 0.0]]
            }
        }
    }

    fn offset_len(o: [f64; 3]) -> f64 {
        (o[0] * o[0] + o[1] * o[1] + o[2] * o[2]).sqrt()
    }

    /// Tool offset in the EE frame from declared intended-tool geometry, not invented +X.
    /// Distal points are those furthest along the live planar reach, averaged so a
    /// two-finger tool yields a centered contact rather than a single corner.
    fn derived_tool_offset_ee(
        model: &realityos_semantics::embodiment::EmbodimentModel,
        ee_name: &str,
        ee_xyz: [f64; 3],
        ee_quat: [f64; 4],
        truth: &VerifierTruth,
    ) -> [f64; 3] {
        let intended =
            declared_manipulation_contact_bodies(model, model.resources.first(), ee_name);
        let reach_n = ee_xyz[0].hypot(ee_xyz[1]);
        let reach = if reach_n > 1e-6 {
            [ee_xyz[0] / reach_n, ee_xyz[1] / reach_n, 0.0]
        } else {
            [1.0, 0.0, 0.0]
        };
        let mut pts: Vec<[f64; 3]> = Vec::new();
        let mut max_along = f64::NEG_INFINITY;
        for g in &model.collision_geoms {
            if !intended.iter().any(|n| n == &g.owner_body) {
                continue;
            }
            let Some(bx) = truth
                .xpos
                .get(&g.owner_body)
                .or_else(|| truth.named_pos.get(&g.owner_body))
            else {
                continue;
            };
            if bx.len() < 3 {
                continue;
            }
            let bq = truth
                .xquat
                .get(&g.owner_body)
                .and_then(|q| {
                    if q.len() >= 4 {
                        Some([q[0], q[1], q[2], q[3]])
                    } else {
                        None
                    }
                })
                .unwrap_or([1.0, 0.0, 0.0, 0.0]);
            let Ok(body) = Se3::try_new([bx[0], bx[1], bx[2]], bq) else {
                continue;
            };
            let gw = body.compose(g.local_pose);
            for local in geom_local_points(&g.shape) {
                let p = gw.transform_point(local);
                if (p[2] - ee_xyz[2]).abs() > 0.012 {
                    continue;
                }
                let along = p[0] * reach[0] + p[1] * reach[1];
                if along > max_along + 1e-4 {
                    max_along = along;
                    pts.clear();
                    pts.push(p);
                } else if (along - max_along).abs() <= 1e-4 {
                    pts.push(p);
                }
            }
        }
        if !pts.is_empty() && max_along.is_finite() {
            let n = pts.len() as f64;
            let avg = [
                pts.iter().map(|p| p[0]).sum::<f64>() / n,
                pts.iter().map(|p| p[1]).sum::<f64>() / n,
                pts.iter().map(|p| p[2]).sum::<f64>() / n,
            ];
            let o = tool_offset_in_ee(ee_quat, avg, ee_xyz);
            if offset_len(o) >= 0.008 {
                return o;
            }
        }
        for b in &intended {
            if let Some(p) = truth.xpos.get(b).or_else(|| truth.named_pos.get(b)) {
                if p.len() >= 3 {
                    let o = tool_offset_in_ee(ee_quat, [p[0], p[1], p[2]], ee_xyz);
                    if offset_len(o) >= 0.008 {
                        return o;
                    }
                }
            }
        }
        [0.0, 0.0, 0.0]
    }

    fn planar_tool_axis(quat: [f64; 4], tool_off: [f64; 3]) -> Option<[f64; 3]> {
        let w = rotate_by_quat(quat, tool_off);
        let n = (w[0] * w[0] + w[1] * w[1]).sqrt();
        if n < 1e-6 {
            return None;
        }
        Some([w[0] / n, w[1] / n, 0.0])
    }

    fn desired_planar_push(goal: &PlanarObjectGoal, start_xy: [f64; 2]) -> [f64; 3] {
        let t = goal.target_xy.unwrap_or([start_xy[0] + 0.03, start_xy[1]]);
        let d = [t[0] - start_xy[0], t[1] - start_xy[1]];
        let n = d[0].hypot(d[1]);
        if n > 1e-6 {
            [d[0] / n, d[1] / n, 0.0]
        } else {
            [1.0, 0.0, 0.0]
        }
    }

    fn select_seed_aligned<'a>(
        cloud: &'a [SampledEePose],
        tool_off: [f64; 3],
        desired: [f64; 3],
        joints: &[realityos_semantics::embodiment::Joint],
    ) -> Option<&'a SampledEePose> {
        // Leave stroke room inside the planar workspace (~0.41 m max reach).
        const R_LO: f64 = 0.20;
        const R_HI: f64 = 0.34;
        const R_PREF: f64 = 0.28;
        const MIN_MARGIN: f64 = 0.04;
        let mut best: Option<(&'a SampledEePose, f64, f64, f64)> = None;
        for s in cloud {
            let r = s.xyz[0].hypot(s.xyz[1]);
            if !(R_LO..=R_HI).contains(&r) {
                continue;
            }
            let margin = named_joint_limit_margin(&s.q, &s.joint_names, joints);
            if margin < MIN_MARGIN {
                continue;
            }
            let d = match planar_tool_axis(s.quat_wxyz, tool_off) {
                Some(ax) => ax[0] * desired[0] + ax[1] * desired[1],
                None => continue,
            };
            if d < 0.5 {
                continue;
            }
            let r_score = -(r - R_PREF).abs();
            let better = match best {
                None => true,
                Some((_, bd, bm, br)) => {
                    d > bd + 1e-9
                        || ((d - bd).abs() <= 1e-9 && margin > bm + 1e-9)
                        || ((d - bd).abs() <= 1e-9 && (margin - bm).abs() <= 1e-9 && r_score > br)
                }
            };
            if better {
                best = Some((s, d, margin, r_score));
            }
        }
        best.map(|(s, _, _, _)| s)
    }

    fn apply_sample_qpos(
        model: &realityos_semantics::embodiment::EmbodimentModel,
        ee: &str,
        qpos: &mut [f64],
        sample: &SampledEePose,
    ) {
        if let Some(chain) = model.ee_joint_chain(ee) {
            for (name, qi) in chain.iter().zip(sample.q.iter()) {
                if let Some(j) = model.joints.iter().find(|j| j.name == *name) {
                    if let Some(adr) = j.qpos_adr {
                        if let Some(slot) = qpos.get_mut(adr as usize) {
                            *slot = *qi;
                        }
                    }
                }
            }
        }
    }

    fn closed_loop_on_bundle(
        bundle_id: &str,
        start_xy: [f64; 2],
        goal: &PlanarObjectGoal,
        perturb_after: Option<[f64; 2]>,
        seed: u64,
    ) -> Result<ClosedLoopTrace, String> {
        let bundle = RobotBundle::load(corpus::robot_dir(bundle_id))
            .map_err(|e| format!("load {bundle_id}: {e}"))?;
        if bundle.manifest.end_effectors.is_empty() {
            return Err("no_end_effector".into());
        }
        let (mut probe, man) = load_and_normalize(&bundle, &template_objects(true), 0)?;
        let t0 = truth_of(&mut probe)?;
        let ee = ee_from_truth(&t0, &bundle).ok_or_else(|| "no_ee_pose".to_string())?;
        let discovered =
            crate::resource_discover::discover_resources(&bundle, &man, &probe.inspect);
        let mut last_loaded = Some((probe, man.clone()));
        let mut qualified = Vec::new();
        for r in &discovered {
            if let Ok((_, q)) = crate::resource_qualify::qualify_resource(&bundle, r) {
                qualified.push(q);
            }
        }
        let mut model = embodiment_from_manifest(&bundle, &man);
        let ee_name = bundle.manifest.end_effectors[0].name.clone();
        let chain_len = model.ee_joint_chain(&ee_name).map(|c| c.len()).unwrap_or(0);
        if chain_len < 3 {
            if let Some((inst, _)) = last_loaded.take() {
                checkin_worker(inst);
            }
            return Err("insufficient_serial_chain".into());
        }
        model.resources = qualified;
        let ee_xyz0 = ee_from_truth(&t0, &bundle).unwrap_or(ee);
        let ee_q0 = ee_quat_from_truth(&t0, &ee_name);
        let tool_off = derived_tool_offset_ee(&model, &ee_name, ee_xyz0, ee_q0, &t0);
        let sha = "goal-directed-loop";
        let size = 0.025;
        let half = [size, size, size];
        let face_gap = 0.015;
        let _ = ee;
        let mut qpos = t0.qpos.clone();
        let mut cloud0 = build_ee_cloud(&model, &ee_name, &qpos, seed);
        let desired = desired_planar_push(goal, start_xy);
        if let Some(s) = midreach_aligned_seed(&model, &ee_name, desired, ee_xyz0[2], tool_off) {
            cloud0.insert(0, s);
        }
        let seed_pose = select_seed_aligned(&cloud0, tool_off, desired, &model.joints)
            .or_else(|| cloud0.first());
        let (mut xy, z) = if let Some(s) = seed_pose {
            apply_sample_qpos(&model, &ee_name, &mut qpos, s);
            let place_push = planar_tool_axis(s.quat_wxyz, tool_off).unwrap_or(desired);
            let center = object_center_for_sample(s, tool_off, place_push, half, face_gap)
                .unwrap_or([s.xyz[0] + size + face_gap, s.xyz[1], s.xyz[2]]);
            ([center[0], center[1]], center[2])
        } else {
            ([ee_xyz0[0] + size + face_gap, ee_xyz0[1]], ee_xyz0[2])
        };
        let mut goal = goal.clone();
        if let Some(t) = goal.target_xy {
            goal.target_xy = Some([xy[0] - start_xy[0] + t[0], xy[1] - start_xy[1] + t[1]]);
        }
        let goal = goal;
        let mut yaw = 0.0;
        let mut established_face: Option<String> = None;
        let mut state = LoopState::default();
        let mut trace = ClosedLoopTrace::new(bundle_id);
        trace.start_xy = xy;
        trace.goal_xy = goal.target_xy;
        trace.goal_yaw = goal.target_yaw;
        let mut last_obs: Option<WorldObservation> = None;
        let mut k = 0u32;
        loop {
            let obs = WorldObservation {
                object_id: "obj0".into(),
                xy,
                yaw,
                robot_q: qpos.clone(),
                freshness_ok: true,
                intended_contact_face: established_face.clone(),
                authority_ok: true,
                observed_at_s: k as f64,
            };
            if evaluate_goal_error(xy, yaw, &goal).reached {
                let step = receding_horizon_step(
                    &obs,
                    &goal,
                    &[],
                    state,
                    last_obs.as_ref().map(|o| (o, None)),
                );
                state = step.state.clone();
                record_action(&mut trace, &step.record, None, None, None);
                break;
            }
            let mut cands = generate_planar_push_candidates(
                "obj0",
                pose(xy, z),
                [size, size, size],
                [0.0, 0.0, 1.0],
                0.015,
                0.03,
            );
            let object = BoxObject {
                center: [xy[0], xy[1], z],
                half_extents: [size, size, size],
                quat_wxyz: [1.0, 0.0, 0.0, 0.0],
            };
            let support = SupportPlane {
                origin: [xy[0], xy[1], z - size],
                normal: [0.0, 0.0, 1.0],
            };
            let cloud = build_ee_cloud(&model, &ee_name, &qpos, seed.wrapping_add(k as u64));
            let ee_xyz = cloud.first().map(|s| s.xyz).unwrap_or(ee);
            let mut proven: std::collections::BTreeMap<
                String,
                Result<ContactManeuver, ContactInfeasible>,
            > = std::collections::BTreeMap::new();
            for c in &cands {
                if proven.contains_key(&c.face_id) {
                    continue;
                }
                proven.insert(
                    c.face_id.clone(),
                    prove_face(
                        &model,
                        &ee_name,
                        &cloud,
                        object,
                        support,
                        c.push_direction_world,
                        c.stroke_m,
                        ee_xyz,
                        tool_off,
                    ),
                );
            }
            for c in &mut cands {
                let (reachable, collision, executable, maneuver, why) = match proven.get(&c.face_id)
                {
                    Some(Ok(m)) => {
                        let close = {
                            let d = [
                                c.contact_point_world[0] - m.contact_point[0],
                                c.contact_point_world[1] - m.contact_point[1],
                            ];
                            (d[0] * d[0] + d[1] * d[1]).sqrt() < 0.03
                        };
                        if close {
                            c.contact_point_world = m.contact_point;
                            c.contact_normal_world = m.contact_normal;
                            c.push_direction_world = m.push_direction;
                            (Some(true), Some(true), Some(true), Some(m.clone()), None)
                        } else {
                            (
                                Some(true),
                                Some(true),
                                Some(false),
                                None,
                                Some("NON_EXECUTABLE".into()),
                            )
                        }
                    }
                    Some(Err(e)) => {
                        let (r, col, ex) = flags_from_infeasible(*e);
                        (r, col, ex, None, Some(e.as_str().to_string()))
                    }
                    None => (Some(false), None, None, None, Some("NO_PROOF".into())),
                };
                c.maneuver = maneuver;
                let mech = fill_mechanics_from_q(
                    &model,
                    &ee_name,
                    &qpos,
                    c.contact_point_world,
                    yaw,
                    0.05,
                    0.3,
                );
                let ctx = EvaluationContext {
                    goal: goal.clone(),
                    object_xy: xy,
                    object_yaw: yaw,
                    object_com_world: [xy[0], xy[1], z],
                    mechanics_template: Some(mech),
                    authority_ok: true,
                    robot_provided: true,
                    robot_reachable: reachable,
                    collision_admissible: collision,
                    executable_witness: executable,
                    robot_reject_reason: why,
                };
                evaluate_candidate(c, &ctx);
            }
            let step = receding_horizon_step(
                &obs,
                &goal,
                &cands,
                state,
                last_obs.as_ref().map(|o| (o, None)),
            );
            state = step.state.clone();
            if matches!(
                step.record.decision,
                LoopDecision::Halt | LoopDecision::Refuse
            ) || matches!(
                step.record.outcome,
                GoalLoopOutcome::GoalReached
                    | GoalLoopOutcome::GoalCurrentlyUnachievable
                    | GoalLoopOutcome::InsufficientPhysicalEvidence
                    | GoalLoopOutcome::AuthorityRefusal
                    | GoalLoopOutcome::InsufficientEvidence
                    | GoalLoopOutcome::GoalPhysicallyInfeasible
            ) {
                record_action(&mut trace, &step.record, step.selected.as_ref(), None, None);
                break;
            }
            let Some(sel) = step.selected.clone() else {
                record_action(&mut trace, &step.record, None, None, None);
                break;
            };
            if !sel.executable_for_plant || sel.maneuver.is_none() {
                let mut rec = step.record.clone();
                rec.decision = LoopDecision::Refuse;
                rec.outcome = GoalLoopOutcome::GoalCurrentlyUnachievable;
                rec.selection_rationale = "NO_EXECUTABLE_SELECTED_WITNESS".into();
                record_action(&mut trace, &rec, Some(&sel), None, None);
                break;
            }
            let mech = fill_mechanics_from_q(
                &model,
                &ee_name,
                &qpos,
                sel.contact_point_world,
                yaw,
                0.05,
                0.3,
            );
            let init = initiation_from_candidate(&mech, &sel, [xy[0], xy[1], z], yaw, true);
            let frozen = FrozenMechanicsPrediction::freeze(
                sel.witness.clone().unwrap_or_else(|| {
                    realityos_semantics::effect_feasibility::evaluate_planar_twist_direction(&init)
                }),
                &init,
            );
            assert!(
                !frozen.contains_privileged_force(),
                "privileged simulator force leaked into predictor"
            );
            let mut sc = push_scenario(
                [xy[0], xy[1], z],
                size,
                0.05,
                0.3,
                sel.push_direction_world,
                sel.stroke_m.max(0.03),
                seed.wrapping_add(k as u64),
            );
            sc.world_construction =
                realityos_semantics::contact_maneuver::WorldConstructionMode::FixedWorld;
            let loaded = last_loaded.take();
            let maneuver = sel.maneuver.clone();
            let qpos_now = qpos.clone();
            let run = with_episode_qpos(Some(&qpos_now), || {
                with_injected_push_maneuver(maneuver, || {
                    run_skill_episode_ex(&bundle, &model, &[], &sc, sha, "PUSH", loaded, None)
                })
            });
            let (ep, mut inst, man) = run?;
            let mut truth = truth_of(&mut inst).unwrap_or_default();
            if k == 0 {
                if let Some(p) = perturb_after {
                    let cur = body_xyz(&truth, "obj0").unwrap_or([xy[0], xy[1], z]);
                    let disp = [cur[0] + p[0], cur[1] + p[1], cur[2]];
                    let _ = inst.set_body_pos("obj0", disp);
                    if let Ok(t) = truth_of(&mut inst) {
                        truth = t;
                    }
                }
            }
            let (nxy, nyaw) = object_xy_yaw(&ep, &truth);
            last_loaded = Some((inst, man));
            qpos = truth.qpos.clone();
            let contact_established = ep.ctrl_writes > 0
                && (ep.intended_tool_contact
                    || ep.contact_pose_reached
                    || ep.had_feasible_contact_maneuver);
            let after = WorldObservation {
                object_id: "obj0".into(),
                xy: nxy,
                yaw: nyaw,
                robot_q: qpos.clone(),
                freshness_ok: true,
                intended_contact_face: if contact_established {
                    Some(sel.face_id.clone())
                } else {
                    None
                },
                authority_ok: ep.unauthorized_writes == 0,
                observed_at_s: (k + 1) as f64,
            };
            let mut rec = record_after_with_goal(step, &after, &goal);
            let blocked = ep.ctrl_writes == 0
                || matches!(
                    ep.failure_taxonomy.as_deref(),
                    Some("COLLISION_INADMISSIBLE") | Some("NO_FEASIBLE_CONTACT_POSE")
                );
            if blocked {
                let key = sel.action_key();
                if !rec.state.forbidden_action_keys.contains(&key) {
                    rec.state.forbidden_action_keys.push(key);
                }
                rec.record.decision = LoopDecision::Recover;
            }
            record_action(
                &mut trace,
                &rec.record,
                Some(&sel),
                Some(&ep),
                Some(&frozen),
            );
            if ep.unauthorized_writes > 0 {
                checkin_worker(last_loaded.take().unwrap().0);
                return Ok(trace);
            }
            xy = nxy;
            yaw = nyaw;
            established_face = after.intended_contact_face.clone();
            last_obs = Some(after);
            state = rec.state;
            k += 1;
            if rec.record.outcome == GoalLoopOutcome::GoalReached {
                break;
            }
        }
        let terminal = [
            "GoalReached",
            "GoalCurrentlyUnachievable",
            "InsufficientPhysicalEvidence",
            "AuthorityRefusal",
            "InsufficientEvidence",
            "GoalPhysicallyInfeasible",
            "ModelNotApplicable",
        ]
        .contains(&trace.final_outcome.as_str());
        if !terminal {
            let obs = WorldObservation {
                object_id: "obj0".into(),
                xy,
                yaw,
                robot_q: qpos,
                freshness_ok: true,
                intended_contact_face: established_face,
                authority_ok: true,
                observed_at_s: k as f64,
            };
            state.attempts = goal.max_bounded_attempts;
            let step = receding_horizon_step(
                &obs,
                &goal,
                &[],
                state,
                last_obs.as_ref().map(|o| (o, None)),
            );
            record_action(&mut trace, &step.record, None, None, None);
        }
        if let Some((inst, _)) = last_loaded {
            checkin_worker(inst);
        }
        Ok(trace)
    }

    #[test]
    fn mode_b_executable_witness_from_observed_q() {
        if !ensure_mujoco_or_skip() {
            write_scratch(
                "mode-b-witness.log",
                "ensure_mujoco_or_skip() == false; MuJoCo worker not started\n",
            );
            return;
        }
        let bundle = RobotBundle::load(corpus::robot_dir("arm_gripper")).expect("bundle");
        let (mut probe, man) =
            load_and_normalize(&bundle, &template_objects(true), 0).expect("load");
        let t0 = truth_of(&mut probe).expect("truth");
        let discovered =
            crate::resource_discover::discover_resources(&bundle, &man, &probe.inspect);
        let mut qualified = Vec::new();
        for r in &discovered {
            if let Ok((_, q)) = crate::resource_qualify::qualify_resource(&bundle, r) {
                qualified.push(q);
            }
        }
        let mut model = embodiment_from_manifest(&bundle, &man);
        model.resources = qualified;
        let ee_name = bundle.manifest.end_effectors[0].name.clone();
        let ee_xyz = ee_from_truth(&t0, &bundle).expect("ee");
        let ee_q = ee_quat_from_truth(&t0, &ee_name);
        let tool_off = derived_tool_offset_ee(&model, &ee_name, ee_xyz, ee_q, &t0);
        let qpos = t0.qpos.clone();
        let mut cloud = build_ee_cloud(&model, &ee_name, &qpos, 21);
        let desired = [1.0, 0.0, 0.0];
        if let Some(s) = midreach_aligned_seed(&model, &ee_name, desired, ee_xyz[2], tool_off) {
            cloud.insert(0, s);
        }
        let half = [0.025, 0.025, 0.025];
        let sample = midreach_aligned_seed(&model, &ee_name, desired, ee_xyz[2], tool_off)
            .or_else(|| select_seed_aligned(&cloud, tool_off, desired, &model.joints).cloned())
            .expect("aligned seed");
        let place_push = planar_tool_axis(sample.quat_wxyz, tool_off).unwrap_or(desired);
        let center = object_center_for_sample(&sample, tool_off, place_push, half, 0.015)
            .expect("object_center_for_sample");
        let object = BoxObject {
            center,
            half_extents: half,
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let support = SupportPlane {
            origin: [center[0], center[1], center[2] - half[2]],
            normal: [0.0, 0.0, 1.0],
        };
        let (proved, funnel) = prove_face_funnel(
            &model,
            &ee_name,
            std::slice::from_ref(&sample),
            object,
            support,
            place_push,
            0.03,
            sample.xyz,
            tool_off,
        );
        write_scratch(
            "mode-b-witness.log",
            &format!(
                "ee_xyz={ee_xyz:?} tool_off={tool_off:?} seed={:?} object={center:?} place_push={place_push:?} funnel={funnel:?} proved={}\n",
                sample.xyz,
                match &proved {
                    Ok(m) => format!("Ok(contact={:?})", m.contact_point),
                    Err(e) => format!("Err({e:?})"),
                }
            ),
        );
        checkin_worker(probe);
        let m = proved.expect("Mode B must produce an executable contact from observed q onto the observed object without moving the world");
        assert_eq!(
            m.object_center, center,
            "FixedWorld: object must stay where observation placed it"
        );
        assert!(
            (m.object_center[2] - sample.xyz[2]).abs() < 0.04,
            "object must sit on the live EE plane, object_z={} ee_z={}",
            m.object_center[2],
            sample.xyz[2]
        );
        let w = m.executable.as_ref().expect("executable witness");
        assert!(
            execution_block_reason(w).is_none(),
            "witness blocked: {:?}",
            execution_block_reason(w)
        );
    }

    fn assert_trace_honest(t: &ClosedLoopTrace, pass: i32) {
        assert_eq!(t.unauthorized_writes, 0, "pass {pass}");
        assert_eq!(t.authority_violations, 0, "pass {pass}");
        assert_eq!(t.evidence_status, SIMULATION_ONLY);
        for act in &t.actions {
            assert_eq!(
                act.get("privileged_force_in_predictor")
                    .and_then(|v| v.as_bool()),
                Some(false)
            );
        }
        let o = t.final_outcome.as_str();
        assert!(
            matches!(
                o,
                "GoalReached"
                    | "GoalCurrentlyUnachievable"
                    | "InsufficientPhysicalEvidence"
                    | "AuthorityRefusal"
                    | "InsufficientEvidence"
                    | "GoalPhysicallyInfeasible"
                    | "ModelNotApplicable"
            ),
            "pass {pass}: must halt with GOAL_REACHED or an honest refuse, got {o}"
        );
    }

    #[test]
    fn closed_loop_mujoco_goal_directed() {
        if !ensure_mujoco_or_skip() {
            write_scratch(
                "mujoco-unavailable.log",
                "ensure_mujoco_or_skip() == false; MuJoCo worker not started\n",
            );
            return;
        }
        let mut traces = Vec::new();
        let mut last_err = None;
        for pass in 1..=2 {
            let g = trans_goal([0.03, 0.0], 0.02, 4);
            let a = closed_loop_on_bundle("arm_gripper", [0.0, 0.0], &g, None, 21);
            let b = closed_loop_on_bundle("arm_gripper", [0.10, 0.0], &g, None, 22);
            match (a, b) {
                (Ok(ta), Ok(tb)) => {
                    assert_trace_honest(&ta, pass);
                    assert_trace_honest(&tb, pass);
                    if !ta.selected_faces.is_empty() && !tb.selected_faces.is_empty() {
                        assert_ne!(
                            ta.selected_faces, tb.selected_faces,
                            "different initial states must discover different sequences: A={:?} B={:?}",
                            ta.selected_faces, tb.selected_faces
                        );
                    }
                    let mut row = json!({
                        "pass": pass,
                        "from_neg_x": ta,
                        "from_pos_x": tb,
                    });
                    if pass == 1 {
                        let mut g_yaw = g.clone();
                        g_yaw.target_yaw = Some(0.35);
                        g_yaw.orientation_tolerance_rad = 0.15;
                        if let Ok(tc) = closed_loop_on_bundle(
                            "arm_gripper",
                            [0.0, 0.0],
                            &g,
                            Some([0.0, 0.03]),
                            21,
                        ) {
                            assert_trace_honest(&tc, pass);
                            row["perturbed"] = json!(tc);
                        }
                        if let Ok(td) =
                            closed_loop_on_bundle("arm_gripper", [0.0, 0.0], &g_yaw, None, 24)
                        {
                            assert_trace_honest(&td, pass);
                            row["yaw_and_translation"] = json!(td);
                        }
                        let mut emb = serde_json::Map::new();
                        for id in corpus::corpus_ids() {
                            if let Ok(t) = closed_loop_on_bundle(id, [0.0, 0.0], &g, None, 25) {
                                assert_trace_honest(&t, pass);
                                emb.insert(id.to_string(), json!(t.selected_faces));
                            }
                        }
                        row["embodiment_sequences"] = json!(emb);
                    }
                    traces.push(row);
                }
                (ea, eb) => {
                    last_err = Some(format!("{ea:?} {eb:?}"));
                }
            }
        }
        if traces.is_empty() {
            write_scratch(
                "closed-loop-mujoco.json",
                &format!("{{\"error\":{}}}", json!(last_err)),
            );
            if let Some(e) = last_err {
                panic!("closed-loop mujoco failed: {e}");
            }
        } else {
            write_scratch(
                "closed-loop-mujoco.json",
                &serde_json::to_string_pretty(&json!({
                    "metal": false,
                    "evidence_status": SIMULATION_ONLY,
                    "passes": traces,
                }))
                .unwrap(),
            );
            let any_reached = traces.iter().any(|row| {
                [
                    "from_neg_x",
                    "from_pos_x",
                    "perturbed",
                    "yaw_and_translation",
                ]
                .iter()
                .any(|k| {
                    row.get(*k).and_then(|v| v.get("final_outcome")) == Some(&json!("GoalReached"))
                })
            });
            assert!(
                any_reached,
                "closed loop must reach PlanarObjectGoal at least once; traces={}",
                serde_json::to_string(&traces).unwrap_or_default()
            );
        }
    }

    #[test]
    fn predictor_inputs_exclude_privileged_fields() {
        let g = trans_goal([0.2, 0.0], 0.01, 4);
        let cands = eval_at([0.0, 0.0], &g, true);
        let sel = select_interaction(&cands, &[]).clone();
        let SelectionOutcome::Selected { index, .. } = sel else {
            panic!("{sel:?}");
        };
        let init = initiation_from_candidate(
            &mechanics_template(0.05, 0.2, 20.0),
            &cands[index],
            [0.0, 0.0, 0.03],
            0.0,
            true,
        );
        let s = serde_json::to_string(&init).unwrap();
        for needle in [
            "actuator_force",
            "normal_force",
            "tangential_force",
            "cfrc_ext",
            "PRIVILEGED",
        ] {
            assert!(!s.contains(needle), "predictor contains {needle}");
        }
    }
}
