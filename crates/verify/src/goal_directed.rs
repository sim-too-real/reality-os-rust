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
    use crate::manipulation::{run_skill_episode_ex, template_objects};
    use crate::manipulation_scenarios::{ManipulationScenario, Polarity};
    use crate::manipulation_verify::body_xyz;
    use crate::mujoco_exec::{checkin_worker, ensure_mujoco_or_skip};
    use crate::observation::VerifierTruth;
    use crate::runner::load_and_normalize;
    use crate::semantics_map::embodiment_from_manifest;
    use realityos_physics::{PressureDistribution, SupportFrictionModel};
    use realityos_semantics::effect_feasibility::PlanarPushInitiation;
    use realityos_semantics::goal_loop::{
        receding_horizon_step, record_after_with_goal, GoalLoopOutcome, LoopDecision, LoopState,
        WorldObservation,
    };
    use realityos_semantics::pair_friction::PairFriction;
    use realityos_semantics::physical_interaction::{
        evaluate_all, generate_planar_push_candidates, initiation_from_candidate,
        select_interaction, EvaluationContext, SelectionOutcome,
    };
    use realityos_semantics::planar_goal::{
        yaw_from_quat_wxyz, InteractionFamily, PlanarObjectGoal, SafetyConstraints,
    };
    use realityos_semantics::provenance::Provenanced;
    use realityos_semantics::transform::Se3;
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
        let xy = ep
            .object_evidence
            .get("pose")
            .and_then(|p| p.as_array())
            .and_then(|a| Some([a.first()?.as_f64()?, a.get(1)?.as_f64()?]))
            .or_else(|| body_xyz(truth, "obj0").map(|p| [p[0], p[1]]))
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
        checkin_worker(probe);
        let mut qualified = Vec::new();
        for r in &discovered {
            if let Ok((_, q)) = crate::resource_qualify::qualify_resource(&bundle, r) {
                qualified.push(q);
            }
        }
        let mut model = embodiment_from_manifest(&bundle, &man);
        let ee_name = bundle.manifest.end_effectors[0].name.as_str();
        let chain_len = model.ee_joint_chain(ee_name).map(|c| c.len()).unwrap_or(0);
        if chain_len < 3 {
            return Err("insufficient_serial_chain".into());
        }
        model.resources = qualified;
        let sha = "goal-directed-loop";
        let size = 0.04;
        let z = ee[2];
        let anchor = [ee[0] + 0.055, ee[1], z];
        let mut xy = [anchor[0] + start_xy[0], anchor[1] + start_xy[1]];
        let mut goal = goal.clone();
        if let Some(t) = goal.target_xy {
            goal.target_xy = Some([anchor[0] + t[0], anchor[1] + t[1]]);
        }
        let goal = goal;
        let mut yaw = 0.0;
        let mut face: Option<String> = None;
        let mut state = LoopState::default();
        let mut trace = ClosedLoopTrace::new(bundle_id);
        trace.start_xy = xy;
        trace.goal_xy = goal.target_xy;
        trace.goal_yaw = goal.target_yaw;
        let mut last_loaded = None;
        for k in 0..goal.max_bounded_attempts {
            let mut cands = generate_planar_push_candidates(
                "obj0",
                pose(xy, z),
                [size, size, size],
                [0.0, 0.0, 1.0],
                0.01,
                0.03,
            );
            let ctx = EvaluationContext {
                goal: goal.clone(),
                object_xy: xy,
                object_yaw: yaw,
                object_com_world: [xy[0], xy[1], z],
                mechanics_template: Some(mechanics_template(0.05, 0.3, 20.0)),
                authority_ok: true,
                robot_provided: false,
                robot_reachable: None,
                collision_admissible: None,
                executable_witness: None,
            };
            evaluate_all(&mut cands, &ctx);
            let obs = WorldObservation {
                object_id: "obj0".into(),
                xy,
                yaw,
                robot_q: vec![],
                freshness_ok: true,
                intended_contact_face: face.clone(),
                authority_ok: true,
                observed_at_s: k as f64,
            };
            let step = receding_horizon_step(&obs, &goal, &cands, state, None);
            if step.record.outcome == GoalLoopOutcome::GoalReached
                || step.record.decision == LoopDecision::Refuse
                || step.record.decision == LoopDecision::Halt
            {
                record_action(&mut trace, &step.record, step.selected.as_ref(), None, None);
                break;
            }
            let Some(sel) = step.selected.clone() else {
                record_action(&mut trace, &step.record, None, None, None);
                break;
            };
            let init = initiation_from_candidate(
                &mechanics_template(0.05, 0.3, 20.0),
                &sel,
                [xy[0], xy[1], z],
                yaw,
                true,
            );
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
            sc.world_construction = if k == 0 {
                realityos_semantics::contact_maneuver::WorldConstructionMode::CapabilitySynthesis
            } else {
                realityos_semantics::contact_maneuver::WorldConstructionMode::FixedWorld
            };
            if k == 1 {
                if let Some(p) = perturb_after {
                    sc.objects[1]["pos"] = json!([xy[0] + p[0], xy[1] + p[1], z]);
                }
            }
            let loaded = last_loaded.take();
            let (ep, mut inst, man) =
                run_skill_episode_ex(&bundle, &model, &[], &sc, sha, "PUSH", loaded, None)?;
            let truth = truth_of(&mut inst).unwrap_or_default();
            let (nxy, nyaw) = object_xy_yaw(&ep, &truth);
            last_loaded = Some((inst, man));
            let after = WorldObservation {
                object_id: "obj0".into(),
                xy: nxy,
                yaw: nyaw,
                robot_q: truth.qpos.clone(),
                freshness_ok: true,
                intended_contact_face: Some(sel.face_id.clone()),
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
            face = Some(sel.face_id.clone());
            state = rec.state;
            if rec.record.outcome == GoalLoopOutcome::GoalReached {
                break;
            }
        }
        if let Some((inst, _)) = last_loaded {
            checkin_worker(inst);
        }
        Ok(trace)
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
            let g = trans_goal([0.06, 0.0], 0.02, 3);
            let a = closed_loop_on_bundle("arm_gripper", [0.0, 0.0], &g, None, 21);
            let b = closed_loop_on_bundle("arm_gripper", [0.10, 0.0], &g, None, 22);
            match (a, b) {
                (Ok(ta), Ok(tb)) => {
                    assert_trace_honest(&ta, pass);
                    assert_trace_honest(&tb, pass);
                    assert!(
                        !ta.selected_faces.is_empty() || !tb.selected_faces.is_empty(),
                        "loop produced no actions"
                    );
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
                            23,
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
