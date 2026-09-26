use realityos_semantics::contact_maneuver::ContactManeuver;
use realityos_semantics::maneuver_witness::{
    ExecutableContactManeuver, ManeuverPhase, PhaseTransition, TransitionKind,
};
use realityos_semantics::physical_interaction::PhysicalInteractionCandidate;
use realityos_semantics::planar_goal::GoalProgressClass;
use realityos_semantics::probe_selection::BeliefRobustness;
use realityos_semantics::recoverability::RecoverabilityClass;
use realityos_semantics::transform::Se3;

/// Synthetic execution proof for selector tests. Runtime paths must use the
/// maneuver produced by robot and collision evaluators.
pub(crate) fn attach_test_execution_proofs(candidates: &mut [PhysicalInteractionCandidate]) {
    for candidate in candidates {
        let strict = candidate.funnel.stage
            == realityos_semantics::physical_interaction::FunnelStage::GoalUseful
            && candidate.goal_progress == Some(GoalProgressClass::StrictProgress);
        candidate.decision_robustness = Some(if strict {
            BeliefRobustness::RobustStrictProgress
        } else {
            BeliefRobustness::Ambiguous
        });
        candidate.decision_recoverability = Some(if strict {
            RecoverabilityClass::ProgressAndRecoverable
        } else {
            RecoverabilityClass::ProgressButRecoverabilityUnknown
        });
        if !strict {
            continue;
        }

        let pose = |xyz| Se3 {
            xyz,
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let contact = candidate.contact_point_world;
        let approach = candidate.approach_point_world;
        let offset = |scale: f64| {
            [
                candidate.push_direction_world[0] * candidate.stroke_m * scale,
                candidate.push_direction_world[1] * candidate.stroke_m * scale,
                candidate.push_direction_world[2] * candidate.stroke_m * scale,
            ]
        };
        let mid = offset(0.5);
        let end = offset(1.0);
        let add = |a: [f64; 3], b: [f64; 3]| [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
        let phase = |xyz| ManeuverPhase::positional(pose(xyz), vec![0.0], vec![0.0], 0.0, 0.0);
        let transition = |kind| PhaseTransition::feasible(kind, 1, 0.5);
        candidate.executable_for_plant = true;
        candidate.maneuver = Some(ContactManeuver {
            contact_pose: pose(contact),
            approach_pose: pose(approach),
            contact_point: contact,
            contact_normal: candidate.contact_normal_world,
            push_direction: candidate.push_direction_world,
            requested_stroke: candidate.stroke_m,
            available_stroke: candidate.stroke_m,
            support_clearance: 0.01,
            joint_margin: 0.5,
            orientation_error: 0.0,
            object_center: [0.0, 0.0, 0.03],
            support_top_z: 0.0,
            sampled_q: vec![0.0],
            executable: Some(ExecutableContactManeuver {
                joint_names: vec!["test_joint".into()],
                start_q: vec![0.0],
                approach: phase(approach),
                contact: phase(contact),
                mid_stroke: phase(add(contact, mid)),
                end_stroke: phase(add(contact, end)),
                current_to_approach: transition(TransitionKind::CurrentToApproach),
                approach_to_contact: transition(TransitionKind::ApproachToContact),
                contact_to_mid: transition(TransitionKind::ContactToMidStroke),
                mid_to_end: transition(TransitionKind::MidToEndStroke),
            }),
        });
    }
}
