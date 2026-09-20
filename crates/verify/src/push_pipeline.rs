//! Causal PUSH pipeline. Post-hoc labels from evidence, not robot identity.

use serde::{Deserialize, Serialize};

use crate::failure_diagnosis::FieldTag;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum PushStage {
    TargetAvailable,
    Reachable,
    ApproachReached,
    ContactEstablished,
    ContactMaintained,
    PushStrokeExecuted,
    ObjectDisplaced,
    DisplacementDirectionValid,
    DisplacementMagnitudeValid,
    TaskVerified,
}

impl PushStage {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::TargetAvailable => "TARGET_AVAILABLE",
            Self::Reachable => "REACHABLE",
            Self::ApproachReached => "APPROACH_REACHED",
            Self::ContactEstablished => "CONTACT_ESTABLISHED",
            Self::ContactMaintained => "CONTACT_MAINTAINED",
            Self::PushStrokeExecuted => "PUSH_STROKE_EXECUTED",
            Self::ObjectDisplaced => "OBJECT_DISPLACED",
            Self::DisplacementDirectionValid => "DISPLACEMENT_DIRECTION_VALID",
            Self::DisplacementMagnitudeValid => "DISPLACEMENT_MAGNITUDE_VALID",
            Self::TaskVerified => "TASK_VERIFIED",
        }
    }

    pub fn all() -> &'static [Self] {
        &[
            Self::TargetAvailable,
            Self::Reachable,
            Self::ApproachReached,
            Self::ContactEstablished,
            Self::ContactMaintained,
            Self::PushStrokeExecuted,
            Self::ObjectDisplaced,
            Self::DisplacementDirectionValid,
            Self::DisplacementMagnitudeValid,
            Self::TaskVerified,
        ]
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PushEvidence {
    pub target_available: bool,
    pub reachable: bool,
    pub approach_reached: bool,
    pub contact_established: bool,
    pub contact_maintained: bool,
    pub stroke_executed: bool,
    pub object_displaced: bool,
    pub direction_valid: bool,
    pub magnitude_valid: bool,
    pub task_verified: bool,
    pub expected_refusal: bool,
    #[serde(default)]
    pub feasible_maneuver: bool,
    #[serde(default)]
    pub contact_pose_reached: bool,
    #[serde(default)]
    pub intended_tool_contact: bool,
    #[serde(default)]
    pub unintended_robot_contact: bool,
    #[serde(default)]
    pub apparent_robot_object_contact: bool,
    #[serde(default)]
    pub current_to_approach_feasible: bool,
    #[serde(default)]
    pub approach_q_reached: bool,
    #[serde(default)]
    pub contact_q_reached: bool,
    #[serde(default)]
    pub mid_q_reached: bool,
    #[serde(default)]
    pub end_q_reached: bool,
    #[serde(default)]
    pub executed_witness_q: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PushPipelineTrace {
    pub first_stage_entered: Option<PushStage>,
    pub last_stage_completed: Option<PushStage>,
    pub earliest_failed_stage: Option<PushStage>,
}

/// Walk the pipeline in order. Stops at the first false stage that is
/// required. Expected refusals do not claim TASK_VERIFIED.
pub fn classify_push_pipeline(ev: &PushEvidence) -> PushPipelineTrace {
    if ev.expected_refusal {
        return PushPipelineTrace {
            first_stage_entered: Some(PushStage::TargetAvailable),
            last_stage_completed: None,
            earliest_failed_stage: None,
        };
    }
    let flags = [
        (PushStage::TargetAvailable, ev.target_available),
        (PushStage::Reachable, ev.reachable),
        (PushStage::ApproachReached, ev.approach_reached),
        (PushStage::ContactEstablished, ev.contact_established),
        (PushStage::ContactMaintained, ev.contact_maintained),
        (PushStage::PushStrokeExecuted, ev.stroke_executed),
        (PushStage::ObjectDisplaced, ev.object_displaced),
        (PushStage::DisplacementDirectionValid, ev.direction_valid),
        (PushStage::DisplacementMagnitudeValid, ev.magnitude_valid),
        (PushStage::TaskVerified, ev.task_verified),
    ];
    let mut first = None;
    let mut last = None;
    let mut failed = None;
    for (stage, ok) in flags {
        if ok {
            if first.is_none() {
                first = Some(stage);
            }
            last = Some(stage);
        } else {
            failed = Some(stage);
            if first.is_none() {
                first = Some(stage);
            }
            break;
        }
    }
    PushPipelineTrace {
        first_stage_entered: first,
        last_stage_completed: last,
        earliest_failed_stage: failed,
    }
}

fn taxonomy_is_slip(taxonomy: &str) -> bool {
    taxonomy == "SLIP" || taxonomy == "SLIP_AROUND_OBJECT" || taxonomy.contains("SLIP")
}

/// Support/table/floor is not PUSH contact. The other body must be the object.
pub fn is_support_surface(name: &str) -> bool {
    realityos_semantics::contact::is_support_surface_name(name)
}

/// Old broad metric: any non-support robot body vs object. Not PUSH contact.
pub fn names_are_apparent_robot_object_contact(a: &str, b: &str, object_id: &str) -> bool {
    if object_id.is_empty() {
        return false;
    }
    let obj_a = a.contains(object_id);
    let obj_b = b.contains(object_id);
    if obj_a == obj_b {
        return false;
    }
    let other = if obj_a { b } else { a };
    !is_support_surface(other)
}

/// PUSH contact: target object ↔ declared manipulation contact geometry.
pub fn names_are_ee_object_contact(a: &str, b: &str, object_id: &str, intended: &[String]) -> bool {
    realityos_semantics::contact::names_are_intended_tool_object_contact(a, b, object_id, intended)
}

pub fn evidence_from_episode_fields(
    task_result: &str,
    taxonomy: &str,
    evidence: &[String],
    has_ee_object_contact: bool,
    expected_refusal: bool,
) -> PushEvidence {
    let miss = taxonomy == "MISS";
    let unreachable = taxonomy == "UNREACHABLE";
    let success = task_result == "success";
    if expected_refusal {
        return PushEvidence {
            target_available: !taxonomy.contains("STALE"),
            reachable: !unreachable,
            approach_reached: false,
            contact_established: false,
            contact_maintained: false,
            stroke_executed: false,
            object_displaced: false,
            direction_valid: false,
            magnitude_valid: false,
            task_verified: false,
            expected_refusal: true,
            feasible_maneuver: false,
            contact_pose_reached: false,
            intended_tool_contact: false,
            unintended_robot_contact: false,
            apparent_robot_object_contact: false,
            current_to_approach_feasible: false,
            approach_q_reached: false,
            contact_q_reached: false,
            mid_q_reached: false,
            end_q_reached: false,
            executed_witness_q: false,
        };
    }
    let contacted = has_ee_object_contact;
    let displaced = evidence
        .iter()
        .any(|e| e == "object_displaced_along_direction");
    PushEvidence {
        target_available: !taxonomy.contains("STALE"),
        reachable: !unreachable,
        approach_reached: !miss || contacted,
        contact_established: contacted,
        contact_maintained: contacted && !taxonomy_is_slip(taxonomy),
        stroke_executed: contacted && taxonomy != "CONTROLLER_FAILURE",
        object_displaced: displaced,
        direction_valid: displaced && taxonomy != "UNEXPECTED_CONTACT",
        magnitude_valid: displaced && success,
        task_verified: success,
        expected_refusal: false,
        feasible_maneuver: false,
        contact_pose_reached: false,
        intended_tool_contact: contacted,
        unintended_robot_contact: false,
        apparent_robot_object_contact: false,
        current_to_approach_feasible: false,
        approach_q_reached: false,
        contact_q_reached: false,
        mid_q_reached: false,
        end_q_reached: false,
        executed_witness_q: false,
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaggedField {
    pub name: &'static str,
    pub tag: FieldTag,
}

pub fn push_diagnostic_field_catalog() -> &'static [TaggedField] {
    &[
        TaggedField {
            name: "episode_id",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "skill",
            tag: FieldTag::PolicyVisibleRuntime,
        },
        TaggedField {
            name: "topology_n_joints",
            tag: FieldTag::PolicyVisibleRuntime,
        },
        TaggedField {
            name: "world_features",
            tag: FieldTag::PolicyVisibleRuntime,
        },
        TaggedField {
            name: "object_features",
            tag: FieldTag::PolicyVisibleRuntime,
        },
        TaggedField {
            name: "object_bounds",
            tag: FieldTag::PolicyVisibleRuntime,
        },
        TaggedField {
            name: "requested_push_direction",
            tag: FieldTag::PolicyVisibleRuntime,
        },
        TaggedField {
            name: "requested_push_distance",
            tag: FieldTag::PolicyVisibleRuntime,
        },
        TaggedField {
            name: "approach_evidence",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "contact_evidence",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "contact_established",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "stroke_evidence",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "displacement_evidence",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "object_displaced",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "authority_verdict",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "controller_outcome",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "verifier_result",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "first_stage_entered",
            tag: FieldTag::TargetLabel,
        },
        TaggedField {
            name: "last_stage_completed",
            tag: FieldTag::TargetLabel,
        },
        TaggedField {
            name: "earliest_failed_stage",
            tag: FieldTag::TargetLabel,
        },
        TaggedField {
            name: "final_task_result",
            tag: FieldTag::TargetLabel,
        },
        TaggedField {
            name: "failure_taxonomy",
            tag: FieldTag::TargetLabel,
        },
        TaggedField {
            name: "pre_contact_taxonomy",
            tag: FieldTag::TargetLabel,
        },
        TaggedField {
            name: "feasible_contact_maneuver",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "contact_pose_reached",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "approach_pose_reached",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "mujoco_ee_object_contact",
            tag: FieldTag::PrivilegedSimLabelOnly,
        },
        TaggedField {
            name: "intended_tool_contact",
            tag: FieldTag::PrivilegedSimLabelOnly,
        },
        TaggedField {
            name: "unintended_robot_contact",
            tag: FieldTag::PrivilegedSimLabelOnly,
        },
        TaggedField {
            name: "support_contact",
            tag: FieldTag::PrivilegedSimLabelOnly,
        },
        TaggedField {
            name: "self_collision",
            tag: FieldTag::PrivilegedSimLabelOnly,
        },
        TaggedField {
            name: "obstacle_contact",
            tag: FieldTag::PrivilegedSimLabelOnly,
        },
        TaggedField {
            name: "apparent_robot_object_contact",
            tag: FieldTag::PrivilegedSimLabelOnly,
        },
        TaggedField {
            name: "world_construction",
            tag: FieldTag::TargetLabel,
        },
        TaggedField {
            name: "selected_rank_why",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "provenance",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "mujoco_contact_force",
            tag: FieldTag::PrivilegedSimLabelOnly,
        },
        TaggedField {
            name: "robot_id",
            tag: FieldTag::IdentitySplitOnly,
        },
        TaggedField {
            name: "unauthorized_writes",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "current_to_approach_feasible",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "approach_q_reached",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "contact_q_reached",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "executed_witness_q",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "n_geometric_candidates",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "n_ik_solutions",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "n_complete_witnesses",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "n_joint_margin_valid",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "n_transition_valid",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "n_executable_candidates",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "n_selected_executable",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "select_last_block_reason",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "first_divergence_layer",
            tag: FieldTag::TargetLabel,
        },
        TaggedField {
            name: "first_divergence_reason",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "first_divergence_phase",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "joint_error_before",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "joint_error_after",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "cartesian_residual_before",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "cartesian_residual_after",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "expected_fk_ee_from_got",
            tag: FieldTag::PolicyVisibleRuntime,
        },
        TaggedField {
            name: "expected_fk_ee_from_target",
            tag: FieldTag::PolicyVisibleRuntime,
        },
        TaggedField {
            name: "post_hoc_mujoco_ee",
            tag: FieldTag::PrivilegedSimLabelOnly,
        },
        TaggedField {
            name: "fk_mujoco_residual",
            tag: FieldTag::PrivilegedSimLabelOnly,
        },
        TaggedField {
            name: "witness_execution_steps",
            tag: FieldTag::PostHocObserved,
        },
        TaggedField {
            name: "witness_execution_trace",
            tag: FieldTag::PostHocObserved,
        },
    ]
}

/// Attach catalog provenance tags. Privileged sim fields stay labeled;
/// they are never a runtime policy input.
pub fn tagged_push_record(values: &[(&'static str, serde_json::Value)]) -> serde_json::Value {
    let mut out = serde_json::Map::new();
    for (name, value) in values {
        let tag = push_diagnostic_field_catalog()
            .iter()
            .find(|f| f.name == *name)
            .map(|f| f.tag)
            .unwrap_or(FieldTag::PostHocObserved);
        out.insert(
            (*name).to_string(),
            serde_json::json!({ "value": value, "tag": tag }),
        );
    }
    serde_json::Value::Object(out)
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PushFunnel {
    pub n: u64,
    pub n_contact: u64,
    pub n_approach: u64,
    pub n_contact_maintained: u64,
    pub n_stroke: u64,
    pub n_displaced: u64,
    pub n_direction_ok: u64,
    pub n_task_success: u64,
    pub n_task_success_given_contact: u64,
    pub n_unauthorized_writes: u64,
    #[serde(default)]
    pub n_positive_reachable: u64,
    #[serde(default)]
    pub n_feasible_maneuver: u64,
    #[serde(default)]
    pub n_contact_pose_reached: u64,
    #[serde(default)]
    pub n_intended_tool_contact: u64,
    #[serde(default)]
    pub n_unintended_robot_contact: u64,
    #[serde(default)]
    pub n_apparent_robot_object_contact: u64,
    #[serde(default)]
    pub n_current_to_approach_feasible: u64,
    #[serde(default)]
    pub n_approach_q_reached: u64,
    #[serde(default)]
    pub n_contact_q_reached: u64,
    #[serde(default)]
    pub n_mid_q_reached: u64,
    #[serde(default)]
    pub n_end_q_reached: u64,
    #[serde(default)]
    pub n_executed_witness_q: u64,
    #[serde(default)]
    pub n_approach_given_feasible: u64,
    #[serde(default)]
    pub n_geometric_candidates: u64,
    #[serde(default)]
    pub n_ik_solutions: u64,
    #[serde(default)]
    pub n_phase_ik_attempts: u64,
    #[serde(default)]
    pub n_phase_ik_successes: u64,
    #[serde(default)]
    pub n_complete_ik_chains: u64,
    #[serde(default)]
    pub n_complete_witnesses: u64,
    #[serde(default)]
    pub n_joint_margin_valid: u64,
    #[serde(default)]
    pub n_transition_valid: u64,
    #[serde(default)]
    pub n_executable_candidates: u64,
    #[serde(default)]
    pub n_selected_executable: u64,
}

impl PushFunnel {
    pub fn absorb(&mut self, ev: &PushEvidence, unauthorized: u64) {
        self.n += 1;
        self.n_unauthorized_writes += unauthorized;
        if !ev.expected_refusal && ev.reachable {
            self.n_positive_reachable += 1;
        }
        if ev.feasible_maneuver {
            self.n_feasible_maneuver += 1;
            if ev.approach_reached {
                self.n_approach_given_feasible += 1;
            }
        }
        if ev.contact_pose_reached {
            self.n_contact_pose_reached += 1;
        }
        if ev.approach_reached {
            self.n_approach += 1;
        }
        if ev.intended_tool_contact {
            self.n_intended_tool_contact += 1;
        }
        if ev.unintended_robot_contact {
            self.n_unintended_robot_contact += 1;
        }
        if ev.apparent_robot_object_contact {
            self.n_apparent_robot_object_contact += 1;
        }
        if ev.current_to_approach_feasible {
            self.n_current_to_approach_feasible += 1;
        }
        if ev.approach_q_reached {
            self.n_approach_q_reached += 1;
        }
        if ev.contact_q_reached {
            self.n_contact_q_reached += 1;
        }
        if ev.mid_q_reached {
            self.n_mid_q_reached += 1;
        }
        if ev.end_q_reached {
            self.n_end_q_reached += 1;
        }
        if ev.executed_witness_q {
            self.n_executed_witness_q += 1;
        }
        if ev.contact_established {
            self.n_contact += 1;
            if ev.task_verified {
                self.n_task_success_given_contact += 1;
            }
            if ev.contact_maintained {
                self.n_contact_maintained += 1;
            }
        }
        if ev.stroke_executed {
            self.n_stroke += 1;
            if ev.object_displaced {
                self.n_displaced += 1;
                if ev.direction_valid {
                    self.n_direction_ok += 1;
                }
            }
        }
        if ev.task_verified {
            self.n_task_success += 1;
        }
    }

    pub fn p_contact_given_approach(&self) -> f64 {
        if self.n_approach == 0 {
            0.0
        } else {
            self.n_contact as f64 / self.n_approach as f64
        }
    }

    pub fn p_task_given_contact(&self) -> f64 {
        if self.n_contact == 0 {
            0.0
        } else {
            self.n_task_success_given_contact as f64 / self.n_contact as f64
        }
    }

    pub fn p_contact_maintained_given_contact(&self) -> f64 {
        if self.n_contact == 0 {
            0.0
        } else {
            self.n_contact_maintained as f64 / self.n_contact as f64
        }
    }

    pub fn p_stroke_given_contact(&self) -> f64 {
        if self.n_contact == 0 {
            0.0
        } else {
            self.n_stroke as f64 / self.n_contact as f64
        }
    }

    pub fn p_displaced_given_stroke(&self) -> f64 {
        if self.n_stroke == 0 {
            0.0
        } else {
            self.n_displaced as f64 / self.n_stroke as f64
        }
    }

    pub fn p_direction_given_displacement(&self) -> f64 {
        if self.n_displaced == 0 {
            0.0
        } else {
            self.n_direction_ok as f64 / self.n_displaced as f64
        }
    }

    pub fn p_approach_given_feasible_maneuver(&self) -> f64 {
        if self.n_feasible_maneuver == 0 {
            0.0
        } else {
            self.n_approach_given_feasible as f64 / self.n_feasible_maneuver as f64
        }
    }

    pub fn p_approach_q_given_feasible_maneuver(&self) -> f64 {
        if self.n_feasible_maneuver == 0 {
            0.0
        } else {
            self.n_approach_q_reached as f64 / self.n_feasible_maneuver as f64
        }
    }

    pub fn p_current_to_approach_given_feasible(&self) -> f64 {
        if self.n_feasible_maneuver == 0 {
            0.0
        } else {
            self.n_current_to_approach_feasible as f64 / self.n_feasible_maneuver as f64
        }
    }

    pub fn p_contact_given_positive_reachable(&self) -> f64 {
        if self.n_positive_reachable == 0 {
            0.0
        } else {
            self.n_contact as f64 / self.n_positive_reachable as f64
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use realityos_semantics::push::{effective_push_distance, push_stroke_success_radius};

    #[test]
    fn radius_strictly_less_than_meaningful_stroke() {
        for d in [0.012, 0.02, 0.03, 0.05, 0.08, 0.12] {
            let stroke = effective_push_distance(d);
            let r = push_stroke_success_radius(stroke);
            assert!(r < stroke, "radius {r} must be < stroke {stroke}");
        }
    }

    #[test]
    fn contact_without_displacement_fails_at_object_displaced() {
        let ev = PushEvidence {
            target_available: true,
            reachable: true,
            approach_reached: true,
            contact_established: true,
            contact_maintained: true,
            stroke_executed: true,
            object_displaced: false,
            direction_valid: false,
            magnitude_valid: false,
            task_verified: false,
            expected_refusal: false,
            feasible_maneuver: false,
            contact_pose_reached: false,
            ..Default::default()
        };
        let t = classify_push_pipeline(&ev);
        assert_eq!(t.first_stage_entered, Some(PushStage::TargetAvailable));
        assert_eq!(t.last_stage_completed, Some(PushStage::PushStrokeExecuted));
        assert_eq!(t.earliest_failed_stage, Some(PushStage::ObjectDisplaced));
    }

    #[test]
    fn miss_fails_at_contact() {
        let ev = evidence_from_episode_fields("fail", "MISS", &[], false, false);
        let t = classify_push_pipeline(&ev);
        assert_eq!(t.earliest_failed_stage, Some(PushStage::ApproachReached));
    }

    #[test]
    fn slip_around_object_is_not_contact_maintained() {
        let ev = evidence_from_episode_fields(
            "fail",
            "SLIP_AROUND_OBJECT",
            &["controlled_contact_established".into()],
            true,
            false,
        );
        assert!(ev.contact_established);
        assert!(
            !ev.contact_maintained,
            "SLIP_AROUND_OBJECT is contact lost, not a string-equal SLIP miss"
        );
        let t = classify_push_pipeline(&ev);
        assert_eq!(t.earliest_failed_stage, Some(PushStage::ContactMaintained));
    }

    #[test]
    fn table_or_floor_contact_is_not_ee_object_contact() {
        assert!(!names_are_ee_object_contact(
            "table",
            "obj0",
            "obj0",
            &["finger".into()]
        ));
        assert!(!names_are_ee_object_contact(
            "obj0",
            "floor",
            "obj0",
            &["finger".into()]
        ));
        assert!(!names_are_ee_object_contact(
            "world",
            "obj0",
            "obj0",
            &["finger".into()]
        ));
        assert!(
            !names_are_ee_object_contact("link7", "obj0", "obj0", &["finger".into()]),
            "forearm/link collision is not PUSH contact"
        );
        assert!(names_are_ee_object_contact(
            "obj0",
            "finger",
            "obj0",
            &["finger".into()]
        ));
        assert!(names_are_apparent_robot_object_contact(
            "link7", "obj0", "obj0"
        ));
        assert!(names_are_apparent_robot_object_contact(
            "obj0", "finger", "obj0"
        ));
    }

    #[test]
    fn expected_refusal_does_not_establish_push_contact() {
        let ev = evidence_from_episode_fields(
            "refuse",
            "UNREACHABLE",
            &["controlled_contact_established".into()],
            true,
            true,
        );
        assert!(!ev.contact_established);
        assert!(!ev.contact_maintained);
        assert!(!ev.stroke_executed);
        assert!(!ev.task_verified);
    }

    #[test]
    fn verified_success_completes_pipeline() {
        let ev = evidence_from_episode_fields(
            "success",
            "",
            &[
                "controlled_contact_established".into(),
                "object_displaced_along_direction".into(),
            ],
            true,
            false,
        );
        let t = classify_push_pipeline(&ev);
        assert_eq!(t.earliest_failed_stage, None);
        assert_eq!(t.last_stage_completed, Some(PushStage::TaskVerified));
    }

    #[test]
    fn catalog_separates_privileged_sim_from_runtime() {
        let cat = push_diagnostic_field_catalog();
        assert!(cat.iter().any(|f| {
            f.name == "mujoco_contact_force" && f.tag == FieldTag::PrivilegedSimLabelOnly
        }));
        assert!(cat.iter().any(|f| {
            f.name == "requested_push_direction" && f.tag == FieldTag::PolicyVisibleRuntime
        }));
        assert!(cat
            .iter()
            .any(|f| f.name == "robot_id" && f.tag == FieldTag::IdentitySplitOnly));
        assert!(!cat.iter().any(|f| {
            f.tag == FieldTag::PolicyVisibleRuntime && f.name == "mujoco_contact_force"
        }));
    }

    #[test]
    fn catalog_covers_required_diagnostic_fields() {
        let names: Vec<&str> = push_diagnostic_field_catalog()
            .iter()
            .map(|f| f.name)
            .collect();
        for required in [
            "episode_id",
            "skill",
            "topology_n_joints",
            "world_features",
            "object_features",
            "requested_push_direction",
            "requested_push_distance",
            "approach_evidence",
            "contact_evidence",
            "stroke_evidence",
            "displacement_evidence",
            "authority_verdict",
            "controller_outcome",
            "verifier_result",
            "earliest_failed_stage",
            "final_task_result",
            "failure_taxonomy",
            "pre_contact_taxonomy",
            "feasible_contact_maneuver",
            "n_geometric_candidates",
            "n_ik_solutions",
            "n_complete_witnesses",
            "n_joint_margin_valid",
            "n_transition_valid",
            "n_executable_candidates",
            "n_selected_executable",
            "select_last_block_reason",
            "provenance",
            "mujoco_contact_force",
            "mujoco_ee_object_contact",
            "intended_tool_contact",
            "unintended_robot_contact",
            "support_contact",
            "self_collision",
            "obstacle_contact",
            "apparent_robot_object_contact",
            "world_construction",
            "robot_id",
            "unauthorized_writes",
        ] {
            assert!(
                names.contains(&required),
                "missing catalog field {required}"
            );
        }
    }

    #[test]
    fn tagged_record_keeps_privileged_sim_off_runtime() {
        let rec = tagged_push_record(&[
            (
                "requested_push_direction",
                serde_json::json!([1.0, 0.0, 0.0]),
            ),
            ("mujoco_contact_force", serde_json::json!(0.0)),
            ("robot_id", serde_json::json!("split-only")),
        ]);
        assert_eq!(
            rec["mujoco_contact_force"]["tag"],
            "PRIVILEGED_SIM_LABEL_ONLY"
        );
        assert_eq!(
            rec["requested_push_direction"]["tag"],
            "POLICY_VISIBLE_RUNTIME"
        );
        assert_eq!(rec["robot_id"]["tag"], "IDENTITY_SPLIT_ONLY");
        assert_ne!(
            rec["mujoco_contact_force"]["tag"],
            rec["requested_push_direction"]["tag"]
        );
    }

    #[test]
    fn catalog_keeps_privileged_ee_object_contact_off_runtime() {
        let cat = push_diagnostic_field_catalog();
        assert!(cat.iter().any(|f| {
            f.name == "mujoco_ee_object_contact" && f.tag == FieldTag::PrivilegedSimLabelOnly
        }));
        assert!(cat
            .iter()
            .any(|f| { f.name == "pre_contact_taxonomy" && f.tag == FieldTag::TargetLabel }));
        assert!(!cat.iter().any(|f| {
            f.tag == FieldTag::PolicyVisibleRuntime && f.name == "mujoco_ee_object_contact"
        }));
        assert!(cat
            .iter()
            .any(|f| { f.name == "approach_pose_reached" && f.tag == FieldTag::PostHocObserved }));
        assert!(cat.iter().any(|f| {
            f.name == "post_hoc_mujoco_ee" && f.tag == FieldTag::PrivilegedSimLabelOnly
        }));
        assert!(cat.iter().any(|f| {
            f.name == "fk_mujoco_residual" && f.tag == FieldTag::PrivilegedSimLabelOnly
        }));
        assert!(cat.iter().any(|f| {
            f.name == "expected_fk_ee_from_got" && f.tag == FieldTag::PolicyVisibleRuntime
        }));
        assert!(cat
            .iter()
            .any(|f| { f.name == "first_divergence_layer" && f.tag == FieldTag::TargetLabel }));
        assert!(cat
            .iter()
            .any(|f| { f.name == "approach_q_reached" && f.tag == FieldTag::PostHocObserved }));
    }

    #[test]
    fn no_feasible_taxonomy_helper_still_defaults_approach_true() {
        let ev =
            evidence_from_episode_fields("refuse", "NO_FEASIBLE_CONTACT_POSE", &[], false, false);
        assert!(
            ev.approach_reached,
            "helper uses !MISS; episode overlay must replace this"
        );
    }

    #[test]
    fn funnel_p_task_given_contact() {
        let mut f = PushFunnel::default();
        let contact_fail = PushEvidence {
            target_available: true,
            reachable: true,
            approach_reached: true,
            contact_established: true,
            contact_maintained: true,
            stroke_executed: true,
            object_displaced: false,
            direction_valid: false,
            magnitude_valid: false,
            task_verified: false,
            expected_refusal: false,
            feasible_maneuver: true,
            contact_pose_reached: true,
            ..Default::default()
        };
        let contact_ok = PushEvidence {
            object_displaced: true,
            direction_valid: true,
            magnitude_valid: true,
            task_verified: true,
            ..contact_fail.clone()
        };
        f.absorb(&contact_fail, 0);
        f.absorb(&contact_ok, 0);
        assert_eq!(f.n_contact, 2);
        assert!((f.p_task_given_contact() - 0.5).abs() < 1e-12);
        assert_eq!(f.n_unauthorized_writes, 0);
        assert_eq!(f.n_positive_reachable, 2);
        assert!((f.p_contact_given_positive_reachable() - 1.0).abs() < 1e-12);
        assert_eq!(f.n_feasible_maneuver, 2);
        assert!((f.p_approach_given_feasible_maneuver() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn p_displaced_given_stroke_stays_in_unit_interval() {
        let mut f = PushFunnel::default();
        let displaced_without_stroke = PushEvidence {
            target_available: true,
            reachable: true,
            approach_reached: true,
            contact_established: false,
            contact_maintained: false,
            stroke_executed: false,
            object_displaced: true,
            direction_valid: true,
            magnitude_valid: true,
            task_verified: true,
            expected_refusal: false,
            feasible_maneuver: false,
            contact_pose_reached: false,
            ..Default::default()
        };
        let stroke_without_displace = PushEvidence {
            object_displaced: false,
            direction_valid: false,
            magnitude_valid: false,
            task_verified: false,
            stroke_executed: true,
            contact_established: true,
            contact_maintained: true,
            ..displaced_without_stroke.clone()
        };
        for _ in 0..3 {
            f.absorb(&displaced_without_stroke, 0);
        }
        f.absorb(&stroke_without_displace, 0);
        let p = f.p_displaced_given_stroke();
        assert!(
            (0.0..=1.0).contains(&p),
            "P(object displaced | stroke) must be in [0,1], got {p} (n_displaced={} n_stroke={})",
            f.n_displaced,
            f.n_stroke
        );
        let pd = f.p_direction_given_displacement();
        assert!(
            (0.0..=1.0).contains(&pd),
            "P(correct direction | displacement) must be in [0,1], got {pd}"
        );
    }
}
