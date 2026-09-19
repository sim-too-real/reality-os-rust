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

#[derive(Debug, Clone, Serialize, Deserialize)]
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

pub fn evidence_from_episode_fields(
    task_result: &str,
    taxonomy: &str,
    evidence: &[String],
    has_contact: bool,
    expected_refusal: bool,
) -> PushEvidence {
    let contacted = has_contact
        || evidence
            .iter()
            .any(|e| e == "controlled_contact_established");
    let displaced = evidence
        .iter()
        .any(|e| e == "object_displaced_along_direction");
    let miss = taxonomy == "MISS";
    let unreachable = taxonomy == "UNREACHABLE";
    let success = task_result == "success";
    PushEvidence {
        target_available: !taxonomy.contains("STALE"),
        reachable: !unreachable,
        approach_reached: !miss || contacted,
        contact_established: contacted,
        contact_maintained: contacted && taxonomy != "SLIP",
        stroke_executed: contacted && taxonomy != "CONTROLLER_FAILURE",
        object_displaced: displaced,
        direction_valid: displaced && taxonomy != "UNEXPECTED_CONTACT",
        magnitude_valid: displaced && success,
        task_verified: success,
        expected_refusal,
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
}

impl PushFunnel {
    pub fn absorb(&mut self, ev: &PushEvidence, unauthorized: u64) {
        self.n += 1;
        self.n_unauthorized_writes += unauthorized;
        if ev.approach_reached {
            self.n_approach += 1;
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
        }
        if ev.object_displaced {
            self.n_displaced += 1;
        }
        if ev.direction_valid {
            self.n_direction_ok += 1;
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
            "provenance",
            "mujoco_contact_force",
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
    }
}
