//! Post-hoc manipulation failure diagnosis dataset and baselines.
//!
//! Not pre-action prediction. Not `realityos-decision`. Does not construct
//! `OnlineWrite`, issue commands, or clear ESTOP / integrity abort.

use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};

/// Typed earliest-stage label. Extends `ManipulationFailure` where the
/// episode traces distinguish stages the taxonomy collapses (especially
/// grasp acquisition vs verified hold, and push contact vs displacement).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum EarliestFailureStage {
    Success,
    ExpectedRefusal,
    MissingEvidence,
    Unreachable,
    ApproachFailure,
    ContactNotEstablished,
    ContactLost,
    ContactEstablishedNoDisplacement,
    ContactEstablishedTaskFailed,
    WrongDirection,
    InsufficientDisplacement,
    GraspEmptyClose,
    GraspAcquiredHoldUnverified,
    Slip,
    ControllerFailure,
    VerifierEvidenceInsufficient,
    AuthorityRefusal,
    Unknown,
}

impl EarliestFailureStage {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Success => "SUCCESS",
            Self::ExpectedRefusal => "EXPECTED_REFUSAL",
            Self::MissingEvidence => "MISSING_EVIDENCE",
            Self::Unreachable => "UNREACHABLE",
            Self::ApproachFailure => "APPROACH_FAILURE",
            Self::ContactNotEstablished => "CONTACT_NOT_ESTABLISHED",
            Self::ContactLost => "CONTACT_LOST",
            Self::ContactEstablishedNoDisplacement => "CONTACT_ESTABLISHED_NO_DISPLACEMENT",
            Self::ContactEstablishedTaskFailed => "CONTACT_ESTABLISHED_TASK_FAILED",
            Self::WrongDirection => "WRONG_DIRECTION",
            Self::InsufficientDisplacement => "INSUFFICIENT_DISPLACEMENT",
            Self::GraspEmptyClose => "GRASP_EMPTY_CLOSE",
            Self::GraspAcquiredHoldUnverified => "GRASP_ACQUIRED_HOLD_UNVERIFIED",
            Self::Slip => "SLIP",
            Self::ControllerFailure => "CONTROLLER_FAILURE",
            Self::VerifierEvidenceInsufficient => "VERIFIER_EVIDENCE_INSUFFICIENT",
            Self::AuthorityRefusal => "AUTHORITY_REFUSAL",
            Self::Unknown => "UNKNOWN",
        }
    }

    fn all() -> &'static [Self] {
        &[
            Self::Success,
            Self::ExpectedRefusal,
            Self::MissingEvidence,
            Self::Unreachable,
            Self::ApproachFailure,
            Self::ContactNotEstablished,
            Self::ContactLost,
            Self::ContactEstablishedNoDisplacement,
            Self::ContactEstablishedTaskFailed,
            Self::WrongDirection,
            Self::InsufficientDisplacement,
            Self::GraspEmptyClose,
            Self::GraspAcquiredHoldUnverified,
            Self::Slip,
            Self::ControllerFailure,
            Self::VerifierEvidenceInsufficient,
            Self::AuthorityRefusal,
            Self::Unknown,
        ]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum FieldTag {
    PolicyVisibleRuntime,
    PostHocObserved,
    PrivilegedSimLabelOnly,
    IdentitySplitOnly,
    TargetLabel,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FieldSpec {
    pub name: &'static str,
    pub tag: FieldTag,
}

/// Leakage audit table. Causal features used by baselines must be a subset of
/// `PolicyVisibleRuntime` / `PostHocObserved` and must exclude identity.
pub fn field_catalog() -> &'static [FieldSpec] {
    &[
        FieldSpec {
            name: "skill",
            tag: FieldTag::PolicyVisibleRuntime,
        },
        FieldSpec {
            name: "gripper_class",
            tag: FieldTag::PolicyVisibleRuntime,
        },
        FieldSpec {
            name: "n_commands",
            tag: FieldTag::PostHocObserved,
        },
        FieldSpec {
            name: "n_reach_cmds",
            tag: FieldTag::PostHocObserved,
        },
        FieldSpec {
            name: "n_grasp_cmds",
            tag: FieldTag::PostHocObserved,
        },
        FieldSpec {
            name: "n_push_cmds",
            tag: FieldTag::PostHocObserved,
        },
        FieldSpec {
            name: "ctrl_writes",
            tag: FieldTag::PostHocObserved,
        },
        FieldSpec {
            name: "n_contacts",
            tag: FieldTag::PostHocObserved,
        },
        FieldSpec {
            name: "has_object_tool_contact",
            tag: FieldTag::PostHocObserved,
        },
        FieldSpec {
            name: "has_object_contact",
            tag: FieldTag::PostHocObserved,
        },
        FieldSpec {
            name: "n_authority_allow",
            tag: FieldTag::PostHocObserved,
        },
        FieldSpec {
            name: "n_authority_refuse",
            tag: FieldTag::PostHocObserved,
        },
        FieldSpec {
            name: "object_mass",
            tag: FieldTag::PrivilegedSimLabelOnly,
        },
        FieldSpec {
            name: "object_friction",
            tag: FieldTag::PrivilegedSimLabelOnly,
        },
        FieldSpec {
            name: "object_pose",
            tag: FieldTag::PrivilegedSimLabelOnly,
        },
        FieldSpec {
            name: "contact_fn_ft",
            tag: FieldTag::PrivilegedSimLabelOnly,
        },
        FieldSpec {
            name: "robot_id",
            tag: FieldTag::IdentitySplitOnly,
        },
        FieldSpec {
            name: "model_hash",
            tag: FieldTag::IdentitySplitOnly,
        },
        FieldSpec {
            name: "world_seed",
            tag: FieldTag::IdentitySplitOnly,
        },
        FieldSpec {
            name: "source_path",
            tag: FieldTag::IdentitySplitOnly,
        },
        FieldSpec {
            name: "earliest_stage",
            tag: FieldTag::TargetLabel,
        },
    ]
}

#[derive(Debug, Clone)]
pub struct DiagnosticEpisode {
    pub robot_id: String,
    pub skill: String,
    pub gripper_class: String,
    pub task_result: String,
    pub failure_taxonomy: Option<String>,
    pub expected_refusal: bool,
    pub unauthorized_writes: u64,
    pub evidence_used: Vec<String>,
    pub n_commands: f64,
    pub n_reach_cmds: f64,
    pub n_grasp_cmds: f64,
    pub n_push_cmds: f64,
    pub ctrl_writes: f64,
    pub n_contacts: f64,
    pub has_object_tool_contact: bool,
    pub has_object_contact: bool,
    pub n_authority_allow: f64,
    pub n_authority_refuse: f64,
    pub source_path: String,
    pub target: EarliestFailureStage,
}

impl DiagnosticEpisode {
    /// Causal vector. Identity fields are intentionally absent.
    pub fn causal_features(&self) -> Vec<f64> {
        vec![
            skill_code(&self.skill),
            gripper_code(&self.gripper_class),
            self.n_commands,
            self.n_reach_cmds,
            self.n_grasp_cmds,
            self.n_push_cmds,
            self.ctrl_writes,
            self.n_contacts,
            f64::from(u8::from(self.has_object_tool_contact)),
            f64::from(u8::from(self.has_object_contact)),
            self.n_authority_allow,
            self.n_authority_refuse,
        ]
    }

    pub fn causal_feature_names() -> &'static [&'static str] {
        &[
            "skill",
            "gripper_class",
            "n_commands",
            "n_reach_cmds",
            "n_grasp_cmds",
            "n_push_cmds",
            "ctrl_writes",
            "n_contacts",
            "has_object_tool_contact",
            "has_object_contact",
            "n_authority_allow",
            "n_authority_refuse",
        ]
    }
}

fn skill_code(s: &str) -> f64 {
    match s {
        "skill.grasp" => 1.0,
        "skill.push" => 2.0,
        "skill.release" => 3.0,
        "skill.reach" => 4.0,
        _ => 0.0,
    }
}

fn gripper_code(s: &str) -> f64 {
    let n = s.to_ascii_lowercase();
    if n.contains("tendon") {
        1.0
    } else if n.contains("parallel") || n.contains("coupled") {
        2.0
    } else if n.is_empty() || n == "none" {
        0.0
    } else {
        3.0
    }
}

pub fn evidence_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../docs/superpowers/evidence")
}

const HOLDOUT_ROBOT: &str = "menagerie_wx250s";

const EPISODE_FILES: &[&str] = &[
    "manipulation_panda_grasp.json",
    "manipulation_panda_push.json",
    "manipulation_panda_release.json",
    "manipulation_arm_gripper.json",
    "manipulation_ur5e_push.json",
    "manipulation_iiwa14_push.json",
];

pub fn load_diagnostic_episodes(dir: &Path) -> Result<Vec<DiagnosticEpisode>, String> {
    let mut out = Vec::new();
    for name in EPISODE_FILES {
        let path = dir.join(name);
        let raw = std::fs::read_to_string(&path).map_err(|e| format!("{name}: {e}"))?;
        let v: Value = serde_json::from_str(&raw).map_err(|e| format!("{name}: {e}"))?;
        let default_robot = v
            .get("extra")
            .and_then(|e| e.get("robot"))
            .and_then(Value::as_str)
            .unwrap_or("unknown")
            .to_string();
        let eps = v
            .get("episodes")
            .and_then(Value::as_array)
            .ok_or_else(|| format!("{name}: missing episodes"))?;
        for ep in eps {
            out.push(parse_episode(ep, &default_robot, name));
        }
    }
    Ok(out)
}

fn parse_episode(ep: &Value, default_robot: &str, source: &str) -> DiagnosticEpisode {
    let robot_id = ep
        .get("robot_id")
        .and_then(Value::as_str)
        .unwrap_or(default_robot)
        .to_string();
    let skill = ep
        .get("skill_contract")
        .and_then(Value::as_str)
        .unwrap_or("")
        .to_string();
    let gripper_class = ep
        .get("resource_topology")
        .and_then(Value::as_str)
        .unwrap_or("")
        .to_string();
    let task_result = ep
        .get("task_result")
        .and_then(Value::as_str)
        .unwrap_or("")
        .to_string();
    let failure_taxonomy = ep
        .get("failure_taxonomy")
        .and_then(Value::as_str)
        .map(str::to_string);
    let expected_refusal = ep
        .get("expected_refusal")
        .and_then(Value::as_bool)
        .unwrap_or(false);
    let unauthorized_writes = ep
        .get("unauthorized_writes")
        .and_then(Value::as_u64)
        .unwrap_or(0);
    let evidence_used: Vec<String> = ep
        .get("evidence_used")
        .and_then(Value::as_array)
        .map(|a| {
            a.iter()
                .filter_map(Value::as_str)
                .map(str::to_string)
                .collect()
        })
        .unwrap_or_default();
    let commands = ep.get("commands").and_then(Value::as_array);
    let mut n_reach = 0.0;
    let mut n_grasp = 0.0;
    let mut n_push = 0.0;
    let n_commands = commands.map(|c| c.len() as f64).unwrap_or(0.0);
    if let Some(cmds) = commands {
        for c in cmds {
            match c.get("kind").and_then(Value::as_str).unwrap_or("") {
                "reach" => n_reach += 1.0,
                "grasp" | "resource" => n_grasp += 1.0,
                "push" => n_push += 1.0,
                _ => {}
            }
        }
    }
    let contacts = ep.get("contacts").and_then(Value::as_array);
    let n_contacts = contacts.map(|c| c.len() as f64).unwrap_or(0.0);
    let mut has_object_tool_contact = false;
    let mut has_object_contact = false;
    if let Some(cs) = contacts {
        for c in cs {
            let a = c.get("a").and_then(Value::as_str).unwrap_or("");
            let b = c.get("b").and_then(Value::as_str).unwrap_or("");
            let names = format!("{a} {b}").to_ascii_lowercase();
            let obj = names.contains("obj");
            let tool = names.contains("hand")
                || names.contains("finger")
                || names.contains("gripper")
                || names.contains("ee");
            if obj {
                has_object_contact = true;
            }
            if obj && tool {
                has_object_tool_contact = true;
            }
        }
    }
    let decisions = ep.get("authority_decisions").and_then(Value::as_array);
    let mut n_allow = 0.0;
    let mut n_refuse = 0.0;
    if let Some(ds) = decisions {
        for d in ds {
            let s = d.as_str().unwrap_or("").to_ascii_lowercase();
            if s.contains("refuse") {
                n_refuse += 1.0;
            } else if s.contains("allow") {
                n_allow += 1.0;
            }
        }
    }
    let mut ep = DiagnosticEpisode {
        robot_id,
        skill,
        gripper_class,
        task_result,
        failure_taxonomy,
        expected_refusal,
        unauthorized_writes,
        evidence_used,
        n_commands,
        n_reach_cmds: n_reach,
        n_grasp_cmds: n_grasp,
        n_push_cmds: n_push,
        ctrl_writes: ep.get("ctrl_writes").and_then(Value::as_f64).unwrap_or(0.0),
        n_contacts,
        has_object_tool_contact,
        has_object_contact,
        n_authority_allow: n_allow,
        n_authority_refuse: n_refuse,
        source_path: source.to_string(),
        target: EarliestFailureStage::Unknown,
    };
    ep.target = derive_earliest_stage(&ep);
    ep
}

/// Target label from post-hoc physical evidence. Does not copy `task_result`.
pub fn derive_earliest_stage(ep: &DiagnosticEpisode) -> EarliestFailureStage {
    if ep.unauthorized_writes > 0 {
        return EarliestFailureStage::AuthorityRefusal;
    }
    let tax = ep.failure_taxonomy.as_deref().unwrap_or("");
    let skill = ep.skill.as_str();
    let has_follow = ep.evidence_used.iter().any(|e| e == "object_follows_ee");
    let empty_close =
        tax == "GRIPPER_EMPTY_CLOSE" || ep.evidence_used.iter().any(|e| e == "gripper_empty_close");

    if ep.task_result == "refuse" {
        return match tax {
            "UNREACHABLE" => EarliestFailureStage::Unreachable,
            "STALE_OBJECT" | "STALE_GRIPPER_STATE" | "STALE_EVIDENCE" | "TARGET_STALE" => {
                EarliestFailureStage::MissingEvidence
            }
            "RESOURCE_UNSUPPORTED" | "COUPLING_UNSUPPORTED" | "MODEL_FEATURE_UNSUPPORTED" => {
                EarliestFailureStage::ExpectedRefusal
            }
            "CONTROLLER_FAILURE" => EarliestFailureStage::ControllerFailure,
            "GRIPPER_EMPTY_CLOSE" => EarliestFailureStage::GraspEmptyClose,
            "" => EarliestFailureStage::Unknown,
            _ => EarliestFailureStage::Unknown,
        };
    }

    if skill == "skill.grasp" {
        if empty_close {
            return EarliestFailureStage::GraspEmptyClose;
        }
        if has_follow {
            return EarliestFailureStage::Success;
        }
        if ep.task_result == "success"
            || ep
                .evidence_used
                .iter()
                .any(|e| e == "expected_finger_object_contact")
        {
            return EarliestFailureStage::GraspAcquiredHoldUnverified;
        }
        if tax == "MISS" || tax == "BLOCKED_APPROACH" {
            return EarliestFailureStage::ApproachFailure;
        }
        if !ep.has_object_tool_contact {
            return EarliestFailureStage::ContactNotEstablished;
        }
        return EarliestFailureStage::Unknown;
    }

    if skill == "skill.push" {
        return classify_push_stage(
            ep.task_result.as_str(),
            tax,
            &ep.evidence_used,
            ep.has_object_contact,
        );
    }

    if skill == "skill.release" {
        if ep.task_result == "success" {
            return EarliestFailureStage::Success;
        }
        if tax == "STALE_GRIPPER_STATE" {
            return EarliestFailureStage::MissingEvidence;
        }
        return EarliestFailureStage::Unknown;
    }

    if ep.task_result == "success" {
        EarliestFailureStage::Success
    } else {
        EarliestFailureStage::Unknown
    }
}

/// Earliest PUSH pipeline stage. No robot identity.
pub fn classify_push_stage(
    task_result: &str,
    taxonomy: &str,
    evidence: &[String],
    has_object_contact: bool,
) -> EarliestFailureStage {
    if task_result == "success" {
        return EarliestFailureStage::Success;
    }
    let displaced = evidence
        .iter()
        .any(|e| e == "object_displaced_along_direction");
    let contacted = has_object_contact
        || evidence
            .iter()
            .any(|e| e == "controlled_contact_established");
    if taxonomy == "UNEXPECTED_CONTACT" {
        return EarliestFailureStage::WrongDirection;
    }
    if taxonomy == "OBJECT_NOT_MOVABLE" || evidence.iter().any(|e| e == "object_not_moved") {
        return EarliestFailureStage::ContactEstablishedNoDisplacement;
    }
    if contacted && !displaced {
        if taxonomy == "SLIP_AROUND_OBJECT" || taxonomy == "SLIP" {
            return EarliestFailureStage::Slip;
        }
        return EarliestFailureStage::ContactEstablishedNoDisplacement;
    }
    if taxonomy == "MISS" && !contacted {
        return EarliestFailureStage::ContactNotEstablished;
    }
    if taxonomy == "MISS" {
        return EarliestFailureStage::ApproachFailure;
    }
    if taxonomy == "SLIP_AROUND_OBJECT" || taxonomy == "SLIP" {
        return EarliestFailureStage::Slip;
    }
    EarliestFailureStage::Unknown
}

/// Taxonomy-only rules. Intentionally cannot see contact geometry or
/// `object_follows_ee`, so grasp acquisition-without-hold looks like Success.
pub fn rules_predict(ep: &DiagnosticEpisode) -> EarliestFailureStage {
    if ep.unauthorized_writes > 0 {
        return EarliestFailureStage::AuthorityRefusal;
    }
    let tax = ep.failure_taxonomy.as_deref().unwrap_or("");
    if ep.task_result == "success" && tax.is_empty() {
        return EarliestFailureStage::Success;
    }
    match tax {
        "UNREACHABLE" => EarliestFailureStage::Unreachable,
        "STALE_OBJECT" | "STALE_GRIPPER_STATE" | "STALE_EVIDENCE" | "TARGET_STALE" => {
            EarliestFailureStage::MissingEvidence
        }
        "RESOURCE_UNSUPPORTED" | "COUPLING_UNSUPPORTED" | "MODEL_FEATURE_UNSUPPORTED" => {
            EarliestFailureStage::ExpectedRefusal
        }
        "GRIPPER_EMPTY_CLOSE" => EarliestFailureStage::GraspEmptyClose,
        "CONTROLLER_FAILURE" => EarliestFailureStage::ControllerFailure,
        "SLIP_AROUND_OBJECT" | "SLIP" => EarliestFailureStage::Slip,
        "MISS" | "BLOCKED_APPROACH" => EarliestFailureStage::ApproachFailure,
        "OBJECT_NOT_MOVABLE" | "UNEXPECTED_CONTACT" => {
            EarliestFailureStage::ContactEstablishedTaskFailed
        }
        "" if ep.task_result == "fail" => EarliestFailureStage::Unknown,
        "" if ep.task_result == "refuse" => EarliestFailureStage::Unknown,
        _ => EarliestFailureStage::Unknown,
    }
}

#[derive(Debug, Clone)]
pub struct Split {
    pub train: Vec<DiagnosticEpisode>,
    pub holdout: Vec<DiagnosticEpisode>,
    pub holdout_robot: String,
    pub holdout_metrics_only: bool,
}

pub fn split_wx250s_holdout(episodes: Vec<DiagnosticEpisode>, dir: &Path) -> Split {
    let mut train = Vec::new();
    let mut holdout = Vec::new();
    for e in episodes {
        if e.robot_id == HOLDOUT_ROBOT {
            holdout.push(e);
        } else {
            train.push(e);
        }
    }
    let holdout_path = dir.join("manipulation_holdout_first_score.json");
    let metrics_only = holdout.is_empty() && holdout_path.is_file();
    Split {
        train,
        holdout,
        holdout_robot: HOLDOUT_ROBOT.into(),
        holdout_metrics_only: metrics_only,
    }
}

/// Supplementary structural holdout when wx250s has no episode traces.
pub fn split_holdout_robot(episodes: Vec<DiagnosticEpisode>, robot: &str) -> Split {
    let mut train = Vec::new();
    let mut holdout = Vec::new();
    for e in episodes {
        if e.robot_id == robot {
            holdout.push(e);
        } else {
            train.push(e);
        }
    }
    Split {
        train,
        holdout,
        holdout_robot: robot.into(),
        holdout_metrics_only: false,
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct ClassScores {
    pub accuracy: f64,
    pub macro_f1: f64,
    pub support: BTreeMap<String, usize>,
    pub confusion: BTreeMap<String, BTreeMap<String, usize>>,
    pub brier: f64,
    pub ece: f64,
    pub unknown_rate: f64,
    pub n: usize,
}

#[derive(Debug, Clone, Serialize)]
pub struct EvalReport {
    pub split: String,
    pub train_n: usize,
    pub holdout_n: usize,
    pub holdout_robot: String,
    pub holdout_metrics_only: bool,
    pub rules: ClassScores,
    pub majority: ClassScores,
    pub logistic: ClassScores,
    pub by_skill_rules: BTreeMap<String, f64>,
    pub by_skill_logistic: BTreeMap<String, f64>,
}

fn score(
    y: &[EarliestFailureStage],
    pred: &[EarliestFailureStage],
    probs: Option<&[Vec<f64>]>,
) -> ClassScores {
    assert_eq!(y.len(), pred.len());
    let n = y.len();
    if n == 0 {
        return ClassScores {
            accuracy: 0.0,
            macro_f1: 0.0,
            support: BTreeMap::new(),
            confusion: BTreeMap::new(),
            brier: 0.0,
            ece: 0.0,
            unknown_rate: 0.0,
            n: 0,
        };
    }
    let mut correct = 0usize;
    let mut support = BTreeMap::new();
    let mut confusion: BTreeMap<String, BTreeMap<String, usize>> = BTreeMap::new();
    let mut unknown = 0usize;
    for (t, p) in y.iter().zip(pred.iter()) {
        *support.entry(t.as_str().to_string()).or_insert(0) += 1;
        *confusion
            .entry(t.as_str().to_string())
            .or_default()
            .entry(p.as_str().to_string())
            .or_insert(0) += 1;
        if t == p {
            correct += 1;
        }
        if *p == EarliestFailureStage::Unknown {
            unknown += 1;
        }
    }
    let mut f1s = Vec::new();
    for cls in EarliestFailureStage::all() {
        let mut tp = 0.0;
        let mut fp = 0.0;
        let mut fn_ = 0.0;
        for (t, p) in y.iter().zip(pred.iter()) {
            if *p == *cls && *t == *cls {
                tp += 1.0;
            } else if *p == *cls {
                fp += 1.0;
            } else if *t == *cls {
                fn_ += 1.0;
            }
        }
        if tp + fp + fn_ > 0.0 {
            let prec = if tp + fp > 0.0 { tp / (tp + fp) } else { 0.0 };
            let rec = if tp + fn_ > 0.0 { tp / (tp + fn_) } else { 0.0 };
            f1s.push(if prec + rec > 0.0 {
                2.0 * prec * rec / (prec + rec)
            } else {
                0.0
            });
        }
    }
    let macro_f1 = if f1s.is_empty() {
        0.0
    } else {
        f1s.iter().sum::<f64>() / f1s.len() as f64
    };
    let classes = EarliestFailureStage::all();
    let (brier, ece) = if let Some(pr) = probs {
        let mut brier = 0.0;
        let mut bin_tot = [0.0; 10];
        let mut bin_conf = [0.0; 10];
        let mut bin_acc = [0.0; 10];
        for (i, t) in y.iter().enumerate() {
            let row = &pr[i];
            let mut max_p = 0.0;
            for (k, cls) in classes.iter().enumerate() {
                let p = row.get(k).copied().unwrap_or(0.0);
                let yk = if *t == *cls { 1.0 } else { 0.0 };
                brier += (p - yk) * (p - yk);
                if p > max_p {
                    max_p = p;
                }
            }
            let b = ((max_p * 10.0).floor() as usize).min(9);
            bin_tot[b] += 1.0;
            bin_conf[b] += max_p;
            if pred[i] == *t {
                bin_acc[b] += 1.0;
            }
        }
        brier /= (n * classes.len()) as f64;
        let mut ece = 0.0;
        for b in 0..10 {
            if bin_tot[b] > 0.0 {
                ece += (bin_tot[b] / n as f64)
                    * (bin_acc[b] / bin_tot[b] - bin_conf[b] / bin_tot[b]).abs();
            }
        }
        (brier, ece)
    } else {
        (0.0, 0.0)
    };
    ClassScores {
        accuracy: correct as f64 / n as f64,
        macro_f1,
        support,
        confusion,
        brier,
        ece,
        unknown_rate: unknown as f64 / n as f64,
        n,
    }
}

fn majority_from_train(train: &[DiagnosticEpisode]) -> HashMap<String, EarliestFailureStage> {
    let mut per_skill: HashMap<String, HashMap<EarliestFailureStage, usize>> = HashMap::new();
    for e in train {
        *per_skill
            .entry(e.skill.clone())
            .or_default()
            .entry(e.target)
            .or_insert(0) += 1;
    }
    per_skill
        .into_iter()
        .map(|(sk, counts)| {
            let best = counts
                .into_iter()
                .max_by_key(|(_, n)| *n)
                .map(|(c, _)| c)
                .unwrap_or(EarliestFailureStage::Unknown);
            (sk, best)
        })
        .collect()
}

struct LogReg {
    weights: Vec<Vec<f64>>,
    bias: Vec<f64>,
}

impl LogReg {
    fn train(xs: &[Vec<f64>], ys: &[usize], n_class: usize, steps: usize) -> Self {
        let d = xs.first().map(Vec::len).unwrap_or(0);
        let mut weights = vec![vec![0.0; d]; n_class];
        let mut bias = vec![0.0; n_class];
        if xs.is_empty() {
            return Self { weights, bias };
        }
        let lr = 0.05;
        let l2 = 1e-4;
        for _ in 0..steps {
            for (x, &y) in xs.iter().zip(ys.iter()) {
                let mut logits: Vec<f64> = (0..n_class)
                    .map(|c| bias[c] + dot(&weights[c], x))
                    .collect();
                let m = logits.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                let mut s = 0.0;
                for z in &mut logits {
                    *z = (*z - m).exp();
                    s += *z;
                }
                for c in 0..n_class {
                    let p = logits[c] / s.max(1e-12);
                    let g = p - if c == y { 1.0 } else { 0.0 };
                    for j in 0..d {
                        weights[c][j] -= lr * (g * x[j] + l2 * weights[c][j]);
                    }
                    bias[c] -= lr * g;
                }
            }
        }
        Self { weights, bias }
    }

    fn predict_proba(&self, x: &[f64]) -> Vec<f64> {
        let n_class = self.bias.len();
        let mut logits: Vec<f64> = (0..n_class)
            .map(|c| self.bias[c] + dot(&self.weights[c], x))
            .collect();
        let m = logits.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let mut s = 0.0;
        for z in &mut logits {
            *z = (*z - m).exp();
            s += *z;
        }
        logits.into_iter().map(|z| z / s.max(1e-12)).collect()
    }
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b.iter()).map(|(x, y)| x * y).sum()
}

fn class_index(s: EarliestFailureStage) -> usize {
    EarliestFailureStage::all()
        .iter()
        .position(|c| *c == s)
        .unwrap_or(EarliestFailureStage::all().len() - 1)
}

pub fn evaluate_split(split: &Split) -> EvalReport {
    let classes = EarliestFailureStage::all();
    let n_class = classes.len();
    let xs: Vec<Vec<f64>> = split
        .train
        .iter()
        .map(DiagnosticEpisode::causal_features)
        .collect();
    let ys: Vec<usize> = split.train.iter().map(|e| class_index(e.target)).collect();
    let model = LogReg::train(&xs, &ys, n_class, 8);
    let maj = majority_from_train(&split.train);

    let hold = &split.holdout;
    let y: Vec<_> = hold.iter().map(|e| e.target).collect();
    let rules_p: Vec<_> = hold.iter().map(rules_predict).collect();
    let maj_p: Vec<_> = hold
        .iter()
        .map(|e| *maj.get(&e.skill).unwrap_or(&EarliestFailureStage::Unknown))
        .collect();
    let mut log_p = Vec::new();
    let mut log_pr = Vec::new();
    for e in hold {
        let pr = model.predict_proba(&e.causal_features());
        let idx = pr
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap_or(n_class - 1);
        log_p.push(classes[idx]);
        log_pr.push(pr);
    }

    let mut by_skill_rules = BTreeMap::new();
    let mut by_skill_logistic = BTreeMap::new();
    let mut skill_tot: BTreeMap<String, (usize, usize, usize)> = BTreeMap::new();
    for (e, r, l) in hold
        .iter()
        .zip(rules_p.iter())
        .zip(log_p.iter())
        .map(|((e, r), l)| (e, r, l))
    {
        let ent = skill_tot.entry(e.skill.clone()).or_insert((0, 0, 0));
        ent.0 += 1;
        if *r == e.target {
            ent.1 += 1;
        }
        if *l == e.target {
            ent.2 += 1;
        }
    }
    for (sk, (n, r, l)) in skill_tot {
        by_skill_rules.insert(sk.clone(), r as f64 / n as f64);
        by_skill_logistic.insert(sk, l as f64 / n as f64);
    }

    EvalReport {
        split: format!("holdout:{}", split.holdout_robot),
        train_n: split.train.len(),
        holdout_n: split.holdout.len(),
        holdout_robot: split.holdout_robot.clone(),
        holdout_metrics_only: split.holdout_metrics_only,
        rules: score(&y, &rules_p, None),
        majority: score(&y, &maj_p, None),
        logistic: score(&y, &log_p, Some(&log_pr)),
        by_skill_rules,
        by_skill_logistic,
    }
}

pub fn recommendation(wx: &EvalReport, supp: &EvalReport) -> &'static str {
    if wx.holdout_n == 0 {
        return "NOT YET";
    }
    if supp.logistic.accuracy <= supp.rules.accuracy + 0.02 {
        return "NOT YET";
    }
    if supp.logistic.ece > 0.25 {
        return "NOT YET";
    }
    "NOT YET"
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dir() -> PathBuf {
        evidence_dir()
    }

    #[test]
    fn catalog_excludes_identity_from_causal_features() {
        let names = DiagnosticEpisode::causal_feature_names();
        for banned in [
            "robot_id",
            "vendor",
            "model_hash",
            "source_path",
            "world_seed",
        ] {
            assert!(
                !names.contains(&banned),
                "{banned} leaked into causal features"
            );
        }
        let ident: Vec<_> = field_catalog()
            .iter()
            .filter(|f| f.tag == FieldTag::IdentitySplitOnly)
            .map(|f| f.name)
            .collect();
        assert!(ident.contains(&"robot_id"));
        assert!(ident.contains(&"model_hash"));
    }

    #[test]
    fn push_labels_distinguish_contact_vs_displacement() {
        let eps = load_diagnostic_episodes(&dir()).expect("load");
        let push: Vec<_> = eps
            .iter()
            .filter(|e| e.skill == "skill.push" && e.robot_id.contains("panda"))
            .collect();
        assert!(push.len() >= 20);
        let established_fail = push
            .iter()
            .filter(|e| {
                matches!(
                    e.target,
                    EarliestFailureStage::ContactEstablishedNoDisplacement
                        | EarliestFailureStage::ContactEstablishedTaskFailed
                        | EarliestFailureStage::Slip
                        | EarliestFailureStage::WrongDirection
                        | EarliestFailureStage::InsufficientDisplacement
                )
            })
            .count();
        let success = push
            .iter()
            .filter(|e| e.target == EarliestFailureStage::Success)
            .count();
        assert!(
            established_fail > 0,
            "need contact-without-task-success labels"
        );
        assert!(success > 0);
        assert_ne!(
            established_fail, success,
            "must not copy task_result into a single class"
        );
    }

    #[test]
    fn grasp_labels_distinguish_acquisition_from_verified_hold() {
        let eps = load_diagnostic_episodes(&dir()).expect("load");
        let grasp: Vec<_> = eps
            .iter()
            .filter(|e| e.skill == "skill.grasp" && e.robot_id.contains("panda"))
            .collect();
        let unverified = grasp
            .iter()
            .filter(|e| e.target == EarliestFailureStage::GraspAcquiredHoldUnverified)
            .count();
        let held = grasp
            .iter()
            .filter(|e| e.target == EarliestFailureStage::Success)
            .count();
        let empty = grasp
            .iter()
            .filter(|e| e.target == EarliestFailureStage::GraspEmptyClose)
            .count();
        assert!(
            unverified > held,
            "acquisition-without-hold must dominate verified hold"
        );
        assert!(empty > 0);
        assert!(held > 0, "object_follows_ee should mark verified hold");
    }

    #[test]
    fn push_stage_contact_without_displacement() {
        assert_eq!(
            classify_push_stage(
                "fail",
                "SLIP_AROUND_OBJECT",
                &["controlled_contact_established".into()],
                true
            ),
            EarliestFailureStage::Slip
        );
        assert_eq!(
            classify_push_stage(
                "fail",
                "OBJECT_NOT_MOVABLE",
                &["object_not_moved".into()],
                true
            ),
            EarliestFailureStage::ContactEstablishedNoDisplacement
        );
        assert_eq!(
            classify_push_stage("fail", "MISS", &[], false),
            EarliestFailureStage::ContactNotEstablished
        );
        assert_eq!(
            classify_push_stage(
                "success",
                "",
                &["object_displaced_along_direction".into()],
                true
            ),
            EarliestFailureStage::Success
        );
    }

    #[test]
    fn unknown_when_unjustified() {
        let mut ep = DiagnosticEpisode {
            robot_id: "x".into(),
            skill: "skill.turn".into(),
            gripper_class: String::new(),
            task_result: "fail".into(),
            failure_taxonomy: None,
            expected_refusal: false,
            unauthorized_writes: 0,
            evidence_used: vec![],
            n_commands: 0.0,
            n_reach_cmds: 0.0,
            n_grasp_cmds: 0.0,
            n_push_cmds: 0.0,
            ctrl_writes: 0.0,
            n_contacts: 0.0,
            has_object_tool_contact: false,
            has_object_contact: false,
            n_authority_allow: 0.0,
            n_authority_refuse: 0.0,
            source_path: "t".into(),
            target: EarliestFailureStage::Unknown,
        };
        assert_eq!(derive_earliest_stage(&ep), EarliestFailureStage::Unknown);
        ep.task_result = "refuse".into();
        assert_eq!(derive_earliest_stage(&ep), EarliestFailureStage::Unknown);
    }

    #[test]
    fn wx250s_is_held_out_and_has_no_episode_traces() {
        let dir = dir();
        let eps = load_diagnostic_episodes(&dir).expect("load");
        let split = split_wx250s_holdout(eps, &dir);
        assert_eq!(split.holdout_robot, "menagerie_wx250s");
        assert!(
            split.train.iter().all(|e| e.robot_id != "menagerie_wx250s"),
            "wx250s leaked into train"
        );
        assert!(
            split.holdout_metrics_only && split.holdout.is_empty(),
            "frozen first-score has no episodes; do not invent them"
        );
        let holdout_file = dir.join("manipulation_holdout_first_score.json");
        let raw = std::fs::read_to_string(holdout_file).unwrap();
        assert!(raw.contains("menagerie_wx250s"));
        assert!(!raw.contains("\"episodes\": ["));
    }

    #[test]
    fn rules_majority_logistic_run_on_arm_gripper_holdout() {
        let dir = dir();
        let eps = load_diagnostic_episodes(&dir).expect("load");
        let split = split_holdout_robot(eps, "arm_gripper");
        assert!(split.holdout.len() >= 100);
        assert!(split.train.iter().any(|e| e.robot_id.contains("panda")));
        let report = evaluate_split(&split);
        eprintln!(
            "supp holdout={} train_n={} holdout_n={} rules_acc={:.4} rules_f1={:.4} maj_acc={:.4} log_acc={:.4} log_f1={:.4} log_brier={:.4} log_ece={:.4}",
            report.holdout_robot,
            report.train_n,
            report.holdout_n,
            report.rules.accuracy,
            report.rules.macro_f1,
            report.majority.accuracy,
            report.logistic.accuracy,
            report.logistic.macro_f1,
            report.logistic.brier,
            report.logistic.ece
        );
        eprintln!("supp by_skill_rules={:?}", report.by_skill_rules);
        eprintln!("supp by_skill_logistic={:?}", report.by_skill_logistic);
        assert!(report.rules.n > 0);
        assert!(report.majority.n > 0);
        assert!(report.logistic.n > 0);
        assert!(report.rules.accuracy >= 0.0 && report.rules.accuracy <= 1.0);
        assert!(report.logistic.accuracy >= 0.0 && report.logistic.accuracy <= 1.0);
        let wx = split_wx250s_holdout(load_diagnostic_episodes(&dir).unwrap(), &dir);
        let wx_report = evaluate_split(&wx);
        eprintln!(
            "wx250s holdout_n={} metrics_only={} rec={}",
            wx_report.holdout_n,
            wx_report.holdout_metrics_only,
            recommendation(&wx_report, &report)
        );
        assert_eq!(recommendation(&wx_report, &report), "NOT YET");
    }
}
