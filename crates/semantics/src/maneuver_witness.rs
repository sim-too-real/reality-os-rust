//! Configuration-space witness for an executable PUSH contact maneuver.
//!
//! Named joints only. No robot identity.

pub use crate::command_domain::MIN_NAMED_JOINT_MARGIN_FRAC;
use crate::command_domain::{joint_position_in_bounds, named_joint_limit_margin};
use crate::embodiment::Joint;
use crate::transform::Se3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum TransitionVerdict {
    Feasible,
    Refused,
    Unknown,
}

impl TransitionVerdict {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Feasible => "FEASIBLE",
            Self::Refused => "REFUSED",
            Self::Unknown => "UNKNOWN",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum TransitionKind {
    CurrentToApproach,
    ApproachToContact,
    ContactToMidStroke,
    MidToEndStroke,
}

impl TransitionKind {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::CurrentToApproach => "CURRENT_TO_APPROACH",
            Self::ApproachToContact => "APPROACH_TO_CONTACT",
            Self::ContactToMidStroke => "CONTACT_TO_MID_STROKE",
            Self::MidToEndStroke => "MID_TO_END_STROKE",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ManeuverPhase {
    pub pose: Se3,
    pub q: Vec<f64>,
    pub seed_q: Vec<f64>,
    pub residual: f64,
    pub position_feasible: bool,
    pub orientation_checked: bool,
    pub orientation_error: f64,
    pub full_pose_feasible: bool,
}

impl ManeuverPhase {
    pub fn positional(
        pose: Se3,
        q: Vec<f64>,
        seed_q: Vec<f64>,
        residual: f64,
        orient_err: f64,
    ) -> Self {
        let position_feasible = residual.is_finite() && q.iter().all(|v| v.is_finite());
        Self {
            pose,
            q,
            seed_q,
            residual,
            position_feasible,
            orientation_checked: false,
            orientation_error: orient_err,
            full_pose_feasible: false,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PhaseTransition {
    pub kind: TransitionKind,
    pub verdict: TransitionVerdict,
    pub reason: String,
    pub n_samples: usize,
    pub min_joint_margin: f64,
}

impl PhaseTransition {
    pub fn feasible(kind: TransitionKind, n_samples: usize, min_joint_margin: f64) -> Self {
        Self {
            kind,
            verdict: TransitionVerdict::Feasible,
            reason: String::new(),
            n_samples,
            min_joint_margin,
        }
    }

    pub fn refused(kind: TransitionKind, reason: impl Into<String>, n_samples: usize) -> Self {
        Self {
            kind,
            verdict: TransitionVerdict::Refused,
            reason: reason.into(),
            n_samples,
            min_joint_margin: 0.0,
        }
    }

    pub fn unknown(kind: TransitionKind, reason: impl Into<String>) -> Self {
        Self {
            kind,
            verdict: TransitionVerdict::Unknown,
            reason: reason.into(),
            n_samples: 0,
            min_joint_margin: 0.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExecutableContactManeuver {
    pub joint_names: Vec<String>,
    pub start_q: Vec<f64>,
    pub approach: ManeuverPhase,
    pub contact: ManeuverPhase,
    pub mid_stroke: ManeuverPhase,
    pub end_stroke: ManeuverPhase,
    pub current_to_approach: PhaseTransition,
    pub approach_to_contact: PhaseTransition,
    pub contact_to_mid: PhaseTransition,
    pub mid_to_end: PhaseTransition,
}

impl ExecutableContactManeuver {
    pub fn is_executable(&self) -> bool {
        self.current_to_approach.verdict == TransitionVerdict::Feasible
            && self.approach_to_contact.verdict == TransitionVerdict::Feasible
            && self.contact_to_mid.verdict == TransitionVerdict::Feasible
            && self.mid_to_end.verdict == TransitionVerdict::Feasible
            && !self.approach.q.is_empty()
            && self.approach.q.len() == self.joint_names.len()
            && self.contact.q.len() == self.joint_names.len()
            && self.mid_stroke.q.len() == self.joint_names.len()
            && self.end_stroke.q.len() == self.joint_names.len()
            && self.start_q.len() == self.joint_names.len()
    }

    pub fn has_approach_q(&self) -> bool {
        !self.approach.q.is_empty() && self.approach.q.len() == self.joint_names.len()
    }

    /// Re-bind `CURRENT_Q → APPROACH_Q` to the live named configuration.
    pub fn reassess_current_to_approach(&self, live_q: &[f64], joints: &[Joint]) -> Self {
        let mut w = self.clone();
        if live_q.len() != w.joint_names.len() || live_q.iter().any(|v| !v.is_finite()) {
            return w;
        }
        w.start_q = live_q.to_vec();
        w.current_to_approach = assess_named_interpolation(
            TransitionKind::CurrentToApproach,
            &w.joint_names,
            live_q,
            &w.approach.q,
            joints,
        );
        w
    }
}

/// Max |Δq| across aligned named joints. Length mismatch is infinite error.
pub fn named_q_max_abs_error(got: &[f64], want: &[f64]) -> f64 {
    if got.len() != want.len() || got.is_empty() {
        return f64::INFINITY;
    }
    got.iter()
        .zip(want.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0_f64, f64::max)
}

pub fn named_q_reached(got: &[f64], want: &[f64], tol: f64) -> bool {
    named_q_max_abs_error(got, want) <= tol
}

/// One bounded joint-space step toward `target`. Does not re-solve Cartesian IK.
pub fn step_toward_named_q(
    live: &[f64],
    target: &[f64],
    max_delta: f64,
) -> Result<Vec<f64>, &'static str> {
    if live.len() != target.len() || live.is_empty() {
        return Err("q_len_mismatch");
    }
    if !max_delta.is_finite() || max_delta <= 0.0 {
        return Err("bad_delta");
    }
    Ok(live
        .iter()
        .zip(target.iter())
        .map(|(a, b)| a + (b - a).clamp(-max_delta, max_delta))
        .collect())
}

/// Why this witness must not emit actuator writes. `None` means it may execute.
pub fn execution_block_reason(w: &ExecutableContactManeuver) -> Option<String> {
    if w.is_executable() {
        return None;
    }
    for t in [
        &w.current_to_approach,
        &w.approach_to_contact,
        &w.contact_to_mid,
        &w.mid_to_end,
    ] {
        if t.verdict != TransitionVerdict::Feasible {
            if t.reason.is_empty() {
                return Some(t.kind.as_str().to_string());
            }
            return Some(t.reason.clone());
        }
    }
    if !w.has_approach_q() {
        return Some("NO_APPROACH_Q".into());
    }
    Some("NOT_EXECUTABLE".into())
}

/// Interior+endpoint count from joint displacement, clamped to 5..=20.
pub fn interpolation_count(qa: &[f64], qb: &[f64]) -> usize {
    let n = qa.len().min(qb.len());
    if n == 0 {
        return 5;
    }
    let mut d = 0.0;
    for i in 0..n {
        let e = qa[i] - qb[i];
        d += e * e;
    }
    let mag = d.sqrt();
    ((5.0 + mag * 8.0).round() as usize).clamp(5, 20)
}

/// Linear named-q interpolation including endpoints.
pub fn interpolate_named_q(
    qa: &[f64],
    qb: &[f64],
    n: usize,
) -> Result<Vec<Vec<f64>>, &'static str> {
    if qa.len() != qb.len() || qa.is_empty() {
        return Err("q_len_mismatch");
    }
    if qa.iter().chain(qb.iter()).any(|v| !v.is_finite()) {
        return Err("non_finite");
    }
    let n = n.clamp(5, 20);
    let mut out = Vec::with_capacity(n);
    let denom = (n - 1) as f64;
    for i in 0..n {
        let t = i as f64 / denom;
        let mut q = Vec::with_capacity(qa.len());
        for j in 0..qa.len() {
            q.push(qa[j] * (1.0 - t) + qb[j] * t);
        }
        out.push(q);
    }
    Ok(out)
}

/// Assess a joint-space segment. Name-aligned limits only.
pub fn assess_named_interpolation(
    kind: TransitionKind,
    names: &[String],
    qa: &[f64],
    qb: &[f64],
    joints: &[Joint],
) -> PhaseTransition {
    if names.len() != qa.len() || names.len() != qb.len() || names.is_empty() {
        return PhaseTransition::refused(kind, "q_len_mismatch", 0);
    }
    if qa.iter().chain(qb.iter()).any(|v| !v.is_finite()) {
        return PhaseTransition::refused(kind, "non_finite", 0);
    }
    let n = interpolation_count(qa, qb);
    let samples = match interpolate_named_q(qa, qb, n) {
        Ok(s) => s,
        Err(r) => return PhaseTransition::refused(kind, r, 0),
    };
    let mut min_margin = 1.0_f64;
    for sample in &samples {
        for (i, name) in names.iter().enumerate() {
            let Some(j) = joints.iter().find(|j| j.name == *name) else {
                continue;
            };
            if !joint_position_in_bounds(j, sample[i]) {
                return PhaseTransition::refused(kind, "JOINT_LIMIT", samples.len());
            }
            let (Some(lo), Some(hi)) = (j.q_min.value, j.q_max.value) else {
                continue;
            };
            let span = (hi - lo).abs().max(1e-6);
            let d = (sample[i] - lo).min(hi - sample[i]).max(0.0);
            let frac = d / span;
            if frac + 1e-12 < MIN_NAMED_JOINT_MARGIN_FRAC {
                return PhaseTransition::refused(kind, "JOINT_LIMIT", samples.len());
            }
            min_margin = min_margin.min(frac);
        }
    }
    PhaseTransition::feasible(kind, samples.len(), min_margin.clamp(0.0, 1.0))
}

/// Build a witness. Later-phase `seed_q` must be the previous phase `q`.
pub fn witness_from_continuing_phases(
    joint_names: Vec<String>,
    start_q: Vec<f64>,
    approach: ManeuverPhase,
    contact: ManeuverPhase,
    mid_stroke: ManeuverPhase,
    end_stroke: ManeuverPhase,
    joints: &[Joint],
) -> ExecutableContactManeuver {
    let current_to_approach = assess_named_interpolation(
        TransitionKind::CurrentToApproach,
        &joint_names,
        &start_q,
        &approach.q,
        joints,
    );
    let approach_to_contact = if contact.seed_q == approach.q {
        assess_named_interpolation(
            TransitionKind::ApproachToContact,
            &joint_names,
            &approach.q,
            &contact.q,
            joints,
        )
    } else {
        PhaseTransition::refused(TransitionKind::ApproachToContact, "IK_DISCONTINUITY", 0)
    };
    let contact_to_mid = if mid_stroke.seed_q == contact.q {
        assess_named_interpolation(
            TransitionKind::ContactToMidStroke,
            &joint_names,
            &contact.q,
            &mid_stroke.q,
            joints,
        )
    } else {
        PhaseTransition::refused(TransitionKind::ContactToMidStroke, "IK_DISCONTINUITY", 0)
    };
    let mid_to_end = if end_stroke.seed_q == mid_stroke.q {
        assess_named_interpolation(
            TransitionKind::MidToEndStroke,
            &joint_names,
            &mid_stroke.q,
            &end_stroke.q,
            joints,
        )
    } else {
        PhaseTransition::refused(TransitionKind::MidToEndStroke, "IK_DISCONTINUITY", 0)
    };
    ExecutableContactManeuver {
        joint_names,
        start_q,
        approach,
        contact,
        mid_stroke,
        end_stroke,
        current_to_approach,
        approach_to_contact,
        contact_to_mid,
        mid_to_end,
    }
}

/// Current named-q reach tolerance. Do not raise this to mint success.
pub const NAMED_Q_REACH_TOL_MAX: f64 = 0.20;

/// Material Reality OS FK vs privileged simulator residual (meters).
pub const FK_MUJOCO_RESIDUAL_MATERIAL_M: f64 = 0.05;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum TrackingProgress {
    Reached,
    Progress,
    Stall,
    Diverge,
    LengthMismatch,
}

impl TrackingProgress {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Reached => "REACHED",
            Self::Progress => "PROGRESS",
            Self::Stall => "STALL",
            Self::Diverge => "DIVERGE",
            Self::LengthMismatch => "LENGTH_MISMATCH",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ContinueNamedQ {
    Continue,
    StopReached,
    StopStall,
    StopDiverge,
    StopBudget,
    StopAuthority,
    StopConstraint,
}

impl ContinueNamedQ {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Continue => "CONTINUE",
            Self::StopReached => "STOP_REACHED",
            Self::StopStall => "STOP_STALL",
            Self::StopDiverge => "STOP_DIVERGE",
            Self::StopBudget => "STOP_BUDGET",
            Self::StopAuthority => "STOP_AUTHORITY",
            Self::StopConstraint => "STOP_CONSTRAINT",
        }
    }

    pub fn keep_going(self) -> bool {
        self == Self::Continue
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum FirstDivergenceLayer {
    None,
    WitnessTargetWrong,
    JointNameAlignmentWrong,
    JointActuatorBindingWrong,
    JointToActuatorLoweringWrong,
    CommandModifiedIncorrectly,
    AuthorityRefusal,
    ActuatorCommandIneffective,
    InsufficientExecutionTime,
    InterpolationInappropriate,
    JointTrackingStalled,
    JointTrackingDiverged,
    JointTargetTimeout,
    QReachedPoseMismatch,
    FkModelMismatch,
    Unknown,
}

impl FirstDivergenceLayer {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::None => "NONE",
            Self::WitnessTargetWrong => "WITNESS_TARGET_WRONG",
            Self::JointNameAlignmentWrong => "JOINT_NAME_ALIGNMENT_WRONG",
            Self::JointActuatorBindingWrong => "JOINT_ACTUATOR_BINDING_WRONG",
            Self::JointToActuatorLoweringWrong => "JOINT_TO_ACTUATOR_LOWERING_WRONG",
            Self::CommandModifiedIncorrectly => "COMMAND_MODIFIED_INCORRECTLY",
            Self::AuthorityRefusal => "AUTHORITY_REFUSAL",
            Self::ActuatorCommandIneffective => "ACTUATOR_COMMAND_INEFFECTIVE",
            Self::InsufficientExecutionTime => "INSUFFICIENT_EXECUTION_TIME",
            Self::InterpolationInappropriate => "INTERPOLATION_INAPPROPRIATE",
            Self::JointTrackingStalled => "JOINT_TRACKING_STALLED",
            Self::JointTrackingDiverged => "JOINT_TRACKING_DIVERGED",
            Self::JointTargetTimeout => "JOINT_TARGET_TIMEOUT",
            Self::QReachedPoseMismatch => "Q_REACHED_POSE_MISMATCH",
            Self::FkModelMismatch => "FK_MODEL_MISMATCH",
            Self::Unknown => "UNKNOWN",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NamedQStepTrace {
    pub phase: String,
    pub joint_names: Vec<String>,
    pub live_q_before: Vec<f64>,
    pub phase_target_q: Vec<f64>,
    pub commanded_q: Vec<f64>,
    pub interpolate: bool,
    pub max_delta: f64,
    pub lowering_ok: bool,
    pub lowering_reason: String,
    pub actuator_names: Vec<String>,
    pub actuator_targets: Vec<f64>,
    pub control_mode: String,
    pub ctrlrange: Vec<[f64; 2]>,
    pub authority_verdict: String,
    pub ctrl_applied: Vec<f64>,
    pub duration_s: f64,
    pub live_q_after: Vec<f64>,
    pub joint_error_before: f64,
    pub joint_error_after: f64,
    pub q_velocity_after: Vec<f64>,
    pub error_reduction_ratio: f64,
    pub saturated: bool,
    pub cartesian_residual_before: f64,
    pub cartesian_residual_after: f64,
    pub expected_fk_ee_from_got: [f64; 3],
    pub expected_fk_ee_from_target: [f64; 3],
    pub post_hoc_mujoco_ee: Option<[f64; 3]>,
    pub fk_mujoco_residual: Option<f64>,
    pub progress: TrackingProgress,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FirstDivergence {
    pub layer: FirstDivergenceLayer,
    pub reason: String,
    pub phase: String,
    pub joint_error_before: f64,
    pub joint_error_after: f64,
    pub cartesian_residual_after: f64,
    pub fk_mujoco_residual: Option<f64>,
    pub q_reached: bool,
    pub pose_reached: bool,
}

impl FirstDivergence {
    fn from_step(
        layer: FirstDivergenceLayer,
        reason: impl Into<String>,
        step: &NamedQStepTrace,
        q_reached: bool,
        pose_reached: bool,
    ) -> Self {
        Self {
            layer,
            reason: reason.into(),
            phase: step.phase.clone(),
            joint_error_before: step.joint_error_before,
            joint_error_after: step.joint_error_after,
            cartesian_residual_after: step.cartesian_residual_after,
            fk_mujoco_residual: step.fk_mujoco_residual,
            q_reached,
            pose_reached,
        }
    }
}

/// Classify tracking from measured joint error. Progress requires `error_after < error_before`.
pub fn classify_named_q_progress(
    error_before: f64,
    error_after: f64,
    tol: f64,
) -> TrackingProgress {
    if !error_before.is_finite() || !error_after.is_finite() {
        return TrackingProgress::LengthMismatch;
    }
    if error_after <= tol {
        return TrackingProgress::Reached;
    }
    let eps = 1e-4_f64.max(0.01 * error_before.abs());
    if error_after < error_before - eps {
        TrackingProgress::Progress
    } else if error_after > error_before + eps {
        TrackingProgress::Diverge
    } else {
        TrackingProgress::Stall
    }
}

pub fn continue_named_q_execution(
    progress: TrackingProgress,
    steps_used: usize,
    max_steps: usize,
    authority_ok: bool,
    constraint_ok: bool,
) -> ContinueNamedQ {
    if !authority_ok {
        return ContinueNamedQ::StopAuthority;
    }
    if !constraint_ok || progress == TrackingProgress::LengthMismatch {
        return ContinueNamedQ::StopConstraint;
    }
    match progress {
        TrackingProgress::Reached => ContinueNamedQ::StopReached,
        TrackingProgress::Diverge => ContinueNamedQ::StopDiverge,
        TrackingProgress::Stall => ContinueNamedQ::StopStall,
        TrackingProgress::LengthMismatch => ContinueNamedQ::StopConstraint,
        TrackingProgress::Progress => {
            if steps_used >= max_steps {
                ContinueNamedQ::StopBudget
            } else {
                ContinueNamedQ::Continue
            }
        }
    }
}

pub fn error_reduction_ratio(error_before: f64, error_after: f64) -> f64 {
    if !error_before.is_finite() || !error_after.is_finite() || error_before.abs() < 1e-12 {
        return 0.0;
    }
    (error_before - error_after) / error_before
}

pub fn xyz_residual(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// True iff `commanded` is the bounded step from `live` toward `target`.
pub fn command_is_bounded_step(
    live: &[f64],
    target: &[f64],
    commanded: &[f64],
    max_delta: f64,
) -> bool {
    let Ok(expected) = step_toward_named_q(live, target, max_delta) else {
        return false;
    };
    expected.len() == commanded.len()
        && expected
            .iter()
            .zip(commanded.iter())
            .all(|(a, b)| (a - b).abs() <= 1e-9)
}

fn authority_refused(verdict: &str) -> bool {
    let u = verdict.to_ascii_uppercase();
    u.contains("REFUSE") || u.contains("ABORT") || u.contains("DENY")
}

/// Earliest measured layer where expectation and physical state diverge.
pub fn classify_first_divergence(
    traces: &[NamedQStepTrace],
    q_reached: bool,
    pose_reached: bool,
    pose_tol: f64,
) -> FirstDivergence {
    if traces.is_empty() {
        return FirstDivergence {
            layer: FirstDivergenceLayer::Unknown,
            reason: "no_step_traces".into(),
            phase: String::new(),
            joint_error_before: f64::INFINITY,
            joint_error_after: f64::INFINITY,
            cartesian_residual_after: f64::INFINITY,
            fk_mujoco_residual: None,
            q_reached,
            pose_reached,
        };
    }
    for step in traces {
        if step.joint_names.is_empty()
            || step.live_q_before.len() != step.phase_target_q.len()
            || step.joint_names.len() != step.phase_target_q.len()
            || step.live_q_after.len() != step.phase_target_q.len()
        {
            return FirstDivergence::from_step(
                FirstDivergenceLayer::JointNameAlignmentWrong,
                "named_q_length_mismatch",
                step,
                q_reached,
                pose_reached,
            );
        }
        if !step.lowering_ok {
            let why = if step.lowering_reason.is_empty() {
                "lowering_failed".into()
            } else {
                step.lowering_reason.clone()
            };
            return FirstDivergence::from_step(
                FirstDivergenceLayer::JointToActuatorLoweringWrong,
                why,
                step,
                q_reached,
                pose_reached,
            );
        }
        if authority_refused(&step.authority_verdict) {
            return FirstDivergence::from_step(
                FirstDivergenceLayer::AuthorityRefusal,
                step.authority_verdict.clone(),
                step,
                q_reached,
                pose_reached,
            );
        }
        if step.commanded_q.len() != step.phase_target_q.len()
            || step.commanded_q.iter().any(|v| !v.is_finite())
        {
            return FirstDivergence::from_step(
                FirstDivergenceLayer::CommandModifiedIncorrectly,
                "commanded_q_invalid",
                step,
                q_reached,
                pose_reached,
            );
        }
    }

    let last = traces.last().unwrap();
    let saw_progress = traces.iter().any(|s| {
        s.progress == TrackingProgress::Progress || s.progress == TrackingProgress::Reached
    });
    let first_diverge = traces
        .iter()
        .find(|s| s.progress == TrackingProgress::Diverge);
    let first_stall = traces
        .iter()
        .find(|s| s.progress == TrackingProgress::Stall);

    if !q_reached && !saw_progress {
        if let Some(step) = first_diverge {
            return FirstDivergence::from_step(
                FirstDivergenceLayer::JointTrackingDiverged,
                "joint_error_increased",
                step,
                q_reached,
                pose_reached,
            );
        }
        let step = first_stall.unwrap_or(last);
        return FirstDivergence::from_step(
            FirstDivergenceLayer::ActuatorCommandIneffective,
            "joint_error_never_decreased",
            step,
            q_reached,
            pose_reached,
        );
    }

    if !q_reached {
        if let Some(step) = first_diverge {
            return FirstDivergence::from_step(
                FirstDivergenceLayer::JointTrackingDiverged,
                "joint_error_increased_after_progress",
                step,
                q_reached,
                pose_reached,
            );
        }
        if last.progress == TrackingProgress::Progress {
            return FirstDivergence::from_step(
                FirstDivergenceLayer::InsufficientExecutionTime,
                "error_still_decreasing_at_budget",
                last,
                q_reached,
                pose_reached,
            );
        }
        if let Some(step) = first_stall {
            return FirstDivergence::from_step(
                FirstDivergenceLayer::JointTrackingStalled,
                "joint_error_stopped_decreasing",
                step,
                q_reached,
                pose_reached,
            );
        }
        return FirstDivergence::from_step(
            FirstDivergenceLayer::JointTargetTimeout,
            "named_q_not_reached",
            last,
            q_reached,
            pose_reached,
        );
    }

    let cart = last.cartesian_residual_after;
    if !pose_reached && cart.is_finite() && cart > pose_tol {
        return FirstDivergence::from_step(
            FirstDivergenceLayer::QReachedPoseMismatch,
            "q_reached_fk_cartesian_missed",
            last,
            q_reached,
            pose_reached,
        );
    }
    if let Some(res) = last.fk_mujoco_residual {
        if res.is_finite() && res > FK_MUJOCO_RESIDUAL_MATERIAL_M {
            return FirstDivergence::from_step(
                FirstDivergenceLayer::FkModelMismatch,
                "fk_mujoco_residual_material",
                last,
                q_reached,
                pose_reached,
            );
        }
    }
    if q_reached && pose_reached {
        return FirstDivergence::from_step(
            FirstDivergenceLayer::None,
            "phase_matched_named_q_and_pose",
            last,
            q_reached,
            pose_reached,
        );
    }
    FirstDivergence::from_step(
        FirstDivergenceLayer::Unknown,
        "residuals_insufficient_to_classify",
        last,
        q_reached,
        pose_reached,
    )
}

/// First divergence when a stored witness is blocked before any actuator write.
/// `JOINT_LIMIT` is interpolation onto a bound, not a tracking stall.
pub fn classify_blocked_interpolation(
    live: &[f64],
    target: &[f64],
    names: &[String],
    joints: &[Joint],
    block_reason: &str,
    cartesian_residual: f64,
) -> FirstDivergence {
    let joint_error = named_q_max_abs_error(live, target);
    let margin = named_joint_limit_margin(target, names, joints)
        .min(named_joint_limit_margin(live, names, joints));
    let layer = if block_reason == "JOINT_LIMIT" || block_reason.contains("JOINT_LIMIT") {
        FirstDivergenceLayer::InterpolationInappropriate
    } else if block_reason == "q_len_mismatch" {
        FirstDivergenceLayer::JointNameAlignmentWrong
    } else {
        FirstDivergenceLayer::Unknown
    };
    FirstDivergence {
        layer,
        reason: format!("{block_reason};margin={margin:.6};live_vs_target={joint_error:.6}"),
        phase: "approach".into(),
        joint_error_before: joint_error,
        joint_error_after: joint_error,
        cartesian_residual_after: cartesian_residual,
        fk_mujoco_residual: None,
        q_reached: false,
        pose_reached: false,
    }
}

/// Bounded execution budget from joint displacement. Independent of embodiment identity.
pub fn named_q_execution_budget(live: &[f64], target: &[f64]) -> usize {
    let n = interpolation_count(live, target);
    n.saturating_mul(4).clamp(16, 64)
}

/// Open-loop named-q waypoints. Last sample is the stored target, not a lagging live step.
pub fn named_q_command_schedule(
    live: &[f64],
    target: &[f64],
    interpolate: bool,
) -> Result<Vec<Vec<f64>>, &'static str> {
    if !interpolate {
        if target.is_empty() || live.len() != target.len() {
            return Err("q_len_mismatch");
        }
        return Ok(vec![target.to_vec()]);
    }
    interpolate_named_q(live, target, interpolation_count(live, target))
}

/// Waypoints plus extra holds on the stored target up to the execution budget.
pub fn named_q_hold_schedule(
    live: &[f64],
    target: &[f64],
    interpolate: bool,
) -> Result<Vec<Vec<f64>>, &'static str> {
    let mut wps = named_q_command_schedule(live, target, interpolate)?;
    if wps.is_empty() {
        wps.push(target.to_vec());
    }
    let budget = named_q_execution_budget(live, target);
    while wps.len() < budget {
        wps.push(target.to_vec());
    }
    Ok(wps)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::embodiment::{unknown_se3, JointKind};
    use crate::provenance::Provenanced;

    fn joint(name: &str, lo: f64, hi: f64) -> Joint {
        Joint {
            name: name.into(),
            kind: JointKind::Hinge,
            axis: Provenanced::declared([0.0, 0.0, 1.0], "test", 0.0),
            qpos_dim: 1,
            dof_dim: 1,
            parent_body: "p".into(),
            child_body: name.into(),
            q_min: Provenanced::declared(lo, "test", 0.0),
            q_max: Provenanced::declared(hi, "test", 0.0),
            dq_max: Provenanced::unknown("test", 0.0),
            effort_max: Provenanced::unknown("test", 0.0),
            origin_in_child: Provenanced::declared([0.0, 0.0, 0.0], "test", 0.0),
            parent_to_joint: unknown_se3("test"),
            joint_to_child: unknown_se3("test"),
            qpos_adr: None,
            dof_adr: None,
        }
    }

    fn phase(q: Vec<f64>, seed: Vec<f64>) -> ManeuverPhase {
        ManeuverPhase::positional(Se3::identity(), q, seed, 1e-4, 0.1)
    }

    #[test]
    fn interpolation_count_is_between_five_and_twenty() {
        let n = interpolation_count(&[0.0, 0.0], &[0.1, 0.0]);
        assert!((5..=20).contains(&n), "n={n}");
        let wide = interpolation_count(&[0.0], &[3.0]);
        assert!(wide <= 20);
        let samples = interpolate_named_q(&[0.0, 1.0], &[1.0, 0.0], n).unwrap();
        assert_eq!(samples.len(), n);
        assert_eq!(samples[0], vec![0.0, 1.0]);
        assert_eq!(samples[n - 1], vec![1.0, 0.0]);
    }

    #[test]
    fn interpolation_resting_on_named_joint_bound_is_refused() {
        let names = vec!["arm0".into()];
        let joints = vec![joint("arm0", -2.0944, 2.0944)];
        let t = assess_named_interpolation(
            TransitionKind::CurrentToApproach,
            &names,
            &[0.0],
            &[-2.0944],
            &joints,
        );
        assert_eq!(t.verdict, TransitionVerdict::Refused);
        assert_eq!(t.reason, "JOINT_LIMIT");
        let interior = assess_named_interpolation(
            TransitionKind::CurrentToApproach,
            &names,
            &[0.0],
            &[-1.0],
            &joints,
        );
        assert_eq!(interior.verdict, TransitionVerdict::Feasible);
        assert!(interior.min_joint_margin >= MIN_NAMED_JOINT_MARGIN_FRAC);
    }

    #[test]
    fn blocked_joint_limit_names_interpolation_layer_with_live_target_residuals() {
        let names = vec!["arm0".into()];
        let joints = vec![joint("arm0", -2.0944, 2.0944)];
        let live = [0.0];
        let target = [-2.0944];
        let t = assess_named_interpolation(
            TransitionKind::CurrentToApproach,
            &names,
            &live,
            &target,
            &joints,
        );
        assert_eq!(t.reason, "JOINT_LIMIT");
        let d =
            classify_blocked_interpolation(&live, &target, &names, &joints, &t.reason, f64::NAN);
        assert_eq!(d.layer, FirstDivergenceLayer::InterpolationInappropriate);
        assert!(
            d.joint_error_before.is_finite() && d.joint_error_before > 1.0,
            "live-vs-target error must be measured, not left at 0"
        );
        assert_eq!(d.joint_error_after, d.joint_error_before);
        assert!(d.reason.contains("JOINT_LIMIT"));
        assert!(d.reason.contains("margin"));
        assert_eq!(d.phase, "approach");
        assert!(!d.q_reached);
        let unknown = classify_blocked_interpolation(
            &live,
            &target,
            &names,
            &joints,
            "NOT_EXECUTABLE",
            f64::NAN,
        );
        assert_eq!(unknown.layer, FirstDivergenceLayer::Unknown);
        assert!(unknown.joint_error_before.is_finite());
    }

    #[test]
    fn interpolation_crossing_named_joint_limit_is_refused() {
        let names = vec!["arm0".into()];
        let joints = vec![joint("arm0", -0.5, 0.5)];
        let t = assess_named_interpolation(
            TransitionKind::CurrentToApproach,
            &names,
            &[0.4],
            &[0.8],
            &joints,
        );
        assert_eq!(t.verdict, TransitionVerdict::Refused);
        assert_eq!(t.reason, "JOINT_LIMIT");
        assert!((5..=20).contains(&t.n_samples) || t.n_samples >= 5);
    }

    #[test]
    fn in_range_interpolation_is_feasible_and_name_aligned() {
        let names = vec!["arm0".into(), "arm1".into()];
        let joints = vec![
            joint("finger", 0.0, 0.04),
            joint("arm0", -2.0, 2.0),
            joint("arm1", -2.0, 2.0),
        ];
        let t = assess_named_interpolation(
            TransitionKind::CurrentToApproach,
            &names,
            &[0.5, 0.4],
            &[0.6, 0.3],
            &joints,
        );
        assert_eq!(t.verdict, TransitionVerdict::Feasible);
        assert!(t.min_joint_margin > 0.2, "must not score against finger");
    }

    #[test]
    fn position_reachable_is_not_full_pose_feasible() {
        let p = ManeuverPhase::positional(Se3::identity(), vec![0.1], vec![0.0], 1e-3, 0.2);
        assert!(p.position_feasible);
        assert!(!p.orientation_checked);
        assert!(!p.full_pose_feasible);
    }

    #[test]
    fn continuation_requires_previous_phase_q_as_seed() {
        let names = vec!["j0".into()];
        let joints = vec![joint("j0", -2.0, 2.0)];
        let approach = phase(vec![0.1], vec![0.0]);
        let contact = phase(vec![0.2], approach.q.clone());
        let mid = phase(vec![0.3], contact.q.clone());
        let end = phase(vec![0.4], mid.q.clone());
        let w = witness_from_continuing_phases(
            names.clone(),
            vec![0.0],
            approach.clone(),
            contact.clone(),
            mid,
            end,
            &joints,
        );
        assert_eq!(w.contact.seed_q, w.approach.q);
        assert_eq!(w.mid_stroke.seed_q, w.contact.q);
        assert_eq!(w.end_stroke.seed_q, w.mid_stroke.q);
        assert!(w.is_executable());

        let broken = phase(vec![0.2], vec![9.9]);
        let w2 = witness_from_continuing_phases(
            names,
            vec![0.0],
            approach,
            broken,
            phase(vec![0.3], vec![0.2]),
            phase(vec![0.4], vec![0.3]),
            &joints,
        );
        assert_eq!(w2.approach_to_contact.verdict, TransitionVerdict::Refused);
        assert_eq!(w2.approach_to_contact.reason, "IK_DISCONTINUITY");
        assert!(!w2.is_executable());
    }

    #[test]
    fn current_to_approach_infeasible_is_not_executable() {
        let names = vec!["j0".into()];
        let joints = vec![joint("j0", -0.2, 0.2)];
        let w = witness_from_continuing_phases(
            names,
            vec![0.0],
            phase(vec![0.5], vec![0.0]),
            phase(vec![0.5], vec![0.5]),
            phase(vec![0.5], vec![0.5]),
            phase(vec![0.5], vec![0.5]),
            &joints,
        );
        assert_eq!(w.current_to_approach.verdict, TransitionVerdict::Refused);
        assert!(!w.is_executable());
        assert!(w.has_approach_q());
    }

    #[test]
    fn empty_approach_q_is_not_a_witness() {
        let names = vec!["j0".into()];
        let joints = vec![joint("j0", -2.0, 2.0)];
        let w = witness_from_continuing_phases(
            names,
            vec![0.0],
            phase(vec![], vec![]),
            phase(vec![0.1], vec![]),
            phase(vec![0.1], vec![0.1]),
            phase(vec![0.1], vec![0.1]),
            &joints,
        );
        assert!(!w.has_approach_q());
        assert!(!w.is_executable());
    }

    #[test]
    fn uuid_embodiment_name_does_not_change_interpolation() {
        let names = vec!["j0".into()];
        let joints = vec![joint("j0", -1.0, 1.0)];
        let a = assess_named_interpolation(
            TransitionKind::CurrentToApproach,
            &names,
            &[0.0],
            &[0.2],
            &joints,
        );
        let _robot_a = "aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee";
        let _robot_b = "ffffffff-0000-1111-2222-333333333333";
        let b = assess_named_interpolation(
            TransitionKind::CurrentToApproach,
            &names,
            &[0.0],
            &[0.2],
            &joints,
        );
        assert_eq!(a, b);
        let _ = (_robot_a, _robot_b);
    }

    #[test]
    fn step_toward_named_q_is_bounded_and_reaches() {
        let mut q = vec![0.0, 0.0];
        let t = vec![1.0, -0.5];
        for _ in 0..8 {
            q = step_toward_named_q(&q, &t, 0.2).unwrap();
        }
        assert!(named_q_reached(&q, &t, 1e-9));
        let one = step_toward_named_q(&[0.0], &[1.0], 0.3).unwrap();
        assert!((one[0] - 0.3).abs() < 1e-12);
    }

    #[test]
    fn named_q_reached_is_length_and_tolerance_honest() {
        assert!(named_q_reached(&[0.10, -0.2], &[0.12, -0.19], 0.05));
        assert!(!named_q_reached(&[0.10], &[0.12, -0.19], 0.05));
        assert!(!named_q_reached(&[], &[0.1], 0.5));
        assert!(named_q_max_abs_error(&[0.0], &[0.2]) > 0.15);
        assert!(
            !named_q_reached(&[0.0], &[0.21], NAMED_Q_REACH_TOL_MAX),
            "max |Δq| > 0.20 must not count as reached at the current tolerance"
        );
        assert!(named_q_reached(&[0.0], &[0.20], NAMED_Q_REACH_TOL_MAX));
        assert!(NAMED_Q_REACH_TOL_MAX <= 0.20);
    }

    #[test]
    fn named_q_progress_is_not_stall_or_diverge() {
        let p = classify_named_q_progress(1.0, 0.4, NAMED_Q_REACH_TOL_MAX);
        assert_eq!(p, TrackingProgress::Progress);
        assert!(
            0.4 < 1.0,
            "progress claims require error_after < error_before"
        );
        assert_eq!(
            classify_named_q_progress(1.0, 1.0, NAMED_Q_REACH_TOL_MAX),
            TrackingProgress::Stall
        );
        assert_eq!(
            classify_named_q_progress(0.5, 0.9, NAMED_Q_REACH_TOL_MAX),
            TrackingProgress::Diverge
        );
        assert_eq!(
            classify_named_q_progress(0.5, 0.10, NAMED_Q_REACH_TOL_MAX),
            TrackingProgress::Reached
        );
        assert_eq!(
            classify_named_q_progress(f64::INFINITY, 0.1, NAMED_Q_REACH_TOL_MAX),
            TrackingProgress::LengthMismatch
        );
        assert_ne!(
            classify_named_q_progress(1.0, 0.4, NAMED_Q_REACH_TOL_MAX),
            classify_named_q_progress(1.0, 1.0, NAMED_Q_REACH_TOL_MAX)
        );
        assert_ne!(
            classify_named_q_progress(1.0, 0.4, NAMED_Q_REACH_TOL_MAX),
            classify_named_q_progress(0.5, 0.9, NAMED_Q_REACH_TOL_MAX)
        );
    }

    #[test]
    fn bounded_named_q_step_cannot_overshoot_max_delta() {
        let max_delta = 0.35;
        let live = vec![0.0, -0.2, 1.4];
        let target = vec![2.0, 2.0, -2.0];
        let cmd = step_toward_named_q(&live, &target, max_delta).unwrap();
        for i in 0..live.len() {
            assert!(
                (cmd[i] - live[i]).abs() <= max_delta + 1e-12,
                "step overshot joint {i}"
            );
        }
        assert!(command_is_bounded_step(&live, &target, &cmd, max_delta));
        let jumped = vec![2.0, 2.0, -2.0];
        assert!(!command_is_bounded_step(&live, &target, &jumped, max_delta));
        assert!(step_toward_named_q(&[0.0], &[1.0], 0.0).is_err());
    }

    #[test]
    fn continue_named_q_stops_on_stall_diverge_and_budget() {
        assert_eq!(
            continue_named_q_execution(TrackingProgress::Progress, 3, 16, true, true),
            ContinueNamedQ::Continue
        );
        assert_eq!(
            continue_named_q_execution(TrackingProgress::Reached, 3, 16, true, true),
            ContinueNamedQ::StopReached
        );
        assert_eq!(
            continue_named_q_execution(TrackingProgress::Stall, 3, 16, true, true),
            ContinueNamedQ::StopStall
        );
        assert_eq!(
            continue_named_q_execution(TrackingProgress::Diverge, 3, 16, true, true),
            ContinueNamedQ::StopDiverge
        );
        assert_eq!(
            continue_named_q_execution(TrackingProgress::Progress, 16, 16, true, true),
            ContinueNamedQ::StopBudget
        );
        assert_eq!(
            continue_named_q_execution(TrackingProgress::Progress, 3, 16, false, true),
            ContinueNamedQ::StopAuthority
        );
        assert_eq!(
            continue_named_q_execution(TrackingProgress::LengthMismatch, 1, 16, true, true),
            ContinueNamedQ::StopConstraint
        );
    }

    fn measured_step(
        phase: &str,
        live_before: Vec<f64>,
        target: Vec<f64>,
        commanded: Vec<f64>,
        live_after: Vec<f64>,
        lowering_ok: bool,
        authority: &str,
        cartesian_after: f64,
        fk_mujoco: Option<f64>,
    ) -> NamedQStepTrace {
        let joint_error_before = named_q_max_abs_error(&live_before, &target);
        let joint_error_after = named_q_max_abs_error(&live_after, &target);
        NamedQStepTrace {
            phase: phase.into(),
            joint_names: (0..target.len()).map(|i| format!("j{i}")).collect(),
            live_q_before: live_before,
            phase_target_q: target,
            commanded_q: commanded,
            interpolate: true,
            max_delta: 0.35,
            lowering_ok,
            lowering_reason: String::new(),
            actuator_names: vec!["a0".into()],
            actuator_targets: vec![0.0],
            control_mode: "position".into(),
            ctrlrange: vec![[-2.0, 2.0]],
            authority_verdict: authority.into(),
            ctrl_applied: vec![0.1],
            duration_s: 0.4,
            live_q_after: live_after.clone(),
            joint_error_before,
            joint_error_after,
            q_velocity_after: vec![0.0; live_after.len()],
            error_reduction_ratio: error_reduction_ratio(joint_error_before, joint_error_after),
            saturated: false,
            cartesian_residual_before: 0.2,
            cartesian_residual_after: cartesian_after,
            expected_fk_ee_from_got: [0.0, 0.0, 0.0],
            expected_fk_ee_from_target: [0.1, 0.0, 0.0],
            post_hoc_mujoco_ee: None,
            fk_mujoco_residual: fk_mujoco,
            progress: classify_named_q_progress(
                joint_error_before,
                joint_error_after,
                NAMED_Q_REACH_TOL_MAX,
            ),
        }
    }

    #[test]
    fn first_divergence_names_earliest_measured_layer() {
        let empty = classify_first_divergence(&[], false, false, 0.08);
        assert_eq!(empty.layer, FirstDivergenceLayer::Unknown);
        assert!(!empty.reason.is_empty());

        let lowering = measured_step(
            "approach",
            vec![0.0],
            vec![1.0],
            vec![0.35],
            vec![0.0],
            false,
            "ALLOWED",
            0.3,
            None,
        );
        let d = classify_first_divergence(&[lowering], false, false, 0.08);
        assert_eq!(d.layer, FirstDivergenceLayer::JointToActuatorLoweringWrong);

        let refused = measured_step(
            "approach",
            vec![0.0],
            vec![1.0],
            vec![0.35],
            vec![0.0],
            true,
            "REFUSED",
            0.3,
            None,
        );
        let d = classify_first_divergence(&[refused], false, false, 0.08);
        assert_eq!(d.layer, FirstDivergenceLayer::AuthorityRefusal);

        let ineffective = measured_step(
            "approach",
            vec![0.0],
            vec![1.0],
            vec![0.35],
            vec![0.0],
            true,
            "ALLOWED",
            0.3,
            None,
        );
        let d = classify_first_divergence(&[ineffective.clone(), ineffective], false, false, 0.08);
        assert_eq!(d.layer, FirstDivergenceLayer::ActuatorCommandIneffective);

        let progress_then_stall = vec![
            measured_step(
                "approach",
                vec![0.0],
                vec![1.0],
                vec![0.35],
                vec![0.35],
                true,
                "ALLOWED",
                0.25,
                None,
            ),
            measured_step(
                "approach",
                vec![0.35],
                vec![1.0],
                vec![0.70],
                vec![0.35],
                true,
                "ALLOWED",
                0.25,
                None,
            ),
        ];
        let d = classify_first_divergence(&progress_then_stall, false, false, 0.08);
        assert_eq!(d.layer, FirstDivergenceLayer::JointTrackingStalled);

        let still_moving = measured_step(
            "approach",
            vec![0.0],
            vec![1.2],
            vec![0.35],
            vec![0.35],
            true,
            "ALLOWED",
            0.2,
            None,
        );
        let d = classify_first_divergence(&[still_moving], false, false, 0.08);
        assert_eq!(d.layer, FirstDivergenceLayer::InsufficientExecutionTime);

        let q_ok_pose_bad = measured_step(
            "approach",
            vec![1.0],
            vec![1.0],
            vec![1.0],
            vec![1.0],
            true,
            "ALLOWED",
            0.25,
            Some(0.01),
        );
        let d = classify_first_divergence(&[q_ok_pose_bad], true, false, 0.08);
        assert_eq!(d.layer, FirstDivergenceLayer::QReachedPoseMismatch);

        let fk_mismatch = measured_step(
            "approach",
            vec![1.0],
            vec![1.0],
            vec![1.0],
            vec![1.0],
            true,
            "ALLOWED",
            0.01,
            Some(0.20),
        );
        let d = classify_first_divergence(&[fk_mismatch], true, true, 0.08);
        assert_eq!(d.layer, FirstDivergenceLayer::FkModelMismatch);

        let ok = measured_step(
            "approach",
            vec![1.0],
            vec![1.0],
            vec![1.0],
            vec![1.0],
            true,
            "ALLOWED",
            0.01,
            Some(0.001),
        );
        let d = classify_first_divergence(&[ok], true, true, 0.08);
        assert_eq!(d.layer, FirstDivergenceLayer::None);
    }

    #[test]
    fn lagging_live_bounded_step_never_commands_the_stored_target() {
        let live = vec![0.59, -2.32];
        let target = vec![0.005, -2.094];
        let step = step_toward_named_q(&live, &target, 0.35).unwrap();
        assert!(
            !named_q_reached(&step, &target, NAMED_Q_REACH_TOL_MAX),
            "a 0.35 bounded step from a lagging live q must not count as commanding the target"
        );
        let schedule = named_q_command_schedule(&live, &target, true).unwrap();
        assert!(
            named_q_reached(schedule.last().unwrap(), &target, 1e-12),
            "open-loop named-q schedule must end at the stored target"
        );
        assert!(named_q_execution_budget(&live, &target) >= schedule.len());
        let hold = named_q_hold_schedule(&live, &target, true).unwrap();
        assert_eq!(hold.len(), named_q_execution_budget(&live, &target));
        assert!(named_q_reached(hold.last().unwrap(), &target, 1e-12));
        let direct = named_q_command_schedule(&live, &target, false).unwrap();
        assert_eq!(direct, vec![target]);
    }

    #[test]
    fn live_q_out_of_bounds_blocks_execution_without_cartesian() {
        let names = vec!["j0".into()];
        let joints = vec![joint("j0", -0.2, 0.2)];
        let w = witness_from_continuing_phases(
            names,
            vec![0.0],
            phase(vec![0.1], vec![0.0]),
            phase(vec![0.1], vec![0.1]),
            phase(vec![0.1], vec![0.1]),
            phase(vec![0.1], vec![0.1]),
            &joints,
        );
        assert!(execution_block_reason(&w).is_none());
        let blocked = w.reassess_current_to_approach(&[0.5], &joints);
        assert_eq!(
            blocked.current_to_approach.verdict,
            TransitionVerdict::Refused
        );
        assert_eq!(blocked.current_to_approach.reason, "JOINT_LIMIT");
        assert_eq!(
            execution_block_reason(&blocked).as_deref(),
            Some("JOINT_LIMIT")
        );
    }
}
