//! Configuration-space witness for an executable PUSH contact maneuver.
//!
//! Named joints only. No robot identity.

use crate::command_domain::joint_position_in_bounds;
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
            min_margin = min_margin.min(d / span);
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
