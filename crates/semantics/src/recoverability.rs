//! Conservative recoverability from bounded local consequences.
//! Exact future poses are not required.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum RecoverabilityClass {
    ProgressAndRecoverable,
    ProgressButRecoverabilityUnknown,
    ProgressButCanEnterUnrecoverableState,
    NoProgress,
    PhysicallyInfeasible,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct InteractionRegion {
    pub center_xy: [f64; 2],
    pub radius_m: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RecoverabilityInput {
    pub physically_feasible: bool,
    pub makes_progress: bool,
    pub current_xy: [f64; 2],
    /// Nominal object translation under the candidate. `None` means the bound
    /// is not known.
    pub nominal_dxy: Option<[f64; 2]>,
    pub uncertainty_radius_m: Option<f64>,
    pub region: Option<InteractionRegion>,
    /// Whether a next contact is admissible if the worst-case pose stays inside
    /// `region`. `None` means that predicate is unknown.
    pub next_contact_admissible: Option<bool>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RecoverabilityChoice {
    pub id: String,
    pub class: RecoverabilityClass,
    /// Higher is better immediate progress.
    pub progress: f64,
}

fn hypot2(v: [f64; 2]) -> f64 {
    (v[0] * v[0] + v[1] * v[1]).sqrt()
}

pub fn classify_recoverability(input: &RecoverabilityInput) -> RecoverabilityClass {
    if !input.physically_feasible {
        return RecoverabilityClass::PhysicallyInfeasible;
    }
    if !input.makes_progress {
        return RecoverabilityClass::NoProgress;
    }
    let (Some(dxy), Some(uncertainty), Some(region)) =
        (input.nominal_dxy, input.uncertainty_radius_m, input.region)
    else {
        return RecoverabilityClass::ProgressButRecoverabilityUnknown;
    };
    if !uncertainty.is_finite() || uncertainty < 0.0 || !region.radius_m.is_finite() {
        return RecoverabilityClass::ProgressButRecoverabilityUnknown;
    }
    let landed = [input.current_xy[0] + dxy[0], input.current_xy[1] + dxy[1]];
    let worst = hypot2([
        landed[0] - region.center_xy[0],
        landed[1] - region.center_xy[1],
    ]) + uncertainty;
    if worst > region.radius_m || input.next_contact_admissible == Some(false) {
        return RecoverabilityClass::ProgressButCanEnterUnrecoverableState;
    }
    if input.next_contact_admissible != Some(true) {
        return RecoverabilityClass::ProgressButRecoverabilityUnknown;
    }
    RecoverabilityClass::ProgressAndRecoverable
}

/// Prefer supported progress that stays recoverable over a larger immediate
/// step that can leave the interaction region.
pub fn select_recoverable_progress(choices: &[RecoverabilityChoice]) -> Option<usize> {
    let mut best: Option<(usize, f64)> = None;
    for (index, choice) in choices.iter().enumerate() {
        if choice.class != RecoverabilityClass::ProgressAndRecoverable {
            continue;
        }
        let better = best
            .map(|(_, progress)| choice.progress > progress)
            .unwrap_or(true);
        if better {
            best = Some((index, choice.progress));
        }
    }
    best.map(|(index, _)| index)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn region() -> InteractionRegion {
        InteractionRegion {
            center_xy: [0.0, 0.0],
            radius_m: 0.1,
        }
    }

    #[test]
    fn five_classes_are_distinct_and_come_from_their_own_inputs() {
        let recoverable = classify_recoverability(&RecoverabilityInput {
            physically_feasible: true,
            makes_progress: true,
            current_xy: [0.0, 0.0],
            nominal_dxy: Some([0.02, 0.0]),
            uncertainty_radius_m: Some(0.005),
            region: Some(region()),
            next_contact_admissible: Some(true),
        });
        let unknown = classify_recoverability(&RecoverabilityInput {
            physically_feasible: true,
            makes_progress: true,
            current_xy: [0.0, 0.0],
            nominal_dxy: None,
            uncertainty_radius_m: None,
            region: None,
            next_contact_admissible: None,
        });
        let can_leave = classify_recoverability(&RecoverabilityInput {
            physically_feasible: true,
            makes_progress: true,
            current_xy: [0.09, 0.0],
            nominal_dxy: Some([0.03, 0.0]),
            uncertainty_radius_m: Some(0.01),
            region: Some(region()),
            next_contact_admissible: Some(true),
        });
        let no_progress = classify_recoverability(&RecoverabilityInput {
            physically_feasible: true,
            makes_progress: false,
            current_xy: [0.0, 0.0],
            nominal_dxy: Some([0.0, 0.0]),
            uncertainty_radius_m: Some(0.0),
            region: Some(region()),
            next_contact_admissible: Some(true),
        });
        let infeasible = classify_recoverability(&RecoverabilityInput {
            physically_feasible: false,
            makes_progress: true,
            current_xy: [0.0, 0.0],
            nominal_dxy: Some([0.02, 0.0]),
            uncertainty_radius_m: Some(0.0),
            region: Some(region()),
            next_contact_admissible: Some(true),
        });
        let labels = [recoverable, unknown, can_leave, no_progress, infeasible];
        assert_eq!(
            labels,
            [
                RecoverabilityClass::ProgressAndRecoverable,
                RecoverabilityClass::ProgressButRecoverabilityUnknown,
                RecoverabilityClass::ProgressButCanEnterUnrecoverableState,
                RecoverabilityClass::NoProgress,
                RecoverabilityClass::PhysicallyInfeasible,
            ]
        );
        for (i, a) in labels.iter().enumerate() {
            for b in labels.iter().skip(i + 1) {
                assert_ne!(a, b);
            }
        }
    }

    #[test]
    fn recoverable_progress_beats_a_larger_step_that_can_leave_the_region() {
        let choices = vec![
            RecoverabilityChoice {
                id: "far".into(),
                class: RecoverabilityClass::ProgressButCanEnterUnrecoverableState,
                progress: 0.9,
            },
            RecoverabilityChoice {
                id: "near".into(),
                class: RecoverabilityClass::ProgressAndRecoverable,
                progress: 0.2,
            },
        ];
        let index = select_recoverable_progress(&choices).unwrap();
        assert_eq!(choices[index].id, "near");
        assert!(choices[0].progress > choices[index].progress);
    }
}
