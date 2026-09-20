//! Applicability of the first planar-push quasi-static model.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum RegimeApplicability {
    Applicable,
    ModelNotApplicable,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum AssumptionState {
    Satisfied,
    Violated,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlanarPushAssumptions {
    pub object_supported: AssumptionState,
    pub approximately_planar: AssumptionState,
    pub quasi_static: AssumptionState,
    pub single_intended_contact: AssumptionState,
    pub support_known: AssumptionState,
    pub friction_known: AssumptionState,
    pub mass_known: AssumptionState,
    pub contact_geometry_known: AssumptionState,
    pub no_significant_impact: AssumptionState,
}

impl PlanarPushAssumptions {
    pub fn applicability(&self) -> RegimeApplicability {
        let flags = [
            self.object_supported,
            self.approximately_planar,
            self.quasi_static,
            self.single_intended_contact,
            self.support_known,
            self.contact_geometry_known,
            self.no_significant_impact,
        ];
        if flags.contains(&AssumptionState::Violated) {
            return RegimeApplicability::ModelNotApplicable;
        }
        if flags.contains(&AssumptionState::Unknown) {
            return RegimeApplicability::Unknown;
        }
        RegimeApplicability::Applicable
    }
}

/// Gravity vs support-normal alignment for a supported planar object.
pub fn support_is_horizontal(gravity: [f64; 3], support_normal: [f64; 3]) -> Option<bool> {
    let gn = (gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]).sqrt();
    let nn = (support_normal[0] * support_normal[0]
        + support_normal[1] * support_normal[1]
        + support_normal[2] * support_normal[2])
        .sqrt();
    if gn < 1e-12 || nn < 1e-12 {
        return None;
    }
    let ghat = [gravity[0] / gn, gravity[1] / gn, gravity[2] / gn];
    let nhat = [
        support_normal[0] / nn,
        support_normal[1] / nn,
        support_normal[2] / nn,
    ];
    let aligned = (-ghat[0] * nhat[0] - ghat[1] * nhat[1] - ghat[2] * nhat[2]).abs();
    Some(aligned > 0.95)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn violated_assumption_is_not_applicable() {
        let a = PlanarPushAssumptions {
            object_supported: AssumptionState::Satisfied,
            approximately_planar: AssumptionState::Satisfied,
            quasi_static: AssumptionState::Violated,
            single_intended_contact: AssumptionState::Satisfied,
            support_known: AssumptionState::Satisfied,
            friction_known: AssumptionState::Satisfied,
            mass_known: AssumptionState::Satisfied,
            contact_geometry_known: AssumptionState::Satisfied,
            no_significant_impact: AssumptionState::Satisfied,
        };
        assert_eq!(a.applicability(), RegimeApplicability::ModelNotApplicable);
    }

    #[test]
    fn earth_g_and_up_normal_are_horizontal_support() {
        assert_eq!(
            support_is_horizontal([0.0, 0.0, -9.81], [0.0, 0.0, 1.0]),
            Some(true)
        );
        assert_eq!(
            support_is_horizontal([0.0, 0.0, -9.81], [1.0, 0.0, 0.0]),
            Some(false)
        );
    }
}
