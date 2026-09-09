//! Capability identifiers. Behaviors ask for these, never a robot name.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Capability {
    FloatingBase,
    DifferentialDrive,
    SerialArm,
    SingleDof,
    Bimanual,
    CartesianImpedance,
    ForceTorqueSensing,
    SafeTorqueOff,
}

impl Capability {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::FloatingBase => "floating_base",
            Self::DifferentialDrive => "differential_drive",
            Self::SerialArm => "serial_arm",
            Self::SingleDof => "single_dof",
            Self::Bimanual => "bimanual",
            Self::CartesianImpedance => "cartesian_impedance",
            Self::ForceTorqueSensing => "force_torque_sensing",
            Self::SafeTorqueOff => "safe_torque_off",
        }
    }

    pub fn parse(raw: &str) -> Option<Self> {
        match raw.trim() {
            "floating_base" => Some(Self::FloatingBase),
            "differential_drive" => Some(Self::DifferentialDrive),
            "serial_arm" => Some(Self::SerialArm),
            "single_dof" => Some(Self::SingleDof),
            "bimanual" => Some(Self::Bimanual),
            "cartesian_impedance" => Some(Self::CartesianImpedance),
            "force_torque_sensing" => Some(Self::ForceTorqueSensing),
            "safe_torque_off" => Some(Self::SafeTorqueOff),
            _ => None,
        }
    }
}

/// Morphology class from model data — not a robot product name.
pub fn capabilities_for_kind(kind: &str) -> Vec<Capability> {
    match kind.trim() {
        "biped" | "quadruped" | "humanoid" => vec![Capability::FloatingBase],
        "differential_drive" | "ackermann" => vec![Capability::DifferentialDrive],
        "serial_arm" => vec![Capability::SerialArm, Capability::CartesianImpedance],
        "uniaxial" | "single_dof" => vec![Capability::SingleDof],
        "bimanual" => vec![Capability::Bimanual, Capability::SerialArm],
        "aerial" => vec![Capability::FloatingBase],
        _ => Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kind_is_not_a_robot_name() {
        assert!(capabilities_for_kind("unitree_h1").is_empty());
        assert!(capabilities_for_kind("biped").contains(&Capability::FloatingBase));
        assert!(capabilities_for_kind("serial_arm").contains(&Capability::SerialArm));
    }
}
