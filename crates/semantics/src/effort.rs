//! Physical joint effort from declared forcerange / effort / proven transmission.
//!
//! ctrlrange, PWM, and tendon command coordinates are never FORCE_RANGE or
//! JOINT_TORQUE_LIMIT.

use crate::embodiment::{Actuator, EmbodimentModel, Joint};
use crate::physical_quantity::{EffortDomain, PhysicalEffort, PhysicalFactProvenance};
use crate::provenance::Provenance;

fn range_abs(lo: f64, hi: f64) -> Option<f64> {
    if !lo.is_finite() || !hi.is_finite() {
        return None;
    }
    Some(lo.abs().max(hi.abs()))
}

/// Physical effort at one joint. Command-domain numbers are rejected.
pub fn physical_joint_effort(joint: &Joint, actuator: Option<&Actuator>) -> PhysicalEffort {
    if let Some(&tau) = joint.effort_max.known_value() {
        if tau.is_finite() && tau >= 0.0 {
            return PhysicalEffort::JointTorque {
                tau_abs_nm: tau,
                domain: EffortDomain::JointTorqueLimit,
                provenance: PhysicalFactProvenance::from(joint.effort_max.provenance),
                source: joint.effort_max.source.clone(),
            };
        }
    }
    let Some(act) = actuator else {
        return PhysicalEffort::Unknown {
            reason: "NO_DECLARED_JOINT_EFFORT_OR_ACTUATOR".into(),
        };
    };
    let kind = act.transmission_kind.as_str();
    if kind == "tendon" || kind == "coupled" {
        return PhysicalEffort::Unknown {
            reason: "TENDON_COMMAND_IS_NOT_JOINT_TORQUE".into(),
        };
    }
    let mode = act.control_mode.to_ascii_lowercase();
    if mode == "pwm" {
        return PhysicalEffort::Unknown {
            reason: "PWM_IS_NOT_TORQUE".into(),
        };
    }
    let Some(&[lo, hi]) = act.forcerange.known_value() else {
        if act.ctrlrange.known_value().is_some() {
            return PhysicalEffort::Unknown {
                reason: "CTRLRANGE_IS_NOT_FORCE_RANGE".into(),
            };
        }
        return PhysicalEffort::Unknown {
            reason: "NO_DECLARED_PHYSICAL_EFFORT".into(),
        };
    };
    let Some(force_abs) = range_abs(lo, hi) else {
        return PhysicalEffort::Unknown {
            reason: "NON_FINITE_FORCERANGE".into(),
        };
    };
    let Some(&gear) = act.gear.known_value() else {
        return PhysicalEffort::Unknown {
            reason: "TRANSMISSION_GEAR_UNKNOWN".into(),
        };
    };
    if !gear.is_finite() || gear.abs() < 1e-12 {
        return PhysicalEffort::Unknown {
            reason: "TRANSMISSION_GEAR_UNKNOWN".into(),
        };
    }
    PhysicalEffort::JointTorque {
        tau_abs_nm: force_abs * gear.abs(),
        domain: EffortDomain::ForceRange,
        provenance: PhysicalFactProvenance::from(act.forcerange.provenance),
        source: act.forcerange.source.clone(),
    }
}

pub fn chain_physical_effort(
    model: &EmbodimentModel,
    joint_names: &[String],
) -> Result<Vec<f64>, PhysicalEffort> {
    let mut out = Vec::with_capacity(joint_names.len());
    for name in joint_names {
        let Some(joint) = model.joints.iter().find(|j| j.name == *name) else {
            return Err(PhysicalEffort::Unknown {
                reason: format!("JOINT_NOT_IN_MODEL:{name}"),
            });
        };
        let act = model.actuator_for_joint(name);
        match physical_joint_effort(joint, act) {
            PhysicalEffort::JointTorque { tau_abs_nm, .. } => out.push(tau_abs_nm),
            unknown => return Err(unknown),
        }
    }
    Ok(out)
}

pub fn any_link_com_known(model: &EmbodimentModel) -> bool {
    model.bodies.iter().any(|b| {
        b.com.known_value().is_some()
            && b.mass_kg.known_value().is_some()
            && b.com.provenance != Provenance::Unknown
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::synth_planar_two_link;
    use crate::embodiment::Actuator;
    use crate::provenance::Provenanced;

    fn position_act(name: &str, joint: &str, cr: [f64; 2]) -> Actuator {
        Actuator {
            name: name.into(),
            target_joint: joint.into(),
            control_mode: "position".into(),
            transmission_kind: "joint".into(),
            ctrlrange: Provenanced::declared(cr, "test.ctrlrange", 0.0),
            forcerange: Provenanced::unknown("test", 0.0),
            gear: Provenanced::unknown("test", 0.0),
        }
    }

    #[test]
    fn position_ctrlrange_is_not_joint_torque() {
        let mut m = synth_planar_two_link();
        m.actuators[0] = position_act("a0", "j0", [-1.0, 1.0]);
        let e = physical_joint_effort(&m.joints[0], Some(&m.actuators[0]));
        assert!(e.is_unknown(), "{e:?}");
        match e {
            PhysicalEffort::Unknown { ref reason } => {
                assert_eq!(reason, "CTRLRANGE_IS_NOT_FORCE_RANGE");
            }
            other => panic!("{other:?}"),
        }
        assert!(e.tau_abs_nm().is_none());
    }

    #[test]
    fn tendon_command_is_not_force() {
        let mut m = synth_planar_two_link();
        m.actuators[0] = Actuator {
            name: "tendon0".into(),
            target_joint: "j0".into(),
            control_mode: "position".into(),
            transmission_kind: "tendon".into(),
            ctrlrange: Provenanced::declared([0.0, 255.0], "test.tendon", 0.0),
            forcerange: Provenanced::declared([0.0, 100.0], "test.tendon_force", 0.0),
            gear: Provenanced::declared(1.0, "test", 0.0),
        };
        let e = physical_joint_effort(&m.joints[0], Some(&m.actuators[0]));
        match e {
            PhysicalEffort::Unknown { reason } => {
                assert_eq!(reason, "TENDON_COMMAND_IS_NOT_JOINT_TORQUE");
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn pwm_is_not_torque() {
        let mut m = synth_planar_two_link();
        m.actuators[0].control_mode = "pwm".into();
        m.actuators[0].ctrlrange = Provenanced::declared([0.0, 1.0], "pwm", 0.0);
        let e = physical_joint_effort(&m.joints[0], Some(&m.actuators[0]));
        match e {
            PhysicalEffort::Unknown { reason } => assert_eq!(reason, "PWM_IS_NOT_TORQUE"),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn declared_joint_effort_is_used() {
        let mut m = synth_planar_two_link();
        m.joints[0].effort_max = Provenanced::declared(1.5, "test.effort", 0.0);
        let e = physical_joint_effort(&m.joints[0], Some(&m.actuators[0]));
        assert_eq!(e.tau_abs_nm(), Some(1.5));
    }

    #[test]
    fn forcerange_without_gear_stays_unknown() {
        let mut m = synth_planar_two_link();
        m.actuators[0].forcerange = Provenanced::declared([-4.0, 4.0], "test.fr", 0.0);
        m.actuators[0].gear = Provenanced::unknown("test", 0.0);
        let e = physical_joint_effort(&m.joints[0], Some(&m.actuators[0]));
        match e {
            PhysicalEffort::Unknown { reason } => {
                assert_eq!(reason, "TRANSMISSION_GEAR_UNKNOWN");
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn forcerange_times_declared_gear_is_joint_torque() {
        let mut m = synth_planar_two_link();
        m.actuators[0].forcerange = Provenanced::declared([-2.0, 2.0], "test.fr", 0.0);
        m.actuators[0].gear = Provenanced::declared(3.0, "test.gear", 0.0);
        let e = physical_joint_effort(&m.joints[0], Some(&m.actuators[0]));
        assert_eq!(e.tau_abs_nm(), Some(6.0));
        match e {
            PhysicalEffort::JointTorque { domain, .. } => {
                assert_eq!(domain, EffortDomain::ForceRange);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn unknown_effort_on_a_chain_refuses_the_bound() {
        let m = synth_planar_two_link();
        let err = chain_physical_effort(&m, &["j0".into(), "j1".into()]).unwrap_err();
        assert!(err.is_unknown());
    }
}
