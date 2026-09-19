//! Command domain vs joint-state domain. Physically different quantities
//! are not interchangeable. No robot identity.

use crate::embodiment::{Actuator, Joint};
use crate::skill::SkillRefuse;

/// Authorize a value in the actuator *command* domain (ctrlrange).
///
/// Does not consult joint `q` bounds. A command of 255 inside a declared
/// `[0, 255]` ctrlrange is allowed even when the target joint lives in
/// `[0, 0.04]`.
pub fn authorize_actuator_command(act: &Actuator, command: f64) -> Result<(), SkillRefuse> {
    if !command.is_finite() {
        return Err(SkillRefuse::InvalidCommand);
    }
    let Some([lo, hi]) = act.ctrlrange.value else {
        return Ok(());
    };
    let (lo, hi) = (lo.min(hi), lo.max(hi));
    if command < lo - 1e-12 || command > hi + 1e-12 {
        return Err(SkillRefuse::InvalidCommand);
    }
    Ok(())
}

/// Joint *state* vs named joint limits. Independent of actuator commands.
pub fn joint_position_in_bounds(joint: &Joint, q: f64) -> bool {
    if !q.is_finite() {
        return false;
    }
    if let Some(lo) = joint.q_min.value {
        if q < lo - 1e-12 {
            return false;
        }
    }
    if let Some(hi) = joint.q_max.value {
        if q > hi + 1e-12 {
            return false;
        }
    }
    true
}

pub fn constrain_joint_state(joint: &Joint, q: f64) -> Result<(), SkillRefuse> {
    if joint_position_in_bounds(joint, q) {
        Ok(())
    } else {
        Err(SkillRefuse::InvalidCommand)
    }
}

fn affine_ctrl_to_joint(act: &Actuator, joint: &Joint, command: f64) -> Option<f64> {
    let [clo, chi] = act.ctrlrange.value?;
    let span = chi - clo;
    if !span.is_finite() || span.abs() < 1e-12 {
        return None;
    }
    let lo = joint.q_min.value?;
    let hi = joint.q_max.value?;
    let t = (command - clo) / span;
    Some(lo + t * (hi - lo))
}

/// Predicted joint position from an actuator command via the declared
/// transmission. Used to constrain resulting *state*, never to authorize
/// the command itself.
pub fn predicted_joint_position(act: &Actuator, joint: &Joint, command: f64) -> Option<f64> {
    if !command.is_finite() {
        return None;
    }
    let kind = act.transmission_kind.as_str();
    if kind == "tendon" || kind == "coupled" {
        return affine_ctrl_to_joint(act, joint, command);
    }
    if let Some(g) = act.gear.value {
        if g.is_finite() && g.abs() > 1e-12 && (g - 1.0).abs() > 1e-9 {
            return Some(command / g);
        }
    }
    if let (Some(cr), Some(lo), Some(hi)) =
        (act.ctrlrange.value, joint.q_min.value, joint.q_max.value)
    {
        let same = (cr[0] - lo).abs() < 1e-9 && (cr[1] - hi).abs() < 1e-9;
        if same {
            return Some(command);
        }
        return affine_ctrl_to_joint(act, joint, command);
    }
    Some(command)
}

/// Named-joint margin in [0, 1]. Samples are matched by joint *name*,
/// never by index coincidence with a different joint list.
pub fn named_joint_limit_margin(q: &[f64], names: &[String], joints: &[Joint]) -> f64 {
    if q.is_empty() {
        return 1.0;
    }
    let mut worst = 1.0_f64;
    let n = q.len().min(names.len());
    for i in 0..n {
        let Some(j) = joints.iter().find(|j| j.name == names[i]) else {
            continue;
        };
        let (Some(lo), Some(hi)) = (j.q_min.value, j.q_max.value) else {
            continue;
        };
        let span = (hi - lo).abs().max(1e-6);
        let d = (q[i] - lo).min(hi - q[i]).max(0.0);
        worst = worst.min(d / span);
    }
    worst.clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::embodiment::{unknown_se3, JointKind};
    use crate::provenance::Provenanced;

    fn joint(name: &str, lo: f64, hi: f64) -> Joint {
        Joint {
            name: name.into(),
            kind: JointKind::Slide,
            axis: Provenanced::declared([1.0, 0.0, 0.0], "test", 0.0),
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

    fn act(name: &str, target: &str, kind: &str, cr: [f64; 2], gear: Option<f64>) -> Actuator {
        Actuator {
            name: name.into(),
            target_joint: target.into(),
            control_mode: "position".into(),
            transmission_kind: kind.into(),
            ctrlrange: Provenanced::declared(cr, "test", 0.0),
            forcerange: Provenanced::unknown("test", 0.0),
            gear: match gear {
                Some(g) => Provenanced::declared(g, "test", 0.0),
                None => Provenanced::unknown("test", 0.0),
            },
        }
    }

    #[test]
    fn direct_drive_command_uses_ctrlrange_not_a_different_joint() {
        let a = act("a0", "j0", "joint", [0.0, 1.57], None);
        let j = joint("j0", 0.0, 1.57);
        assert!(authorize_actuator_command(&a, 0.8).is_ok());
        assert!(authorize_actuator_command(&a, 2.0).is_err());
        assert!(constrain_joint_state(&j, 0.8).is_ok());
        assert!(constrain_joint_state(&j, 2.0).is_err());
        let q = predicted_joint_position(&a, &j, 0.8).unwrap();
        assert!((q - 0.8).abs() < 1e-12);
    }

    #[test]
    fn gear_transmission_command_is_not_the_joint_coordinate() {
        let a = act("a_g", "j_g", "gear", [0.0, 2.0], Some(50.0));
        let j = joint("j_g", 0.0, 0.04);
        assert!(authorize_actuator_command(&a, 2.0).is_ok());
        assert!(authorize_actuator_command(&a, 2.1).is_err());
        let q = predicted_joint_position(&a, &j, 2.0).unwrap();
        assert!((q - 0.04).abs() < 1e-12);
        assert!(constrain_joint_state(&j, q).is_ok());
        assert!(
            constrain_joint_state(&j, 2.0).is_err(),
            "command value is not joint state"
        );
    }

    #[test]
    fn tendon_ctrlrange_not_joint_range_allows_full_scale_command() {
        // Generic ctrlrange != joint range (the 255 vs 0.04 class).
        let a = act("split", "finger_a", "tendon", [0.0, 255.0], None);
        let j = joint("finger_a", 0.0, 0.04);
        assert!(
            authorize_actuator_command(&a, 255.0).is_ok(),
            "valid command must not be refused by joint q bounds"
        );
        assert!(authorize_actuator_command(&a, 256.0).is_err());
        assert!(authorize_actuator_command(&a, -1.0).is_err());
        assert!(
            constrain_joint_state(&j, 255.0).is_err(),
            "resulting joint state remains constrainable"
        );
        let q = predicted_joint_position(&a, &j, 255.0).unwrap();
        assert!((q - 0.04).abs() < 1e-9);
        assert!(constrain_joint_state(&j, q).is_ok());
        assert!(constrain_joint_state(&j, 0.0).is_ok());
    }

    #[test]
    fn coupled_fingers_share_command_domain_and_per_joint_state() {
        let a = act("split", "finger_l", "coupled", [0.0, 255.0], None);
        let jl = joint("finger_l", 0.0, 0.04);
        let jr = joint("finger_r", 0.0, 0.04);
        assert!(authorize_actuator_command(&a, 127.5).is_ok());
        let ql = predicted_joint_position(&a, &jl, 127.5).unwrap();
        let qr = predicted_joint_position(&a, &jr, 127.5).unwrap();
        assert!((ql - 0.02).abs() < 1e-9);
        assert!((qr - 0.02).abs() < 1e-9);
        assert!(constrain_joint_state(&jl, ql).is_ok());
        assert!(constrain_joint_state(&jr, qr).is_ok());
    }

    #[test]
    fn named_margin_ignores_index_coincidence_with_other_joints() {
        let finger = joint("finger_a", 0.0, 0.04);
        let arm0 = joint("arm0", -2.0, 2.0);
        let arm1 = joint("arm1", -2.0, 2.0);
        let joints = [finger, arm0, arm1];
        let q = vec![0.5, 0.4];
        let names = vec!["arm0".into(), "arm1".into()];
        let named = named_joint_limit_margin(&q, &names, &joints);
        assert!(
            named > 0.2,
            "name-aligned arm q must not use finger [0,0.04], margin={named}"
        );
        let index_like = named_joint_limit_margin(&q, &["finger_a".into(), "arm0".into()], &joints);
        assert!(
            index_like < 1e-9,
            "q=0.5 is outside finger [0,0.04] when names actually match the finger"
        );
    }

    #[test]
    fn non_finite_command_is_refused() {
        let a = act("a0", "j0", "joint", [0.0, 1.0], None);
        assert!(authorize_actuator_command(&a, f64::NAN).is_err());
        assert!(authorize_actuator_command(&a, f64::INFINITY).is_err());
    }
}
