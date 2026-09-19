//! Reachable workspace sampling from joint limits and FK. No robot identity.

use crate::embodiment::{EmbodimentModel, Joint, JointKind};
use crate::kinematics::forward_kinematics;

/// Joint sample from a unit draw in [0, 1]. Unknown limits use a bounded
/// estimated envelope (hinge ±π, slide ±5 cm), not a calibrated value.
pub fn joint_q_from_unit(joint: &Joint, u: f64) -> f64 {
    let u = u.clamp(0.0, 1.0);
    let (lo, hi) = match (joint.q_min.value, joint.q_max.value) {
        (Some(a), Some(b)) => (a.min(b), a.max(b)),
        _ => match joint.kind {
            JointKind::Hinge => (-std::f64::consts::PI, std::f64::consts::PI),
            JointKind::Slide => (-0.05, 0.05),
            _ => (0.0, 0.0),
        },
    };
    lo + u * (hi - lo)
}

/// FK cloud of EE xyz from unit draws. `units` is row-major: `n_draw * chain.len()`.
pub fn reachable_ee_xyz(model: &EmbodimentModel, ee: &str, units: &[f64]) -> Vec<[f64; 3]> {
    let Some(chain) = model.ee_joint_chain(ee) else {
        return Vec::new();
    };
    let n = chain.len();
    if n == 0 {
        return Vec::new();
    }
    let mut out = Vec::new();
    for chunk in units.chunks(n) {
        if chunk.len() != n {
            break;
        }
        let q: Vec<f64> = chain
            .iter()
            .zip(chunk.iter())
            .filter_map(|(name, u)| {
                model
                    .joints
                    .iter()
                    .find(|j| j.name == *name)
                    .map(|j| joint_q_from_unit(j, *u))
            })
            .collect();
        if q.len() != n {
            continue;
        }
        if let Ok(fk) = forward_kinematics(model, &chain, ee, &q) {
            if fk.ee.xyz.iter().all(|v| v.is_finite()) {
                out.push(fk.ee.xyz);
            }
        }
    }
    out
}

/// Object center so the near face sits at `contact_ee` along `push_dir`.
pub fn push_object_xyz(
    contact_ee: [f64; 3],
    push_dir: [f64; 3],
    object_half: f64,
    face_gap: f64,
) -> [f64; 3] {
    let n = (push_dir[0] * push_dir[0] + push_dir[1] * push_dir[1] + push_dir[2] * push_dir[2])
        .sqrt()
        .max(1e-9);
    let d = [push_dir[0] / n, push_dir[1] / n, push_dir[2] / n];
    let along = object_half.max(0.0) + face_gap.max(0.0);
    [
        contact_ee[0] + d[0] * along,
        contact_ee[1] + d[1] * along,
        contact_ee[2] + d[2] * along,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::synth_planar_two_link;

    #[test]
    fn reachable_cloud_is_finite_and_not_a_single_point() {
        let m = synth_planar_two_link();
        let mut units = Vec::new();
        for i in 0..12u32 {
            let u0 = f64::from(i) / 11.0;
            let u1 = f64::from((i * 3) % 12) / 11.0;
            units.push(u0);
            units.push(u1);
        }
        let cloud = reachable_ee_xyz(&m, "ee", &units);
        assert!(cloud.len() >= 8, "got {} samples", cloud.len());
        let mut xmin = f64::INFINITY;
        let mut xmax = f64::NEG_INFINITY;
        for p in &cloud {
            assert!(p.iter().all(|v| v.is_finite()));
            xmin = xmin.min(p[0]);
            xmax = xmax.max(p[0]);
        }
        assert!(
            xmax - xmin > 0.02,
            "workspace samples must move in x, span={}",
            xmax - xmin
        );
    }

    #[test]
    fn object_center_sits_along_push_direction_from_contact() {
        let contact = [0.2, 0.0, 0.1];
        let obj = push_object_xyz(contact, [1.0, 0.0, 0.0], 0.025, 0.01);
        assert!((obj[0] - 0.235).abs() < 1e-12);
        assert!((obj[1] - 0.0).abs() < 1e-12);
        assert!((obj[2] - 0.1).abs() < 1e-12);
    }
}
