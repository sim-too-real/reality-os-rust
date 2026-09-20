//! Contact-point Jacobian on the shipped FK. Finite-difference residual is evidence,
//! not a second kinematics stack.

use realityos_physics::{translational_jacobian_column, JointMotionKind};

use crate::embodiment::{EmbodimentModel, JointKind};
use crate::kinematics::{forward_kinematics, jacobian_translational_at, FkState};
use crate::skill::SkillRefuse;
use crate::transform::{norm3, sub3};

#[derive(Debug, Clone, PartialEq)]
pub struct ContactJacobianWitness {
    pub joint_names: Vec<String>,
    pub contact_point_world: [f64; 3],
    pub analytic_3xn: Vec<Vec<f64>>,
    pub finite_difference_3xn: Vec<Vec<f64>>,
    pub residual: f64,
}

pub fn jacobian_column_joint_names(model: &EmbodimentModel, chain: &[String]) -> Vec<String> {
    chain
        .iter()
        .filter_map(|name| {
            let j = model.joints.iter().find(|j| j.name == *name)?;
            match j.kind {
                JointKind::Fixed => None,
                _ => Some(name.clone()),
            }
        })
        .collect()
}

fn motion_kind(kind: JointKind) -> Option<JointMotionKind> {
    match kind {
        JointKind::Hinge => Some(JointMotionKind::Revolute),
        JointKind::Slide => Some(JointMotionKind::Prismatic),
        JointKind::Fixed => Some(JointMotionKind::Fixed),
        JointKind::Ball | JointKind::Free | JointKind::Other => None,
    }
}

/// Analytic translational Jacobian at a world contact, using the same hinge/slide
/// columns as `jacobian_translational` (offset `r = contact − origin`).
pub fn analytic_contact_jacobian(fk: &FkState, contact_world: [f64; 3]) -> Vec<Vec<f64>> {
    jacobian_translational_at(fk, contact_world)
}

fn contact_world_at(
    model: &EmbodimentModel,
    chain: &[String],
    ee: &str,
    q: &[f64],
    contact_in_ee: [f64; 3],
) -> Result<[f64; 3], SkillRefuse> {
    let fk = forward_kinematics(model, chain, ee, q)?;
    Ok(fk.ee.transform_point(contact_in_ee))
}

/// Central-difference residual of the shipped contact Jacobian vs shipped FK.
pub fn contact_jacobian_witness(
    model: &EmbodimentModel,
    chain: &[String],
    ee: &str,
    q: &[f64],
    contact_in_ee: [f64; 3],
    eps: f64,
) -> Result<ContactJacobianWitness, SkillRefuse> {
    if !eps.is_finite() || eps <= 0.0 {
        return Err(SkillRefuse::Unsupported);
    }
    let fk = forward_kinematics(model, chain, ee, q)?;
    let contact = fk.ee.transform_point(contact_in_ee);
    let analytic = analytic_contact_jacobian(&fk, contact);
    let names = jacobian_column_joint_names(model, chain);
    let n = names.len();
    if analytic[0].len() != n {
        return Err(SkillRefuse::Unsupported);
    }
    let mut fd = vec![vec![0.0; n], vec![0.0; n], vec![0.0; n]];
    let mut residual = 0.0_f64;
    for (col, name) in names.iter().enumerate() {
        let Some(idx) = chain.iter().position(|n| n == name) else {
            return Err(SkillRefuse::Unsupported);
        };
        let mut qp = q.to_vec();
        let mut qm = q.to_vec();
        qp[idx] += eps;
        qm[idx] -= eps;
        let pp = contact_world_at(model, chain, ee, &qp, contact_in_ee)?;
        let pm = contact_world_at(model, chain, ee, &qm, contact_in_ee)?;
        let col_fd = [
            (pp[0] - pm[0]) / (2.0 * eps),
            (pp[1] - pm[1]) / (2.0 * eps),
            (pp[2] - pm[2]) / (2.0 * eps),
        ];
        fd[0][col] = col_fd[0];
        fd[1][col] = col_fd[1];
        fd[2][col] = col_fd[2];
        for row in 0..3 {
            residual = residual.max((analytic[row][col] - col_fd[row]).abs());
        }
        if let Some(kind) = model
            .joints
            .iter()
            .find(|j| j.name == *name)
            .and_then(|j| motion_kind(j.kind))
        {
            let physics_col = translational_jacobian_column(
                kind,
                fk.axes_world[col],
                fk.joint_origins[col],
                contact,
            )
            .map_err(|_| SkillRefuse::Unreachable)?;
            let shipped = [analytic[0][col], analytic[1][col], analytic[2][col]];
            if norm3(sub3(physics_col, shipped)) > 1e-9 {
                return Err(SkillRefuse::Unsupported);
            }
        }
    }
    Ok(ContactJacobianWitness {
        joint_names: names,
        contact_point_world: contact,
        analytic_3xn: analytic,
        finite_difference_3xn: fd,
        residual,
    })
}

pub fn jacobian_3xn_columns(j: &[Vec<f64>]) -> Vec<[f64; 3]> {
    let n = j.first().map(|r| r.len()).unwrap_or(0);
    (0..n)
        .map(|i| {
            [
                j.first().and_then(|r| r.get(i)).copied().unwrap_or(0.0),
                j.get(1).and_then(|r| r.get(i)).copied().unwrap_or(0.0),
                j.get(2).and_then(|r| r.get(i)).copied().unwrap_or(0.0),
            ]
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::synth_planar_two_link;
    use crate::embodiment::{unknown_se3, Body, Joint, JointKind};
    use crate::provenance::Provenanced;
    use crate::transform::Se3;

    fn three_link() -> crate::embodiment::EmbodimentModel {
        let mut m = synth_planar_two_link();
        const L2: f64 = 0.15;
        const L3: f64 = 0.12;
        m.bodies.push(Body {
            name: "link3".into(),
            parent: Some("link2".into()),
            mass_kg: Provenanced::unknown("test", 0.0),
            com: Provenanced::unknown("test", 0.0),
            inertia: Provenanced::unknown("test", 0.0),
            local_pose: Provenanced::declared(
                Se3::translation([L2, 0.0, 0.0]).unwrap(),
                "test",
                0.0,
            ),
        });
        m.joints.push(Joint {
            name: "j2".into(),
            kind: JointKind::Hinge,
            axis: Provenanced::declared([0.0, 0.0, 1.0], "test", 0.0),
            qpos_dim: 1,
            dof_dim: 1,
            parent_body: "link2".into(),
            child_body: "link3".into(),
            q_min: Provenanced::unknown("test", 0.0),
            q_max: Provenanced::unknown("test", 0.0),
            dq_max: Provenanced::unknown("test", 0.0),
            effort_max: Provenanced::unknown("test", 0.0),
            origin_in_child: Provenanced::declared([0.0, 0.0, 0.0], "test", 0.0),
            parent_to_joint: unknown_se3("test"),
            joint_to_child: unknown_se3("test"),
            qpos_adr: Some(2),
            dof_adr: Some(2),
        });
        if let Some(f) = m.frames.iter_mut().find(|f| f.name == "ee") {
            f.parent_body = "link3".into();
            f.translation = Provenanced::declared([L3, 0.0, 0.0], "test", 0.0);
        }
        m.end_effectors[0].joint_chain = vec!["j0".into(), "j1".into(), "j2".into()];
        m
    }

    #[test]
    fn fd_residual_matches_shipped_jacobian_at_offset() {
        let m = synth_planar_two_link();
        let chain = m.ee_joint_chain("ee").unwrap();
        let w = contact_jacobian_witness(&m, &chain, "ee", &[0.2, -0.3], [0.01, 0.02, 0.0], 1e-6)
            .unwrap();
        assert_eq!(w.joint_names, chain);
        assert!(w.residual < 1e-5, "fd residual {} too large", w.residual);
        assert_eq!(w.analytic_3xn[0].len(), 2);
    }

    #[test]
    fn named_joint_alignment_holds_on_three_link_chain() {
        let m = three_link();
        let chain = m.ee_joint_chain("ee").unwrap();
        assert_eq!(chain.len(), 3);
        let q = [0.1, -0.2, 0.3];
        let w = contact_jacobian_witness(&m, &chain, "ee", &q, [0.0, 0.03, 0.0], 1e-6).unwrap();
        assert_eq!(w.joint_names, vec!["j0", "j1", "j2"]);
        assert!(w.residual < 1e-5, "residual {}", w.residual);
        assert_eq!(w.analytic_3xn[0].len(), 3);
    }
}
