//! Quasi-static gravity / self-load torque from provenanced inertials.
//!
//! Actuator-required gravity load (self-load):
//! τ_self,i = − Σ_bodies axis_i · ((p_com − o_i) × m g)  (revolute)
//! τ_self,i = − axis_i · Σ_bodies (m g)                    (prismatic)
//!
//! This is the generalized force the actuator must supply to hold the chain,
//! matching M q̈ + b(q,q̇) = τ with q̇=q̈=0 (b ≈ gravity). Gravity's applied
//! torque is the negative of this value.
//!
//! No robot identity. No MuJoCo. Missing mass/COM is the caller's UNKNOWN.

use crate::contact::JointMotionKind;
use crate::error::{finite, nonneg, PhysicsError, PhysicsResult};

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// One rigid body with a known world-frame COM.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidBodyInertial {
    pub mass_kg: f64,
    pub com_world: [f64; 3],
}

/// One joint and the downstream body indices it supports.
#[derive(Debug, Clone, PartialEq)]
pub struct JointForGravity {
    pub kind: JointMotionKind,
    pub origin_world: [f64; 3],
    pub axis_world: [f64; 3],
    pub downstream: Vec<usize>,
}

/// Gravity generalized force at each joint. Caller supplies only known inertials.
pub fn gravity_torque_nm(
    joints: &[JointForGravity],
    bodies: &[RigidBodyInertial],
    gravity_m_s2: [f64; 3],
) -> PhysicsResult<Vec<f64>> {
    let g = [
        finite(gravity_m_s2[0], "gravity")?,
        finite(gravity_m_s2[1], "gravity")?,
        finite(gravity_m_s2[2], "gravity")?,
    ];
    for (i, b) in bodies.iter().enumerate() {
        nonneg(b.mass_kg, "mass")?;
        finite(b.com_world[0], "com")?;
        finite(b.com_world[1], "com")?;
        finite(b.com_world[2], "com")?;
        let _ = i;
    }
    let mut out = Vec::with_capacity(joints.len());
    for j in joints {
        let axis = [
            finite(j.axis_world[0], "axis")?,
            finite(j.axis_world[1], "axis")?,
            finite(j.axis_world[2], "axis")?,
        ];
        let origin = [
            finite(j.origin_world[0], "origin")?,
            finite(j.origin_world[1], "origin")?,
            finite(j.origin_world[2], "origin")?,
        ];
        let mut tau = 0.0;
        for &idx in &j.downstream {
            let Some(body) = bodies.get(idx) else {
                return Err(PhysicsError::Unevaluable("downstream_body"));
            };
            let weight = [
                body.mass_kg * g[0],
                body.mass_kg * g[1],
                body.mass_kg * g[2],
            ];
            tau += match j.kind {
                JointMotionKind::Revolute => {
                    let r = sub3(body.com_world, origin);
                    -dot3(axis, cross3(r, weight))
                }
                JointMotionKind::Prismatic => -dot3(axis, weight),
                JointMotionKind::Fixed => 0.0,
            };
        }
        out.push(tau);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    const G: f64 = 9.80665;

    #[test]
    fn two_link_vertical_hinges_at_horizontal() {
        // Links along +x, hinges about +y, g = −z.
        let l1 = 0.4;
        let l2 = 0.3;
        let m1 = 2.0;
        let m2 = 1.0;
        let bodies = [
            RigidBodyInertial {
                mass_kg: m1,
                com_world: [l1 / 2.0, 0.0, 0.0],
            },
            RigidBodyInertial {
                mass_kg: m2,
                com_world: [l1 + l2 / 2.0, 0.0, 0.0],
            },
        ];
        let joints = [
            JointForGravity {
                kind: JointMotionKind::Revolute,
                origin_world: [0.0, 0.0, 0.0],
                axis_world: [0.0, 1.0, 0.0],
                downstream: vec![0, 1],
            },
            JointForGravity {
                kind: JointMotionKind::Revolute,
                origin_world: [l1, 0.0, 0.0],
                axis_world: [0.0, 1.0, 0.0],
                downstream: vec![1],
            },
        ];
        let tau = gravity_torque_nm(&joints, &bodies, [0.0, 0.0, -G]).unwrap();
        let t0 = -G * (m1 * l1 / 2.0 + m2 * (l1 + l2 / 2.0));
        let t1 = -G * m2 * l2 / 2.0;
        assert!((tau[0] - t0).abs() < 1e-12, "{} vs {}", tau[0], t0);
        assert!((tau[1] - t1).abs() < 1e-12, "{} vs {}", tau[1], t1);
    }

    #[test]
    fn vertical_links_have_zero_hinge_gravity() {
        let bodies = [RigidBodyInertial {
            mass_kg: 1.5,
            com_world: [0.0, 0.0, 0.2],
        }];
        let joints = [JointForGravity {
            kind: JointMotionKind::Revolute,
            origin_world: [0.0, 0.0, 0.0],
            axis_world: [0.0, 1.0, 0.0],
            downstream: vec![0],
        }];
        let tau = gravity_torque_nm(&joints, &bodies, [0.0, 0.0, -G]).unwrap();
        assert!(tau[0].abs() < 1e-12, "{}", tau[0]);
    }

    #[test]
    fn planar_z_hinges_with_vertical_g_are_zero() {
        let bodies = [
            RigidBodyInertial {
                mass_kg: 0.35,
                com_world: [0.11, 0.02, 0.08],
            },
            RigidBodyInertial {
                mass_kg: 0.25,
                com_world: [0.31, -0.01, 0.08],
            },
        ];
        let joints = [
            JointForGravity {
                kind: JointMotionKind::Revolute,
                origin_world: [0.0, 0.0, 0.08],
                axis_world: [0.0, 0.0, 1.0],
                downstream: vec![0, 1],
            },
            JointForGravity {
                kind: JointMotionKind::Revolute,
                origin_world: [0.22, 0.0, 0.08],
                axis_world: [0.0, 0.0, 1.0],
                downstream: vec![1],
            },
        ];
        let tau = gravity_torque_nm(&joints, &bodies, [0.0, 0.0, -G]).unwrap();
        assert!(tau[0].abs() < 1e-12 && tau[1].abs() < 1e-12, "{tau:?}");
    }

    #[test]
    fn prismatic_along_gravity_is_subtree_weight() {
        let bodies = [
            RigidBodyInertial {
                mass_kg: 0.8,
                com_world: [0.0, 0.0, 0.1],
            },
            RigidBodyInertial {
                mass_kg: 0.2,
                com_world: [0.0, 0.0, 0.4],
            },
        ];
        let joints = [JointForGravity {
            kind: JointMotionKind::Prismatic,
            origin_world: [0.0, 0.0, 0.0],
            axis_world: [0.0, 0.0, 1.0],
            downstream: vec![0, 1],
        }];
        let tau = gravity_torque_nm(&joints, &bodies, [0.0, 0.0, -G]).unwrap();
        let expected = G * (0.8 + 0.2);
        assert!(
            (tau[0] - expected).abs() < 1e-12,
            "{} vs {}",
            tau[0],
            expected
        );
    }

    #[test]
    fn missing_downstream_index_is_unevaluable() {
        let bodies = [RigidBodyInertial {
            mass_kg: 1.0,
            com_world: [0.1, 0.0, 0.0],
        }];
        let joints = [JointForGravity {
            kind: JointMotionKind::Revolute,
            origin_world: [0.0, 0.0, 0.0],
            axis_world: [0.0, 1.0, 0.0],
            downstream: vec![0, 1],
        }];
        let err = gravity_torque_nm(&joints, &bodies, [0.0, 0.0, -G]).unwrap_err();
        assert_eq!(err, PhysicsError::Unevaluable("downstream_body"));
    }

    #[test]
    fn omitting_downstream_mass_changes_torque() {
        let full = [
            RigidBodyInertial {
                mass_kg: 1.0,
                com_world: [0.2, 0.0, 0.0],
            },
            RigidBodyInertial {
                mass_kg: 3.0,
                com_world: [0.5, 0.0, 0.0],
            },
        ];
        let joints_full = [JointForGravity {
            kind: JointMotionKind::Revolute,
            origin_world: [0.0, 0.0, 0.0],
            axis_world: [0.0, 1.0, 0.0],
            downstream: vec![0, 1],
        }];
        let joints_omit = [JointForGravity {
            kind: JointMotionKind::Revolute,
            origin_world: [0.0, 0.0, 0.0],
            axis_world: [0.0, 1.0, 0.0],
            downstream: vec![0],
        }];
        let a = gravity_torque_nm(&joints_full, &full, [0.0, 0.0, -G]).unwrap();
        let b = gravity_torque_nm(&joints_omit, &full, [0.0, 0.0, -G]).unwrap();
        assert!(
            (a[0] - b[0]).abs() > 1.0,
            "planted missing mass must not average away"
        );
    }
}
