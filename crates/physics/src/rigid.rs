//! Pure-Rust serial FK / RNEA. Narrow ABI — not Pinocchio, not a safety authority.

use crate::error::{finite, PhysicsError, PhysicsResult};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointKind {
    Revolute,
    Prismatic,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SerialJoint {
    pub kind: JointKind,
    pub axis: [f64; 3],
    pub origin: [f64; 3],
    pub mass: f64,
    pub com: [f64; 3],
    pub inertia_diag: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct SerialModel {
    pub joints: Vec<SerialJoint>,
}

pub type Mat4 = [[f64; 4]; 4];

pub trait RigidBodyBackend {
    fn name(&self) -> &'static str;
    fn fk(&self, model: &SerialModel, q: &[f64]) -> PhysicsResult<Vec<Mat4>>;
    fn rnea(
        &self,
        model: &SerialModel,
        q: &[f64],
        dq: &[f64],
        ddq: &[f64],
        gravity: [f64; 3],
    ) -> PhysicsResult<Vec<f64>>;
}

#[derive(Debug, Clone, Copy, Default)]
pub struct NativeRigidBackend;

impl RigidBodyBackend for NativeRigidBackend {
    fn name(&self) -> &'static str {
        "native_rnea"
    }

    fn fk(&self, model: &SerialModel, q: &[f64]) -> PhysicsResult<Vec<Mat4>> {
        fk_serial(model, q)
    }

    fn rnea(
        &self,
        model: &SerialModel,
        q: &[f64],
        dq: &[f64],
        ddq: &[f64],
        gravity: [f64; 3],
    ) -> PhysicsResult<Vec<f64>> {
        rnea_serial(model, q, dq, ddq, gravity)
    }
}

pub fn default_backend() -> NativeRigidBackend {
    NativeRigidBackend
}

fn require_dim(model: &SerialModel, q: &[f64], name: &'static str) -> PhysicsResult<()> {
    if q.len() != model.joints.len() {
        return Err(PhysicsError::DimMismatch {
            name,
            expected: model.joints.len(),
            got: q.len(),
        });
    }
    for (i, v) in q.iter().enumerate() {
        finite(*v, name)?;
        let _ = i;
    }
    Ok(())
}

fn hat(axis: [f64; 3]) -> [f64; 3] {
    let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    if n < 1e-12 {
        [0.0, 0.0, 1.0]
    } else {
        [axis[0] / n, axis[1] / n, axis[2] / n]
    }
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn rot_axis_angle(axis: [f64; 3], q: f64) -> [[f64; 3]; 3] {
    let k = hat(axis);
    let c = q.cos();
    let s = q.sin();
    let v = 1.0 - c;
    [
        [
            c + k[0] * k[0] * v,
            k[0] * k[1] * v - k[2] * s,
            k[0] * k[2] * v + k[1] * s,
        ],
        [
            k[1] * k[0] * v + k[2] * s,
            c + k[1] * k[1] * v,
            k[1] * k[2] * v - k[0] * s,
        ],
        [
            k[2] * k[0] * v - k[1] * s,
            k[2] * k[1] * v + k[0] * s,
            c + k[2] * k[2] * v,
        ],
    ]
}

fn mat_mul3(a: [[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    [
        a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
        a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
        a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
    ]
}

fn identity4() -> Mat4 {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn mul4(a: Mat4, b: Mat4) -> Mat4 {
    let mut o = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            o[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] + a[i][3] * b[3][j];
        }
    }
    o
}

fn joint_transform(joint: &SerialJoint, q: f64) -> Mat4 {
    let mut t = identity4();
    t[0][3] = joint.origin[0];
    t[1][3] = joint.origin[1];
    t[2][3] = joint.origin[2];
    match joint.kind {
        JointKind::Revolute => {
            let r = rot_axis_angle(joint.axis, q);
            for i in 0..3 {
                for j in 0..3 {
                    t[i][j] = r[i][j];
                }
            }
        }
        JointKind::Prismatic => {
            let a = hat(joint.axis);
            t[0][3] += a[0] * q;
            t[1][3] += a[1] * q;
            t[2][3] += a[2] * q;
        }
    }
    t
}

pub fn fk_serial(model: &SerialModel, q: &[f64]) -> PhysicsResult<Vec<Mat4>> {
    require_dim(model, q, "q")?;
    let mut acc = identity4();
    let mut out = Vec::with_capacity(model.joints.len());
    for (j, qi) in model.joints.iter().zip(q) {
        acc = mul4(acc, joint_transform(j, *qi));
        out.push(acc);
    }
    Ok(out)
}

/// Recursive Newton–Euler for a serial chain. Gravity is expressed in the base frame.
pub fn rnea_serial(
    model: &SerialModel,
    q: &[f64],
    dq: &[f64],
    ddq: &[f64],
    gravity: [f64; 3],
) -> PhysicsResult<Vec<f64>> {
    require_dim(model, q, "q")?;
    require_dim(model, dq, "dq")?;
    require_dim(model, ddq, "ddq")?;
    for (i, g) in gravity.iter().enumerate() {
        finite(*g, "gravity")?;
        let _ = i;
    }

    let n = model.joints.len();
    if n == 0 {
        return Ok(Vec::new());
    }

    let mut omega = vec![[0.0; 3]; n];
    let mut alpha = vec![[0.0; 3]; n];
    let mut acc = vec![[0.0; 3]; n];
    let mut z_world = vec![[0.0; 3]; n];
    let mut com_world = vec![[0.0; 3]; n];
    let mut origin_world = vec![[0.0; 3]; n];

    let frames = fk_serial(model, q)?;
    for (i, joint) in model.joints.iter().enumerate() {
        let r = [
            [frames[i][0][0], frames[i][0][1], frames[i][0][2]],
            [frames[i][1][0], frames[i][1][1], frames[i][1][2]],
            [frames[i][2][0], frames[i][2][1], frames[i][2][2]],
        ];
        z_world[i] = mat_mul3(r, hat(joint.axis));
        origin_world[i] = [frames[i][0][3], frames[i][1][3], frames[i][2][3]];
        com_world[i] = add(origin_world[i], mat_mul3(r, joint.com));
    }

    for i in 0..n {
        let w_prev = if i == 0 { [0.0; 3] } else { omega[i - 1] };
        let a_prev = if i == 0 { [0.0; 3] } else { alpha[i - 1] };
        let acc_prev = if i == 0 {
            scale(gravity, -1.0)
        } else {
            acc[i - 1]
        };
        match model.joints[i].kind {
            JointKind::Revolute => {
                omega[i] = add(w_prev, scale(z_world[i], dq[i]));
                alpha[i] = add(
                    add(a_prev, scale(z_world[i], ddq[i])),
                    cross(w_prev, scale(z_world[i], dq[i])),
                );
            }
            JointKind::Prismatic => {
                omega[i] = w_prev;
                alpha[i] = a_prev;
            }
        }
        let r_com = if i == 0 {
            com_world[i]
        } else {
            [
                com_world[i][0] - origin_world[i - 1][0],
                com_world[i][1] - origin_world[i - 1][1],
                com_world[i][2] - origin_world[i - 1][2],
            ]
        };
        let mut linear = add(acc_prev, cross(alpha[i], r_com));
        linear = add(linear, cross(omega[i], cross(omega[i], r_com)));
        if model.joints[i].kind == JointKind::Prismatic {
            linear = add(linear, scale(z_world[i], ddq[i]));
            linear = add(linear, scale(cross(omega[i], z_world[i]), 2.0 * dq[i]));
        }
        acc[i] = linear;
    }

    let mut f = vec![[0.0; 3]; n];
    let mut n_mom = vec![[0.0; 3]; n];
    let mut tau = vec![0.0; n];
    for i in (0..n).rev() {
        let m = model.joints[i].mass.max(0.0);
        let fi = scale(acc[i], m);
        let child_f = if i + 1 < n { f[i + 1] } else { [0.0; 3] };
        f[i] = add(fi, child_f);
        let r_com = if i == 0 {
            com_world[i]
        } else {
            [
                com_world[i][0] - origin_world[i.saturating_sub(1)][0],
                com_world[i][1] - origin_world[i.saturating_sub(1)][1],
                com_world[i][2] - origin_world[i.saturating_sub(1)][2],
            ]
        };
        let ixx = model.joints[i].inertia_diag[0];
        let iyy = model.joints[i].inertia_diag[1];
        let izz = model.joints[i].inertia_diag[2];
        let i_omega = [ixx * omega[i][0], iyy * omega[i][1], izz * omega[i][2]];
        let i_alpha = [ixx * alpha[i][0], iyy * alpha[i][1], izz * alpha[i][2]];
        let mut ni = add(i_alpha, cross(omega[i], i_omega));
        ni = add(ni, cross(r_com, fi));
        if i + 1 < n {
            ni = add(ni, n_mom[i + 1]);
        }
        n_mom[i] = ni;
        tau[i] = match model.joints[i].kind {
            JointKind::Revolute => dot(n_mom[i], z_world[i]),
            JointKind::Prismatic => dot(f[i], z_world[i]),
        };
        finite(tau[i], "tau")?;
    }
    Ok(tau)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pendulum() -> SerialModel {
        SerialModel {
            joints: vec![SerialJoint {
                kind: JointKind::Revolute,
                axis: [0.0, 0.0, 1.0],
                origin: [0.0, 0.0, 0.0],
                mass: 2.0,
                com: [1.0, 0.0, 0.0],
                inertia_diag: [0.0, 0.0, 0.0],
            }],
        }
    }

    #[test]
    fn fk_rejects_dim_mismatch_and_nan() {
        let m = pendulum();
        let b = default_backend();
        assert!(b.fk(&m, &[]).is_err());
        assert!(b.fk(&m, &[f64::NAN]).is_err());
        let t = b.fk(&m, &[0.0]).unwrap();
        assert!((t[0][0][3] - 0.0).abs() < 1e-12);
    }

    #[test]
    fn rnea_static_pendulum_matches_mg_cross() {
        let m = pendulum();
        let b = default_backend();
        let tau = b
            .rnea(&m, &[0.0], &[0.0], &[0.0], [0.0, -9.81, 0.0])
            .unwrap();
        assert!((tau[0].abs() - 2.0 * 9.81).abs() < 1e-6);
        assert_eq!(b.name(), "native_rnea");
    }
}
