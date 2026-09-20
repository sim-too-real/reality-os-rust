//! Finite-face contact manifold. Object-frame geometry only. No robot identity.

use crate::transform::{cross3, norm3, normalize3, scale3, sub3};

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Translational IK residual. Not a contact-geometry or execution tolerance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IkResidual {
    pub translational_m: f64,
}

impl IkResidual {
    pub fn precise_accept_m() -> f64 {
        crate::kinematics::IK_ACCEPT_M
    }

    pub fn is_precise(self) -> bool {
        crate::kinematics::ik_residual_is_precise(self.translational_m)
    }
}

/// Allowed deviation from the designed face (normal) and face edges (tangent).
/// Independent of IK success.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactGeometryTolerance {
    pub normal_m: f64,
    pub tangent_slack_m: f64,
}

impl ContactGeometryTolerance {
    pub fn for_face(normal_m: f64) -> Self {
        Self {
            normal_m: normal_m.max(0.0),
            tangent_slack_m: 1e-4,
        }
    }
}

/// Allowed execution-time pose error. Independent of IK success.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExecutionPoseTolerance {
    pub position_m: f64,
}

impl ExecutionPoseTolerance {
    pub fn from_precise_ik() -> Self {
        Self {
            position_m: crate::kinematics::IK_ACCEPT_M,
        }
    }
}

/// Position-only Cartesian target. Must not be treated as contact-pose proof.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionTarget {
    pub xyz: [f64; 3],
}

/// Constrained contact target on a finite face. Roll may stay free.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactManifoldTarget {
    pub tool_point: [f64; 3],
    pub min_axis_align: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactingBodyKind {
    DeclaredTool,
    Forearm,
    Other,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ManifoldReject {
    TangentUOutside,
    TangentVOutside,
    NormalSeparation,
    OppositeFace,
    InsideSupport,
    WrongOrientation,
    WrongContactingBody,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoxFaceManifold {
    pub origin: [f64; 3],
    pub normal: [f64; 3],
    pub u: [f64; 3],
    pub v: [f64; 3],
    pub u_half: f64,
    pub v_half: f64,
    pub face_gap: f64,
    pub push: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ManifoldCoords {
    pub normal: f64,
    pub u: f64,
    pub v: f64,
}

fn box_half_along(half: [f64; 3], dir: [f64; 3]) -> f64 {
    (half[0] * dir[0].abs() + half[1] * dir[1].abs() + half[2] * dir[2].abs()).max(1e-6)
}

fn tangent_basis(normal: [f64; 3], up: [f64; 3]) -> Option<([f64; 3], [f64; 3])> {
    let n = normalize3(normal)?;
    let up_n = normalize3(up).unwrap_or([0.0, 0.0, 1.0]);
    let mut u = cross3(n, up_n);
    if norm3(u) < 1e-9 {
        u = cross3(n, [1.0, 0.0, 0.0]);
    }
    if norm3(u) < 1e-9 {
        u = cross3(n, [0.0, 1.0, 0.0]);
    }
    let u = normalize3(u)?;
    let v = normalize3(cross3(u, n))?;
    Some((u, v))
}

/// Near-face manifold for a finite box being pushed along `push`.
/// Origin is the geometric face center. Outward normal points toward the tool.
pub fn box_push_face_manifold(
    object_center: [f64; 3],
    half_extents: [f64; 3],
    push: [f64; 3],
    support_normal: [f64; 3],
    face_gap: f64,
) -> Option<BoxFaceManifold> {
    let push = normalize3(push)?;
    let half = box_half_along(half_extents, push);
    let origin = [
        object_center[0] - push[0] * half,
        object_center[1] - push[1] * half,
        object_center[2] - push[2] * half,
    ];
    let normal = scale3(push, -1.0);
    let (u, v) = tangent_basis(normal, support_normal)?;
    Some(BoxFaceManifold {
        origin,
        normal,
        u,
        v,
        u_half: box_half_along(half_extents, u),
        v_half: box_half_along(half_extents, v),
        face_gap: face_gap.max(0.0),
        push,
    })
}

pub fn manifold_coords(manifold: &BoxFaceManifold, tool_point: [f64; 3]) -> ManifoldCoords {
    let d = sub3(tool_point, manifold.origin);
    ManifoldCoords {
        normal: dot3(d, manifold.normal),
        u: dot3(d, manifold.u),
        v: dot3(d, manifold.v),
    }
}

fn support_signed_height(
    point: [f64; 3],
    support_origin: [f64; 3],
    support_normal: [f64; 3],
) -> f64 {
    let n = normalize3(support_normal).unwrap_or([0.0, 0.0, 1.0]);
    dot3(sub3(point, support_origin), n)
}

/// Object-frame membership. `tool_axis_world` is the declared tool approach axis.
pub fn evaluate_box_face_contact(
    tool_point: [f64; 3],
    tool_axis_world: Option<[f64; 3]>,
    contacting: ContactingBodyKind,
    object_center: [f64; 3],
    half_extents: [f64; 3],
    push: [f64; 3],
    support_origin: [f64; 3],
    support_normal: [f64; 3],
    face_gap: f64,
    min_axis_align: f64,
    geom_tol: ContactGeometryTolerance,
) -> Result<ManifoldCoords, ManifoldReject> {
    if contacting != ContactingBodyKind::DeclaredTool {
        return Err(ManifoldReject::WrongContactingBody);
    }
    let Some(manifold) =
        box_push_face_manifold(object_center, half_extents, push, support_normal, face_gap)
    else {
        return Err(ManifoldReject::NormalSeparation);
    };
    let Some(axis) = tool_axis_world.and_then(normalize3) else {
        return Err(ManifoldReject::WrongOrientation);
    };
    if dot3(axis, manifold.push) + 1e-12 < min_axis_align {
        return Err(ManifoldReject::WrongOrientation);
    }
    if support_signed_height(tool_point, support_origin, support_normal) < -geom_tol.tangent_slack_m
    {
        return Err(ManifoldReject::InsideSupport);
    }
    let c = manifold_coords(&manifold, tool_point);
    if c.u.abs() > manifold.u_half + geom_tol.tangent_slack_m {
        return Err(ManifoldReject::TangentUOutside);
    }
    if c.v.abs() > manifold.v_half + geom_tol.tangent_slack_m {
        return Err(ManifoldReject::TangentVOutside);
    }
    let intended = manifold.face_gap;
    if (c.normal - intended).abs() > geom_tol.normal_m {
        if c.normal < -manifold.u_half.min(manifold.v_half).max(1e-6) {
            return Err(ManifoldReject::OppositeFace);
        }
        return Err(ManifoldReject::NormalSeparation);
    }
    if dot3(sub3(tool_point, object_center), manifold.push) > geom_tol.normal_m {
        return Err(ManifoldReject::OppositeFace);
    }
    Ok(c)
}

/// Axis-alignment + in-face tool point + small normal window. Roll is free.
pub fn contact_constraint_residual(
    tool_point: [f64; 3],
    tool_axis_world: Option<[f64; 3]>,
    manifold: &BoxFaceManifold,
    min_axis_align: f64,
) -> (f64, f64, f64, f64) {
    let c = manifold_coords(manifold, tool_point);
    let align = match tool_axis_world.and_then(normalize3) {
        Some(axis) => dot3(axis, manifold.push),
        None => -1.0,
    };
    let axis_err = (min_axis_align - align).max(0.0);
    let u_err = (c.u.abs() - manifold.u_half).max(0.0);
    let v_err = (c.v.abs() - manifold.v_half).max(0.0);
    let n_err = (c.normal - manifold.face_gap).abs();
    (n_err, u_err, v_err, axis_err)
}

#[cfg(test)]
mod tests {
    use super::*;

    struct FaceCase {
        center: [f64; 3],
        half: [f64; 3],
        push: [f64; 3],
        support_origin: [f64; 3],
        support_n: [f64; 3],
    }

    fn box_push_x() -> FaceCase {
        FaceCase {
            center: [0.30, 0.0, 0.16],
            half: [0.03, 0.03, 0.03],
            push: [1.0, 0.0, 0.0],
            support_origin: [0.30, 0.0, 0.13],
            support_n: [0.0, 0.0, 1.0],
        }
    }

    fn tol() -> ContactGeometryTolerance {
        ContactGeometryTolerance::for_face(0.02)
    }

    fn eval(
        tool: [f64; 3],
        axis: Option<[f64; 3]>,
        body: ContactingBodyKind,
    ) -> Result<ManifoldCoords, ManifoldReject> {
        let c = box_push_x();
        evaluate_box_face_contact(
            tool,
            axis,
            body,
            c.center,
            c.half,
            c.push,
            c.support_origin,
            c.support_n,
            0.015,
            0.5,
            tol(),
        )
    }

    #[test]
    fn tolerances_are_independently_inspectable() {
        let ik = IkResidual {
            translational_m: 4e-2,
        };
        assert!(!ik.is_precise());
        assert!(IkResidual {
            translational_m: 5e-4
        }
        .is_precise());
        assert!(IkResidual::precise_accept_m() < 0.01);
        let g = ContactGeometryTolerance::for_face(0.03);
        let e = ExecutionPoseTolerance::from_precise_ik();
        assert_ne!(g.normal_m, e.position_m);
        assert!(e.position_m <= IkResidual::precise_accept_m() + 1e-18);
    }

    #[test]
    fn valid_face_center_is_inside() {
        let tool = [0.30 - 0.03 - 0.015, 0.0, 0.16];
        eval(
            tool,
            Some([1.0, 0.0, 0.0]),
            ContactingBodyKind::DeclaredTool,
        )
        .unwrap();
    }

    #[test]
    fn past_side_edge_is_rejected() {
        let tool = [0.30 - 0.03 - 0.015, 0.08, 0.16];
        assert_eq!(
            eval(
                tool,
                Some([1.0, 0.0, 0.0]),
                ContactingBodyKind::DeclaredTool
            )
            .unwrap_err(),
            ManifoldReject::TangentUOutside
        );
    }

    #[test]
    fn above_finite_face_is_rejected() {
        let tool = [0.30 - 0.03 - 0.015, 0.0, 0.16 + 0.08];
        assert_eq!(
            eval(
                tool,
                Some([1.0, 0.0, 0.0]),
                ContactingBodyKind::DeclaredTool
            )
            .unwrap_err(),
            ManifoldReject::TangentVOutside
        );
    }

    #[test]
    fn below_finite_face_is_rejected() {
        let tool = [0.30 - 0.03 - 0.015, 0.0, 0.16 - 0.08];
        let err = eval(
            tool,
            Some([1.0, 0.0, 0.0]),
            ContactingBodyKind::DeclaredTool,
        )
        .unwrap_err();
        assert!(
            err == ManifoldReject::TangentVOutside || err == ManifoldReject::InsideSupport,
            "got {err:?}"
        );
    }

    #[test]
    fn wrong_orientation_is_rejected() {
        let tool = [0.30 - 0.03 - 0.015, 0.0, 0.16];
        assert_eq!(
            eval(
                tool,
                Some([0.0, 1.0, 0.0]),
                ContactingBodyKind::DeclaredTool
            )
            .unwrap_err(),
            ManifoldReject::WrongOrientation
        );
    }

    #[test]
    fn missing_tool_axis_is_not_treated_as_aligned() {
        let tool = [0.30 - 0.03 - 0.015, 0.0, 0.16];
        assert_eq!(
            eval(tool, None, ContactingBodyKind::DeclaredTool).unwrap_err(),
            ManifoldReject::WrongOrientation
        );
        let f = box_push_x();
        let manifold =
            box_push_face_manifold(f.center, f.half, f.push, f.support_n, 0.015).unwrap();
        let (_n, _u, _v, axis_err) = contact_constraint_residual(tool, None, &manifold, 0.5);
        assert!(
            axis_err + 1e-12 >= 0.5,
            "missing axis must not score as aligned, axis_err={axis_err}"
        );
    }

    #[test]
    fn forearm_is_not_a_contact_candidate() {
        let tool = [0.30 - 0.03 - 0.015, 0.0, 0.16];
        assert_eq!(
            eval(tool, Some([1.0, 0.0, 0.0]), ContactingBodyKind::Forearm).unwrap_err(),
            ManifoldReject::WrongContactingBody
        );
    }

    #[test]
    fn inside_support_is_rejected() {
        let tool = [0.30 - 0.03 - 0.015, 0.0, 0.10];
        assert_eq!(
            eval(
                tool,
                Some([1.0, 0.0, 0.0]),
                ContactingBodyKind::DeclaredTool
            )
            .unwrap_err(),
            ManifoldReject::InsideSupport
        );
    }

    #[test]
    fn opposite_face_is_rejected() {
        let tool = [0.30 + 0.03 + 0.015, 0.0, 0.16];
        let err = eval(
            tool,
            Some([1.0, 0.0, 0.0]),
            ContactingBodyKind::DeclaredTool,
        )
        .unwrap_err();
        assert!(
            err == ManifoldReject::OppositeFace || err == ManifoldReject::NormalSeparation,
            "got {err:?}"
        );
    }

    #[test]
    fn ee_origin_on_face_with_offset_tool_point_is_rejected() {
        let ee = [0.30 - 0.03 - 0.015, 0.0, 0.16];
        eval(ee, Some([1.0, 0.0, 0.0]), ContactingBodyKind::DeclaredTool).unwrap();
        let tool = [ee[0] + 0.04, ee[1] + 0.05, ee[2]];
        assert_eq!(
            eval(
                tool,
                Some([1.0, 0.0, 0.0]),
                ContactingBodyKind::DeclaredTool
            )
            .unwrap_err(),
            ManifoldReject::TangentUOutside
        );
    }
}
