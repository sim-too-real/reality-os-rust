//! Planar wrench / twist with explicit frames. Quasi-static support limit surface
//! (ellipsoidal only when f_max, τ_max, and pressure assumptions are declared).
//!
//! Goyal/Ruina/Papadopoulos limit surface; Lynch/Howe ellipsoid:
//! (fx/f_max)² + (fy/f_max)² + (τ/τ_max)² = 1
//! Instantaneous twist is the outward normal A w with A = diag(1/f_max², 1/f_max², 1/τ_max²).
//! Mason/Lynch motion cone maps friction-cone edge wrenches through that gradient.

use crate::error::{finite, nonneg, positive, PhysicsError, PhysicsResult};
use serde::{Deserialize, Serialize};

fn dot2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn cross2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn add2(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] + b[0], a[1] + b[1]]
}

fn sub2(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

fn scale2(a: [f64; 2], s: f64) -> [f64; 2] {
    [a[0] * s, a[1] * s]
}

fn norm2(a: [f64; 2]) -> f64 {
    dot2(a, a).sqrt()
}

fn rot2(v: [f64; 2], yaw: f64) -> [f64; 2] {
    let c = yaw.cos();
    let s = yaw.sin();
    [c * v[0] - s * v[1], s * v[0] + c * v[1]]
}

fn unit2(v: [f64; 2], name: &'static str) -> PhysicsResult<[f64; 2]> {
    let v = [finite(v[0], name)?, finite(v[1], name)?];
    let n = norm2(v);
    if n <= 0.0 {
        return Err(PhysicsError::NonPositive(name));
    }
    Ok([v[0] / n, v[1] / n])
}

const TWIST_EPS: f64 = 1e-12;
const MODE_EPS: f64 = 1e-9;

/// Named planar frame. Anonymous triples are not used.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum PlanarFrameKind {
    World,
    Object,
    Contact,
}

/// w = [Fx, Fy, τz] in an explicit frame.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PlanarWrench {
    pub fx: f64,
    pub fy: f64,
    pub tau_z: f64,
    pub frame: PlanarFrameKind,
}

impl PlanarWrench {
    pub fn xy(self) -> [f64; 2] {
        [self.fx, self.fy]
    }

    pub fn is_zero(self) -> bool {
        self.fx.abs() < TWIST_EPS && self.fy.abs() < TWIST_EPS && self.tau_z.abs() < TWIST_EPS
    }
}

/// v = [vx, vy, ωz] in an explicit frame.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PlanarTwist {
    pub vx: f64,
    pub vy: f64,
    pub omega_z: f64,
    pub frame: PlanarFrameKind,
}

impl PlanarTwist {
    pub fn xy(self) -> [f64; 2] {
        [self.vx, self.vy]
    }

    pub fn is_zero(self) -> bool {
        self.vx.abs() < TWIST_EPS && self.vy.abs() < TWIST_EPS && self.omega_z.abs() < TWIST_EPS
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum RotationSign {
    Clockwise,
    Counterclockwise,
    TranslationOnly,
    Ambiguous,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ContactMode {
    Sticking,
    SlidingLeft,
    SlidingRight,
    Separating,
    Ambiguous,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum MotionCompatibility {
    MotionCompatible,
    MotionIncompatible,
    MotionBoundarySlip,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum PressureDistribution {
    Unknown,
    DeclaredUniform,
}

/// Support-friction wrench set. Type and pressure assumption are first-class.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum SupportFrictionModel {
    #[default]
    Unknown,
    TranslationalThreshold {
        mu: f64,
        normal_n: f64,
    },
    BoundedFromGeometry {
        f_max: f64,
        tau_bound: f64,
    },
    Ellipsoidal {
        f_max: f64,
        tau_max: f64,
        pressure: PressureDistribution,
    },
}

impl SupportFrictionModel {
    pub fn kind_name(self) -> &'static str {
        match self {
            Self::Unknown => "UNKNOWN",
            Self::TranslationalThreshold { .. } => "TRANSLATIONAL_THRESHOLD",
            Self::BoundedFromGeometry { .. } => "BOUNDED_FROM_GEOMETRY",
            Self::Ellipsoidal { .. } => "ELLIPSOIDAL",
        }
    }

    pub fn pressure_name(self) -> &'static str {
        match self {
            Self::Ellipsoidal { pressure, .. } => match pressure {
                PressureDistribution::Unknown => "UNKNOWN",
                PressureDistribution::DeclaredUniform => "DECLARED_UNIFORM",
            },
            Self::Unknown => "UNKNOWN",
            _ => "NOT_APPLICABLE",
        }
    }
}

/// τ_max = μ N (2/3) R for a circular patch with declared uniform pressure.
pub fn tau_max_uniform_circle(mu: f64, normal_n: f64, radius_m: f64) -> PhysicsResult<f64> {
    Ok(nonneg(mu, "mu")? * nonneg(normal_n, "normal_force")? * (2.0 / 3.0) * positive(radius_m, "radius")?)
}

pub fn f_max_coulomb(mu: f64, normal_n: f64) -> PhysicsResult<f64> {
    Ok(nonneg(mu, "mu")? * nonneg(normal_n, "normal_force")?)
}

/// Project a 3-vector onto the support plane (remove component along n̂).
pub fn project_to_plane(v: [f64; 3], support_normal: [f64; 3]) -> PhysicsResult<[f64; 2]> {
    let n = [
        finite(support_normal[0], "support_normal")?,
        finite(support_normal[1], "support_normal")?,
        finite(support_normal[2], "support_normal")?,
    ];
    let nn = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if nn <= 0.0 {
        return Err(PhysicsError::NonPositive("support_normal"));
    }
    let nh = [n[0] / nn, n[1] / nn, n[2] / nn];
    let v = [finite(v[0], "vec")?, finite(v[1], "vec")?, finite(v[2], "vec")?];
    let vn = v[0] * nh[0] + v[1] * nh[1] + v[2] * nh[2];
    let t = [v[0] - nh[0] * vn, v[1] - nh[1] * vn, v[2] - nh[2] * vn];
    // Plane basis: e1 = world x projected, or world y if n ≈ x.
    let ex = if nh[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let exn = ex[0] * nh[0] + ex[1] * nh[1] + ex[2] * nh[2];
    let e1 = [ex[0] - nh[0] * exn, ex[1] - nh[1] * exn, ex[2] - nh[2] * exn];
    let n1 = (e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2]).sqrt();
    if n1 <= 0.0 {
        return Err(PhysicsError::Singular("plane_basis"));
    }
    let e1 = [e1[0] / n1, e1[1] / n1, e1[2] / n1];
    let e2 = [
        nh[1] * e1[2] - nh[2] * e1[1],
        nh[2] * e1[0] - nh[0] * e1[2],
        nh[0] * e1[1] - nh[1] * e1[0],
    ];
    Ok([
        t[0] * e1[0] + t[1] * e1[1] + t[2] * e1[2],
        t[0] * e2[0] + t[1] * e2[1] + t[2] * e2[2],
    ])
}

/// Contact force at r produces object wrench [f, r × f] in the object frame.
pub fn contact_force_object_wrench(
    force_object_xy: [f64; 2],
    contact_offset_object_xy: [f64; 2],
) -> PhysicsResult<PlanarWrench> {
    let f = [
        finite(force_object_xy[0], "force")?,
        finite(force_object_xy[1], "force")?,
    ];
    let r = [
        finite(contact_offset_object_xy[0], "offset")?,
        finite(contact_offset_object_xy[1], "offset")?,
    ];
    Ok(PlanarWrench {
        fx: f[0],
        fy: f[1],
        tau_z: cross2(r, f),
        frame: PlanarFrameKind::Object,
    })
}

/// Rotate planar xy (world yaw about support normal). τz / ωz unchanged.
pub fn rotate_wrench_world(w: PlanarWrench, yaw_rad: f64) -> PlanarWrench {
    let xy = rot2(w.xy(), yaw_rad);
    PlanarWrench {
        fx: xy[0],
        fy: xy[1],
        tau_z: w.tau_z,
        frame: w.frame,
    }
}

pub fn rotate_twist_world(v: PlanarTwist, yaw_rad: f64) -> PlanarTwist {
    let xy = rot2(v.xy(), yaw_rad);
    PlanarTwist {
        vx: xy[0],
        vy: xy[1],
        omega_z: v.omega_z,
        frame: v.frame,
    }
}

pub fn rotation_sign_from_omega(omega_z: f64, translation: [f64; 2]) -> RotationSign {
    let t = norm2(translation);
    if omega_z.abs() < 1e-9 && t < 1e-12 {
        return RotationSign::Unknown;
    }
    if omega_z.abs() * 1.0 < 1e-6 * (t + 1e-9) {
        return RotationSign::TranslationOnly;
    }
    if omega_z > 0.0 {
        RotationSign::Counterclockwise
    } else {
        RotationSign::Clockwise
    }
}

/// Instantaneous object twist from a support wrench on the ellipsoidal LS.
/// Zero wrench does not produce a claimed sliding twist.
pub fn twist_from_ellipsoidal_wrench(
    wrench: PlanarWrench,
    f_max: f64,
    tau_max: f64,
) -> PhysicsResult<PlanarTwist> {
    if wrench.frame != PlanarFrameKind::Object {
        return Err(PhysicsError::Unevaluable("wrench_frame"));
    }
    if wrench.is_zero() {
        return Err(PhysicsError::Unevaluable("zero_wrench"));
    }
    let fm = positive(f_max, "f_max")?;
    let tm = positive(tau_max, "tau_max")?;
    let vx = wrench.fx / (fm * fm);
    let vy = wrench.fy / (fm * fm);
    let om = wrench.tau_z / (tm * tm);
    Ok(PlanarTwist {
        vx,
        vy,
        omega_z: om,
        frame: PlanarFrameKind::Object,
    })
}

/// Characteristic length c = τ_max / f_max. Normalize (vx, vy, c ω).
pub fn normalize_twist(twist: PlanarTwist, char_length_m: f64) -> PhysicsResult<PlanarTwist> {
    let c = positive(char_length_m, "char_length")?;
    let n = (twist.vx * twist.vx + twist.vy * twist.vy + (c * twist.omega_z).powi(2)).sqrt();
    if n <= 0.0 {
        return Err(PhysicsError::Unevaluable("zero_twist"));
    }
    Ok(PlanarTwist {
        vx: twist.vx / n,
        vy: twist.vy / n,
        omega_z: twist.omega_z / n,
        frame: twist.frame,
    })
}

/// Scale a wrench onto the ellipsoid (quasi-static: motion uses the boundary wrench).
pub fn project_wrench_to_ellipsoid(
    wrench: PlanarWrench,
    f_max: f64,
    tau_max: f64,
) -> PhysicsResult<PlanarWrench> {
    if wrench.is_zero() {
        return Err(PhysicsError::Unevaluable("zero_wrench"));
    }
    let fm = positive(f_max, "f_max")?;
    let tm = positive(tau_max, "tau_max")?;
    let s2 = (wrench.fx / fm).powi(2) + (wrench.fy / fm).powi(2) + (wrench.tau_z / tm).powi(2);
    if s2 <= 0.0 {
        return Err(PhysicsError::Unevaluable("zero_wrench"));
    }
    let s = s2.sqrt();
    Ok(PlanarWrench {
        fx: wrench.fx / s,
        fy: wrench.fy / s,
        tau_z: wrench.tau_z / s,
        frame: wrench.frame,
    })
}

/// Minimum λ ≥ 0 such that λ [f̂, r × f̂] lies on the ellipsoid (initiation).
pub fn lambda_to_limit_surface(
    force_dir_object_xy: [f64; 2],
    contact_offset_object_xy: [f64; 2],
    f_max: f64,
    tau_max: f64,
) -> PhysicsResult<f64> {
    let w = contact_force_object_wrench(force_dir_object_xy, contact_offset_object_xy)?;
    let fm = positive(f_max, "f_max")?;
    let tm = positive(tau_max, "tau_max")?;
    let s2 = (w.fx / fm).powi(2) + (w.fy / fm).powi(2) + (w.tau_z / tm).powi(2);
    if s2 <= 0.0 {
        return Err(PhysicsError::Unevaluable("zero_wrench"));
    }
    Ok(1.0 / s2.sqrt())
}

/// Velocity of the object at contact r given object-frame twist about the origin (COF).
pub fn velocity_at_offset(twist: PlanarTwist, offset_object_xy: [f64; 2]) -> [f64; 2] {
    [
        twist.vx - twist.omega_z * offset_object_xy[1],
        twist.vy + twist.omega_z * offset_object_xy[0],
    ]
}

fn cone_edge_force(n_hat: [f64; 2], t_hat: [f64; 2], mu: f64, left: bool) -> [f64; 2] {
    if left {
        add2(n_hat, scale2(t_hat, mu))
    } else {
        sub2(n_hat, scale2(t_hat, mu))
    }
}

/// Motion-cone edge contact-point velocities from friction-cone edges through the LS.
pub fn motion_cone_edges(
    n_hat_object: [f64; 2],
    mu_tool: f64,
    contact_offset_object_xy: [f64; 2],
    f_max: f64,
    tau_max: f64,
) -> PhysicsResult<([f64; 2], [f64; 2])> {
    let n = unit2(n_hat_object, "contact_normal")?;
    let mu = nonneg(mu_tool, "mu_tool")?;
    let t = [-n[1], n[0]];
    let f_left = cone_edge_force(n, t, mu, true);
    let f_right = cone_edge_force(n, t, mu, false);
    let w_l = project_wrench_to_ellipsoid(
        contact_force_object_wrench(f_left, contact_offset_object_xy)?,
        f_max,
        tau_max,
    )?;
    let w_r = project_wrench_to_ellipsoid(
        contact_force_object_wrench(f_right, contact_offset_object_xy)?,
        f_max,
        tau_max,
    )?;
    let v_l = twist_from_ellipsoidal_wrench(w_l, f_max, tau_max)?;
    let v_r = twist_from_ellipsoidal_wrench(w_r, f_max, tau_max)?;
    Ok((
        velocity_at_offset(v_l, contact_offset_object_xy),
        velocity_at_offset(v_r, contact_offset_object_xy),
    ))
}

/// Contact mode from pusher velocity vs the motion cone. Not “high μ ⇒ stick”.
pub fn contact_mode_from_pusher(
    pusher_vel_object_xy: [f64; 2],
    n_hat_object: [f64; 2],
    mu_tool: f64,
    contact_offset_object_xy: [f64; 2],
    model: SupportFrictionModel,
) -> PhysicsResult<ContactMode> {
    let n = unit2(n_hat_object, "contact_normal")?;
    let vp = [
        finite(pusher_vel_object_xy[0], "pusher_vel")?,
        finite(pusher_vel_object_xy[1], "pusher_vel")?,
    ];
    if norm2(vp) <= MODE_EPS {
        return Ok(ContactMode::Unknown);
    }
    let into = dot2(vp, n);
    if into < -MODE_EPS {
        return Ok(ContactMode::Separating);
    }
    let SupportFrictionModel::Ellipsoidal { f_max, tau_max, pressure } = model else {
        return Ok(ContactMode::Unknown);
    };
    if matches!(pressure, PressureDistribution::Unknown) {
        return Ok(ContactMode::Unknown);
    }
    let (v_left, v_right) = motion_cone_edges(n, mu_tool, contact_offset_object_xy, f_max, tau_max)?;
    // Inside if vp is between v_right and v_left (CCW from right to left about the cone).
    let cr = cross2(v_right, vp);
    let cl = cross2(vp, v_left);
    let span = cross2(v_right, v_left);
    if span.abs() < MODE_EPS {
        return Ok(ContactMode::Ambiguous);
    }
    if cr >= -MODE_EPS && cl >= -MODE_EPS {
        return Ok(ContactMode::Sticking);
    }
    // Outside: nearer edge by angle.
    let dl = {
        let nl = norm2(v_left).max(MODE_EPS);
        let np = norm2(vp).max(MODE_EPS);
        1.0 - (dot2(v_left, vp) / (nl * np)).clamp(-1.0, 1.0)
    };
    let dr = {
        let nr = norm2(v_right).max(MODE_EPS);
        let np = norm2(vp).max(MODE_EPS);
        1.0 - (dot2(v_right, vp) / (nr * np)).clamp(-1.0, 1.0)
    };
    if dl <= dr {
        Ok(ContactMode::SlidingLeft)
    } else {
        Ok(ContactMode::SlidingRight)
    }
}

pub fn motion_compatibility(
    requested_twist: PlanarTwist,
    contact_offset_object_xy: [f64; 2],
    n_hat_object: [f64; 2],
    mu_tool: f64,
    model: SupportFrictionModel,
) -> PhysicsResult<MotionCompatibility> {
    let SupportFrictionModel::Ellipsoidal { f_max, tau_max, pressure } = model else {
        return Ok(MotionCompatibility::Unknown);
    };
    if matches!(pressure, PressureDistribution::Unknown) {
        return Ok(MotionCompatibility::Unknown);
    }
    let vc = velocity_at_offset(requested_twist, contact_offset_object_xy);
    let n = unit2(n_hat_object, "contact_normal")?;
    if dot2(vc, n) < -MODE_EPS {
        return Ok(MotionCompatibility::MotionIncompatible);
    }
    let (v_left, v_right) = motion_cone_edges(n, mu_tool, contact_offset_object_xy, f_max, tau_max)?;
    let cr = cross2(v_right, vc);
    let cl = cross2(vc, v_left);
    if cr.abs() < 1e-6 || cl.abs() < 1e-6 {
        return Ok(MotionCompatibility::MotionBoundarySlip);
    }
    if cr >= 0.0 && cl >= 0.0 {
        Ok(MotionCompatibility::MotionCompatible)
    } else {
        Ok(MotionCompatibility::MotionIncompatible)
    }
}

/// Twist from a declared ellipsoidal model and a contact force (not pusher velocity).
pub fn twist_from_contact_force(
    force_object_xy: [f64; 2],
    contact_offset_object_xy: [f64; 2],
    model: SupportFrictionModel,
) -> PhysicsResult<(PlanarTwist, RotationSign)> {
    match model {
        SupportFrictionModel::Ellipsoidal {
            f_max,
            tau_max,
            pressure,
        } => {
            if matches!(pressure, PressureDistribution::Unknown) {
                return Err(PhysicsError::Unevaluable("pressure_unknown"));
            }
            let w = contact_force_object_wrench(force_object_xy, contact_offset_object_xy)?;
            if w.is_zero() {
                return Err(PhysicsError::Unevaluable("zero_wrench"));
            }
            let wb = project_wrench_to_ellipsoid(w, f_max, tau_max)?;
            let tw = twist_from_ellipsoidal_wrench(wb, f_max, tau_max)?;
            let sign = rotation_sign_from_omega(tw.omega_z, tw.xy());
            Ok((tw, sign))
        }
        SupportFrictionModel::Unknown => Err(PhysicsError::Unevaluable("support_unknown")),
        SupportFrictionModel::TranslationalThreshold { .. }
        | SupportFrictionModel::BoundedFromGeometry { .. } => {
            Err(PhysicsError::Unevaluable("support_not_ellipsoidal"))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ellip(f: f64, t: f64) -> SupportFrictionModel {
        SupportFrictionModel::Ellipsoidal {
            f_max: f,
            tau_max: t,
            pressure: PressureDistribution::DeclaredUniform,
        }
    }

    #[test]
    fn contact_force_moment_is_r_cross_f() {
        let w = contact_force_object_wrench([1.0, 0.0], [0.0, 0.04]).unwrap();
        assert!((w.tau_z + 0.04).abs() < 1e-12);
        assert_eq!(w.frame, PlanarFrameKind::Object);
        let m = contact_force_object_wrench([1.0, 0.0], [0.0, -0.04]).unwrap();
        assert!((m.tau_z - 0.04).abs() < 1e-12);
        assert!((w.tau_z + m.tau_z).abs() < 1e-12);
    }

    #[test]
    fn zero_wrench_does_not_claim_sliding_twist() {
        let z = PlanarWrench {
            fx: 0.0,
            fy: 0.0,
            tau_z: 0.0,
            frame: PlanarFrameKind::Object,
        };
        let err = twist_from_ellipsoidal_wrench(z, 1.0, 0.1).unwrap_err();
        assert_eq!(err, PhysicsError::Unevaluable("zero_wrench"));
    }

    #[test]
    fn centered_force_is_translation_only() {
        let (tw, sign) = twist_from_contact_force([2.0, 0.0], [0.0, 0.0], ellip(1.0, 0.05)).unwrap();
        assert_eq!(sign, RotationSign::TranslationOnly);
        assert!(tw.omega_z.abs() < 1e-12);
        assert!(tw.vx > 0.0);
        assert!(tw.vy.abs() < 1e-12);
        assert_eq!(tw.frame, PlanarFrameKind::Object);
    }

    #[test]
    fn mirrored_offset_flips_rotation_sign() {
        let model = ellip(1.0, 0.05);
        let (_, a) = twist_from_contact_force([1.0, 0.0], [0.0, 0.03], model).unwrap();
        let (_, b) = twist_from_contact_force([1.0, 0.0], [0.0, -0.03], model).unwrap();
        assert_eq!(a, RotationSign::Clockwise);
        assert_eq!(b, RotationSign::Counterclockwise);
        assert_ne!(a, b);
    }

    #[test]
    fn world_yaw_rotates_translation_not_omega() {
        let (tw, _) = twist_from_contact_force([1.0, 0.0], [0.0, 0.0], ellip(1.0, 0.05)).unwrap();
        let r = rotate_twist_world(tw, std::f64::consts::FRAC_PI_2);
        assert!(r.vx.abs() < 1e-12);
        assert!(r.vy > 0.0);
        assert!((r.omega_z - tw.omega_z).abs() < 1e-12);
    }

    #[test]
    fn scaling_force_does_not_change_twist_direction() {
        let model = ellip(2.0, 0.1);
        let (a, sa) = twist_from_contact_force([1.0, 0.2], [0.01, 0.03], model).unwrap();
        let (b, sb) = twist_from_contact_force([4.0, 0.8], [0.01, 0.03], model).unwrap();
        assert_eq!(sa, sb);
        let c = 0.1 / 2.0;
        let na = normalize_twist(a, c).unwrap();
        let nb = normalize_twist(b, c).unwrap();
        assert!((na.vx - nb.vx).abs() < 1e-9);
        assert!((na.vy - nb.vy).abs() < 1e-9);
        assert!((na.omega_z - nb.omega_z).abs() < 1e-9);
    }

    #[test]
    fn unknown_pressure_is_not_auto_uniform() {
        let model = SupportFrictionModel::Ellipsoidal {
            f_max: 1.0,
            tau_max: 0.05,
            pressure: PressureDistribution::Unknown,
        };
        let err = twist_from_contact_force([1.0, 0.0], [0.0, 0.03], model).unwrap_err();
        assert_eq!(err, PhysicsError::Unevaluable("pressure_unknown"));
        assert_ne!(model.pressure_name(), "DECLARED_UNIFORM");
    }

    #[test]
    fn tangential_pusher_velocity_can_leave_sticking() {
        let model = ellip(1.0, 0.05);
        let n = [1.0, 0.0];
        let r = [0.0, 0.0];
        let stick = contact_mode_from_pusher([1.0, 0.0], n, 0.4, r, model).unwrap();
        let slide = contact_mode_from_pusher([1.0, 3.0], n, 0.4, r, model).unwrap();
        assert_eq!(stick, ContactMode::Sticking);
        assert_ne!(slide, ContactMode::Sticking);
        assert!(matches!(
            slide,
            ContactMode::SlidingLeft | ContactMode::SlidingRight
        ));
    }

    #[test]
    fn pulling_pusher_is_separating() {
        let model = ellip(1.0, 0.05);
        let m = contact_mode_from_pusher([-1.0, 0.0], [1.0, 0.0], 0.8, [0.0, 0.0], model).unwrap();
        assert_eq!(m, ContactMode::Separating);
    }

    #[test]
    fn high_mu_alone_does_not_force_sticking() {
        let model = ellip(1.0, 0.05);
        let m = contact_mode_from_pusher([0.1, 5.0], [1.0, 0.0], 5.0, [0.0, 0.0], model).unwrap();
        assert_ne!(m, ContactMode::Sticking);
    }

    #[test]
    fn requested_twist_inside_cone_is_compatible() {
        let model = ellip(1.0, 0.05);
        let tw = PlanarTwist {
            vx: 1.0,
            vy: 0.0,
            omega_z: 0.0,
            frame: PlanarFrameKind::Object,
        };
        let c = motion_compatibility(tw, [0.0, 0.0], [1.0, 0.0], 0.5, model).unwrap();
        assert_eq!(c, MotionCompatibility::MotionCompatible);
        let bad = PlanarTwist {
            vx: 0.1,
            vy: 4.0,
            omega_z: 0.0,
            frame: PlanarFrameKind::Object,
        };
        let i = motion_compatibility(bad, [0.0, 0.0], [1.0, 0.0], 0.5, model).unwrap();
        assert_eq!(i, MotionCompatibility::MotionIncompatible);
    }

    #[test]
    fn frames_are_explicit_on_wrench_and_twist() {
        let w = contact_force_object_wrench([0.0, 1.0], [0.02, 0.0]).unwrap();
        assert_eq!(w.frame, PlanarFrameKind::Object);
        let (tw, _) = twist_from_contact_force([0.0, 1.0], [0.02, 0.0], ellip(1.0, 0.04)).unwrap();
        assert_eq!(tw.frame, PlanarFrameKind::Object);
    }

    #[test]
    fn uniform_circle_tau_max_is_two_thirds_mu_n_r() {
        let t = tau_max_uniform_circle(0.3, 10.0, 0.05).unwrap();
        assert!((t - 0.3 * 10.0 * (2.0 / 3.0) * 0.05).abs() < 1e-12);
    }
}
