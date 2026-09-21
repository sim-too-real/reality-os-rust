//! Object-level planar goal and dimensionally-sane error / progress.

use realityos_physics::{rotate_twist_world, PlanarFrameKind, PlanarTwist};
use serde::{Deserialize, Serialize};

const PROGRESS_EPS: f64 = 1e-6;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum InteractionFamily {
    PlanarPush,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum GoalProgressClass {
    StrictProgress,
    Neutral,
    Regression,
    Ambiguous,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PlanarRegion {
    pub center_xy: [f64; 2],
    pub half_extents_xy: [f64; 2],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SafetyConstraints {
    pub require_authority: bool,
    pub require_fresh_observation: bool,
    pub max_stroke_m: f64,
}

impl Default for SafetyConstraints {
    fn default() -> Self {
        Self {
            require_authority: true,
            require_fresh_observation: true,
            max_stroke_m: 0.05,
        }
    }
}

/// Smallest meaningful object-level goal. Not natural language.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlanarObjectGoal {
    pub object_id: String,
    pub world_id: String,
    pub model_id: String,
    pub target_xy: Option<[f64; 2]>,
    pub target_xy_region: Option<PlanarRegion>,
    pub target_yaw: Option<f64>,
    pub target_yaw_interval: Option<[f64; 2]>,
    pub translation_tolerance_m: f64,
    pub orientation_tolerance_rad: f64,
    pub freshness_s: f64,
    pub allowed_interaction_family: InteractionFamily,
    pub safety: SafetyConstraints,
    pub max_bounded_attempts: u32,
}

impl PlanarObjectGoal {
    pub fn translation_active(&self) -> bool {
        self.target_xy.is_some() || self.target_xy_region.is_some()
    }

    pub fn orientation_active(&self) -> bool {
        self.target_yaw.is_some() || self.target_yaw_interval.is_some()
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GoalError {
    pub translation_residual_m: f64,
    pub orientation_residual_rad: f64,
    pub translation_normalized: f64,
    pub orientation_normalized: f64,
    pub combined: f64,
    /// World XY: current minus closest point of the target (empty if inactive).
    pub translation_error_xy: [f64; 2],
    /// wrap(current_yaw − target_yaw). Zero if orientation is inactive.
    pub signed_yaw_error: f64,
    pub translation_active: bool,
    pub orientation_active: bool,
    pub reached: bool,
}

pub fn wrap_pi(rad: f64) -> f64 {
    if !rad.is_finite() {
        return rad;
    }
    let two_pi = std::f64::consts::PI * 2.0;
    let mut x = rad.rem_euclid(two_pi);
    if x > std::f64::consts::PI {
        x -= two_pi;
    }
    x
}

pub fn yaw_from_quat_wxyz(q: [f64; 4]) -> f64 {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

fn hypot2(v: [f64; 2]) -> f64 {
    (v[0] * v[0] + v[1] * v[1]).sqrt()
}

fn closest_point_in_region(xy: [f64; 2], region: PlanarRegion) -> [f64; 2] {
    let hx = region.half_extents_xy[0].abs();
    let hy = region.half_extents_xy[1].abs();
    [
        xy[0].clamp(region.center_xy[0] - hx, region.center_xy[0] + hx),
        xy[1].clamp(region.center_xy[1] - hy, region.center_xy[1] + hy),
    ]
}

fn translation_target(xy: [f64; 2], goal: &PlanarObjectGoal) -> Option<[f64; 2]> {
    if let Some(region) = goal.target_xy_region {
        Some(closest_point_in_region(xy, region))
    } else {
        goal.target_xy
    }
}

fn signed_yaw_to_goal(yaw: f64, goal: &PlanarObjectGoal) -> f64 {
    if let Some([lo, hi]) = goal.target_yaw_interval {
        let y = wrap_pi(yaw);
        let a = wrap_pi(lo);
        let b = wrap_pi(hi);
        let (lo_w, hi_w) = if a <= b { (a, b) } else { (b, a) };
        if y >= lo_w && y <= hi_w {
            return 0.0;
        }
        let d_lo = wrap_pi(y - lo_w);
        let d_hi = wrap_pi(y - hi_w);
        if d_lo.abs() <= d_hi.abs() {
            d_lo
        } else {
            d_hi
        }
    } else if let Some(t) = goal.target_yaw {
        wrap_pi(yaw - t)
    } else {
        0.0
    }
}

pub fn evaluate_goal_error(xy: [f64; 2], yaw: f64, goal: &PlanarObjectGoal) -> GoalError {
    let t_active = goal.translation_active();
    let o_active = goal.orientation_active();
    let (err_xy, t_res) = if t_active {
        let tgt = translation_target(xy, goal).unwrap_or(xy);
        let e = [xy[0] - tgt[0], xy[1] - tgt[1]];
        (e, hypot2(e))
    } else {
        ([0.0, 0.0], 0.0)
    };
    let yaw_err = if o_active {
        signed_yaw_to_goal(yaw, goal)
    } else {
        0.0
    };
    let t_tol = goal.translation_tolerance_m.max(1e-12);
    let o_tol = goal.orientation_tolerance_rad.max(1e-12);
    let t_n = if t_active { t_res / t_tol } else { 0.0 };
    let o_n = if o_active { yaw_err.abs() / o_tol } else { 0.0 };
    let combined = match (t_active, o_active) {
        (true, true) => (t_n * t_n + o_n * o_n).sqrt(),
        (true, false) => t_n,
        (false, true) => o_n,
        (false, false) => 0.0,
    };
    let t_ok = !t_active || t_res <= goal.translation_tolerance_m;
    let o_ok = !o_active || yaw_err.abs() <= goal.orientation_tolerance_rad;
    GoalError {
        translation_residual_m: t_res,
        orientation_residual_rad: yaw_err.abs(),
        translation_normalized: t_n,
        orientation_normalized: o_n,
        combined,
        translation_error_xy: err_xy,
        signed_yaw_error: yaw_err,
        translation_active: t_active,
        orientation_active: o_active,
        reached: t_ok && o_ok && (t_active || o_active),
    }
}

pub fn twist_in_world(twist: PlanarTwist, object_yaw: f64) -> PlanarTwist {
    match twist.frame {
        PlanarFrameKind::World => twist,
        PlanarFrameKind::Object | PlanarFrameKind::Contact => rotate_twist_world(twist, object_yaw),
    }
}

/// Directional derivative of the explicit normalized goal-error.
pub fn predicted_error_derivative(
    error: &GoalError,
    twist: PlanarTwist,
    goal: &PlanarObjectGoal,
    object_yaw: f64,
) -> Option<f64> {
    if !twist.vx.is_finite() || !twist.vy.is_finite() || !twist.omega_z.is_finite() {
        return None;
    }
    let w = twist_in_world(twist, object_yaw);
    let t_tol = goal.translation_tolerance_m.max(1e-12);
    let o_tol = goal.orientation_tolerance_rad.max(1e-12);
    let de_t = if error.translation_active {
        let r = error.translation_residual_m;
        if r < 1e-12 {
            0.0
        } else {
            (error.translation_error_xy[0] * w.vx + error.translation_error_xy[1] * w.vy)
                / (r * t_tol)
        }
    } else {
        0.0
    };
    let de_o = if error.orientation_active {
        if error.orientation_residual_rad < 1e-12 {
            0.0
        } else {
            error.signed_yaw_error.signum() * w.omega_z / o_tol
        }
    } else {
        0.0
    };
    match (error.translation_active, error.orientation_active) {
        (true, true) => {
            if error.combined < 1e-12 {
                Some(0.0)
            } else {
                Some(
                    (error.translation_normalized * de_t + error.orientation_normalized * de_o)
                        / error.combined,
                )
            }
        }
        (true, false) => Some(de_t),
        (false, true) => Some(de_o),
        (false, false) => None,
    }
}

pub fn classify_predicted_progress(
    error: &GoalError,
    twist: Option<PlanarTwist>,
    goal: &PlanarObjectGoal,
    object_yaw: f64,
) -> GoalProgressClass {
    let Some(tw) = twist else {
        return GoalProgressClass::Ambiguous;
    };
    let Some(de) = predicted_error_derivative(error, tw, goal, object_yaw) else {
        return GoalProgressClass::Ambiguous;
    };
    if de < -PROGRESS_EPS {
        GoalProgressClass::StrictProgress
    } else if de > PROGRESS_EPS {
        GoalProgressClass::Regression
    } else {
        GoalProgressClass::Neutral
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn trans_goal(target: [f64; 2], tol: f64) -> PlanarObjectGoal {
        PlanarObjectGoal {
            object_id: "obj0".into(),
            world_id: "world0".into(),
            model_id: "model0".into(),
            target_xy: Some(target),
            target_xy_region: None,
            target_yaw: None,
            target_yaw_interval: None,
            translation_tolerance_m: tol,
            orientation_tolerance_rad: 0.1,
            freshness_s: 1.0,
            allowed_interaction_family: InteractionFamily::PlanarPush,
            safety: SafetyConstraints::default(),
            max_bounded_attempts: 8,
        }
    }

    fn yaw_goal(target: f64, tol: f64) -> PlanarObjectGoal {
        let mut g = trans_goal([0.0, 0.0], 0.1);
        g.target_xy = None;
        g.target_yaw = Some(target);
        g.orientation_tolerance_rad = tol;
        g
    }

    fn both_goal(xy: [f64; 2], yaw: f64) -> PlanarObjectGoal {
        let mut g = trans_goal(xy, 0.02);
        g.target_yaw = Some(yaw);
        g.orientation_tolerance_rad = 0.1;
        g
    }

    fn tw(vx: f64, vy: f64, w: f64) -> PlanarTwist {
        PlanarTwist {
            vx,
            vy,
            omega_z: w,
            frame: PlanarFrameKind::World,
        }
    }

    #[test]
    fn translation_twist_toward_target_is_strict_progress() {
        let g = trans_goal([0.10, 0.0], 0.01);
        let e = evaluate_goal_error([0.0, 0.0], 0.0, &g);
        assert!((e.translation_normalized - 10.0).abs() < 1e-9);
        assert!(!e.orientation_active);
        assert_eq!(
            classify_predicted_progress(&e, Some(tw(1.0, 0.0, 0.0)), &g, 0.0),
            GoalProgressClass::StrictProgress
        );
        assert_eq!(
            classify_predicted_progress(&e, Some(tw(-1.0, 0.0, 0.0)), &g, 0.0),
            GoalProgressClass::Regression
        );
        assert_eq!(
            classify_predicted_progress(&e, Some(tw(0.0, 1.0, 0.0)), &g, 0.0),
            GoalProgressClass::Neutral
        );
        assert_eq!(
            classify_predicted_progress(&e, None, &g, 0.0),
            GoalProgressClass::Ambiguous
        );
    }

    #[test]
    fn yaw_progress_uses_signed_wrap_not_a_hidden_weight() {
        let g = yaw_goal(0.5, 0.1);
        let e = evaluate_goal_error([0.0, 0.0], 0.0, &g);
        assert!(e.signed_yaw_error < 0.0);
        assert_eq!(
            classify_predicted_progress(&e, Some(tw(0.0, 0.0, 1.0)), &g, 0.0),
            GoalProgressClass::StrictProgress
        );
        assert_eq!(
            classify_predicted_progress(&e, Some(tw(0.0, 0.0, -1.0)), &g, 0.0),
            GoalProgressClass::Regression
        );
    }

    #[test]
    fn combined_metric_is_explicit_hypot_of_normalized_residuals() {
        let g = both_goal([0.04, 0.0], 0.2);
        let e = evaluate_goal_error([0.0, 0.0], 0.0, &g);
        let expect = (e.translation_normalized.powi(2) + e.orientation_normalized.powi(2)).sqrt();
        assert!((e.combined - expect).abs() < 1e-12);
        assert!(!e.reached);
        let reached = evaluate_goal_error([0.04, 0.0], 0.2, &g);
        assert!(reached.reached);
    }

    #[test]
    fn region_goal_is_zero_inside_and_progress_toward_it() {
        let mut g = trans_goal([0.0, 0.0], 0.01);
        g.target_xy = None;
        g.target_xy_region = Some(PlanarRegion {
            center_xy: [0.2, 0.0],
            half_extents_xy: [0.05, 0.05],
        });
        let inside = evaluate_goal_error([0.18, 0.01], 0.0, &g);
        assert!(inside.reached);
        assert!(inside.translation_residual_m < 1e-12);
        let outside = evaluate_goal_error([0.0, 0.0], 0.0, &g);
        assert!(!outside.reached);
        assert_eq!(
            classify_predicted_progress(&outside, Some(tw(1.0, 0.0, 0.0)), &g, 0.0),
            GoalProgressClass::StrictProgress
        );
    }

    #[test]
    fn object_frame_twist_is_rotated_into_world() {
        let g = trans_goal([1.0, 0.0], 0.05);
        let e = evaluate_goal_error([0.0, 0.0], std::f64::consts::FRAC_PI_2, &g);
        let obj = PlanarTwist {
            vx: 1.0,
            vy: 0.0,
            omega_z: 0.0,
            frame: PlanarFrameKind::Object,
        };
        // Object +X at yaw=π/2 is world +Y, which is perpendicular to the +X target.
        assert_eq!(
            classify_predicted_progress(&e, Some(obj), &g, std::f64::consts::FRAC_PI_2),
            GoalProgressClass::Neutral
        );
    }
}
