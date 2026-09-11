use crate::provenance::Provenanced;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Se3 {
    pub xyz: [f64; 3],
    pub quat_wxyz: [f64; 4],
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum TransformError {
    #[error("calibration epoch mismatch")]
    EpochMismatch,
    #[error("non-finite translation")]
    NonFiniteTranslation,
    #[error("non-finite rotation")]
    NonFiniteRotation,
    #[error("invalid rotation")]
    InvalidRotation,
    #[error("transform path disconnected")]
    Disconnected,
    #[error("transform graph cycle")]
    Cycle,
    #[error("stale transform")]
    Stale,
    #[error("missing pose value")]
    MissingPose,
}

impl Se3 {
    pub fn identity() -> Self {
        Self {
            xyz: [0.0, 0.0, 0.0],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        }
    }

    pub fn translation(xyz: [f64; 3]) -> Result<Self, TransformError> {
        Self::try_new(xyz, [1.0, 0.0, 0.0, 0.0])
    }

    pub fn try_new(xyz: [f64; 3], quat_wxyz: [f64; 4]) -> Result<Self, TransformError> {
        if !xyz.iter().all(|v| v.is_finite()) {
            return Err(TransformError::NonFiniteTranslation);
        }
        if !quat_wxyz.iter().all(|v| v.is_finite()) {
            return Err(TransformError::NonFiniteRotation);
        }
        let n = quat_norm(quat_wxyz);
        if n < 1e-12 {
            return Err(TransformError::InvalidRotation);
        }
        Ok(Self {
            xyz,
            quat_wxyz: [
                quat_wxyz[0] / n,
                quat_wxyz[1] / n,
                quat_wxyz[2] / n,
                quat_wxyz[3] / n,
            ],
        })
    }

    pub fn from_axis_angle(axis: [f64; 3], angle: f64) -> Result<Self, TransformError> {
        if !axis.iter().all(|v| v.is_finite()) || !angle.is_finite() {
            return Err(TransformError::NonFiniteRotation);
        }
        let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        if n < 1e-12 {
            return Err(TransformError::InvalidRotation);
        }
        let a = [axis[0] / n, axis[1] / n, axis[2] / n];
        let half = angle * 0.5;
        let s = half.sin();
        Self::try_new([0.0, 0.0, 0.0], [half.cos(), a[0] * s, a[1] * s, a[2] * s])
    }

    pub fn validate(self) -> Result<Self, TransformError> {
        Self::try_new(self.xyz, self.quat_wxyz)
    }

    pub fn inverse(self) -> Self {
        let qinv = quat_conj(self.quat_wxyz);
        let xyz = scale3(rotate_by_quat(qinv, self.xyz), -1.0);
        Self {
            xyz,
            quat_wxyz: qinv,
        }
    }

    /// Compose `self * other`: apply `other` first, then `self` (parent * child).
    pub fn compose(self, other: Self) -> Self {
        let quat_wxyz = quat_mul(self.quat_wxyz, other.quat_wxyz);
        let xyz = add3(self.xyz, rotate_by_quat(self.quat_wxyz, other.xyz));
        Self { xyz, quat_wxyz }
    }

    pub fn rotate(self, v: [f64; 3]) -> [f64; 3] {
        rotate_by_quat(self.quat_wxyz, v)
    }

    pub fn transform_point(self, v: [f64; 3]) -> [f64; 3] {
        add3(self.xyz, self.rotate(v))
    }

    pub fn rotation_matrix(self) -> [[f64; 3]; 3] {
        quat_to_mat(self.quat_wxyz)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TransformEdge {
    pub parent: String,
    pub child: String,
    pub pose: Provenanced<Se3>,
    pub timestamp_s: f64,
    pub calibration_epoch: String,
    pub source: String,
}

impl TransformEdge {
    pub fn identity(
        parent: impl Into<String>,
        child: impl Into<String>,
        epoch: impl Into<String>,
        timestamp_s: f64,
    ) -> Self {
        let parent = parent.into();
        let child = child.into();
        let source = format!("transform.{parent}.{child}");
        Self {
            parent,
            child,
            pose: Provenanced::declared(Se3::identity(), &source, timestamp_s),
            timestamp_s,
            calibration_epoch: epoch.into(),
            source,
        }
    }

    pub fn from_se3(
        parent: impl Into<String>,
        child: impl Into<String>,
        pose: Se3,
        epoch: impl Into<String>,
        timestamp_s: f64,
        source: impl Into<String>,
    ) -> Self {
        let source = source.into();
        Self {
            parent: parent.into(),
            child: child.into(),
            pose: Provenanced::declared(pose, &source, timestamp_s),
            timestamp_s,
            calibration_epoch: epoch.into(),
            source,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TransformGraph {
    pub calibration_epoch: String,
    edges: Vec<TransformEdge>,
}

impl TransformGraph {
    pub fn new(epoch: impl Into<String>) -> Self {
        Self {
            calibration_epoch: epoch.into(),
            edges: Vec::new(),
        }
    }

    pub fn edges(&self) -> &[TransformEdge] {
        &self.edges
    }

    pub fn insert(&mut self, edge: TransformEdge) -> Result<(), TransformError> {
        if edge.calibration_epoch != self.calibration_epoch {
            return Err(TransformError::EpochMismatch);
        }
        for existing in &self.edges {
            if existing.calibration_epoch != edge.calibration_epoch {
                return Err(TransformError::EpochMismatch);
            }
        }
        self.edges.push(edge);
        Ok(())
    }

    pub fn lookup(&self, parent: &str, child: &str) -> Option<&TransformEdge> {
        self.edges
            .iter()
            .find(|e| e.parent == parent && e.child == child)
    }

    pub fn inverse_transform(&self, parent: &str, child: &str) -> Result<Se3, TransformError> {
        let edge = self
            .lookup(parent, child)
            .ok_or(TransformError::Disconnected)?;
        let pose = edge.pose.value.ok_or(TransformError::MissingPose)?;
        Ok(pose.inverse())
    }

    pub fn resolve(
        &self,
        from: &str,
        to: &str,
        now_s: f64,
        freshness_s: Option<f64>,
    ) -> Result<Se3, TransformError> {
        if from == to {
            return Ok(Se3::identity());
        }
        let path = self.path(from, to)?;
        let mut acc = Se3::identity();
        let mut cur = from;
        for hop in &path {
            if hop.calibration_epoch != self.calibration_epoch {
                return Err(TransformError::EpochMismatch);
            }
            if let Some(fresh) = freshness_s {
                if now_s - hop.timestamp_s > fresh {
                    return Err(TransformError::Stale);
                }
            }
            let pose = hop.pose.value.ok_or(TransformError::MissingPose)?;
            if hop.parent == cur && hop.child != cur {
                acc = acc.compose(pose);
                cur = hop.child.as_str();
            } else if hop.child == cur && hop.parent != cur {
                acc = acc.compose(pose.inverse());
                cur = hop.parent.as_str();
            } else {
                return Err(TransformError::Disconnected);
            }
        }
        if cur != to {
            return Err(TransformError::Disconnected);
        }
        Ok(acc)
    }

    pub fn diagnose(&self) -> Vec<String> {
        let mut out = Vec::new();
        let mut names = Vec::new();
        for e in &self.edges {
            if !names.iter().any(|n| n == &e.parent) {
                names.push(e.parent.clone());
            }
            if !names.iter().any(|n| n == &e.child) {
                names.push(e.child.clone());
            }
        }
        for a in &names {
            for b in &names {
                if a == b {
                    continue;
                }
                match self.path(a, b) {
                    Err(TransformError::Disconnected) => {
                        out.push(format!("disconnected:{a}->{b}"));
                    }
                    Err(TransformError::Cycle) => out.push(format!("cycle:{a}->{b}")),
                    _ => {}
                }
            }
        }
        out
    }

    fn path(&self, from: &str, to: &str) -> Result<Vec<TransformEdge>, TransformError> {
        if from == to {
            return Ok(Vec::new());
        }
        let mut visited = vec![from.to_string()];
        let mut queue = vec![(from.to_string(), Vec::new())];
        let mut qi = 0;
        while qi < queue.len() {
            let (cur, trail) = queue[qi].clone();
            qi += 1;
            if trail.len() > self.edges.len() + 1 {
                return Err(TransformError::Cycle);
            }
            for edge in &self.edges {
                let nxt = if edge.parent == cur {
                    Some(edge.child.clone())
                } else if edge.child == cur {
                    Some(edge.parent.clone())
                } else {
                    None
                };
                let Some(nxt) = nxt else { continue };
                if visited.iter().any(|v| v == &nxt) {
                    continue;
                }
                let mut next_trail = trail.clone();
                next_trail.push(edge.clone());
                if nxt == to {
                    return Ok(next_trail);
                }
                visited.push(nxt.clone());
                queue.push((nxt, next_trail));
            }
        }
        Err(TransformError::Disconnected)
    }
}

pub fn quat_norm(q: [f64; 4]) -> f64 {
    (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt()
}

pub fn quat_conj(q: [f64; 4]) -> [f64; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

pub fn quat_mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

pub fn rotate_by_quat(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let qv = [0.0, v[0], v[1], v[2]];
    let r = quat_mul(quat_mul(q, qv), quat_conj(q));
    [r[1], r[2], r[3]]
}

pub fn quat_to_mat(q: [f64; 4]) -> [[f64; 3]; 3] {
    let w = q[0];
    let x = q[1];
    let y = q[2];
    let z = q[3];
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

pub fn add3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn scale3(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub fn norm3(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

pub fn normalize3(v: [f64; 3]) -> Option<[f64; 3]> {
    let n = norm3(v);
    if n < 1e-12 {
        None
    } else {
        Some([v[0] / n, v[1] / n, v[2] / n])
    }
}

pub fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn refuses_to_mix_calibration_epochs() {
        let mut g = TransformGraph::new("e0");
        g.insert(TransformEdge::identity("world", "base", "e0", 1.0))
            .unwrap();
        let err = g
            .insert(TransformEdge::identity("base", "ee", "e1", 1.0))
            .unwrap_err();
        assert_eq!(err, TransformError::EpochMismatch);
    }

    #[test]
    fn compose_and_inverse_roundtrip() {
        let a = Se3::try_new([0.1, -0.2, 0.3], [0.9238795, 0.0, 0.3826834, 0.0]).unwrap();
        let b = Se3::from_axis_angle([0.0, 0.0, 1.0], 0.4).unwrap();
        let c = a.compose(b);
        let back = c.compose(b.inverse());
        assert!(norm3(sub3(back.xyz, a.xyz)) < 1e-9);
    }

    #[test]
    fn path_world_base_shoulder_tool_and_reverse() {
        let mut g = TransformGraph::new("e0");
        let world_base = Se3::try_new([1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]).unwrap();
        let base_shoulder = Se3::try_new([0.0, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0]).unwrap();
        let shoulder_tool = Se3::try_new([0.3, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]).unwrap();
        g.insert(TransformEdge::from_se3(
            "world", "base", world_base, "e0", 1.0, "test",
        ))
        .unwrap();
        g.insert(TransformEdge::from_se3(
            "base",
            "shoulder",
            base_shoulder,
            "e0",
            1.0,
            "test",
        ))
        .unwrap();
        g.insert(TransformEdge::from_se3(
            "shoulder",
            "tool",
            shoulder_tool,
            "e0",
            1.0,
            "test",
        ))
        .unwrap();

        let fwd = g.resolve("world", "tool", 1.0, Some(1.0)).unwrap();
        assert!((fwd.xyz[0] - 1.3).abs() < 1e-12);
        assert!((fwd.xyz[2] - 0.2).abs() < 1e-12);
        let rev = g.resolve("tool", "world", 1.0, Some(1.0)).unwrap();
        let origin = rev.transform_point(fwd.xyz);
        assert!(norm3(origin) < 1e-12);
    }

    #[test]
    fn rotated_base_is_not_subtraction() {
        let yaw = Se3::from_axis_angle([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_2).unwrap();
        let world_base = Se3::try_new([1.0, 0.0, 0.0], yaw.quat_wxyz).unwrap();
        let mut g = TransformGraph::new("e0");
        g.insert(TransformEdge::from_se3(
            "world", "base", world_base, "e0", 1.0, "test",
        ))
        .unwrap();
        let world_target = [1.0, 1.0, 0.0];
        let t_base_from_world = g.resolve("world", "base", 1.0, None).unwrap().inverse();
        let in_base = t_base_from_world.transform_point(world_target);
        let subtraction_only = [world_target[0] - 1.0, world_target[1] - 0.0, 0.0];
        assert!(
            norm3(sub3(in_base, subtraction_only)) > 0.5,
            "subtraction-only must fail the rotated-base case, got {in_base:?} vs {subtraction_only:?}"
        );
        assert!(
            (in_base[0] - 1.0).abs() < 1e-9,
            "expected +X in base, got {in_base:?}"
        );
        assert!(in_base[1].abs() < 1e-9);
    }

    #[test]
    fn zero_axis_angle_is_invalid() {
        assert_eq!(
            Se3::from_axis_angle([0.0, 0.0, 0.0], 1.0).unwrap_err(),
            TransformError::InvalidRotation
        );
    }

    #[test]
    fn stale_edge_is_refused() {
        let mut g = TransformGraph::new("e0");
        g.insert(TransformEdge::identity("world", "base", "e0", 1.0))
            .unwrap();
        let err = g.resolve("world", "base", 3.0, Some(0.25)).unwrap_err();
        assert_eq!(err, TransformError::Stale);
    }

    #[test]
    fn unknown_pose_is_not_identity() {
        let mut g = TransformGraph::new("e0");
        let mut e = TransformEdge::identity("world", "base", "e0", 1.0);
        e.pose = Provenanced::unknown("test", 1.0);
        g.insert(e).unwrap();
        assert_eq!(
            g.resolve("world", "base", 1.0, None).unwrap_err(),
            TransformError::MissingPose
        );
    }
}
