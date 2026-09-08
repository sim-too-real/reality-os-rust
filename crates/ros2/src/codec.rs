//! Pure codecs. No I/O. No authority. Dict-twin of ROS messages.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointState {
    pub name: Vec<String>,
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
    pub effort: Vec<f64>,
    pub timestamp_s: f64,
}

impl JointState {
    pub fn to_samples(&self) -> Vec<(String, f64)> {
        let mut s = Vec::new();
        for (i, n) in self.name.iter().enumerate() {
            if let Some(p) = self.position.get(i) {
                s.push((format!("{n}_pos"), *p));
            }
            if let Some(v) = self.velocity.get(i) {
                s.push((format!("{n}_vel"), *v));
            }
            if let Some(e) = self.effort.get(i) {
                s.push((format!("{n}_eff"), *e));
            }
        }
        s
    }

    pub fn finite(&self) -> bool {
        self.position
            .iter()
            .chain(&self.velocity)
            .chain(&self.effort)
            .all(|x| x.is_finite())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Wrench {
    pub force_n: [f64; 3],
    pub torque_nm: [f64; 3],
    pub timestamp_s: f64,
    pub frame_id: String,
}

impl Wrench {
    pub fn force_norm_n(&self) -> f64 {
        let [x, y, z] = self.force_n;
        (x * x + y * y + z * z).sqrt()
    }

    pub fn to_samples(&self) -> Vec<(String, f64)> {
        vec![
            ("force_x_n".into(), self.force_n[0]),
            ("force_y_n".into(), self.force_n[1]),
            ("force_z_n".into(), self.force_n[2]),
            ("force_norm_n".into(), self.force_norm_n()),
            ("torque_x_nm".into(), self.torque_nm[0]),
            ("torque_y_nm".into(), self.torque_nm[1]),
            ("torque_z_nm".into(), self.torque_nm[2]),
        ]
    }
}

/// Command the robot already accepts. Published only after Governor write.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointCommand {
    pub position: Vec<f64>,
    pub effort: Vec<f64>,
    pub command_id: String,
    pub metal: bool,
}

impl JointCommand {
    pub fn from_action(command_id: &str, action: &[f64]) -> Self {
        Self {
            position: Vec::new(),
            effort: action.to_vec(),
            command_id: command_id.into(),
            metal: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrench_norm_is_euclidean() {
        let w = Wrench {
            force_n: [3.0, 4.0, 0.0],
            torque_nm: [0.0, 0.0, 0.0],
            timestamp_s: 0.0,
            frame_id: "ft".into(),
        };
        assert!((w.force_norm_n() - 5.0).abs() < 1e-12);
    }
}
