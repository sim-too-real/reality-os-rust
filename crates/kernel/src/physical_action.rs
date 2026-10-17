//! Exact action authority shared by reasoning, grants, and execution.
//!
//! These values describe a bounded transaction. They do not create authority
//! on their own; an issued grant must bind the exact value.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AbortHoldMode {
    HoldCurrentJointPosition,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WrenchLimitScope {
    pub sensor_id: String,
    pub sensor_frame: String,
    pub caps: [f64; 6],
    pub policy_digest: String,
    pub abort_contract_id: String,
}

impl WrenchLimitScope {
    fn validate(&self) -> Result<(), &'static str> {
        if self.sensor_id.trim().is_empty() {
            return Err("wrench_sensor_id");
        }
        if self.sensor_frame.trim().is_empty() {
            return Err("wrench_sensor_frame");
        }
        if self.caps.iter().any(|cap| !cap.is_finite() || *cap <= 0.0) {
            return Err("wrench_caps");
        }
        if !is_sha256(&self.policy_digest) {
            return Err("wrench_policy_digest");
        }
        if self.abort_contract_id.trim().is_empty() {
            return Err("wrench_abort_contract");
        }
        Ok(())
    }

    fn exactly_matches(&self, other: &Self) -> bool {
        self.sensor_id == other.sensor_id
            && self.sensor_frame == other.sensor_frame
            && self
                .caps
                .iter()
                .zip(other.caps)
                .all(|(left, right)| left.to_bits() == right.to_bits())
            && self.policy_digest == other.policy_digest
            && self.abort_contract_id == other.abort_contract_id
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WrenchAbortContract {
    pub contract_id: String,
    pub hold_mode: AbortHoldMode,
    pub allowed_stop_actuator_ids: Vec<String>,
    pub max_time_to_stop_s: f64,
    pub max_stop_writes: u32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WrenchLimitPolicy {
    pub scope: WrenchLimitScope,
    pub sensor_calibration_hash: String,
    pub sensor_sample_rate_hz: f64,
    pub max_rise_rate: [f64; 6],
    pub max_sensing_latency_s: f64,
    pub max_abort_latency_s: f64,
    pub max_sample_age_s: f64,
    pub max_inter_sample_gap_s: f64,
    pub max_cross_sensor_skew_s: f64,
    pub monitor_id: String,
    pub compensation_digest: String,
    pub abort_contract: WrenchAbortContract,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("invalid wrench limit policy: {0}")]
pub struct WrenchPolicyError(pub &'static str);

impl WrenchLimitPolicy {
    pub fn digest(&self) -> String {
        fn write_string(hash: &mut Sha256, value: &str) {
            hash.update((value.len() as u64).to_le_bytes());
            hash.update(value.as_bytes());
        }
        fn write_f64(hash: &mut Sha256, value: f64) {
            hash.update(value.to_bits().to_le_bytes());
        }

        let mut hash = Sha256::new();
        hash.update(b"realityos.wrench_limit_policy/1\0");
        write_string(&mut hash, &self.scope.sensor_id);
        write_string(&mut hash, &self.scope.sensor_frame);
        for cap in self.scope.caps {
            write_f64(&mut hash, cap);
        }
        write_string(&mut hash, &self.scope.abort_contract_id);
        write_string(&mut hash, &self.sensor_calibration_hash);
        write_f64(&mut hash, self.sensor_sample_rate_hz);
        for rise_rate in self.max_rise_rate {
            write_f64(&mut hash, rise_rate);
        }
        write_f64(&mut hash, self.max_sensing_latency_s);
        write_f64(&mut hash, self.max_abort_latency_s);
        write_f64(&mut hash, self.max_sample_age_s);
        write_f64(&mut hash, self.max_inter_sample_gap_s);
        write_f64(&mut hash, self.max_cross_sensor_skew_s);
        write_string(&mut hash, &self.monitor_id);
        write_string(&mut hash, &self.compensation_digest);
        write_string(&mut hash, &self.abort_contract.contract_id);
        match self.abort_contract.hold_mode {
            AbortHoldMode::HoldCurrentJointPosition => hash.update([0]),
        }
        hash.update((self.abort_contract.allowed_stop_actuator_ids.len() as u64).to_le_bytes());
        for actuator_id in &self.abort_contract.allowed_stop_actuator_ids {
            write_string(&mut hash, actuator_id);
        }
        write_f64(&mut hash, self.abort_contract.max_time_to_stop_s);
        hash.update(self.abort_contract.max_stop_writes.to_le_bytes());
        hex::encode(hash.finalize())
    }

    pub fn validate(&self) -> Result<(), WrenchPolicyError> {
        self.scope.validate().map_err(WrenchPolicyError)?;
        if self.sensor_calibration_hash.trim().is_empty() {
            return Err(WrenchPolicyError("sensor_calibration_hash"));
        }
        if !self.sensor_sample_rate_hz.is_finite() || self.sensor_sample_rate_hz <= 0.0 {
            return Err(WrenchPolicyError("sensor_sample_rate_hz"));
        }
        if self
            .max_rise_rate
            .iter()
            .any(|rate| !rate.is_finite() || *rate < 0.0)
        {
            return Err(WrenchPolicyError("max_rise_rate"));
        }
        if !self.max_sensing_latency_s.is_finite() || self.max_sensing_latency_s < 0.0 {
            return Err(WrenchPolicyError("max_sensing_latency_s"));
        }
        if !self.max_abort_latency_s.is_finite() || self.max_abort_latency_s <= 0.0 {
            return Err(WrenchPolicyError("max_abort_latency_s"));
        }
        if !self.max_sample_age_s.is_finite() || self.max_sample_age_s <= 0.0 {
            return Err(WrenchPolicyError("max_sample_age_s"));
        }
        if !self.max_inter_sample_gap_s.is_finite() || self.max_inter_sample_gap_s <= 0.0 {
            return Err(WrenchPolicyError("max_inter_sample_gap_s"));
        }
        if !self.max_cross_sensor_skew_s.is_finite() || self.max_cross_sensor_skew_s < 0.0 {
            return Err(WrenchPolicyError("max_cross_sensor_skew_s"));
        }
        if self.monitor_id.trim().is_empty() {
            return Err(WrenchPolicyError("monitor_id"));
        }
        if !is_sha256(&self.compensation_digest) {
            return Err(WrenchPolicyError("compensation_digest"));
        }
        let contract = &self.abort_contract;
        if contract.contract_id.trim().is_empty()
            || contract.contract_id != self.scope.abort_contract_id
        {
            return Err(WrenchPolicyError("abort_contract_id"));
        }
        if !contract.max_time_to_stop_s.is_finite()
            || contract.max_time_to_stop_s <= 0.0
            || contract.max_time_to_stop_s > self.max_abort_latency_s
            || contract.max_stop_writes == 0
        {
            return Err(WrenchPolicyError("abort_contract_limits"));
        }
        let mut stop_actuators = std::collections::BTreeSet::new();
        if contract.allowed_stop_actuator_ids.is_empty()
            || contract
                .allowed_stop_actuator_ids
                .iter()
                .any(|id| id.trim().is_empty() || !stop_actuators.insert(id))
        {
            return Err(WrenchPolicyError("allowed_stop_actuator_ids"));
        }
        if self.scope.policy_digest != self.digest() {
            return Err(WrenchPolicyError("policy_digest_mismatch"));
        }
        Ok(())
    }
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PhysicalActionEnvelope {
    LinearStroke {
        requested_m: f64,
        maximum_m: f64,
        wrench_limit: Option<WrenchLimitScope>,
    },
    PinchGrasp {
        resource_id: String,
        finger_joint_names: Vec<String>,
        open_positions: Vec<f64>,
        close_positions: Vec<f64>,
        per_finger_force_ceiling_n: f64,
        lift_trajectory_digest: String,
        maximum_lift_m: f64,
    },
}

impl PhysicalActionEnvelope {
    /// Checks the envelope's structure and numeric bounds independent of a model.
    pub fn validate(&self) -> Result<(), &'static str> {
        match self {
            Self::LinearStroke {
                requested_m,
                maximum_m,
                wrench_limit,
            } => {
                if !requested_m.is_finite() || *requested_m <= 0.0 {
                    return Err("requested_stroke");
                }
                if !maximum_m.is_finite() || *maximum_m < *requested_m {
                    return Err("maximum_stroke");
                }
                if let Some(scope) = wrench_limit {
                    scope.validate()?;
                }
            }
            Self::PinchGrasp {
                resource_id,
                finger_joint_names,
                open_positions,
                close_positions,
                per_finger_force_ceiling_n,
                lift_trajectory_digest,
                maximum_lift_m,
            } => {
                if resource_id.trim().is_empty() {
                    return Err("resource_id");
                }
                if finger_joint_names.len() != 2
                    || finger_joint_names.iter().any(|name| name.trim().is_empty())
                    || finger_joint_names[0] == finger_joint_names[1]
                {
                    return Err("finger_joint_names");
                }
                if open_positions.len() != finger_joint_names.len()
                    || close_positions.len() != finger_joint_names.len()
                {
                    return Err("finger_position_dimensions");
                }
                for (&open, &close) in open_positions.iter().zip(close_positions) {
                    if !open.is_finite() || !close.is_finite() || !(close - open).is_finite() {
                        return Err("finger_positions");
                    }
                }
                if !per_finger_force_ceiling_n.is_finite() || *per_finger_force_ceiling_n <= 0.0 {
                    return Err("force_ceiling");
                }
                if lift_trajectory_digest.len() != 64
                    || !lift_trajectory_digest
                        .bytes()
                        .all(|byte| byte.is_ascii_hexdigit())
                {
                    return Err("lift_trajectory_digest");
                }
                if !maximum_lift_m.is_finite() || *maximum_lift_m < 0.02 {
                    return Err("maximum_lift");
                }
            }
        }
        Ok(())
    }

    /// Validation required at any grant or control-write boundary.
    pub fn validate_for_execution(&self) -> Result<(), &'static str> {
        self.validate()?;
        if let Self::LinearStroke { wrench_limit, .. } = self {
            if wrench_limit.is_none() {
                return Err("wrench_limit_required");
            }
        }
        Ok(())
    }

    /// Returns the command and maximum for a linear action, if this is one.
    pub fn linear_stroke_bounds(&self) -> Option<(f64, f64)> {
        match self {
            Self::LinearStroke {
                requested_m,
                maximum_m,
                ..
            } => Some((*requested_m, *maximum_m)),
            Self::PinchGrasp { .. } => None,
        }
    }

    /// Compares the exact structured authority, including each float bit pattern.
    pub fn exactly_matches(&self, other: &Self) -> bool {
        match (self, other) {
            (
                Self::LinearStroke {
                    requested_m: left_requested,
                    maximum_m: left_maximum,
                    wrench_limit: left_wrench_limit,
                },
                Self::LinearStroke {
                    requested_m: right_requested,
                    maximum_m: right_maximum,
                    wrench_limit: right_wrench_limit,
                },
            ) => {
                left_requested.to_bits() == right_requested.to_bits()
                    && left_maximum.to_bits() == right_maximum.to_bits()
                    && match (left_wrench_limit, right_wrench_limit) {
                        (None, None) => true,
                        (Some(left), Some(right)) => left.exactly_matches(right),
                        _ => false,
                    }
            }
            (
                Self::PinchGrasp {
                    resource_id: left_resource,
                    finger_joint_names: left_names,
                    open_positions: left_open,
                    close_positions: left_close,
                    per_finger_force_ceiling_n: left_force,
                    lift_trajectory_digest: left_lift_digest,
                    maximum_lift_m: left_lift,
                },
                Self::PinchGrasp {
                    resource_id: right_resource,
                    finger_joint_names: right_names,
                    open_positions: right_open,
                    close_positions: right_close,
                    per_finger_force_ceiling_n: right_force,
                    lift_trajectory_digest: right_lift_digest,
                    maximum_lift_m: right_lift,
                },
            ) => {
                left_resource == right_resource
                    && left_names == right_names
                    && exact_float_vec(left_open, right_open)
                    && exact_float_vec(left_close, right_close)
                    && left_force.to_bits() == right_force.to_bits()
                    && left_lift_digest == right_lift_digest
                    && left_lift.to_bits() == right_lift.to_bits()
            }
            _ => false,
        }
    }
}

fn exact_float_vec(left: &[f64], right: &[f64]) -> bool {
    left.len() == right.len()
        && left
            .iter()
            .zip(right)
            .all(|(left, right)| left.to_bits() == right.to_bits())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_wrench_policy() -> WrenchLimitPolicy {
        let mut policy = WrenchLimitPolicy {
            scope: WrenchLimitScope {
                sensor_id: "ft-tool".into(),
                sensor_frame: "tool0".into(),
                caps: [20.0, 20.0, 30.0, 2.0, 2.0, 2.0],
                policy_digest: String::new(),
                abort_contract_id: "hold-on-trip".into(),
            },
            sensor_calibration_hash: "sensor-calibration-v1".into(),
            sensor_sample_rate_hz: 125.0,
            max_rise_rate: [10.0; 6],
            max_sensing_latency_s: 0.01,
            max_abort_latency_s: 0.03,
            max_sample_age_s: 0.02,
            max_inter_sample_gap_s: 0.01,
            max_cross_sensor_skew_s: 0.005,
            monitor_id: "wrench-monitor-v1".into(),
            compensation_digest: "c".repeat(64),
            abort_contract: WrenchAbortContract {
                contract_id: "hold-on-trip".into(),
                hold_mode: AbortHoldMode::HoldCurrentJointPosition,
                allowed_stop_actuator_ids: vec!["tool-x".into(), "tool-y".into()],
                max_time_to_stop_s: 0.03,
                max_stop_writes: 2,
            },
        };
        policy.scope.policy_digest = policy.digest();
        policy
    }

    #[test]
    fn linear_stroke_scope_binds_all_six_wrench_caps() {
        let policy = valid_wrench_policy();
        let left = PhysicalActionEnvelope::LinearStroke {
            requested_m: 0.008,
            maximum_m: 0.02,
            wrench_limit: Some(policy.scope.clone()),
        };
        for axis in 0..6 {
            let mut changed_scope = policy.scope.clone();
            changed_scope.caps[axis] = f64::from_bits(changed_scope.caps[axis].to_bits() + 1);
            let right = PhysicalActionEnvelope::LinearStroke {
                requested_m: 0.008,
                maximum_m: 0.02,
                wrench_limit: Some(changed_scope),
            };
            assert!(!left.exactly_matches(&right), "axis {axis} must bind");
        }
        let mut changed_policy_digest = policy.scope.clone();
        let first_digest_char = if changed_policy_digest.policy_digest.starts_with('0') {
            "1"
        } else {
            "0"
        };
        changed_policy_digest
            .policy_digest
            .replace_range(..1, first_digest_char);
        let changed = PhysicalActionEnvelope::LinearStroke {
            requested_m: 0.008,
            maximum_m: 0.02,
            wrench_limit: Some(changed_policy_digest),
        };
        assert!(!left.exactly_matches(&changed));
    }

    #[test]
    fn linear_stroke_rejects_missing_or_invalid_wrench_scope() {
        let missing = PhysicalActionEnvelope::LinearStroke {
            requested_m: 0.008,
            maximum_m: 0.02,
            wrench_limit: None,
        };
        assert!(missing.validate().is_ok());
        assert!(missing.validate_for_execution().is_err());

        let mut invalid_scope = valid_wrench_policy().scope;
        invalid_scope.caps[0] = f64::NAN;
        let invalid = PhysicalActionEnvelope::LinearStroke {
            requested_m: 0.008,
            maximum_m: 0.02,
            wrench_limit: Some(invalid_scope),
        };
        assert!(invalid.validate_for_execution().is_err());
    }

    #[test]
    fn policy_digest_binds_sensor_calibration_rate_latency_rise_rates_and_abort_contract() {
        let baseline = valid_wrench_policy();
        assert!(baseline.validate().is_ok());
        let digest = baseline.digest();

        let mut changed = baseline.clone();
        changed.scope.sensor_frame.push_str("_other");
        assert_ne!(digest, changed.digest());
        for axis in 0..6 {
            let mut changed = baseline.clone();
            changed.scope.caps[axis] = f64::from_bits(changed.scope.caps[axis].to_bits() + 1);
            assert_ne!(digest, changed.digest(), "cap axis {axis} must bind");
        }
        let mut changed = baseline.clone();
        changed.sensor_calibration_hash.push_str("-other");
        assert_ne!(digest, changed.digest());
        let mut changed = baseline.clone();
        changed.sensor_sample_rate_hz += 1.0;
        assert_ne!(digest, changed.digest());
        let mut changed = baseline.clone();
        changed.scope.sensor_id.push_str("-other");
        assert_ne!(digest, changed.digest());
        let mut changed = baseline.clone();
        changed.max_sensing_latency_s = f64::from_bits(changed.max_sensing_latency_s.to_bits() + 1);
        assert_ne!(digest, changed.digest());
        let mut changed = baseline.clone();
        changed.max_abort_latency_s = f64::from_bits(changed.max_abort_latency_s.to_bits() + 1);
        assert_ne!(digest, changed.digest());
        let mut changed = baseline.clone();
        changed.max_sample_age_s = f64::from_bits(changed.max_sample_age_s.to_bits() + 1);
        assert_ne!(digest, changed.digest());
        let mut changed = baseline.clone();
        changed.max_inter_sample_gap_s =
            f64::from_bits(changed.max_inter_sample_gap_s.to_bits() + 1);
        assert_ne!(digest, changed.digest());
        let mut changed = baseline.clone();
        changed.max_cross_sensor_skew_s =
            f64::from_bits(changed.max_cross_sensor_skew_s.to_bits() + 1);
        assert_ne!(digest, changed.digest());
        let mut changed = baseline.clone();
        changed.max_rise_rate[5] = f64::from_bits(changed.max_rise_rate[5].to_bits() + 1);
        assert_ne!(digest, changed.digest());
        let mut changed = baseline.clone();
        changed.monitor_id.push_str("-other");
        assert_ne!(digest, changed.digest());
        let mut changed = baseline.clone();
        changed.compensation_digest.replace_range(..1, "d");
        assert_ne!(digest, changed.digest());
        let mut changed = baseline.clone();
        changed.abort_contract.contract_id.push_str("-other");
        assert_ne!(digest, changed.digest());
        let mut changed = baseline.clone();
        changed
            .abort_contract
            .allowed_stop_actuator_ids
            .push("tool-z".into());
        assert_ne!(digest, changed.digest());
        let mut changed = baseline.clone();
        changed.abort_contract.max_time_to_stop_s =
            f64::from_bits(changed.abort_contract.max_time_to_stop_s.to_bits() + 1);
        assert_ne!(digest, changed.digest());
        let mut changed = baseline;
        changed.abort_contract.max_stop_writes += 1;
        assert_ne!(digest, changed.digest());
    }

    #[test]
    fn abort_contract_rejects_empty_or_duplicate_stop_actuators() {
        let mut policy = valid_wrench_policy();
        policy.abort_contract.allowed_stop_actuator_ids.clear();
        policy.scope.policy_digest = policy.digest();
        assert!(policy.validate().is_err());

        let mut policy = valid_wrench_policy();
        policy
            .abort_contract
            .allowed_stop_actuator_ids
            .push("tool-x".into());
        policy.scope.policy_digest = policy.digest();
        assert!(policy.validate().is_err());
    }

    #[test]
    fn exact_match_distinguishes_signed_zero_in_position_vectors() {
        let left = PhysicalActionEnvelope::PinchGrasp {
            resource_id: "gripper".into(),
            finger_joint_names: vec!["left".into(), "right".into()],
            open_positions: vec![-0.0, 0.0],
            close_positions: vec![0.01, 0.01],
            per_finger_force_ceiling_n: 2.0,
            lift_trajectory_digest: "a".repeat(64),
            maximum_lift_m: 0.02,
        };
        let mut right = left.clone();
        if let PhysicalActionEnvelope::PinchGrasp { open_positions, .. } = &mut right {
            open_positions[0] = 0.0;
        }
        assert!(!left.exactly_matches(&right));
    }
}
