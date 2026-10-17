//! Typed simulation-only force/torque sensor profiles and MuJoCo output parsing.
//!
//! These observations are simulation evidence only and must never be promoted
//! to hardware provenance.

use realityos_kernel::physical_action::WrenchLimitPolicy;
use realityos_semantics::observation::{wrench_sample_digest, WrenchSample};
use realityos_semantics::sensor::WrenchSampleOrigin;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum WrenchSensorError {
    #[error("invalid simulation wrench sensor profile: {0}")]
    Profile(&'static str),
    #[error("invalid simulation wrench worker response: {0}")]
    Worker(&'static str),
    #[error("invalid simulation wrench sample: {0}")]
    Sample(&'static str),
    #[error("wrench profile serialization failed: {0}")]
    Serialization(#[from] serde_json::Error),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SimulationWrenchSensorProfile {
    pub sensor_id: String,
    pub model_id: String,
    pub calibration_hash: String,
    pub sensor_frame: String,
    pub units: String,
    pub sample_rate_hz: f64,
    pub latency_s: f64,
    pub resolution: [f64; 6],
    pub bias: [f64; 6],
    pub error_bounds: [f64; 6],
    pub seeded_noise_bound: [f64; 6],
    pub limit_policy: WrenchLimitPolicy,
}

impl SimulationWrenchSensorProfile {
    pub fn validate(&self) -> Result<(), WrenchSensorError> {
        if self.sensor_id.trim().is_empty()
            || self.model_id.trim().is_empty()
            || self.sensor_frame.trim().is_empty()
        {
            return Err(WrenchSensorError::Profile("identity"));
        }
        if self.units != "N,N*m" {
            return Err(WrenchSensorError::Profile("units"));
        }
        if self.calibration_hash.len() != 64
            || !self
                .calibration_hash
                .bytes()
                .all(|byte| byte.is_ascii_hexdigit())
        {
            return Err(WrenchSensorError::Profile("calibration_hash"));
        }
        if !self.sample_rate_hz.is_finite() || self.sample_rate_hz <= 0.0 {
            return Err(WrenchSensorError::Profile("sample_rate_hz"));
        }
        if !self.latency_s.is_finite() || self.latency_s < 0.0 {
            return Err(WrenchSensorError::Profile("latency_s"));
        }
        for axis in 0..6 {
            if !self.resolution[axis].is_finite() || self.resolution[axis] <= 0.0 {
                return Err(WrenchSensorError::Profile("resolution"));
            }
            if !self.bias[axis].is_finite() {
                return Err(WrenchSensorError::Profile("bias"));
            }
            if !self.error_bounds[axis].is_finite() || self.error_bounds[axis] < 0.0 {
                return Err(WrenchSensorError::Profile("error_bounds"));
            }
            if !self.seeded_noise_bound[axis].is_finite()
                || self.seeded_noise_bound[axis] < 0.0
                || self.seeded_noise_bound[axis] + self.resolution[axis] * 0.5
                    > self.error_bounds[axis] + f64::EPSILON
            {
                return Err(WrenchSensorError::Profile("seeded_noise_bound"));
            }
        }
        self.limit_policy
            .validate()
            .map_err(|_| WrenchSensorError::Profile("limit_policy"))?;
        if self.limit_policy.scope.sensor_id != self.sensor_id
            || self.limit_policy.scope.sensor_frame != self.sensor_frame
            || self.limit_policy.sensor_calibration_hash != self.calibration_hash
            || self.limit_policy.sensor_sample_rate_hz.to_bits() != self.sample_rate_hz.to_bits()
            || self.limit_policy.max_sensing_latency_s.to_bits() != self.latency_s.to_bits()
        {
            return Err(WrenchSensorError::Profile("policy_scope_mismatch"));
        }
        let period = 1.0 / self.sample_rate_hz;
        if self.limit_policy.max_inter_sample_gap_s < period {
            return Err(WrenchSensorError::Profile("sample_gap_below_period"));
        }
        Ok(())
    }

    /// Require exact integration-step representation for sampling and declared
    /// sensing limits. Silently rounding any of these values would change the
    /// policy that the authority digest approved.
    pub fn validate_for_timestep(&self, timestep_s: f64) -> Result<(), WrenchSensorError> {
        self.validate()?;
        if !timestep_s.is_finite() || timestep_s <= 0.0 {
            return Err(WrenchSensorError::Profile("model_timestep"));
        }
        let sample_period = 1.0 / self.sample_rate_hz;
        if !is_integral_step_count(sample_period / timestep_s) {
            return Err(WrenchSensorError::Profile("sample_rate_not_representable"));
        }
        if !is_integral_step_count(self.limit_policy.max_inter_sample_gap_s / timestep_s) {
            return Err(WrenchSensorError::Profile("max_gap_not_representable"));
        }
        if self.latency_s > 0.0 && !is_integral_step_count(self.latency_s / timestep_s) {
            return Err(WrenchSensorError::Profile("latency_not_representable"));
        }
        Ok(())
    }

    pub fn sample_interval_steps(&self, timestep_s: f64) -> Result<u32, WrenchSensorError> {
        self.validate_for_timestep(timestep_s)?;
        let steps = (1.0 / self.sample_rate_hz / timestep_s).round();
        u32::try_from(steps as u64).map_err(|_| WrenchSensorError::Profile("sample_interval_steps"))
    }

    pub fn digest(&self) -> Result<String, WrenchSensorError> {
        self.validate()?;
        fn write_string(hash: &mut Sha256, value: &str) {
            hash.update((value.len() as u64).to_le_bytes());
            hash.update(value.as_bytes());
        }
        fn write_f64(hash: &mut Sha256, value: f64) {
            hash.update(value.to_bits().to_le_bytes());
        }

        let mut hash = Sha256::new();
        hash.update(b"realityos.simulation_wrench_sensor_profile/1\0");
        write_string(&mut hash, &self.sensor_id);
        write_string(&mut hash, &self.model_id);
        write_string(&mut hash, &self.calibration_hash);
        write_string(&mut hash, &self.sensor_frame);
        write_string(&mut hash, &self.units);
        write_f64(&mut hash, self.sample_rate_hz);
        write_f64(&mut hash, self.latency_s);
        for value in self.resolution {
            write_f64(&mut hash, value);
        }
        for value in self.bias {
            write_f64(&mut hash, value);
        }
        for value in self.error_bounds {
            write_f64(&mut hash, value);
        }
        for value in self.seeded_noise_bound {
            write_f64(&mut hash, value);
        }
        write_string(&mut hash, &self.limit_policy.digest());
        Ok(hex::encode(hash.finalize()))
    }
}

fn is_integral_step_count(value: f64) -> bool {
    value.is_finite()
        && value >= 1.0
        && (value - value.round()).abs() <= 1e-9 * value.abs().max(1.0)
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SimulationWrenchWindow {
    pub samples: Vec<WrenchSample>,
    pub max_gap_s: f64,
    pub compensation_digest: String,
}

impl SimulationWrenchWindow {
    /// Convert the worker's identity-free numeric samples into typed simulation
    /// evidence. Body/geom/contact IDs are deliberately absent from this wire
    /// projection.
    pub fn from_worker_response(
        response: &Value,
        profile: &SimulationWrenchSensorProfile,
        transform_epoch: &str,
    ) -> Result<Self, WrenchSensorError> {
        profile.validate()?;
        if transform_epoch.trim().is_empty() {
            return Err(WrenchSensorError::Sample("transform_epoch"));
        }
        if response["ok"] != true {
            return Err(WrenchSensorError::Worker("not_ok"));
        }
        let records = response["samples"]
            .as_array()
            .ok_or(WrenchSensorError::Worker("samples_missing"))?;
        let mut samples = Vec::with_capacity(records.len());
        let mut max_gap_s: f64 = 0.0;
        let mut previous_capture: Option<f64> = None;
        let mut previous_sequence: Option<u64> = None;
        for record in records {
            let sequence = record["sequence"]
                .as_u64()
                .ok_or(WrenchSensorError::Worker("sequence"))?;
            let capture_s = record["capture_s"]
                .as_f64()
                .ok_or(WrenchSensorError::Worker("capture_s"))?;
            let receive_s = record["receive_s"]
                .as_f64()
                .ok_or(WrenchSensorError::Worker("receive_s"))?;
            let wrench = parse_six(&record["wrench"]).ok_or(WrenchSensorError::Worker("wrench"))?;
            let error_bounds = parse_six(&record["error_bounds"])
                .ok_or(WrenchSensorError::Worker("error_bounds"))?;
            if error_bounds
                .iter()
                .zip(profile.error_bounds)
                .any(|(actual, expected)| actual.to_bits() != expected.to_bits())
            {
                return Err(WrenchSensorError::Sample("error_bounds_mismatch"));
            }
            if !capture_s.is_finite()
                || capture_s < 0.0
                || !receive_s.is_finite()
                || (receive_s - capture_s - profile.latency_s).abs() > 1e-9
            {
                return Err(WrenchSensorError::Sample("timestamps"));
            }
            if previous_sequence.is_some_and(|previous| sequence != previous + 1) {
                return Err(WrenchSensorError::Sample("sequence_order"));
            }
            if let Some(previous) = previous_capture {
                let gap = capture_s - previous;
                if !gap.is_finite() || gap <= 0.0 {
                    return Err(WrenchSensorError::Sample("capture_order"));
                }
                max_gap_s = max_gap_s.max(gap);
            }
            let mut sample = WrenchSample {
                sensor_id: profile.sensor_id.clone(),
                sensor_frame: profile.sensor_frame.clone(),
                transform_epoch: transform_epoch.into(),
                units: profile.units.clone(),
                wrench,
                error_bounds,
                capture_s,
                receive_s,
                sequence,
                clock_domain: "mujoco_simulation_clock".into(),
                origin: WrenchSampleOrigin::Simulation,
                digest: String::new(),
            };
            sample.digest = wrench_sample_digest(&sample);
            sample.validate().map_err(WrenchSensorError::Sample)?;
            samples.push(sample);
            previous_capture = Some(capture_s);
            previous_sequence = Some(sequence);
        }
        Ok(Self {
            samples,
            max_gap_s,
            compensation_digest: profile.limit_policy.compensation_digest.clone(),
        })
    }
}

fn parse_six(value: &Value) -> Option<[f64; 6]> {
    let values = value.as_array()?;
    if values.len() != 6 {
        return None;
    }
    let mut out = [0.0; 6];
    for (index, value) in values.iter().enumerate() {
        out[index] = value.as_f64()?;
        if !out[index].is_finite() {
            return None;
        }
    }
    Some(out)
}

/// The contact frame axes are stored as rows in MuJoCo's transposed
/// `mjContact.frame` representation. The contact force points from geom 1 to
/// geom 2; moments are shifted from the contact point to the sensor origin.
pub fn transform_contact_wrench_world(
    contact_frame_rows: [[f64; 3]; 3],
    contact_wrench: [f64; 6],
    contact_position_world: [f64; 3],
    sensor_origin_world: [f64; 3],
    force_on_geom2: bool,
) -> [f64; 6] {
    let sign = if force_on_geom2 { 1.0 } else { -1.0 };
    let mut force_world = [0.0; 3];
    let mut torque_world = [0.0; 3];
    for axis in 0..3 {
        for world_axis in 0..3 {
            force_world[world_axis] +=
                sign * contact_frame_rows[axis][world_axis] * contact_wrench[axis];
            torque_world[world_axis] +=
                sign * contact_frame_rows[axis][world_axis] * contact_wrench[axis + 3];
        }
    }
    let r = [
        contact_position_world[0] - sensor_origin_world[0],
        contact_position_world[1] - sensor_origin_world[1],
        contact_position_world[2] - sensor_origin_world[2],
    ];
    let moment_arm = [
        r[1] * force_world[2] - r[2] * force_world[1],
        r[2] * force_world[0] - r[0] * force_world[2],
        r[0] * force_world[1] - r[1] * force_world[0],
    ];
    [
        force_world[0],
        force_world[1],
        force_world[2],
        torque_world[0] + moment_arm[0],
        torque_world[1] + moment_arm[1],
        torque_world[2] + moment_arm[2],
    ]
}

/// Return the signed wrench applied to geom1 (negative) or geom2 (positive).
pub fn contact_force_on_geom(contact_wrench: [f64; 6], is_geom1: bool) -> [f64; 6] {
    let sign = if is_geom1 { -1.0 } else { 1.0 };
    contact_wrench.map(|value| sign * value)
}

/// Sum all contact loads on the instrumented tool subtree, including contacts
/// with bodies that were not the task's expected target.
pub fn aggregate_contact_wrenches(contributions: &[[f64; 6]]) -> [f64; 6] {
    let mut total = [0.0; 6];
    for wrench in contributions {
        for axis in 0..6 {
            total[axis] += wrench[axis];
        }
    }
    total
}
