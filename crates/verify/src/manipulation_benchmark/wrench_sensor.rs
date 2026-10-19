//! Typed simulation-only force/torque sensor profiles and MuJoCo output parsing.
//!
//! These observations are simulation evidence only and must never be promoted
//! to hardware provenance.

use realityos_kernel::physical_action::WrenchLimitPolicy;
use realityos_semantics::observation::{wrench_sample_digest, WrenchSample};
use realityos_semantics::push_probe::WrenchCompensation;
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
    /// Worst-case per-axis model uncertainty for the same-state, contact-free
    /// wrench prediction. Zero is only valid for the registered deterministic
    /// MuJoCo model; hardware profiles must use a different model identity.
    #[serde(default)]
    pub free_space_model_error_bounds: [f64; 6],
    pub seeded_noise_bound: [f64; 6],
    pub limit_policy: WrenchLimitPolicy,
}

impl SimulationWrenchSensorProfile {
    pub fn compensation_policy_digest_for(
        model_id: &str,
        compensation: &WrenchCompensation,
    ) -> String {
        let mut hash = Sha256::new();
        hash.update(b"realityos.simulation_wrench_compensation/1\0");
        hash.update((model_id.len() as u64).to_le_bytes());
        hash.update(model_id.as_bytes());
        hash.update((compensation.digest.len() as u64).to_le_bytes());
        hash.update(compensation.digest.as_bytes());
        hex::encode(hash.finalize())
    }

    pub fn compensation_policy_digest(&self) -> String {
        let mut compensation = self.compensation();
        for axis in 0..6 {
            compensation.residual_error[axis] += self.free_space_model_error_bounds[axis];
        }
        compensation.digest = compensation.digest();
        Self::compensation_policy_digest_for(&self.model_id, &compensation)
    }

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
            if !self.free_space_model_error_bounds[axis].is_finite()
                || self.free_space_model_error_bounds[axis] < 0.0
            {
                return Err(WrenchSensorError::Profile("free_space_model_error_bounds"));
            }
            if !self.seeded_noise_bound[axis].is_finite()
                || self.seeded_noise_bound[axis] < 0.0
                || self.seeded_noise_bound[axis] + self.resolution[axis] * 0.5
                    > self.error_bounds[axis] + f64::EPSILON
            {
                return Err(WrenchSensorError::Profile("seeded_noise_bound"));
            }
        }
        if self.model_id != "mujoco-force-torque-sensor-free-space-model-v1"
            && self
                .free_space_model_error_bounds
                .iter()
                .any(|bound| *bound != 0.0)
        {
            return Err(WrenchSensorError::Profile("free_space_model_identity"));
        }
        self.limit_policy
            .validate()
            .map_err(|_| WrenchSensorError::Profile("limit_policy"))?;
        if self.limit_policy.compensation_digest != self.compensation_policy_digest() {
            return Err(WrenchSensorError::Profile("compensation_digest_mismatch"));
        }
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

    /// Static sensor bias and residual uncertainty. The sensor model's dynamic
    /// stationary-load compensation is separately bound by `model_id` in the
    /// policy compensation digest.
    pub fn compensation(&self) -> WrenchCompensation {
        let mut compensation = WrenchCompensation {
            bias: self.bias,
            gravity_wrench: [0.0; 6],
            residual_error: self.error_bounds,
            digest: String::new(),
        };
        compensation.digest = compensation.digest();
        compensation
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
        hash.update(b"realityos.simulation_wrench_sensor_profile/2\0");
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
        for value in self.free_space_model_error_bounds {
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
    #[serde(default)]
    pub sample_free_space_wrenches: Vec<[f64; 6]>,
    /// Per-sample uncertainty paired with `sample_free_space_wrenches`.
    #[serde(default)]
    pub sample_free_space_wrench_error_bounds: Vec<[f64; 6]>,
    #[serde(default)]
    pub sensor_positions: Vec<[f64; 3]>,
    #[serde(default)]
    pub sensor_orientations: Vec<[f64; 4]>,
    #[serde(default)]
    pub precontact_baseline: Option<WrenchSample>,
    #[serde(default)]
    pub precontact_baseline_status: Option<String>,
    #[serde(default)]
    pub precontact_baseline_position: Option<[f64; 3]>,
    #[serde(default)]
    pub precontact_baseline_orientation: Option<[f64; 4]>,
    #[serde(default)]
    pub triggering_sample: Option<WrenchSample>,
    #[serde(default)]
    pub monitor_gravity_reference: Option<WrenchSample>,
    #[serde(default)]
    pub monitor_gravity_reference_status: Option<String>,
    #[serde(default)]
    pub monitor_gravity_reference_position: Option<[f64; 3]>,
    #[serde(default)]
    pub monitor_gravity_reference_orientation: Option<[f64; 4]>,
    pub max_gap_s: f64,
    pub compensation_digest: String,
}

/// Subtract the registered sensor bias and a contact-free stationary load
/// reference from a measured sample. The reference load is attached to the
/// instrumented child subtree, so rotate its force and moment into the current
/// sensor frame without translating its origin with the sensor. The v1 model
/// retains its historical point-wrench translation for old evidence profiles;
/// v2 fixes that model for body-fixed loads. The free-space-model v1 monitor
/// reference keeps those same stationary-load semantics; its separate,
/// per-sample model wrench is used by probe inference. Returned error bounds
/// include both measurements and the selected wrench-frame transform.
pub fn gravity_compensated_wrench(
    sample: &WrenchSample,
    sample_position: [f64; 3],
    sample_orientation: [f64; 4],
    bias: [f64; 6],
    model_id: &str,
    gravity_reference: Option<(&WrenchSample, [f64; 3], [f64; 4])>,
) -> Result<([f64; 6], [f64; 6]), WrenchAbortReason> {
    if !matches!(
        model_id,
        "mujoco-force-torque-sensor-gravity-baseline-v1"
            | "mujoco-force-torque-sensor-gravity-baseline-v2"
            | "mujoco-force-torque-sensor-free-space-model-v1"
    ) {
        return Err(WrenchAbortReason::MonitorFault);
    }
    if sample.validate().is_err() || sample.origin != WrenchSampleOrigin::Simulation {
        return Err(WrenchAbortReason::MonitorFault);
    }
    let mut compensated = [0.0; 6];
    let mut error = sample.error_bounds;
    for axis in 0..6 {
        if !sample.wrench[axis].is_finite()
            || !sample.error_bounds[axis].is_finite()
            || sample.error_bounds[axis] < 0.0
            || !bias[axis].is_finite()
        {
            return Err(WrenchAbortReason::MonitorFault);
        }
        compensated[axis] = sample.wrench[axis] - bias[axis];
    }
    let Some((reference, reference_position, reference_orientation)) = gravity_reference else {
        return Ok((compensated, error));
    };
    if reference.validate().is_err()
        || reference.sensor_id != sample.sensor_id
        || reference.sensor_frame != sample.sensor_frame
        || reference.transform_epoch != sample.transform_epoch
        || reference.origin != WrenchSampleOrigin::Simulation
        || reference.sequence > sample.sequence
        || reference.capture_s > sample.capture_s
        || !sample_position
            .iter()
            .chain(reference_position.iter())
            .all(|v| v.is_finite())
    {
        return Err(WrenchAbortReason::MonitorFault);
    }
    let current_rotation = quaternion_rotation(sample_orientation)?;
    let reference_rotation = quaternion_rotation(reference_orientation)?;
    let mut reference_wrench = [0.0; 6];
    for axis in 0..6 {
        if !reference.wrench[axis].is_finite()
            || !reference.error_bounds[axis].is_finite()
            || reference.error_bounds[axis] < 0.0
            || !bias[axis].is_finite()
        {
            return Err(WrenchAbortReason::MonitorFault);
        }
        reference_wrench[axis] = reference.wrench[axis] - bias[axis];
    }
    let force_world = matrix_vector(
        reference_rotation,
        [
            reference_wrench[0],
            reference_wrench[1],
            reference_wrench[2],
        ],
    );
    let mut torque_world_at_sample = matrix_vector(
        reference_rotation,
        [
            reference_wrench[3],
            reference_wrench[4],
            reference_wrench[5],
        ],
    );
    let force_current = matrix_transpose_vector(current_rotation, force_world);
    let mut torque_error_world = matrix_abs_vector(
        reference_rotation,
        [
            reference.error_bounds[3],
            reference.error_bounds[4],
            reference.error_bounds[5],
        ],
    );
    if model_id == "mujoco-force-torque-sensor-gravity-baseline-v1" {
        let arm = [
            reference_position[0] - sample_position[0],
            reference_position[1] - sample_position[1],
            reference_position[2] - sample_position[2],
        ];
        torque_world_at_sample = add3(torque_world_at_sample, cross3(arm, force_world));
        let force_error_world = matrix_abs_vector(
            reference_rotation,
            [
                reference.error_bounds[0],
                reference.error_bounds[1],
                reference.error_bounds[2],
            ],
        );
        torque_error_world = add3(
            torque_error_world,
            [
                arm[1].abs() * force_error_world[2] + arm[2].abs() * force_error_world[1],
                arm[2].abs() * force_error_world[0] + arm[0].abs() * force_error_world[2],
                arm[0].abs() * force_error_world[1] + arm[1].abs() * force_error_world[0],
            ],
        );
    }
    let torque_current = matrix_transpose_vector(current_rotation, torque_world_at_sample);
    for axis in 0..3 {
        compensated[axis] -= force_current[axis];
        compensated[axis + 3] -= torque_current[axis];
    }

    let reference_force_error_world = matrix_abs_vector(
        reference_rotation,
        [
            reference.error_bounds[0],
            reference.error_bounds[1],
            reference.error_bounds[2],
        ],
    );
    let force_error_current =
        matrix_transpose_abs_vector(current_rotation, reference_force_error_world);
    let torque_error_current = matrix_transpose_abs_vector(current_rotation, torque_error_world);
    for axis in 0..3 {
        error[axis] += force_error_current[axis];
        error[axis + 3] += torque_error_current[axis];
        if !compensated[axis].is_finite()
            || !compensated[axis + 3].is_finite()
            || !error[axis].is_finite()
            || !error[axis + 3].is_finite()
        {
            return Err(WrenchAbortReason::MonitorFault);
        }
    }
    Ok((compensated, error))
}

fn quaternion_rotation(quaternion: [f64; 4]) -> Result<[[f64; 3]; 3], WrenchAbortReason> {
    let [w, x, y, z] = quaternion;
    let norm_squared = w * w + x * x + y * y + z * z;
    if quaternion.iter().any(|value| !value.is_finite()) || (norm_squared - 1.0).abs() > 1e-5 {
        return Err(WrenchAbortReason::MonitorFault);
    }
    Ok([
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
    ])
}

fn matrix_vector(matrix: [[f64; 3]; 3], vector: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|row| {
        (0..3)
            .map(|column| matrix[row][column] * vector[column])
            .sum()
    })
}

fn matrix_transpose_vector(matrix: [[f64; 3]; 3], vector: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|column| (0..3).map(|row| matrix[row][column] * vector[row]).sum())
}

fn matrix_abs_vector(matrix: [[f64; 3]; 3], vector: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|row| {
        (0..3)
            .map(|column| matrix[row][column].abs() * vector[column])
            .sum()
    })
}

fn matrix_transpose_abs_vector(matrix: [[f64; 3]; 3], vector: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|column| {
        (0..3)
            .map(|row| matrix[row][column].abs() * vector[row])
            .sum()
    })
}

fn add3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|axis| left[axis] + right[axis])
}

fn cross3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum WrenchAbortReason {
    Limit {
        axis: usize,
        measured_upper_bound: f64,
        cap: f64,
    },
    MissingSample,
    StaleSample,
    GapExceeded,
    DigestMismatch,
    UnexpectedContact,
    ActuatorSaturation {
        actuator_ids: Vec<String>,
    },
    MonitorFault,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WrenchAbortEvent {
    pub reason: WrenchAbortReason,
    #[serde(default)]
    pub triggering_sample: Option<WrenchSample>,
    #[serde(default)]
    pub triggering_sensor_position: Option<[f64; 3]>,
    #[serde(default)]
    pub triggering_sensor_orientation: Option<[f64; 4]>,
    pub triggered_at_s: f64,
    pub stopped_at_s: f64,
    pub stop_write_count: u32,
    pub contract_id: String,
    pub stop_verified: bool,
    #[serde(default)]
    pub stop_actuator_ids: Vec<String>,
    #[serde(default)]
    pub stop_error: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MonitoredStepResult {
    pub state: Value,
    pub wrench_window: SimulationWrenchWindow,
    pub abort: Option<WrenchAbortEvent>,
    pub control_correction_max_rad: f64,
}

/// Conservatively bound every wrench axis using measurement error, sample age,
/// the registered rise rate, sensor latency, and abort latency. Any telemetry
/// defect fails closed with a typed reason so the caller can invoke only the
/// pre-authorized stop contract.
pub fn conservative_wrench_upper_bound(
    sample: Option<&WrenchSample>,
    previous_capture_s: Option<f64>,
    now_s: f64,
    policy: &WrenchLimitPolicy,
) -> Result<[f64; 6], WrenchAbortReason> {
    if policy.validate().is_err() {
        return Err(WrenchAbortReason::DigestMismatch);
    }
    let Some(sample) = sample else {
        return Err(WrenchAbortReason::MissingSample);
    };
    if sample.validate().is_err()
        || sample.sensor_id != policy.scope.sensor_id
        || sample.sensor_frame != policy.scope.sensor_frame
        || sample.origin != WrenchSampleOrigin::Simulation
    {
        return Err(WrenchAbortReason::DigestMismatch);
    }
    if !now_s.is_finite() || now_s < sample.receive_s || now_s < sample.capture_s {
        return Err(WrenchAbortReason::MissingSample);
    }
    let age_s = now_s - sample.capture_s;
    if age_s > policy.max_sample_age_s {
        return Err(WrenchAbortReason::StaleSample);
    }
    if previous_capture_s.is_some_and(|previous| {
        !previous.is_finite()
            || sample.capture_s <= previous
            || sample.capture_s - previous > policy.max_inter_sample_gap_s
    }) {
        return Err(WrenchAbortReason::GapExceeded);
    }
    let mut upper_bounds = [0.0; 6];
    for (axis, upper_slot) in upper_bounds.iter_mut().enumerate() {
        let upper_bound = sample.wrench[axis].abs()
            + sample.error_bounds[axis]
            + policy.max_rise_rate[axis]
                * (age_s + policy.max_sensing_latency_s + policy.max_abort_latency_s);
        if !upper_bound.is_finite() {
            return Err(WrenchAbortReason::MonitorFault);
        }
        *upper_slot = upper_bound;
        if upper_bound >= policy.scope.caps[axis] {
            return Err(WrenchAbortReason::Limit {
                axis,
                measured_upper_bound: upper_bound,
                cap: policy.scope.caps[axis],
            });
        }
    }
    Ok(upper_bounds)
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
        let mut sample_free_space_wrenches = Vec::with_capacity(records.len());
        let mut sample_free_space_wrench_error_bounds = Vec::with_capacity(records.len());
        let mut sensor_positions = Vec::with_capacity(records.len());
        let mut sensor_orientations = Vec::with_capacity(records.len());
        let mut max_gap_s: f64 = 0.0;
        let mut previous_capture: Option<f64> = None;
        let mut previous_sequence: Option<u64> = None;
        for record in records {
            let (sample, sensor_position, sensor_quaternion) =
                parse_worker_sample(record, profile, transform_epoch)?;
            let model_free_space_wrench = if record["model_free_space_wrench"].is_null() {
                if profile.model_id == "mujoco-force-torque-sensor-free-space-model-v1" {
                    return Err(WrenchSensorError::Sample("model_free_space_wrench_missing"));
                }
                [0.0; 6]
            } else {
                parse_six(&record["model_free_space_wrench"])
                    .ok_or(WrenchSensorError::Sample("model_free_space_wrench"))?
            };
            let model_free_space_error_bound =
                if record["model_free_space_wrench_error_bounds"].is_null() {
                    if profile.model_id == "mujoco-force-torque-sensor-free-space-model-v1" {
                        return Err(WrenchSensorError::Sample(
                            "model_free_space_wrench_error_bounds_missing",
                        ));
                    }
                    [0.0; 6]
                } else {
                    parse_six(&record["model_free_space_wrench_error_bounds"]).ok_or(
                        WrenchSensorError::Sample("model_free_space_wrench_error_bounds"),
                    )?
                };
            if model_free_space_error_bound != profile.free_space_model_error_bounds {
                return Err(WrenchSensorError::Sample(
                    "model_free_space_wrench_error_bounds_mismatch",
                ));
            }
            if model_free_space_error_bound
                .iter()
                .any(|value| !value.is_finite() || *value < 0.0)
            {
                return Err(WrenchSensorError::Sample(
                    "model_free_space_wrench_error_bounds_invalid",
                ));
            }
            let sequence = sample.sequence;
            let capture_s = sample.capture_s;
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
            samples.push(sample);
            sample_free_space_wrenches.push(model_free_space_wrench);
            sample_free_space_wrench_error_bounds.push(model_free_space_error_bound);
            sensor_positions.push(sensor_position);
            sensor_orientations.push(sensor_quaternion);
            previous_capture = Some(capture_s);
            previous_sequence = Some(sequence);
        }
        let (precontact_baseline, precontact_baseline_position, precontact_baseline_orientation) =
            if response["precontact_baseline"].is_null() {
                (None, None, None)
            } else {
                let (baseline, position, orientation) = parse_worker_sample(
                    &response["precontact_baseline"],
                    profile,
                    transform_epoch,
                )?;
                if worker_sample_is_future(&baseline, &samples, response) {
                    return Err(WrenchSensorError::Sample("precontact_baseline_future"));
                }
                if let Some(index) = samples
                    .iter()
                    .position(|sample| sample.sequence == baseline.sequence)
                {
                    if samples[index] != baseline
                        || sensor_positions[index] != position
                        || sensor_orientations[index] != orientation
                    {
                        return Err(WrenchSensorError::Sample("baseline_sample_mismatch"));
                    }
                }
                (Some(baseline), Some(position), Some(orientation))
            };
        let (
            monitor_gravity_reference,
            monitor_gravity_reference_position,
            monitor_gravity_reference_orientation,
        ) = if response["monitor_gravity_reference"].is_null() {
            (None, None, None)
        } else {
            let (reference, position, orientation) = parse_worker_sample(
                &response["monitor_gravity_reference"],
                profile,
                transform_epoch,
            )?;
            if worker_sample_is_future(&reference, &samples, response) {
                return Err(WrenchSensorError::Sample(
                    "monitor_gravity_reference_future",
                ));
            }
            if let Some(index) = samples
                .iter()
                .position(|sample| sample.sequence == reference.sequence)
            {
                if samples[index] != reference
                    || sensor_positions[index] != position
                    || sensor_orientations[index] != orientation
                {
                    return Err(WrenchSensorError::Sample(
                        "monitor_gravity_reference_mismatch",
                    ));
                }
            }
            (Some(reference), Some(position), Some(orientation))
        };
        let triggering_sample = if response["triggering_sample"].is_null() {
            None
        } else {
            Some(parse_worker_sample(&response["triggering_sample"], profile, transform_epoch)?.0)
        };
        Ok(Self {
            samples,
            sample_free_space_wrenches,
            sample_free_space_wrench_error_bounds,
            sensor_positions,
            sensor_orientations,
            precontact_baseline,
            precontact_baseline_status: response["precontact_baseline_status"]
                .as_str()
                .map(str::to_owned),
            precontact_baseline_position,
            precontact_baseline_orientation,
            triggering_sample,
            monitor_gravity_reference,
            monitor_gravity_reference_status: response["monitor_gravity_reference_status"]
                .as_str()
                .map(str::to_owned),
            monitor_gravity_reference_position,
            monitor_gravity_reference_orientation,
            max_gap_s,
            compensation_digest: profile.limit_policy.compensation_digest.clone(),
        })
    }
}

fn worker_sample_is_future(
    candidate: &WrenchSample,
    samples: &[WrenchSample],
    response: &Value,
) -> bool {
    let after_last_sample = samples.last().is_some_and(|last| {
        candidate.sequence > last.sequence || candidate.capture_s > last.capture_s + 1e-12
    });
    let state = &response["state"];
    let after_final_state = !state.is_null()
        && !state["time"]
            .as_f64()
            .is_some_and(|time_s| time_s.is_finite() && candidate.capture_s <= time_s + 1e-12);
    after_last_sample || after_final_state
}

fn parse_worker_sample(
    record: &Value,
    profile: &SimulationWrenchSensorProfile,
    transform_epoch: &str,
) -> Result<(WrenchSample, [f64; 3], [f64; 4]), WrenchSensorError> {
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
    let sensor_position = parse_three(&record["sensor_position"])
        .ok_or(WrenchSensorError::Worker("sensor_position"))?;
    let sensor_quaternion = parse_four(&record["sensor_quaternion"])
        .ok_or(WrenchSensorError::Worker("sensor_quaternion"))?;
    let error_bounds =
        parse_six(&record["error_bounds"]).ok_or(WrenchSensorError::Worker("error_bounds"))?;
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
    Ok((sample, sensor_position, sensor_quaternion))
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

fn parse_three(value: &Value) -> Option<[f64; 3]> {
    let values = value.as_array()?;
    if values.len() != 3 {
        return None;
    }
    let out = [
        values[0].as_f64()?,
        values[1].as_f64()?,
        values[2].as_f64()?,
    ];
    out.iter().all(|value| value.is_finite()).then_some(out)
}

fn parse_four(value: &Value) -> Option<[f64; 4]> {
    let values = value.as_array()?;
    if values.len() != 4 {
        return None;
    }
    let out = [
        values[0].as_f64()?,
        values[1].as_f64()?,
        values[2].as_f64()?,
        values[3].as_f64()?,
    ];
    if out.iter().any(|value| !value.is_finite())
        || (out.iter().map(|value| value * value).sum::<f64>() - 1.0).abs() > 1e-5
    {
        return None;
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
