//! Conservative friction inference from synchronized wrench and motion evidence.

use crate::observation::WrenchSample;
use crate::physical_belief::{
    BeliefEvidenceReference, BeliefUpdateError, PhysicalParameter, PhysicalParameterBelief,
};
use crate::sensor::WrenchSampleOrigin;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

pub type WrenchEvidenceOrigin = WrenchSampleOrigin;

const STANDARD_GRAVITY_M_S2: f64 = 9.80665;
const SUPPORT_NORMAL_TOLERANCE: f64 = 1e-6;
const ROTATION_TOLERANCE: f64 = 1e-6;
const MAX_VERTICAL_TO_HORIZONTAL_PROBE_RATIO: f64 = 0.05;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WrenchTransform {
    /// Rotation from the named sensor frame into the level support frame.
    pub rotation_to_support: [[f64; 3]; 3],
    /// Sensor-origin position relative to the support origin, in support axes.
    pub translation_to_support_m: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProbeMotionWindow {
    pub start_s: f64,
    pub end_s: f64,
    pub displacement_m: [f64; 3],
    pub max_speed_m_s: f64,
    pub max_quasi_static_speed_m_s: f64,
    pub pose_resolution_m: f64,
    pub first_motion_s: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WrenchCompensation {
    pub bias: [f64; 6],
    pub gravity_wrench: [f64; 6],
    pub residual_error: [f64; 6],
    pub digest: String,
}

impl WrenchCompensation {
    /// Canonical SHA-256 over the compensation terms, excluding the digest itself.
    pub fn digest(&self) -> String {
        let mut bytes = b"realityos.wrench_compensation/1\0".to_vec();
        push_f64s(&mut bytes, &self.bias);
        push_f64s(&mut bytes, &self.gravity_wrench);
        push_f64s(&mut bytes, &self.residual_error);
        sha256(&bytes)
    }

    pub fn validate(&self) -> Result<(), FrictionNoInferenceReason> {
        if self
            .bias
            .iter()
            .chain(self.gravity_wrench.iter())
            .any(|value| !value.is_finite())
            || self
                .residual_error
                .iter()
                .any(|value| !value.is_finite() || *value < 0.0)
            || !is_sha256(&self.digest)
            || self.digest != self.digest()
        {
            return Err(FrictionNoInferenceReason::InvalidCompensation);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WrenchInterval {
    pub lower: [f64; 6],
    pub upper: [f64; 6],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IntendedContactEvidence {
    pub observation_epoch: String,
    pub collision_model_digest: String,
    pub tool_body_id: String,
    pub target_body_id: String,
    pub plausible_contact_sets: Vec<Vec<String>>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PushProbeMeasurement {
    pub samples: Vec<WrenchSample>,
    #[serde(default)]
    pub precontact_baseline: Option<WrenchSample>,
    /// Optional per-sample gravity wrench in that sample's sensor frame.
    /// Simulated probes derive these values from the ordered no-contact baseline.
    #[serde(default)]
    pub sample_gravity_wrenches: Vec<[f64; 6]>,
    #[serde(default)]
    pub sample_sensor_transforms: Vec<WrenchTransform>,
    pub sensor_to_support: WrenchTransform,
    pub compensation: WrenchCompensation,
    pub mass_kg: [f64; 2],
    pub support_normal_world: [f64; 3],
    pub motion: ProbeMotionWindow,
    pub prior_support_friction: [f64; 2],
    pub current_observation_epoch: String,
    pub current_collision_model_digest: String,
    pub now_s: f64,
    pub max_sample_age_s: f64,
    pub max_inter_sample_gap_s: f64,
    pub max_sample_motion_skew_s: f64,
    pub max_wrench_rise_rate: [f64; 6],
    pub intended_contact: IntendedContactEvidence,
    pub saturated: bool,
    pub origin: WrenchEvidenceOrigin,
}

impl PushProbeMeasurement {
    /// Digest all inputs in fixed order so a belief reference identifies the exact evidence.
    pub fn digest(&self) -> String {
        let mut bytes = b"realityos.push_probe_measurement/2\0".to_vec();
        bytes.extend_from_slice(&(self.samples.len() as u64).to_le_bytes());
        for sample in &self.samples {
            push_string(&mut bytes, &sample.digest);
        }
        match &self.precontact_baseline {
            Some(sample) => {
                bytes.push(1);
                push_string(&mut bytes, &sample.digest);
            }
            None => bytes.push(0),
        }
        bytes.extend_from_slice(&(self.sample_gravity_wrenches.len() as u64).to_le_bytes());
        for gravity_wrench in &self.sample_gravity_wrenches {
            push_f64s(&mut bytes, gravity_wrench);
        }
        bytes.extend_from_slice(&(self.sample_sensor_transforms.len() as u64).to_le_bytes());
        for transform in &self.sample_sensor_transforms {
            for row in transform.rotation_to_support {
                push_f64s(&mut bytes, &row);
            }
            push_f64s(&mut bytes, &transform.translation_to_support_m);
        }
        for row in self.sensor_to_support.rotation_to_support {
            push_f64s(&mut bytes, &row);
        }
        push_f64s(&mut bytes, &self.sensor_to_support.translation_to_support_m);
        push_f64s(&mut bytes, &self.compensation.bias);
        push_f64s(&mut bytes, &self.compensation.gravity_wrench);
        push_f64s(&mut bytes, &self.compensation.residual_error);
        push_string(&mut bytes, &self.compensation.digest);
        push_f64s(&mut bytes, &self.mass_kg);
        push_f64s(&mut bytes, &self.support_normal_world);
        push_f64(&mut bytes, self.motion.start_s);
        push_f64(&mut bytes, self.motion.end_s);
        push_f64s(&mut bytes, &self.motion.displacement_m);
        push_f64(&mut bytes, self.motion.max_speed_m_s);
        push_f64(&mut bytes, self.motion.max_quasi_static_speed_m_s);
        push_f64(&mut bytes, self.motion.pose_resolution_m);
        push_opt_f64(&mut bytes, self.motion.first_motion_s);
        push_f64s(&mut bytes, &self.prior_support_friction);
        push_string(&mut bytes, &self.current_observation_epoch);
        push_string(&mut bytes, &self.current_collision_model_digest);
        push_f64(&mut bytes, self.now_s);
        push_f64(&mut bytes, self.max_sample_age_s);
        push_f64(&mut bytes, self.max_inter_sample_gap_s);
        push_f64(&mut bytes, self.max_sample_motion_skew_s);
        push_f64s(&mut bytes, &self.max_wrench_rise_rate);
        push_string(&mut bytes, &self.intended_contact.observation_epoch);
        push_string(&mut bytes, &self.intended_contact.collision_model_digest);
        push_string(&mut bytes, &self.intended_contact.tool_body_id);
        push_string(&mut bytes, &self.intended_contact.target_body_id);
        bytes.extend_from_slice(
            &(self.intended_contact.plausible_contact_sets.len() as u64).to_le_bytes(),
        );
        for contact_set in &self.intended_contact.plausible_contact_sets {
            bytes.extend_from_slice(&(contact_set.len() as u64).to_le_bytes());
            for body_id in contact_set {
                push_string(&mut bytes, body_id);
            }
        }
        bytes.push(u8::from(self.saturated));
        push_string(&mut bytes, self.origin.as_str());
        sha256(&bytes)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FrictionInferenceMode {
    Sticking,
    SlipOnset,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FrictionNoInferenceReason {
    InvalidMeasurement,
    InvalidCompensation,
    InvalidTransform,
    UnsupportedSupport,
    NonPlanarMotion,
    DynamicMotion,
    UnbracketedMotion,
    UnknownMass,
    InvalidPrior,
    StaleEvidence,
    PrecontactBaselineGapExceeded,
    AmbiguousContact,
    NoIntendedContact,
    ActuatorSaturated,
    SyntheticEvidence,
    OriginMismatch,
    InvalidSample,
    InsufficientSamples,
    PrecontactBaselineMissing,
    NonMonotonicSamples,
    SampleGapExceeded,
    StaleSample,
    TimeSkewExceeded,
    NonPositiveNormalLoad,
    Unidentifiable,
    EmptyIntersection,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum FrictionInferenceResult {
    Updated {
        interval: [f64; 2],
        mode: FrictionInferenceMode,
    },
    NoInference(FrictionNoInferenceReason),
}

/// Convert the complete sensor wrench to a target-object wrench in support axes.
/// The sample is the wrench on the tool, so its compensated wrench is negated.
/// The torque is shifted from sensor origin to support origin using `r × F`.
pub fn transform_sensor_wrench_to_support(
    sample: &WrenchSample,
    transform: &WrenchTransform,
    compensation: &WrenchCompensation,
    time_skew_s: f64,
    max_wrench_rise_rate: [f64; 6],
) -> Result<WrenchInterval, FrictionNoInferenceReason> {
    sample
        .validate()
        .map_err(|_| FrictionNoInferenceReason::InvalidSample)?;
    compensation.validate()?;
    validate_transform(transform)?;
    if !time_skew_s.is_finite()
        || time_skew_s < 0.0
        || max_wrench_rise_rate
            .iter()
            .any(|rate| !rate.is_finite() || *rate < 0.0)
    {
        return Err(FrictionNoInferenceReason::InvalidMeasurement);
    }

    let mut sensor_lower = [0.0; 6];
    let mut sensor_upper = [0.0; 6];
    for axis in 0..6 {
        let center =
            sample.wrench[axis] - compensation.bias[axis] - compensation.gravity_wrench[axis];
        let error = sample.error_bounds[axis]
            + compensation.residual_error[axis]
            + max_wrench_rise_rate[axis] * time_skew_s;
        // The tool sensor reports load on the tool; the target receives the opposite wrench.
        sensor_lower[axis] = -(center + error);
        sensor_upper[axis] = -(center - error);
    }

    let mut result = WrenchInterval {
        lower: [0.0; 6],
        upper: [0.0; 6],
    };
    for row in 0..3 {
        let (lower, upper) = linear_interval(
            &transform.rotation_to_support[row],
            &sensor_lower[..3],
            &sensor_upper[..3],
        );
        result.lower[row] = lower;
        result.upper[row] = upper;
    }
    let rotated_force_lower = [result.lower[0], result.lower[1], result.lower[2]];
    let rotated_force_upper = [result.upper[0], result.upper[1], result.upper[2]];
    for row in 0..3 {
        let (torque_lower, torque_upper) = linear_interval(
            &transform.rotation_to_support[row],
            &sensor_lower[3..],
            &sensor_upper[3..],
        );
        let cross_lower = cross_component_interval(
            row,
            transform.translation_to_support_m,
            rotated_force_lower,
            rotated_force_upper,
        )
        .0;
        let cross_upper = cross_component_interval(
            row,
            transform.translation_to_support_m,
            rotated_force_lower,
            rotated_force_upper,
        )
        .1;
        result.lower[row + 3] = torque_lower + cross_lower;
        result.upper[row + 3] = torque_upper + cross_upper;
    }
    if result
        .lower
        .iter()
        .chain(result.upper.iter())
        .any(|value| !value.is_finite())
    {
        return Err(FrictionNoInferenceReason::InvalidTransform);
    }
    Ok(result)
}

pub fn infer_support_friction(measurement: &PushProbeMeasurement) -> FrictionInferenceResult {
    let fail = FrictionInferenceResult::NoInference;
    if measurement.origin == WrenchSampleOrigin::Synthetic {
        return fail(FrictionNoInferenceReason::SyntheticEvidence);
    }
    if measurement.origin != WrenchSampleOrigin::Hardware
        && measurement.origin != WrenchSampleOrigin::Simulation
    {
        return fail(FrictionNoInferenceReason::OriginMismatch);
    }
    if measurement.saturated {
        return fail(FrictionNoInferenceReason::ActuatorSaturated);
    }
    if !valid_interval(measurement.mass_kg) || measurement.mass_kg[0] <= 0.0 {
        return fail(FrictionNoInferenceReason::UnknownMass);
    }
    if !valid_interval(measurement.prior_support_friction)
        || measurement.prior_support_friction[0] < 0.0
    {
        return fail(FrictionNoInferenceReason::InvalidPrior);
    }
    if !measurement.current_observation_epoch.trim().is_empty()
        && measurement.current_observation_epoch == measurement.intended_contact.observation_epoch
        && !measurement.current_collision_model_digest.trim().is_empty()
        && is_sha256(&measurement.current_collision_model_digest)
        && measurement.current_collision_model_digest
            == measurement.intended_contact.collision_model_digest
    {
        // Exact freshness and model identity are checked here before using the sample window.
    } else {
        return fail(FrictionNoInferenceReason::StaleEvidence);
    }
    if measurement.intended_contact.tool_body_id.trim().is_empty()
        || measurement
            .intended_contact
            .target_body_id
            .trim()
            .is_empty()
        || measurement.intended_contact.tool_body_id == measurement.intended_contact.target_body_id
    {
        return fail(FrictionNoInferenceReason::NoIntendedContact);
    }
    if measurement
        .intended_contact
        .plausible_contact_sets
        .is_empty()
    {
        return fail(FrictionNoInferenceReason::NoIntendedContact);
    }
    if measurement.intended_contact.plausible_contact_sets.len() != 1 {
        return fail(FrictionNoInferenceReason::AmbiguousContact);
    }
    let mut observed_pair = measurement.intended_contact.plausible_contact_sets[0].clone();
    observed_pair.sort();
    observed_pair.dedup();
    let mut intended_pair = vec![
        measurement.intended_contact.tool_body_id.clone(),
        measurement.intended_contact.target_body_id.clone(),
    ];
    intended_pair.sort();
    if observed_pair != intended_pair {
        return fail(FrictionNoInferenceReason::AmbiguousContact);
    }
    if measurement
        .intended_contact
        .observation_epoch
        .trim()
        .is_empty()
        || !valid_support_normal(measurement.support_normal_world)
    {
        return fail(FrictionNoInferenceReason::UnsupportedSupport);
    }
    if let Err(reason) = measurement.compensation.validate() {
        return fail(reason);
    }
    if let Err(reason) = validate_transform(&measurement.sensor_to_support) {
        return fail(reason);
    }
    if !valid_timing_policy(measurement)
        || !valid_motion_window(&measurement.motion)
        || !measurement.now_s.is_finite()
        || measurement.now_s < measurement.motion.end_s
    {
        return fail(FrictionNoInferenceReason::InvalidMeasurement);
    }
    if measurement.motion.max_speed_m_s > measurement.motion.max_quasi_static_speed_m_s {
        return fail(FrictionNoInferenceReason::DynamicMotion);
    }

    // Allow sub-resolution seating noise and a small, bounded relative offset.
    // Larger vertical motion can change support loading enough to invalidate
    // the planar friction model.
    let vertical = measurement.motion.displacement_m[2].abs();
    let horizontal =
        measurement.motion.displacement_m[0].hypot(measurement.motion.displacement_m[1]);
    let resolution = measurement.motion.pose_resolution_m;
    if vertical > resolution
        && vertical > MAX_VERTICAL_TO_HORIZONTAL_PROBE_RATIO * horizontal.max(resolution)
    {
        return fail(FrictionNoInferenceReason::NonPlanarMotion);
    }
    let displacement =
        measurement.motion.displacement_m[0].hypot(measurement.motion.displacement_m[1]);
    let mode = match measurement.motion.first_motion_s {
        None if displacement <= measurement.motion.pose_resolution_m => {
            FrictionInferenceMode::Sticking
        }
        None => return fail(FrictionNoInferenceReason::UnbracketedMotion),
        Some(first_motion_s)
            if first_motion_s >= measurement.motion.start_s
                && first_motion_s <= measurement.motion.end_s
                && displacement > measurement.motion.pose_resolution_m =>
        {
            FrictionInferenceMode::SlipOnset
        }
        Some(_) => return fail(FrictionNoInferenceReason::UnbracketedMotion),
    };

    if measurement.samples.len() < 2 {
        return fail(FrictionNoInferenceReason::InsufficientSamples);
    }
    if measurement.precontact_baseline.is_none() {
        return fail(FrictionNoInferenceReason::PrecontactBaselineMissing);
    }
    if (!measurement.sample_gravity_wrenches.is_empty()
        && measurement.sample_gravity_wrenches.len() != measurement.samples.len())
        || (!measurement.sample_sensor_transforms.is_empty()
            && measurement.sample_sensor_transforms.len() != measurement.samples.len())
        || measurement
            .sample_gravity_wrenches
            .iter()
            .flatten()
            .any(|value| !value.is_finite())
    {
        return fail(FrictionNoInferenceReason::InvalidMeasurement);
    }
    if let Err(reason) = validate_samples(measurement) {
        return fail(reason);
    }

    let bounds = match mode {
        FrictionInferenceMode::Sticking => {
            let Some(sample) = measurement.samples.last() else {
                return fail(FrictionNoInferenceReason::InsufficientSamples);
            };
            let skew_s = (sample.capture_s - measurement.motion.end_s).abs();
            if skew_s > measurement.max_sample_motion_skew_s {
                return fail(FrictionNoInferenceReason::TimeSkewExceeded);
            }
            let compensation = match compensation_for_sample(measurement, sample) {
                Ok(compensation) => compensation,
                Err(reason) => return fail(reason),
            };
            let transform = match transform_for_sample(measurement, sample) {
                Ok(transform) => transform,
                Err(reason) => return fail(reason),
            };
            let wrench = match transform_sensor_wrench_to_support(
                sample,
                &transform,
                &compensation,
                skew_s,
                measurement.max_wrench_rise_rate,
            ) {
                Ok(wrench) => wrench,
                Err(reason) => return fail(reason),
            };
            let force = tangential_force_bounds(&wrench);
            let normal = normal_load_bounds(&wrench, measurement.mass_kg);
            if normal[0] <= 0.0 {
                return fail(FrictionNoInferenceReason::NonPositiveNormalLoad);
            }
            if force[0] <= 0.0 {
                return fail(FrictionNoInferenceReason::Unidentifiable);
            }
            [force[0] / normal[1], measurement.prior_support_friction[1]]
        }
        FrictionInferenceMode::SlipOnset => {
            let onset = measurement
                .motion
                .first_motion_s
                .expect("slip mode has onset");
            let before = measurement
                .samples
                .iter()
                .rev()
                .find(|sample| sample.capture_s < onset);
            let after = measurement
                .samples
                .iter()
                .find(|sample| sample.capture_s >= onset);
            let (Some(before), Some(after)) = (before, after) else {
                return fail(FrictionNoInferenceReason::UnbracketedMotion);
            };
            let before_skew_s = (onset - before.capture_s).abs();
            let after_skew_s = (after.capture_s - onset).abs();
            if before_skew_s > measurement.max_sample_motion_skew_s
                || after_skew_s > measurement.max_sample_motion_skew_s
            {
                return fail(FrictionNoInferenceReason::TimeSkewExceeded);
            }
            let before_compensation = match compensation_for_sample(measurement, before) {
                Ok(compensation) => compensation,
                Err(reason) => return fail(reason),
            };
            let after_compensation = match compensation_for_sample(measurement, after) {
                Ok(compensation) => compensation,
                Err(reason) => return fail(reason),
            };
            let before_transform = match transform_for_sample(measurement, before) {
                Ok(transform) => transform,
                Err(reason) => return fail(reason),
            };
            let after_transform = match transform_for_sample(measurement, after) {
                Ok(transform) => transform,
                Err(reason) => return fail(reason),
            };
            let before_wrench = match transform_sensor_wrench_to_support(
                before,
                &before_transform,
                &before_compensation,
                before_skew_s,
                measurement.max_wrench_rise_rate,
            ) {
                Ok(wrench) => wrench,
                Err(reason) => return fail(reason),
            };
            let after_wrench = match transform_sensor_wrench_to_support(
                after,
                &after_transform,
                &after_compensation,
                after_skew_s,
                measurement.max_wrench_rise_rate,
            ) {
                Ok(wrench) => wrench,
                Err(reason) => return fail(reason),
            };
            let pre_force = tangential_force_bounds(&before_wrench);
            let pre_normal = normal_load_bounds(&before_wrench, measurement.mass_kg);
            let post_force = tangential_force_bounds(&after_wrench);
            let post_normal = normal_load_bounds(&after_wrench, measurement.mass_kg);
            if pre_normal[0] <= 0.0 || post_normal[0] <= 0.0 {
                return fail(FrictionNoInferenceReason::NonPositiveNormalLoad);
            }
            if pre_force[0] <= 0.0 || post_force[1] <= 0.0 {
                return fail(FrictionNoInferenceReason::Unidentifiable);
            }
            if std::env::var_os("REALITYOS_TRACE_FRICTION_INFERENCE").is_some() {
                eprintln!(
                    "probe-friction-slip onset={onset:.6} before_t={:.6} after_t={:.6} mass={:?} before_raw={:?} after_raw={:?} before_wrench={:?} after_wrench={:?} pre_force={:?} pre_normal={:?} post_force={:?} post_normal={:?}",
                    before.capture_s,
                    after.capture_s,
                    measurement.mass_kg,
                    before.wrench,
                    after.wrench,
                    before_wrench,
                    after_wrench,
                    pre_force,
                    pre_normal,
                    post_force,
                    post_normal,
                );
            }
            [pre_force[0] / pre_normal[1], post_force[1] / post_normal[0]]
        }
    };
    if bounds.iter().any(|value| !value.is_finite()) || bounds[0] < 0.0 {
        return fail(FrictionNoInferenceReason::Unidentifiable);
    }
    if bounds[0] > bounds[1] {
        return fail(FrictionNoInferenceReason::EmptyIntersection);
    }
    let interval = [
        measurement.prior_support_friction[0].max(bounds[0]),
        measurement.prior_support_friction[1].min(bounds[1]),
    ];
    if interval[0] > interval[1] {
        return fail(FrictionNoInferenceReason::EmptyIntersection);
    }
    FrictionInferenceResult::Updated { interval, mode }
}

fn compensation_for_sample(
    measurement: &PushProbeMeasurement,
    sample: &WrenchSample,
) -> Result<WrenchCompensation, FrictionNoInferenceReason> {
    let mut compensation = measurement.compensation.clone();
    if !measurement.sample_gravity_wrenches.is_empty() {
        let index = measurement
            .samples
            .iter()
            .position(|candidate| candidate.sequence == sample.sequence)
            .ok_or(FrictionNoInferenceReason::InvalidSample)?;
        compensation.gravity_wrench = measurement.sample_gravity_wrenches[index];
        compensation.digest = compensation.digest();
    }
    Ok(compensation)
}

fn transform_for_sample(
    measurement: &PushProbeMeasurement,
    sample: &WrenchSample,
) -> Result<WrenchTransform, FrictionNoInferenceReason> {
    if measurement.sample_sensor_transforms.is_empty() {
        return Ok(measurement.sensor_to_support.clone());
    }
    let index = measurement
        .samples
        .iter()
        .position(|candidate| candidate.sequence == sample.sequence)
        .ok_or(FrictionNoInferenceReason::InvalidSample)?;
    let transform = measurement.sample_sensor_transforms[index].clone();
    validate_transform(&transform)?;
    Ok(transform)
}

impl PhysicalParameterBelief {
    /// Apply only a validated estimator result and retain its exact evidence lineage.
    pub fn intersect_support_friction_with_probe(
        &mut self,
        interval: [f64; 2],
        measurement: &PushProbeMeasurement,
    ) -> Result<[f64; 2], BeliefUpdateError> {
        self.intersect_interval_with_evidence(
            PhysicalParameter::SupportFriction,
            interval,
            BeliefEvidenceReference {
                measurement_digest: measurement.digest(),
                origin: measurement.origin,
            },
        )
    }
}

fn validate_samples(measurement: &PushProbeMeasurement) -> Result<(), FrictionNoInferenceReason> {
    if measurement.samples.is_empty() {
        return Err(FrictionNoInferenceReason::InsufficientSamples);
    }
    let baseline = measurement
        .precontact_baseline
        .as_ref()
        .ok_or(FrictionNoInferenceReason::PrecontactBaselineMissing)?;
    baseline
        .validate()
        .map_err(|_| FrictionNoInferenceReason::InvalidSample)?;
    if baseline.origin != measurement.origin {
        return Err(FrictionNoInferenceReason::OriginMismatch);
    }
    if baseline.transform_epoch != measurement.current_observation_epoch {
        return Err(FrictionNoInferenceReason::StaleEvidence);
    }
    if let Some(first) = measurement.samples.first() {
        if baseline.sensor_id != first.sensor_id
            || baseline.sensor_frame != first.sensor_frame
            || baseline.units != first.units
            || baseline.clock_domain != first.clock_domain
            || baseline.sequence >= first.sequence
            || baseline.capture_s >= first.capture_s
        {
            return Err(FrictionNoInferenceReason::StaleEvidence);
        }
        if first.capture_s - baseline.capture_s > measurement.max_inter_sample_gap_s {
            return Err(FrictionNoInferenceReason::PrecontactBaselineGapExceeded);
        }
    }
    let mut prior: Option<&WrenchSample> = None;
    for sample in &measurement.samples {
        sample
            .validate()
            .map_err(|_| FrictionNoInferenceReason::InvalidSample)?;
        if sample.origin == WrenchSampleOrigin::Synthetic {
            return Err(FrictionNoInferenceReason::SyntheticEvidence);
        }
        if sample.origin != measurement.origin {
            return Err(FrictionNoInferenceReason::OriginMismatch);
        }
        if sample.transform_epoch != measurement.current_observation_epoch {
            return Err(FrictionNoInferenceReason::StaleEvidence);
        }
        if sample.capture_s < measurement.motion.start_s
            || sample.capture_s > measurement.motion.end_s
            || sample.receive_s > measurement.now_s
            || sample.receive_s - sample.capture_s > measurement.max_sample_age_s
        {
            return Err(FrictionNoInferenceReason::StaleSample);
        }
        if let Some(previous) = prior {
            let capture_gap = sample.capture_s - previous.capture_s;
            let receive_gap = sample.receive_s - previous.receive_s;
            if sample.sensor_id != previous.sensor_id
                || sample.sensor_frame != previous.sensor_frame
                || sample.clock_domain != previous.clock_domain
                || sample.sequence <= previous.sequence
                || capture_gap <= 0.0
                || receive_gap < 0.0
            {
                return Err(FrictionNoInferenceReason::NonMonotonicSamples);
            }
            if capture_gap > measurement.max_inter_sample_gap_s {
                return Err(FrictionNoInferenceReason::SampleGapExceeded);
            }
        }
        prior = Some(sample);
    }
    if prior
        .is_some_and(|sample| measurement.now_s - sample.receive_s > measurement.max_sample_age_s)
    {
        return Err(FrictionNoInferenceReason::StaleSample);
    }
    Ok(())
}

fn valid_timing_policy(measurement: &PushProbeMeasurement) -> bool {
    measurement.now_s.is_finite()
        && measurement.now_s >= 0.0
        && measurement.max_sample_age_s.is_finite()
        && measurement.max_sample_age_s > 0.0
        && measurement.max_inter_sample_gap_s.is_finite()
        && measurement.max_inter_sample_gap_s > 0.0
        && measurement.max_sample_motion_skew_s.is_finite()
        && measurement.max_sample_motion_skew_s >= 0.0
        && measurement
            .max_wrench_rise_rate
            .iter()
            .all(|rate| rate.is_finite() && *rate >= 0.0)
}

fn valid_motion_window(motion: &ProbeMotionWindow) -> bool {
    motion.start_s.is_finite()
        && motion.end_s.is_finite()
        && motion.start_s >= 0.0
        && motion.end_s > motion.start_s
        && motion.displacement_m.iter().all(|value| value.is_finite())
        && motion.max_speed_m_s.is_finite()
        && motion.max_speed_m_s >= 0.0
        && motion.max_quasi_static_speed_m_s.is_finite()
        && motion.max_quasi_static_speed_m_s > 0.0
        && motion.pose_resolution_m.is_finite()
        && motion.pose_resolution_m > 0.0
        && match motion.first_motion_s {
            None => true,
            Some(first) => first.is_finite() && first >= motion.start_s && first <= motion.end_s,
        }
}

fn valid_support_normal(normal: [f64; 3]) -> bool {
    normal.iter().all(|value| value.is_finite())
        && (norm3(normal) - 1.0).abs() <= SUPPORT_NORMAL_TOLERANCE
        && normal[0].abs() <= SUPPORT_NORMAL_TOLERANCE
        && normal[1].abs() <= SUPPORT_NORMAL_TOLERANCE
        && (normal[2] - 1.0).abs() <= SUPPORT_NORMAL_TOLERANCE
}

fn validate_transform(transform: &WrenchTransform) -> Result<(), FrictionNoInferenceReason> {
    if transform
        .rotation_to_support
        .iter()
        .flatten()
        .chain(transform.translation_to_support_m.iter())
        .any(|value| !value.is_finite())
    {
        return Err(FrictionNoInferenceReason::InvalidTransform);
    }
    for row in 0..3 {
        for column in 0..3 {
            let dot = (0..3)
                .map(|axis| {
                    transform.rotation_to_support[row][axis]
                        * transform.rotation_to_support[column][axis]
                })
                .sum::<f64>();
            let expected = if row == column { 1.0 } else { 0.0 };
            if (dot - expected).abs() > ROTATION_TOLERANCE {
                return Err(FrictionNoInferenceReason::InvalidTransform);
            }
        }
    }
    let r = transform.rotation_to_support;
    let determinant = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
    if (determinant - 1.0).abs() > ROTATION_TOLERANCE {
        return Err(FrictionNoInferenceReason::InvalidTransform);
    }
    Ok(())
}

fn tangential_force_bounds(wrench: &WrenchInterval) -> [f64; 2] {
    let x = magnitude_interval(wrench.lower[0], wrench.upper[0]);
    let y = magnitude_interval(wrench.lower[1], wrench.upper[1]);
    [x[0].hypot(y[0]), x[1].hypot(y[1])]
}

fn normal_load_bounds(wrench: &WrenchInterval, mass_kg: [f64; 2]) -> [f64; 2] {
    [
        mass_kg[0] * STANDARD_GRAVITY_M_S2 - wrench.upper[2],
        mass_kg[1] * STANDARD_GRAVITY_M_S2 - wrench.lower[2],
    ]
}

fn magnitude_interval(lower: f64, upper: f64) -> [f64; 2] {
    let maximum = lower.abs().max(upper.abs());
    if lower <= 0.0 && upper >= 0.0 {
        [0.0, maximum]
    } else {
        [lower.abs().min(upper.abs()), maximum]
    }
}

fn linear_interval(coefficients: &[f64], lower: &[f64], upper: &[f64]) -> (f64, f64) {
    let mut result_lower = 0.0;
    let mut result_upper = 0.0;
    for ((coefficient, low), high) in coefficients.iter().zip(lower).zip(upper) {
        if *coefficient >= 0.0 {
            result_lower += coefficient * low;
            result_upper += coefficient * high;
        } else {
            result_lower += coefficient * high;
            result_upper += coefficient * low;
        }
    }
    (result_lower, result_upper)
}

fn cross_component_interval(
    component: usize,
    translation: [f64; 3],
    force_lower: [f64; 3],
    force_upper: [f64; 3],
) -> (f64, f64) {
    let (left_axis, right_axis) = match component {
        0 => (1, 2),
        1 => (2, 0),
        _ => (0, 1),
    };
    let left = scale_interval(
        translation[left_axis],
        force_lower[right_axis],
        force_upper[right_axis],
    );
    let right = scale_interval(
        translation[right_axis],
        force_lower[left_axis],
        force_upper[left_axis],
    );
    (left.0 - right.1, left.1 - right.0)
}

fn scale_interval(coefficient: f64, lower: f64, upper: f64) -> (f64, f64) {
    if coefficient >= 0.0 {
        (coefficient * lower, coefficient * upper)
    } else {
        (coefficient * upper, coefficient * lower)
    }
}

fn valid_interval(interval: [f64; 2]) -> bool {
    interval[0].is_finite() && interval[1].is_finite() && interval[0] <= interval[1]
}

fn norm3(vector: [f64; 3]) -> f64 {
    vector[0].hypot(vector[1]).hypot(vector[2])
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

fn push_string(bytes: &mut Vec<u8>, value: &str) {
    bytes.extend_from_slice(&(value.len() as u64).to_le_bytes());
    bytes.extend_from_slice(value.as_bytes());
}

fn push_f64(bytes: &mut Vec<u8>, value: f64) {
    bytes.extend_from_slice(&value.to_bits().to_le_bytes());
}

fn push_f64s(bytes: &mut Vec<u8>, values: &[f64]) {
    for value in values {
        push_f64(bytes, *value);
    }
}

fn push_opt_f64(bytes: &mut Vec<u8>, value: Option<f64>) {
    match value {
        Some(value) => {
            bytes.push(1);
            push_f64(bytes, value);
        }
        None => bytes.push(0),
    }
}

fn sha256(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
}
