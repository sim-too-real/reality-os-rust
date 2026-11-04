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
    /// True when motion exceeded pose resolution before the force lower bound
    /// could resolve a slip onset. Inference must then fail closed.
    #[serde(default)]
    pub unresolved_motion_before_force: bool,
}

/// A raw simulator contact pair captured alongside a probe pose sample.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProbeContactPair {
    pub body1: String,
    pub geom1_name: String,
    pub body2: String,
    pub geom2_name: String,
}

impl ProbeContactPair {
    pub fn contains_bodies(&self, first: &str, second: &str) -> bool {
        (self.body1 == first && self.body2 == second)
            || (self.body1 == second && self.body2 == first)
    }

    fn validate(&self) -> Result<(), &'static str> {
        if self.body1.trim().is_empty()
            || self.geom1_name.trim().is_empty()
            || self.body2.trim().is_empty()
            || self.geom2_name.trim().is_empty()
        {
            return Err("contact_pair_identity");
        }
        Ok(())
    }
}

/// A raw MuJoCo contact-force snapshot paired with one object-pose sample.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProbeContactForceSample {
    pub contact_pair: ProbeContactPair,
    pub position_world_m: [f64; 3],
    /// Contact frame rows in world coordinates, flattened in row-major order.
    pub frame: [f64; 9],
    /// MuJoCo contact friction: two sliding, one torsional, and two rolling terms.
    pub friction: [f64; 5],
    /// Force and torque in MuJoCo's contact frame.
    pub force_local: [f64; 3],
    pub torque_local: [f64; 3],
    pub normal_force_n: f64,
    pub tangential_force_n: f64,
    pub force_available: bool,
}

impl ProbeContactForceSample {
    fn validate(&self) -> Result<(), &'static str> {
        self.contact_pair.validate()?;
        if self.position_world_m.iter().any(|value| !value.is_finite()) {
            return Err("contact_force_position");
        }
        if self.frame.iter().any(|value| !value.is_finite()) {
            return Err("contact_force_frame");
        }
        if self
            .friction
            .iter()
            .any(|value| !value.is_finite() || *value < 0.0)
        {
            return Err("contact_force_friction");
        }
        if self
            .force_local
            .iter()
            .chain(self.torque_local.iter())
            .any(|value| !value.is_finite())
        {
            return Err("contact_force_wrench");
        }
        if !self.normal_force_n.is_finite()
            || self.normal_force_n < 0.0
            || !self.tangential_force_n.is_finite()
            || self.tangential_force_n < 0.0
        {
            return Err("contact_force_magnitude");
        }
        Ok(())
    }
}

/// A target-body pose captured at a simulation step during a push probe.
/// `wrench_sequence` is present when a force sample was captured at the same
/// simulator time, making the two evidence streams directly pairable offline.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProbeObjectPoseSample {
    pub object_id: String,
    pub simulator_step: u64,
    #[serde(default)]
    pub wrench_sequence: Option<u64>,
    /// `Some([])` records no contacts at this exact simulator step; `None`
    /// identifies legacy pose evidence that did not record contacts.
    #[serde(default)]
    pub contact_pairs: Option<Vec<ProbeContactPair>>,
    /// Contact forces in the exact order of `contact_pairs`; `None` is legacy evidence.
    #[serde(default)]
    pub contact_force_samples: Option<Vec<ProbeContactForceSample>>,
    pub capture_s: f64,
    pub clock_domain: String,
    pub position_world_m: [f64; 3],
    /// MuJoCo quaternion order: w, x, y, z.
    pub orientation_world_wxyz: [f64; 4],
    pub digest: String,
}

impl ProbeObjectPoseSample {
    pub fn digest(&self) -> String {
        let mut bytes = if self.contact_force_samples.is_some() {
            b"realityos.probe_object_pose_sample/3\0".to_vec()
        } else if self.contact_pairs.is_some() {
            b"realityos.probe_object_pose_sample/2\0".to_vec()
        } else {
            b"realityos.probe_object_pose_sample/1\0".to_vec()
        };
        push_string(&mut bytes, &self.object_id);
        bytes.extend_from_slice(&self.simulator_step.to_le_bytes());
        match self.wrench_sequence {
            Some(sequence) => {
                bytes.push(1);
                bytes.extend_from_slice(&sequence.to_le_bytes());
            }
            None => bytes.push(0),
        }
        if let Some(contact_pairs) = &self.contact_pairs {
            push_contact_pairs(&mut bytes, contact_pairs);
        }
        if let Some(contact_force_samples) = &self.contact_force_samples {
            push_contact_force_samples(&mut bytes, contact_force_samples);
        }
        push_f64(&mut bytes, self.capture_s);
        push_string(&mut bytes, &self.clock_domain);
        push_f64s(&mut bytes, &self.position_world_m);
        push_f64s(&mut bytes, &self.orientation_world_wxyz);
        sha256(&bytes)
    }

    pub fn validate(&self) -> Result<(), &'static str> {
        if self.object_id.trim().is_empty() || self.clock_domain.trim().is_empty() {
            return Err("identity");
        }
        if self.simulator_step == 0 {
            return Err("simulator_step");
        }
        if !self.capture_s.is_finite() || self.capture_s < 0.0 {
            return Err("timestamp");
        }
        if self.position_world_m.iter().any(|value| !value.is_finite()) {
            return Err("position");
        }
        if let Some(contact_pairs) = &self.contact_pairs {
            for pair in contact_pairs {
                pair.validate()?;
            }
        }
        if let Some(contact_force_samples) = &self.contact_force_samples {
            let Some(contact_pairs) = self.contact_pairs.as_ref() else {
                return Err("contact_force_pair_mismatch");
            };
            if contact_pairs.len() != contact_force_samples.len() {
                return Err("contact_force_pair_mismatch");
            }
            for (pair, force_sample) in contact_pairs.iter().zip(contact_force_samples) {
                force_sample.validate()?;
                if pair != &force_sample.contact_pair {
                    return Err("contact_force_pair_mismatch");
                }
            }
        }
        if self
            .orientation_world_wxyz
            .iter()
            .any(|value| !value.is_finite())
            || (self
                .orientation_world_wxyz
                .iter()
                .map(|value| value * value)
                .sum::<f64>()
                - 1.0)
                .abs()
                > ROTATION_TOLERANCE
        {
            return Err("orientation");
        }
        if !is_sha256(&self.digest) || self.digest != self.digest() {
            return Err("digest_mismatch");
        }
        Ok(())
    }
}

/// A time-aligned target pose trace for independently auditing probe motion.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProbeObjectPoseTrace {
    pub object_id: String,
    pub clock_domain: String,
    pub start_s: f64,
    pub start_position_world_m: [f64; 3],
    /// MuJoCo quaternion order: w, x, y, z.
    pub start_orientation_world_wxyz: [f64; 4],
    /// Wrench sequence sampled at `start_s` before the first per-step pose
    /// trace sample. This binds a bootstrap wrench to the sealed start pose.
    #[serde(default)]
    pub start_wrench_sequence: Option<u64>,
    /// Raw contacts at the exact start pose. `None` denotes legacy evidence.
    #[serde(default)]
    pub start_contact_pairs: Option<Vec<ProbeContactPair>>,
    pub samples: Vec<ProbeObjectPoseSample>,
    pub digest: String,
}

impl ProbeObjectPoseTrace {
    pub fn digest(&self) -> String {
        let mut bytes = if self.has_contact_force_trace() {
            b"realityos.probe_object_pose_trace/4\0".to_vec()
        } else if self.has_contact_trace() {
            b"realityos.probe_object_pose_trace/3\0".to_vec()
        } else {
            match self.start_wrench_sequence {
                Some(_) => b"realityos.probe_object_pose_trace/2\0".to_vec(),
                None => b"realityos.probe_object_pose_trace/1\0".to_vec(),
            }
        };
        push_string(&mut bytes, &self.object_id);
        push_string(&mut bytes, &self.clock_domain);
        push_f64(&mut bytes, self.start_s);
        push_f64s(&mut bytes, &self.start_position_world_m);
        push_f64s(&mut bytes, &self.start_orientation_world_wxyz);
        if let Some(sequence) = self.start_wrench_sequence {
            bytes.extend_from_slice(&sequence.to_le_bytes());
        }
        if let Some(contact_pairs) = &self.start_contact_pairs {
            push_contact_pairs(&mut bytes, contact_pairs);
        }
        bytes.extend_from_slice(&(self.samples.len() as u64).to_le_bytes());
        for sample in &self.samples {
            push_string(&mut bytes, &sample.digest);
        }
        sha256(&bytes)
    }

    pub fn has_contact_trace(&self) -> bool {
        self.start_contact_pairs.is_some()
            && self
                .samples
                .iter()
                .all(|sample| sample.contact_pairs.is_some())
    }

    pub fn has_contact_force_trace(&self) -> bool {
        self.has_contact_trace()
            && self
                .samples
                .iter()
                .all(|sample| sample.contact_force_samples.is_some())
    }

    pub fn contains_contact_between(&self, first: &str, second: &str) -> bool {
        self.start_contact_pairs
            .iter()
            .chain(
                self.samples
                    .iter()
                    .filter_map(|sample| sample.contact_pairs.as_ref()),
            )
            .flatten()
            .any(|pair| pair.contains_bodies(first, second))
    }

    pub fn validate(&self) -> Result<(), &'static str> {
        if self.object_id.trim().is_empty() || self.clock_domain.trim().is_empty() {
            return Err("identity");
        }
        if !self.start_s.is_finite() || self.start_s < 0.0 {
            return Err("start_timestamp");
        }
        if self
            .start_position_world_m
            .iter()
            .any(|value| !value.is_finite())
        {
            return Err("start_position");
        }
        if self
            .start_orientation_world_wxyz
            .iter()
            .any(|value| !value.is_finite())
            || (self
                .start_orientation_world_wxyz
                .iter()
                .map(|value| value * value)
                .sum::<f64>()
                - 1.0)
                .abs()
                > ROTATION_TOLERANCE
        {
            return Err("start_orientation");
        }
        if self.samples.is_empty() {
            return Err("empty_trace");
        }
        let recorded_contact_lists = usize::from(self.start_contact_pairs.is_some())
            + self
                .samples
                .iter()
                .filter(|sample| sample.contact_pairs.is_some())
                .count();
        if recorded_contact_lists != 0 && recorded_contact_lists != self.samples.len() + 1 {
            return Err("incomplete_contact_trace");
        }
        let recorded_contact_force_lists = self
            .samples
            .iter()
            .filter(|sample| sample.contact_force_samples.is_some())
            .count();
        if recorded_contact_force_lists != 0 && recorded_contact_force_lists != self.samples.len() {
            return Err("incomplete_contact_force_trace");
        }
        if let Some(contact_pairs) = &self.start_contact_pairs {
            for pair in contact_pairs {
                pair.validate()?;
            }
        }
        if self.start_wrench_sequence.is_some_and(|sequence| {
            self.samples
                .iter()
                .any(|sample| sample.wrench_sequence == Some(sequence))
        }) {
            return Err("duplicate_start_wrench_pair");
        }
        let mut previous_step: Option<u64> = None;
        let mut previous_time: Option<f64> = None;
        for (index, sample) in self.samples.iter().enumerate() {
            sample.validate()?;
            if sample.object_id != self.object_id || sample.clock_domain != self.clock_domain {
                return Err("sample_identity_mismatch");
            }
            if let Some(previous) = previous_step {
                if sample.simulator_step != previous.saturating_add(1) {
                    return Err("step_gap_or_order");
                }
            } else if sample.simulator_step == 0 {
                return Err("step_order");
            }
            if sample.capture_s < self.start_s
                || previous_time.is_some_and(|previous| sample.capture_s <= previous)
            {
                return Err("time_order");
            }
            if index == 0
                && sample.capture_s == self.start_s
                && (sample.position_world_m != self.start_position_world_m
                    || sample.orientation_world_wxyz != self.start_orientation_world_wxyz)
            {
                return Err("start_pose_mismatch");
            }
            previous_step = Some(sample.simulator_step);
            previous_time = Some(sample.capture_s);
        }
        if !is_sha256(&self.digest) || self.digest != self.digest() {
            return Err("digest_mismatch");
        }
        Ok(())
    }

    /// Return the interval containing the first XY displacement threshold
    /// crossing, measured from the original probe origin.
    pub fn first_xy_motion_bracket(&self, threshold_m: f64) -> Option<[f64; 2]> {
        if !threshold_m.is_finite() || threshold_m <= 0.0 {
            return None;
        }
        let mut previous_time = self.start_s;
        for sample in &self.samples {
            let dx = sample.position_world_m[0] - self.start_position_world_m[0];
            let dy = sample.position_world_m[1] - self.start_position_world_m[1];
            if dx.hypot(dy) > threshold_m {
                return Some([previous_time, sample.capture_s]);
            }
            previous_time = sample.capture_s;
        }
        None
    }
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
    /// Optional same-state model wrench without environment contact, in each sample's sensor frame.
    #[serde(default)]
    pub sample_free_space_wrenches: Vec<[f64; 6]>,
    /// Per-sample uncertainty paired with each free-space model prediction.
    #[serde(default)]
    pub sample_free_space_wrench_error_bounds: Vec<[f64; 6]>,
    #[serde(default)]
    pub sample_sensor_transforms: Vec<WrenchTransform>,
    pub sensor_to_support: WrenchTransform,
    pub compensation: WrenchCompensation,
    pub mass_kg: [f64; 2],
    pub support_normal_world: [f64; 3],
    pub motion: ProbeMotionWindow,
    #[serde(default)]
    pub motion_trace: Option<ProbeObjectPoseTrace>,
    pub prior_support_friction: [f64; 2],
    pub current_observation_epoch: String,
    pub current_collision_model_digest: String,
    pub now_s: f64,
    pub max_sample_age_s: f64,
    pub max_inter_sample_gap_s: f64,
    pub max_sample_motion_skew_s: f64,
    pub max_wrench_rise_rate: [f64; 6],
    pub intended_contact: IntendedContactEvidence,
    /// True when runtime contacts on the instrumented tool subtree were not
    /// limited to the authorized target object.
    #[serde(default)]
    pub runtime_contact_ambiguous: bool,
    pub saturated: bool,
    pub origin: WrenchEvidenceOrigin,
}

impl PushProbeMeasurement {
    /// Digest all inputs in fixed order so a belief reference identifies the exact evidence.
    pub fn digest(&self) -> String {
        let mut bytes = if self.motion_trace.is_some() {
            b"realityos.push_probe_measurement/7\0".to_vec()
        } else if self.motion.unresolved_motion_before_force {
            b"realityos.push_probe_measurement/6\0".to_vec()
        } else {
            // Preserve the v5 digest for existing records with the new field's
            // default value so historical sidecars remain independently verifiable.
            b"realityos.push_probe_measurement/5\0".to_vec()
        };
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
        bytes.extend_from_slice(&(self.sample_free_space_wrenches.len() as u64).to_le_bytes());
        for free_space_wrench in &self.sample_free_space_wrenches {
            push_f64s(&mut bytes, free_space_wrench);
        }
        bytes.extend_from_slice(
            &(self.sample_free_space_wrench_error_bounds.len() as u64).to_le_bytes(),
        );
        for error_bound in &self.sample_free_space_wrench_error_bounds {
            push_f64s(&mut bytes, error_bound);
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
        if self.motion.unresolved_motion_before_force {
            bytes.push(1);
        }
        if let Some(trace) = &self.motion_trace {
            push_string(&mut bytes, &trace.digest);
        }
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
        bytes.push(u8::from(self.runtime_contact_ambiguous));
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

/// What an updated probe friction interval actually estimates.
///
/// Seed 21007 showed wrench-derived onset T/N can overlap contact T/N while
/// remaining disjoint from the simulator Coulomb parameter. Callers must not
/// treat this interval as identified Coulomb μ.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum FrictionInferredQuantity {
    /// Tool-wrench tangential/normal force-ratio envelope at sticking or slip onset.
    #[default]
    OnsetOrStickingForceRatio,
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
    UnexpectedContact,
    SyntheticEvidence,
    HardwareOriginUntrusted,
    OriginMismatch,
    InvalidSample,
    InvalidMotionTrace,
    InsufficientSamples,
    PrecontactBaselineMissing,
    NonMonotonicSamples,
    SampleGapExceeded,
    StaleSample,
    TimeSkewExceeded,
    NonPositiveNormalLoad,
    Unidentifiable,
    /// Single-measurement friction bounds are empty (lo > hi).
    EmptyIntersection,
    /// Measurement bounds are non-empty but do not overlap the prior support.
    /// Distinct from EmptyIntersection so successive-probe cadence failures are
    /// not conflated with malformed single-measurement intervals.
    PriorMeasurementIncompatible,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum FrictionInferenceResult {
    Updated {
        interval: [f64; 2],
        mode: FrictionInferenceMode,
        #[serde(default)]
        inferred_quantity: FrictionInferredQuantity,
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
    // A public enum value is not proof that evidence came through a trusted
    // device adapter. Until a registered hardware adapter can attest its
    // device identity and provenance, hardware-origin updates fail closed.
    if measurement.origin == WrenchSampleOrigin::Hardware {
        return fail(FrictionNoInferenceReason::HardwareOriginUntrusted);
    }
    if measurement.origin != WrenchSampleOrigin::Simulation {
        return fail(FrictionNoInferenceReason::OriginMismatch);
    }
    if measurement.runtime_contact_ambiguous {
        return fail(FrictionNoInferenceReason::UnexpectedContact);
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
    if measurement.motion.unresolved_motion_before_force {
        return fail(FrictionNoInferenceReason::UnbracketedMotion);
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
    if (!measurement.sample_free_space_wrenches.is_empty()
        && measurement.sample_free_space_wrenches.len() != measurement.samples.len())
        || (!measurement.sample_free_space_wrench_error_bounds.is_empty()
            && measurement.sample_free_space_wrench_error_bounds.len() != measurement.samples.len())
        || (measurement.sample_free_space_wrenches.is_empty()
            != measurement.sample_free_space_wrench_error_bounds.is_empty())
        || (!measurement.sample_sensor_transforms.is_empty()
            && measurement.sample_sensor_transforms.len() != measurement.samples.len())
        || measurement
            .sample_free_space_wrenches
            .iter()
            .flatten()
            .any(|value| !value.is_finite())
        || measurement
            .sample_free_space_wrench_error_bounds
            .iter()
            .flatten()
            .any(|value| !value.is_finite() || *value < 0.0)
    {
        return fail(FrictionNoInferenceReason::InvalidMeasurement);
    }
    if let Err(reason) = validate_samples(measurement) {
        return fail(reason);
    }
    if measurement.motion_trace.as_ref().is_some_and(|trace| {
        trace.has_contact_trace()
            && !trace.contains_contact_between(
                &measurement.intended_contact.tool_body_id,
                &measurement.intended_contact.target_body_id,
            )
    }) {
        return fail(FrictionNoInferenceReason::NoIntendedContact);
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
            [pre_force[0] / pre_normal[1], post_force[1] / post_normal[0]]
        }
    };
    if bounds.iter().any(|value| !value.is_finite()) || bounds[0] < 0.0 {
        return fail(FrictionNoInferenceReason::Unidentifiable);
    }
    match intersect_measurement_bounds_with_prior(
        bounds,
        measurement.prior_support_friction,
    ) {
        Ok(interval) => FrictionInferenceResult::Updated {
            interval,
            mode,
            inferred_quantity: FrictionInferredQuantity::OnsetOrStickingForceRatio,
        },
        Err(reason) => fail(reason),
    }
}

/// Intersect single-measurement friction bounds with the prior support.
///
/// - empty measurement bounds → `EmptyIntersection`
/// - non-empty bounds outside prior → `PriorMeasurementIncompatible`
pub fn intersect_measurement_bounds_with_prior(
    bounds: [f64; 2],
    prior_support_friction: [f64; 2],
) -> Result<[f64; 2], FrictionNoInferenceReason> {
    if bounds[0] > bounds[1] {
        return Err(FrictionNoInferenceReason::EmptyIntersection);
    }
    let interval = [
        prior_support_friction[0].max(bounds[0]),
        prior_support_friction[1].min(bounds[1]),
    ];
    if interval[0] > interval[1] {
        return Err(FrictionNoInferenceReason::PriorMeasurementIncompatible);
    }
    Ok(interval)
}

/// Cadence policy for successive probes after a first SlipOnset narrowing.
///
/// Single-measurement `infer_support_friction` stays strict. These policies only
/// resolve what the belief should do when a later onset is prior-incompatible.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SuccessiveProbeCadencePolicy {
    /// Status quo: prior-incompatible later onsets reject.
    StrictIntersect,
    /// Keep the first SlipOnset posterior; quarantine the later bounds.
    RetainFirstOnPriorIncompatible,
    /// When later bounds sit entirely above the first SlipOnset posterior,
    /// quarantine the first as under-load/provisional and accept
    /// later ∩ prior_before_first_onset (higher load family).
    PreferHigherLoadFamilyOnIncompatible,
}

/// Result of applying a successive-probe cadence policy.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SuccessiveProbeCadenceOutcome {
    Accepted {
        interval: [f64; 2],
    },
    RetainedFirst {
        retained_interval: [f64; 2],
        quarantined_bounds: [f64; 2],
    },
    EscalatedHigherLoadFamily {
        interval: [f64; 2],
        quarantined_first_interval: [f64; 2],
        measurement_bounds: [f64; 2],
    },
    Rejected {
        reason: FrictionNoInferenceReason,
    },
}

/// Resolve successive-probe belief when a new measurement may not intersect the
/// current prior. Does not change force/contact thresholds.
pub fn apply_successive_probe_cadence(
    policy: SuccessiveProbeCadencePolicy,
    current_prior: [f64; 2],
    measurement_bounds: [f64; 2],
    first_slip_onset: Option<[f64; 2]>,
    prior_before_first_onset: Option<[f64; 2]>,
) -> SuccessiveProbeCadenceOutcome {
    match intersect_measurement_bounds_with_prior(measurement_bounds, current_prior) {
        Ok(interval) => SuccessiveProbeCadenceOutcome::Accepted { interval },
        Err(FrictionNoInferenceReason::EmptyIntersection) => {
            SuccessiveProbeCadenceOutcome::Rejected {
                reason: FrictionNoInferenceReason::EmptyIntersection,
            }
        }
        Err(FrictionNoInferenceReason::PriorMeasurementIncompatible) => match policy {
            SuccessiveProbeCadencePolicy::StrictIntersect => {
                SuccessiveProbeCadenceOutcome::Rejected {
                    reason: FrictionNoInferenceReason::PriorMeasurementIncompatible,
                }
            }
            SuccessiveProbeCadencePolicy::RetainFirstOnPriorIncompatible => {
                if first_slip_onset.is_some() {
                    SuccessiveProbeCadenceOutcome::RetainedFirst {
                        retained_interval: current_prior,
                        quarantined_bounds: measurement_bounds,
                    }
                } else {
                    SuccessiveProbeCadenceOutcome::Rejected {
                        reason: FrictionNoInferenceReason::PriorMeasurementIncompatible,
                    }
                }
            }
            SuccessiveProbeCadencePolicy::PreferHigherLoadFamilyOnIncompatible => {
                let (Some(first), Some(orig_prior)) =
                    (first_slip_onset, prior_before_first_onset)
                else {
                    return SuccessiveProbeCadenceOutcome::Rejected {
                        reason: FrictionNoInferenceReason::PriorMeasurementIncompatible,
                    };
                };
                // Entirely above the first posterior → treat as a higher load family.
                if !(measurement_bounds[0] > first[1]) {
                    return SuccessiveProbeCadenceOutcome::Rejected {
                        reason: FrictionNoInferenceReason::PriorMeasurementIncompatible,
                    };
                }
                match intersect_measurement_bounds_with_prior(measurement_bounds, orig_prior) {
                    Ok(interval) => SuccessiveProbeCadenceOutcome::EscalatedHigherLoadFamily {
                        interval,
                        quarantined_first_interval: first,
                        measurement_bounds,
                    },
                    Err(reason) => SuccessiveProbeCadenceOutcome::Rejected { reason },
                }
            }
        },
        Err(reason) => SuccessiveProbeCadenceOutcome::Rejected { reason },
    }
}

/// Offline helper: lower bound at/above `mu_required` can prove planar push
/// mechanics. Not an admission claim.
pub fn friction_interval_proves_mu_required(interval: [f64; 2], mu_required: f64) -> bool {
    interval[0].is_finite()
        && interval[1].is_finite()
        && interval[0] <= interval[1]
        && interval[0] + 1e-12 >= mu_required
}

/// Policy for the **first** SlipOnset when measurement bounds are prior-incompatible.
///
/// Distinct from [`SuccessiveProbeCadencePolicy`]: PreferHigherLoadFamily requires a
/// prior first-onset posterior. These policies do not change contact/force Governor
/// thresholds. Live `record_probe_measurement` stays on RetainDeclaredPrior until an
/// authorized wire decides otherwise.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FirstOnsetOutsidePriorPolicy {
    /// Status quo: keep declared prior; reject PriorMeasurementIncompatible.
    RetainDeclaredPrior,
    /// When first-onset bounds sit entirely above `prior.upper`, replace the prior
    /// envelope with the measurement bounds (discards declared prior; research-only).
    NarrowToMeasurementOnEntirelyAbovePrior,
    /// Name the hole: declared prior is not credible for this scene; do not invent
    /// an Updated interval.
    RefusePriorEnvelopeAsNotCredible,
}

/// Result of applying a first-onset-outside-prior policy.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum FirstOnsetOutsidePriorOutcome {
    AcceptedByIntersect {
        interval: [f64; 2],
    },
    AcceptedByReplacingPrior {
        interval: [f64; 2],
        quarantined_prior: [f64; 2],
    },
    RefusedPriorEnvelope {
        measurement_bounds: [f64; 2],
        declared_prior: [f64; 2],
    },
    Rejected {
        reason: FrictionNoInferenceReason,
    },
}

/// Resolve first-onset belief when measurement may not intersect the declared prior.
/// Does not change force/contact thresholds. Does not invent measurements.
pub fn apply_first_onset_outside_prior(
    policy: FirstOnsetOutsidePriorPolicy,
    declared_prior: [f64; 2],
    measurement_bounds: [f64; 2],
) -> FirstOnsetOutsidePriorOutcome {
    match intersect_measurement_bounds_with_prior(measurement_bounds, declared_prior) {
        Ok(interval) => FirstOnsetOutsidePriorOutcome::AcceptedByIntersect { interval },
        Err(FrictionNoInferenceReason::EmptyIntersection) => {
            FirstOnsetOutsidePriorOutcome::Rejected {
                reason: FrictionNoInferenceReason::EmptyIntersection,
            }
        }
        Err(FrictionNoInferenceReason::PriorMeasurementIncompatible) => match policy {
            FirstOnsetOutsidePriorPolicy::RetainDeclaredPrior => {
                FirstOnsetOutsidePriorOutcome::Rejected {
                    reason: FrictionNoInferenceReason::PriorMeasurementIncompatible,
                }
            }
            FirstOnsetOutsidePriorPolicy::NarrowToMeasurementOnEntirelyAbovePrior => {
                if !(measurement_bounds[0] > declared_prior[1]
                    && measurement_bounds[0] <= measurement_bounds[1])
                {
                    return FirstOnsetOutsidePriorOutcome::Rejected {
                        reason: FrictionNoInferenceReason::PriorMeasurementIncompatible,
                    };
                }
                FirstOnsetOutsidePriorOutcome::AcceptedByReplacingPrior {
                    interval: measurement_bounds,
                    quarantined_prior: declared_prior,
                }
            }
            FirstOnsetOutsidePriorPolicy::RefusePriorEnvelopeAsNotCredible => {
                FirstOnsetOutsidePriorOutcome::RefusedPriorEnvelope {
                    measurement_bounds,
                    declared_prior,
                }
            }
        },
        Err(reason) => FirstOnsetOutsidePriorOutcome::Rejected { reason },
    }
}

/// Offline honesty metric when first-onset bounds sit entirely above `prior.upper`.
///
/// Returns `(measurement.lower − prior.upper) / prior.upper` when the geometry is
/// `entirely_above_prior` with a positive finite prior upper bound. Does not invent
/// measurements, change Governor thresholds, or select a live belief policy.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum PriorEnvelopeMissMargin {
    EntirelyAbovePrior {
        absolute_miss: f64,
        relative_miss: f64,
        declared_prior: [f64; 2],
        measurement_bounds: [f64; 2],
    },
    NotEntirelyAbovePrior {
        declared_prior: [f64; 2],
        measurement_bounds: [f64; 2],
    },
    InvalidInputs {
        reason: String,
    },
}

pub fn prior_envelope_miss_margin(
    declared_prior: [f64; 2],
    measurement_bounds: [f64; 2],
) -> PriorEnvelopeMissMargin {
    if !(declared_prior[0].is_finite()
        && declared_prior[1].is_finite()
        && declared_prior[0] <= declared_prior[1]
        && measurement_bounds[0].is_finite()
        && measurement_bounds[1].is_finite()
        && measurement_bounds[0] <= measurement_bounds[1])
    {
        return PriorEnvelopeMissMargin::InvalidInputs {
            reason: "non_finite_or_unordered_interval".into(),
        };
    }
    if !(declared_prior[1] > 0.0) {
        return PriorEnvelopeMissMargin::InvalidInputs {
            reason: "non_positive_prior_upper".into(),
        };
    }
    if measurement_bounds[0] > declared_prior[1] {
        let absolute_miss = measurement_bounds[0] - declared_prior[1];
        PriorEnvelopeMissMargin::EntirelyAbovePrior {
            absolute_miss,
            relative_miss: absolute_miss / declared_prior[1],
            declared_prior,
            measurement_bounds,
        }
    } else {
        PriorEnvelopeMissMargin::NotEntirelyAbovePrior {
            declared_prior,
            measurement_bounds,
        }
    }
}

fn push_contact_pairs(bytes: &mut Vec<u8>, contact_pairs: &[ProbeContactPair]) {
    bytes.extend_from_slice(&(contact_pairs.len() as u64).to_le_bytes());
    for pair in contact_pairs {
        push_string(bytes, &pair.body1);
        push_string(bytes, &pair.geom1_name);
        push_string(bytes, &pair.body2);
        push_string(bytes, &pair.geom2_name);
    }
}

fn push_contact_force_samples(bytes: &mut Vec<u8>, samples: &[ProbeContactForceSample]) {
    bytes.extend_from_slice(&(samples.len() as u64).to_le_bytes());
    for sample in samples {
        push_contact_pairs(bytes, std::slice::from_ref(&sample.contact_pair));
        push_f64s(bytes, &sample.position_world_m);
        push_f64s(bytes, &sample.frame);
        push_f64s(bytes, &sample.friction);
        push_f64s(bytes, &sample.force_local);
        push_f64s(bytes, &sample.torque_local);
        push_f64(bytes, sample.normal_force_n);
        push_f64(bytes, sample.tangential_force_n);
        bytes.push(u8::from(sample.force_available));
    }
}

fn compensation_for_sample(
    measurement: &PushProbeMeasurement,
    sample: &WrenchSample,
) -> Result<WrenchCompensation, FrictionNoInferenceReason> {
    let mut compensation = measurement.compensation.clone();
    if !measurement.sample_free_space_wrenches.is_empty() {
        let index = measurement
            .samples
            .iter()
            .position(|candidate| candidate.sequence == sample.sequence)
            .ok_or(FrictionNoInferenceReason::InvalidSample)?;
        compensation.gravity_wrench = measurement.sample_free_space_wrenches[index];
        for axis in 0..6 {
            compensation.residual_error[axis] +=
                measurement.sample_free_space_wrench_error_bounds[index][axis];
        }
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
    if let Some(trace) = &measurement.motion_trace {
        validate_motion_trace(measurement, trace)?;
    }
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

fn validate_motion_trace(
    measurement: &PushProbeMeasurement,
    trace: &ProbeObjectPoseTrace,
) -> Result<(), FrictionNoInferenceReason> {
    trace
        .validate()
        .map_err(|_| FrictionNoInferenceReason::InvalidMotionTrace)?;
    if trace.object_id != measurement.intended_contact.target_body_id
        || trace.start_s != measurement.motion.start_s
        || trace.samples.last().map(|sample| sample.capture_s) != Some(measurement.motion.end_s)
        || trace.clock_domain
            != measurement
                .samples
                .first()
                .map(|sample| sample.clock_domain.as_str())
                .unwrap_or_default()
    {
        return Err(FrictionNoInferenceReason::InvalidMotionTrace);
    }
    let start = trace.start_position_world_m;
    let end = trace
        .samples
        .last()
        .ok_or(FrictionNoInferenceReason::InvalidMotionTrace)?
        .position_world_m;
    let displacement = [end[0] - start[0], end[1] - start[1], end[2] - start[2]];
    if displacement
        .iter()
        .zip(measurement.motion.displacement_m.iter())
        .any(|(actual, recorded)| (actual - recorded).abs() > 1e-9)
    {
        return Err(FrictionNoInferenceReason::InvalidMotionTrace);
    }
    for sample in &measurement.samples {
        let mut matching_poses = trace.samples.iter().filter(|pose| {
            pose.wrench_sequence == Some(sample.sequence) && pose.capture_s == sample.capture_s
        });
        let paired_start_pose = trace.start_wrench_sequence == Some(sample.sequence)
            && sample.capture_s == trace.start_s;
        let explicit_pose_count = matching_poses.by_ref().count();
        if explicit_pose_count + usize::from(paired_start_pose) != 1 {
            return Err(FrictionNoInferenceReason::InvalidMotionTrace);
        }
    }
    if let Some(start_sequence) = trace.start_wrench_sequence {
        let mut matching_samples = measurement.samples.iter().filter(|sample| {
            sample.sequence == start_sequence && sample.capture_s == trace.start_s
        });
        if matching_samples.next().is_none() || matching_samples.next().is_some() {
            return Err(FrictionNoInferenceReason::InvalidMotionTrace);
        }
    }
    for pose in trace
        .samples
        .iter()
        .filter(|pose| pose.wrench_sequence.is_some())
    {
        let mut matching_samples = measurement.samples.iter().filter(|sample| {
            Some(sample.sequence) == pose.wrench_sequence && sample.capture_s == pose.capture_s
        });
        if matching_samples.next().is_none() || matching_samples.next().is_some() {
            return Err(FrictionNoInferenceReason::InvalidMotionTrace);
        }
    }
    let first_motion = trace.first_xy_motion_bracket(measurement.motion.pose_resolution_m);
    match (first_motion, measurement.motion.first_motion_s) {
        (Some([_, crossing_end]), Some(resolved_s)) if resolved_s >= crossing_end => {}
        (Some(_), None) if measurement.motion.unresolved_motion_before_force => {}
        (None, None) if !measurement.motion.unresolved_motion_before_force => {}
        _ => return Err(FrictionNoInferenceReason::InvalidMotionTrace),
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

#[cfg(test)]
mod probe_object_pose_trace_tests {
    use super::{ProbeObjectPoseSample, ProbeObjectPoseTrace};
    use serde_json::{json, Value};

    fn pose_sample_json(step: u64, with_force_samples: bool, friction: f64) -> Value {
        let pair = json!({
            "body1": "obj0",
            "geom1_name": "obj0_geom",
            "body2": "table",
            "geom2_name": "table_geom"
        });
        let mut sample = json!({
            "object_id": "obj0",
            "simulator_step": step,
            "wrench_sequence": step + 10,
            "contact_pairs": [pair.clone()],
            "capture_s": step as f64 * 0.002,
            "clock_domain": "mujoco_simulation_clock",
            "position_world_m": [0.0, 0.0, 0.4],
            "orientation_world_wxyz": [1.0, 0.0, 0.0, 0.0],
            "digest": ""
        });
        if with_force_samples {
            sample["contact_force_samples"] = json!([{
                "contact_pair": pair,
                "position_world_m": [0.0, 0.0, 0.42],
                "frame": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                "friction": [friction, friction, 0.01, 0.01, 0.01],
                "force_local": [2.0, 0.25, 0.0],
                "torque_local": [0.0, 0.0, 0.0],
                "normal_force_n": 2.0,
                "tangential_force_n": 0.25,
                "force_available": true
            }]);
        }
        let parsed: ProbeObjectPoseSample =
            serde_json::from_value(sample.clone()).expect("pose sample parses");
        sample["digest"] = json!(parsed.digest());
        sample
    }

    #[test]
    fn probe_object_pose_digest_binds_contact_force_samples() {
        let first: ProbeObjectPoseSample = serde_json::from_value(pose_sample_json(1, true, 0.5))
            .expect("sample with contact force data parses");
        let changed: ProbeObjectPoseSample = serde_json::from_value(pose_sample_json(1, true, 0.6))
            .expect("sample with changed contact friction parses");

        assert_ne!(
            first.digest(),
            changed.digest(),
            "changing the recorded contact friction must change the pose digest"
        );
        assert_eq!(
            serde_json::to_value(&first).expect("sample serializes")["contact_force_samples"][0]
                ["friction"][0],
            json!(0.5),
            "contact force data must survive serde roundtrip"
        );
    }

    #[test]
    fn probe_object_pose_sample_preserves_empty_contact_force_samples() {
        let mut value = pose_sample_json(1, false, 0.5);
        value["contact_force_samples"] = json!([]);
        let sample: ProbeObjectPoseSample =
            serde_json::from_value(value).expect("empty contact force list parses");
        let serialized = serde_json::to_value(sample).expect("sample serializes");

        assert_eq!(serialized["contact_force_samples"], json!([]));
    }

    #[test]
    fn probe_object_pose_trace_rejects_partial_contact_force_samples() {
        let mut trace = json!({
            "object_id": "obj0",
            "clock_domain": "mujoco_simulation_clock",
            "start_s": 0.0,
            "start_position_world_m": [0.0, 0.0, 0.4],
            "start_orientation_world_wxyz": [1.0, 0.0, 0.0, 0.0],
            "start_wrench_sequence": 10,
            "start_contact_pairs": [],
            "samples": [
                pose_sample_json(1, true, 0.5),
                pose_sample_json(2, false, 0.5)
            ],
            "digest": ""
        });
        let parsed: ProbeObjectPoseTrace =
            serde_json::from_value(trace.clone()).expect("trace parses");
        trace["digest"] = json!(parsed.digest());
        let trace: ProbeObjectPoseTrace = serde_json::from_value(trace).expect("trace parses");

        assert_eq!(trace.validate(), Err("incomplete_contact_force_trace"));
    }
}
