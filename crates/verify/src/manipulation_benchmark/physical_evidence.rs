//! Physical sensor provenance validation. This module never authorizes hardware writes.

use super::scenario::BENCHMARK_SCHEMA_VERSION;
use super::BenchmarkError;
use crate::observation::{PolicyObservation, VisionMode};
use realityos_semantics::observation::{joint_sample_digest, ObservationFrame};
use realityos_semantics::sensor::SensorClass;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PhysicalSensorRequirement {
    pub sensor_id: String,
    pub device_serial: String,
    pub model_id: String,
    pub calibration_hash: String,
    pub frame: String,
    pub units: String,
    pub max_age_s: f64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PhysicalSensorIdentity {
    pub sensor_id: String,
    pub device_serial: String,
    pub model_id: String,
    pub calibration_hash: String,
    pub frame: String,
    pub units: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorOrigin {
    Hardware,
    Simulation,
    Synthetic,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysicalSensorTrace {
    pub observation: ObservationFrame,
    pub sensors: Vec<PhysicalSensorIdentity>,
    pub origin: SensorOrigin,
    /// The exact policy input derived from `observation`, before action selection.
    pub policy_observation: PolicyObservation,
    /// Joint order binds the policy vector to individually provenance-checked samples.
    pub joint_order: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PhysicalEvidenceStatus {
    Unverified,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PhysicalObservationEvidence {
    pub schema_version: u32,
    pub status: PhysicalEvidenceStatus,
    pub sensor_provenance_valid: bool,
    pub requires_independent_measurement: bool,
    pub trace_digest: String,
    pub frame_id: String,
    pub policy_observation_id: String,
    pub sensor_ids: Vec<String>,
    pub captured_at_s: f64,
    pub received_at_s: f64,
}

pub fn validate_physical_observation(
    trace: &PhysicalSensorTrace,
    requirements: &[PhysicalSensorRequirement],
    now_s: f64,
) -> Result<PhysicalObservationEvidence, BenchmarkError> {
    let reject = |reason: &str| {
        BenchmarkError::InvalidTrial(format!("physical sensor evidence rejected: {reason}"))
    };
    if trace.origin != SensorOrigin::Hardware {
        return Err(reject("sensor origin is not hardware"));
    }
    if requirements.is_empty() {
        return Err(reject("no required sensor provenance was declared"));
    }
    if !now_s.is_finite() || now_s < 0.0 {
        return Err(reject("validation time is not finite and non-negative"));
    }
    if trace.observation.frame_id.trim().is_empty()
        || trace.observation.transform_epoch.trim().is_empty()
        || !trace.observation.as_of_s.is_finite()
        || trace.observation.as_of_s > now_s
    {
        return Err(reject("observation frame identity or timestamp is invalid"));
    }
    if trace.policy_observation.mode == VisionMode::PerfectPerception {
        return Err(reject(
            "perfect-perception policy input is simulator privileged state",
        ));
    }
    if trace.policy_observation.observation_id != trace.observation.frame_id
        || !trace.policy_observation.timestamp_s.is_finite()
        || (trace.policy_observation.timestamp_s - trace.observation.as_of_s).abs() > 1e-6
        || trace.policy_observation.timestamp_s > now_s
        || trace.policy_observation.robot_id.trim().is_empty()
        || trace.policy_observation.model_hash.trim().is_empty()
        || trace.policy_observation.task_id.trim().is_empty()
        || trace.policy_observation.qpos.is_empty()
        || trace.policy_observation.qpos.len() != trace.joint_order.len()
        || trace.policy_observation.qvel.len() != trace.joint_order.len()
        || trace
            .policy_observation
            .qpos
            .iter()
            .chain(&trace.policy_observation.qvel)
            .any(|value| !value.is_finite())
    {
        return Err(reject(
            "policy observation is malformed or not tied to the sensor frame",
        ));
    }
    let max_age_s = requirements
        .iter()
        .map(|requirement| requirement.max_age_s)
        .fold(f64::INFINITY, f64::min);
    if !max_age_s.is_finite()
        || max_age_s <= 0.0
        || now_s - trace.policy_observation.timestamp_s > max_age_s
    {
        return Err(reject("policy observation is stale"));
    }

    let mut required_by_id = BTreeMap::new();
    for requirement in requirements {
        if requirement.sensor_id.trim().is_empty()
            || requirement.device_serial.trim().is_empty()
            || requirement.model_id.trim().is_empty()
            || requirement.calibration_hash.trim().is_empty()
            || requirement.frame.trim().is_empty()
            || requirement.units.trim().is_empty()
            || !requirement.max_age_s.is_finite()
            || requirement.max_age_s <= 0.0
            || required_by_id
                .insert(requirement.sensor_id.as_str(), requirement)
                .is_some()
        {
            return Err(reject(
                "sensor requirement is empty, invalid, or duplicated",
            ));
        }
    }

    let mut identity_by_id = BTreeMap::new();
    for identity in &trace.sensors {
        if identity_by_id
            .insert(identity.sensor_id.as_str(), identity)
            .is_some()
        {
            return Err(reject("sensor identity is duplicated"));
        }
        let Some(requirement) = required_by_id.get(identity.sensor_id.as_str()) else {
            return Err(reject("trace contains an undeclared sensor"));
        };
        if identity.device_serial != requirement.device_serial
            || identity.model_id != requirement.model_id
            || identity.calibration_hash != requirement.calibration_hash
            || identity.frame != requirement.frame
            || identity.units != requirement.units
        {
            return Err(reject(
                "sensor serial, model, calibration, frame, or units mismatch",
            ));
        }
    }
    if identity_by_id.len() != required_by_id.len() {
        return Err(reject("one or more required sensor identities are missing"));
    }

    let mut seen_observations = BTreeSet::new();
    let mut observation_class_by_id = BTreeMap::new();
    let mut min_capture = f64::INFINITY;
    let mut max_receive: f64 = 0.0;
    for observation in &trace.observation.observations {
        let Some(requirement) = required_by_id.get(observation.sensor_id.as_str()) else {
            return Err(reject("observation contains an undeclared sensor"));
        };
        if !seen_observations.insert(observation.sensor_id.as_str())
            || observation.robot_frame != requirement.frame
            || observation.calibration_hash != requirement.calibration_hash
            || !observation.capture_s.is_finite()
            || !observation.receive_s.is_finite()
            || !observation.expires_at_s.is_finite()
            || observation.capture_s < 0.0
            || observation.receive_s < observation.capture_s
            || observation.receive_s > now_s
            || now_s - observation.receive_s > requirement.max_age_s
            || now_s > observation.expires_at_s
            || observation.sequence == 0
            || observation.digest.trim().is_empty()
            || is_simulation_clock(&observation.clock_domain)
        {
            return Err(reject(
                "sensor observation is stale, malformed, or untrusted",
            ));
        }
        min_capture = min_capture.min(observation.capture_s);
        max_receive = max_receive.max(observation.receive_s);
        observation_class_by_id.insert(observation.sensor_id.as_str(), observation.class);
    }
    if seen_observations.len() != required_by_id.len() {
        return Err(reject("one or more required sensor channels are missing"));
    }

    let mut seen_joints = BTreeSet::new();
    for sample in &trace.observation.joint_state {
        let Some(requirement) = required_by_id.get(sample.sensor_id.as_str()) else {
            return Err(reject("joint sample uses an undeclared sensor"));
        };
        if !seen_joints.insert(sample.joint_name.as_str())
            || sample.calibration_id != requirement.calibration_hash
            || sample.transform_epoch != trace.observation.transform_epoch
            || !sample.q.is_finite()
            || sample.dq.map_or(true, |value| !value.is_finite())
            || !sample.capture_s.is_finite()
            || !sample.receive_s.is_finite()
            || sample.receive_s < sample.capture_s
            || sample.receive_s > now_s
            || now_s - sample.receive_s > requirement.max_age_s
            || sample.digest
                != joint_sample_digest(
                    &sample.joint_name,
                    sample.q,
                    &sample.sensor_id,
                    &sample.calibration_id,
                    &sample.transform_epoch,
                )
        {
            return Err(reject(
                "joint sample is stale, malformed, or has a bad digest",
            ));
        }
    }
    if trace.joint_order.is_empty()
        || trace.observation.missing_required(&trace.joint_order)
        || trace
            .observation
            .wrong_epoch(&trace.observation.transform_epoch)
        || trace.joint_order.iter().collect::<BTreeSet<_>>().len() != trace.joint_order.len()
    {
        return Err(reject(
            "required joint samples are missing or from another transform epoch",
        ));
    }
    for (index, joint_name) in trace.joint_order.iter().enumerate() {
        let sample = trace
            .observation
            .joint_state
            .iter()
            .find(|sample| sample.joint_name == *joint_name)
            .ok_or_else(|| reject("policy joint order references a missing sample"))?;
        if trace.policy_observation.qpos[index].to_bits() != sample.q.to_bits()
            || sample
                .dq
                .is_some_and(|dq| trace.policy_observation.qvel[index].to_bits() != dq.to_bits())
        {
            return Err(reject(
                "policy joint state differs from its sensor-derived sample",
            ));
        }
    }
    let sensor_ids = identity_by_id
        .keys()
        .map(|sensor_id| (*sensor_id).to_string())
        .collect::<Vec<_>>();
    for detection in &trace.policy_observation.detections {
        if !identity_by_id.contains_key(detection.source.as_str())
            || !observation_class_by_id
                .get(detection.source.as_str())
                .is_some_and(|class| {
                    matches!(
                        class,
                        SensorClass::RgbCamera | SensorClass::DepthCamera | SensorClass::Rgbd
                    )
                })
            || detection.pose.is_empty()
            || detection.pose.iter().any(|value| !value.is_finite())
        {
            return Err(reject(
                "policy detection has no valid hardware sensor source",
            ));
        }
    }
    let serialized = serde_json::to_vec(trace)?;
    let mut digest = Sha256::new();
    digest.update(b"realityos.manipulation_benchmark/physical_sensor_trace/1\0");
    digest.update(serialized);

    Ok(PhysicalObservationEvidence {
        schema_version: BENCHMARK_SCHEMA_VERSION,
        status: PhysicalEvidenceStatus::Unverified,
        sensor_provenance_valid: true,
        requires_independent_measurement: true,
        trace_digest: hex::encode(digest.finalize()),
        frame_id: trace.observation.frame_id.clone(),
        policy_observation_id: trace.policy_observation.observation_id.clone(),
        sensor_ids,
        captured_at_s: min_capture,
        received_at_s: max_receive,
    })
}

fn is_simulation_clock(clock_domain: &str) -> bool {
    let clock = clock_domain.trim().to_ascii_lowercase();
    clock == "sim"
        || clock.contains("simulator")
        || clock.contains("simulation")
        || clock.contains("mujoco")
        || clock.contains("synthetic")
}
