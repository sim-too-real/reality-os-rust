#!/usr/bin/env python3
"""Independent, standard-library replay of supported probe sidecar branches."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import sys
from pathlib import Path
from typing import Any

FORMAT = "realityos.probe_measurement_sidecar"
OUTPUT_FORMAT = "realityos.independent_probe_interval_reference"
STANDARD_GRAVITY_M_S2 = 9.80665
SUPPORT_NORMAL_TOLERANCE = 1e-6
ROTATION_TOLERANCE = 1e-6
MAX_VERTICAL_TO_HORIZONTAL_PROBE_RATIO = 0.05
INTERVAL_TOLERANCE = 1e-12


class ReplayError(ValueError):
    """The record is invalid, mismatched, or outside this verifier's scope."""


def _fail(message: str) -> None:
    raise ReplayError(message)


def _require(condition: bool, message: str) -> None:
    if not condition:
        _fail(message)


def _u64(value: Any) -> bytes:
    _require(isinstance(value, int) and not isinstance(value, bool), "invalid unsigned integer")
    _require(0 <= value < 2**64, "unsigned integer out of range")
    return struct.pack("<Q", value)


def _f64(value: Any) -> bytes:
    _require(isinstance(value, (int, float)) and not isinstance(value, bool), "invalid f64")
    number = float(value)
    _require(math.isfinite(number), "non-finite f64")
    return struct.pack("<d", number)


def _push_string(data: bytearray, value: Any) -> None:
    _require(isinstance(value, str), "expected string")
    encoded = value.encode("utf-8")
    data.extend(_u64(len(encoded)))
    data.extend(encoded)


def _push_f64s(data: bytearray, values: Any, length: int | None = None) -> None:
    _require(isinstance(values, list), "expected numeric array")
    if length is not None:
        _require(len(values) == length, "numeric array has wrong length")
    for value in values:
        data.extend(_f64(value))


def _sha256(data: bytes | bytearray) -> str:
    return hashlib.sha256(data).hexdigest()


def _is_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdefABCDEF" for character in value)
    )


def _push_contact_pairs(data: bytearray, pairs: Any) -> None:
    _require(isinstance(pairs, list), "contact pairs must be an array")
    data.extend(_u64(len(pairs)))
    for pair in pairs:
        _require(isinstance(pair, dict), "contact pair must be an object")
        for key in ("body1", "geom1_name", "body2", "geom2_name"):
            value = pair.get(key)
            _require(isinstance(value, str) and value.strip(), "invalid contact-pair identity")
            _push_string(data, value)


def _push_contact_force_samples(data: bytearray, samples: Any) -> None:
    """Match crates/semantics push_probe::push_contact_force_samples."""
    _require(isinstance(samples, list), "contact force samples must be an array")
    data.extend(_u64(len(samples)))
    for sample in samples:
        _require(isinstance(sample, dict), "contact force sample must be an object")
        pair = sample.get("contact_pair")
        _require(isinstance(pair, dict), "contact force sample missing contact_pair")
        _push_contact_pairs(data, [pair])
        _push_f64s(data, sample.get("position_world_m"), 3)
        _push_f64s(data, sample.get("frame"), 9)
        _push_f64s(data, sample.get("friction"), 5)
        _push_f64s(data, sample.get("force_local"), 3)
        _push_f64s(data, sample.get("torque_local"), 3)
        data.extend(_f64(sample.get("normal_force_n")))
        data.extend(_f64(sample.get("tangential_force_n")))
        available = sample.get("force_available")
        _require(isinstance(available, bool), "contact force_available must be bool")
        data.append(int(available))


def _wrench_sample_digest(sample: dict[str, Any]) -> str:
    data = bytearray(b"realityos.wrench_sample/1\0")
    for key in (
        "sensor_id",
        "sensor_frame",
        "transform_epoch",
        "units",
        "clock_domain",
        "origin",
    ):
        _push_string(data, sample.get(key))
    _push_f64s(data, sample.get("wrench"), 6)
    _push_f64s(data, sample.get("error_bounds"), 6)
    data.extend(_f64(sample.get("capture_s")))
    data.extend(_f64(sample.get("receive_s")))
    data.extend(_u64(sample.get("sequence")))
    return _sha256(data)


def _validate_wrench_sample(sample: Any) -> None:
    _require(isinstance(sample, dict), "wrench sample must be an object")
    for key in ("sensor_id", "sensor_frame", "transform_epoch", "clock_domain"):
        _require(isinstance(sample.get(key), str) and sample[key].strip(), "invalid sample identity")
    _require(sample.get("units") == "N,N*m", "unsupported wrench units")
    capture_s = float(sample.get("capture_s"))
    receive_s = float(sample.get("receive_s"))
    _require(capture_s >= 0.0 and receive_s >= capture_s, "invalid sample timestamps")
    _push_f64s(bytearray(), sample.get("wrench"), 6)
    errors = sample.get("error_bounds")
    _push_f64s(bytearray(), errors, 6)
    _require(all(float(error) >= 0.0 for error in errors), "negative sample error bound")
    _require(sample.get("origin") in ("simulation", "hardware", "synthetic"), "unknown sample origin")
    digest = sample.get("digest")
    _require(_is_sha256(digest) and digest == _wrench_sample_digest(sample), "sample digest mismatch")


def _pose_sample_digest(sample: dict[str, Any]) -> str:
    contacts = sample.get("contact_pairs")
    forces = sample.get("contact_force_samples")
    if forces is not None:
        _require(isinstance(forces, list), "contact force samples must be an array")
        _require(isinstance(contacts, list), "complete contact trace required")
        data = bytearray(b"realityos.probe_object_pose_sample/3\0")
    elif contacts is not None:
        _require(isinstance(contacts, list), "complete contact trace required")
        data = bytearray(b"realityos.probe_object_pose_sample/2\0")
    else:
        data = bytearray(b"realityos.probe_object_pose_sample/1\0")
    _push_string(data, sample.get("object_id"))
    data.extend(_u64(sample.get("simulator_step")))
    sequence = sample.get("wrench_sequence")
    if sequence is None:
        data.append(0)
    else:
        data.append(1)
        data.extend(_u64(sequence))
    if contacts is not None:
        _push_contact_pairs(data, contacts)
    if forces is not None:
        _push_contact_force_samples(data, forces)
    data.extend(_f64(sample.get("capture_s")))
    _push_string(data, sample.get("clock_domain"))
    _push_f64s(data, sample.get("position_world_m"), 3)
    _push_f64s(data, sample.get("orientation_world_wxyz"), 4)
    return _sha256(data)


def _trace_digest(trace: dict[str, Any]) -> str:
    samples = trace.get("samples")
    _require(isinstance(samples, list), "motion trace samples must be an array")
    start_contacts = trace.get("start_contact_pairs")
    has_contact = isinstance(start_contacts, list) and all(
        isinstance(sample.get("contact_pairs"), list) for sample in samples
    )
    has_force = has_contact and all(
        isinstance(sample.get("contact_force_samples"), list) for sample in samples
    )
    start_sequence = trace.get("start_wrench_sequence")
    if has_force:
        data = bytearray(b"realityos.probe_object_pose_trace/4\0")
    elif has_contact:
        data = bytearray(b"realityos.probe_object_pose_trace/3\0")
    elif start_sequence is not None:
        data = bytearray(b"realityos.probe_object_pose_trace/2\0")
    else:
        data = bytearray(b"realityos.probe_object_pose_trace/1\0")
    _push_string(data, trace.get("object_id"))
    _push_string(data, trace.get("clock_domain"))
    data.extend(_f64(trace.get("start_s")))
    _push_f64s(data, trace.get("start_position_world_m"), 3)
    _push_f64s(data, trace.get("start_orientation_world_wxyz"), 4)
    if start_sequence is not None:
        data.extend(_u64(start_sequence))
    if isinstance(start_contacts, list):
        _push_contact_pairs(data, start_contacts)
    data.extend(_u64(len(samples)))
    for sample in samples:
        _push_string(data, sample.get("digest"))
    return _sha256(data)


def _orientation_valid(quaternion: Any) -> bool:
    if not isinstance(quaternion, list) or len(quaternion) != 4:
        return False
    if any(not math.isfinite(float(value)) for value in quaternion):
        return False
    return abs(sum(float(value) ** 2 for value in quaternion) - 1.0) <= ROTATION_TOLERANCE


def _validate_contact_pairs(pairs: Any) -> None:
    _require(isinstance(pairs, list), "contact pairs must be an array")
    for pair in pairs:
        _require(isinstance(pair, dict), "contact pair must be an object")
        for key in ("body1", "geom1_name", "body2", "geom2_name"):
            _require(isinstance(pair.get(key), str) and pair[key].strip(), "invalid contact-pair identity")


def _validate_motion_trace(trace: Any, measurement: dict[str, Any]) -> bool:
    _require(isinstance(trace, dict), "complete motion trace required")
    samples = trace.get("samples")
    _require(isinstance(samples, list) and samples, "empty motion trace")
    _require(all(isinstance(sample, dict) for sample in samples), "pose sample must be an object")
    start_contacts = trace.get("start_contact_pairs")
    contact_counts = int(start_contacts is not None) + sum(
        int(sample.get("contact_pairs") is not None) for sample in samples
    )
    _require(
        contact_counts == len(samples) + 1,
        "incomplete contact trace",
    )
    _validate_contact_pairs(start_contacts)

    target_id = measurement["intended_contact"]["target_body_id"]
    motion = measurement["motion"]
    _require(trace.get("object_id") == target_id, "motion trace target mismatch")
    _require(trace.get("start_s") == motion.get("start_s"), "motion trace start mismatch")
    _require(samples[-1].get("capture_s") == motion.get("end_s"), "motion trace end mismatch")

    start_position = trace.get("start_position_world_m")
    _push_f64s(bytearray(), start_position, 3)
    _require(_orientation_valid(trace.get("start_orientation_world_wxyz")), "invalid trace start orientation")
    _require(isinstance(start_contacts, list), "complete contact trace required")
    if trace.get("start_wrench_sequence") is not None:
        _u64(trace["start_wrench_sequence"])

    recorded_contact_lists = [start_contacts]
    previous_step: int | None = None
    previous_time = float(trace["start_s"])
    for index, sample in enumerate(samples):
        _require(sample.get("object_id") == target_id, "pose sample target mismatch")
        _require(sample.get("clock_domain") == trace.get("clock_domain"), "pose clock mismatch")
        step = sample.get("simulator_step")
        _require(isinstance(step, int) and not isinstance(step, bool) and step > 0, "invalid simulator step")
        if previous_step is not None:
            _require(step == previous_step + 1, "pose-step gap or order")
        capture_s = float(sample.get("capture_s"))
        _require(
            capture_s >= float(trace["start_s"])
            and (index == 0 or capture_s > previous_time),
            "pose-time order",
        )
        if index == 0 and capture_s == float(trace["start_s"]):
            _require(
                sample.get("position_world_m") == start_position
                and sample.get("orientation_world_wxyz") == trace.get("start_orientation_world_wxyz"),
                "start pose mismatch",
            )
        position = sample.get("position_world_m")
        _push_f64s(bytearray(), position, 3)
        _require(_orientation_valid(sample.get("orientation_world_wxyz")), "invalid pose orientation")
        pairs = sample.get("contact_pairs")
        _validate_contact_pairs(pairs)
        digest = sample.get("digest")
        _require(_is_sha256(digest) and digest == _pose_sample_digest(sample), "pose sample digest mismatch")
        recorded_contact_lists.append(pairs)
        previous_step = step
        previous_time = capture_s

    _require(isinstance(trace.get("clock_domain"), str) and trace["clock_domain"].strip(), "invalid trace clock")
    _require(math.isfinite(float(trace["start_s"])) and float(trace["start_s"]) >= 0.0, "invalid trace start time")
    end_position = samples[-1]["position_world_m"]
    displacement = [float(end_position[i]) - float(start_position[i]) for i in range(3)]
    recorded_displacement = motion.get("displacement_m")
    _push_f64s(bytearray(), recorded_displacement, 3)
    _require(
        all(abs(displacement[i] - float(recorded_displacement[i])) <= 1e-9 for i in range(3)),
        "motion trace displacement mismatch",
    )

    pose_by_sequence: dict[int, list[dict[str, Any]]] = {}
    for sample in samples:
        sequence = sample.get("wrench_sequence")
        if sequence is not None:
            _u64(sequence)
            pose_by_sequence.setdefault(sequence, []).append(sample)
    start_sequence = trace.get("start_wrench_sequence")
    for wrench in measurement.get("samples", []):
        sequence = wrench.get("sequence")
        paired_start = start_sequence == sequence and wrench.get("capture_s") == trace.get("start_s")
        explicit = [
            pose
            for pose in pose_by_sequence.get(sequence, [])
            if pose.get("capture_s") == wrench.get("capture_s")
        ]
        _require(len(explicit) + int(paired_start) == 1, "wrench-pose alignment mismatch")
    if start_sequence is not None:
        start_matches = [
            wrench
            for wrench in measurement.get("samples", [])
            if wrench.get("sequence") == start_sequence and wrench.get("capture_s") == trace.get("start_s")
        ]
        _require(len(start_matches) == 1, "wrench-pose alignment mismatch")
    for pose in samples:
        sequence = pose.get("wrench_sequence")
        if sequence is not None:
            paired_wrenches = [
                wrench
                for wrench in measurement.get("samples", [])
                if wrench.get("sequence") == sequence and wrench.get("capture_s") == pose.get("capture_s")
            ]
            _require(len(paired_wrenches) == 1, "wrench-pose alignment mismatch")

    resolution = float(motion["pose_resolution_m"])
    previous_time = float(trace["start_s"])
    crossing_end: float | None = None
    for sample in samples:
        dx = float(sample["position_world_m"][0]) - float(start_position[0])
        dy = float(sample["position_world_m"][1]) - float(start_position[1])
        if math.hypot(dx, dy) > resolution:
            crossing_end = float(sample["capture_s"])
            break
        previous_time = float(sample["capture_s"])
    first_motion = motion.get("first_motion_s")
    if crossing_end is not None and first_motion is not None:
        _require(float(first_motion) >= crossing_end, "motion onset precedes trace crossing")
    elif crossing_end is None and first_motion is None and not motion.get("unresolved_motion_before_force", False):
        pass
    else:
        _fail("motion onset does not match pose trace")

    trace_digest = trace.get("digest")
    _require(_is_sha256(trace_digest) and trace_digest == _trace_digest(trace), "motion trace digest mismatch")
    tool_id = measurement["intended_contact"]["tool_body_id"]
    has_intended_contact = any(
        (pair.get("body1") == tool_id and pair.get("body2") == target_id)
        or (pair.get("body1") == target_id and pair.get("body2") == tool_id)
        for contacts in recorded_contact_lists
        for pair in contacts
    )
    return has_intended_contact


def _measurement_digest(measurement: dict[str, Any]) -> str:
    trace = measurement.get("motion_trace")
    motion = measurement["motion"]
    intended = measurement["intended_contact"]
    data = bytearray(
        b"realityos.push_probe_measurement/7\0"
        if trace is not None
        else b"realityos.push_probe_measurement/6\0"
        if motion.get("unresolved_motion_before_force", False)
        else b"realityos.push_probe_measurement/5\0"
    )
    samples = measurement.get("samples")
    _require(isinstance(samples, list), "measurement samples must be an array")
    data.extend(_u64(len(samples)))
    for sample in samples:
        _push_string(data, sample.get("digest"))

    baseline = measurement.get("precontact_baseline")
    if baseline is None:
        data.append(0)
    else:
        data.append(1)
        _push_string(data, baseline.get("digest"))

    free_wrenches = measurement.get("sample_free_space_wrenches", [])
    _require(isinstance(free_wrenches, list), "free-space wrench array invalid")
    data.extend(_u64(len(free_wrenches)))
    for wrench in free_wrenches:
        _push_f64s(data, wrench, 6)
    free_errors = measurement.get("sample_free_space_wrench_error_bounds", [])
    _require(isinstance(free_errors, list), "free-space error array invalid")
    data.extend(_u64(len(free_errors)))
    for errors in free_errors:
        _push_f64s(data, errors, 6)

    sample_transforms = measurement.get("sample_sensor_transforms", [])
    _require(isinstance(sample_transforms, list), "sample transform array invalid")
    data.extend(_u64(len(sample_transforms)))
    for transform in sample_transforms:
        _push_transform_values(data, transform)
    _push_transform_values(data, measurement.get("sensor_to_support"))

    compensation = measurement.get("compensation")
    _push_f64s(data, compensation.get("bias"), 6)
    _push_f64s(data, compensation.get("gravity_wrench"), 6)
    _push_f64s(data, compensation.get("residual_error"), 6)
    _push_string(data, compensation.get("digest"))
    _push_f64s(data, measurement.get("mass_kg"), 2)
    _push_f64s(data, measurement.get("support_normal_world"), 3)

    for key in ("start_s", "end_s"):
        data.extend(_f64(motion.get(key)))
    _push_f64s(data, motion.get("displacement_m"), 3)
    for key in ("max_speed_m_s", "max_quasi_static_speed_m_s", "pose_resolution_m"):
        data.extend(_f64(motion.get(key)))
    first_motion = motion.get("first_motion_s")
    if first_motion is None:
        data.append(0)
    else:
        data.append(1)
        data.extend(_f64(first_motion))
    if motion.get("unresolved_motion_before_force", False):
        data.append(1)
    if trace is not None:
        _push_string(data, trace.get("digest"))

    _push_f64s(data, measurement.get("prior_support_friction"), 2)
    _push_string(data, measurement.get("current_observation_epoch"))
    _push_string(data, measurement.get("current_collision_model_digest"))
    for key in (
        "now_s",
        "max_sample_age_s",
        "max_inter_sample_gap_s",
        "max_sample_motion_skew_s",
    ):
        data.extend(_f64(measurement.get(key)))
    _push_f64s(data, measurement.get("max_wrench_rise_rate"), 6)
    for key in (
        "observation_epoch",
        "collision_model_digest",
        "tool_body_id",
        "target_body_id",
    ):
        _push_string(data, intended.get(key))
    contact_sets = intended.get("plausible_contact_sets")
    _require(isinstance(contact_sets, list), "plausible contact sets invalid")
    data.extend(_u64(len(contact_sets)))
    for contact_set in contact_sets:
        _require(isinstance(contact_set, list), "plausible contact set invalid")
        data.extend(_u64(len(contact_set)))
        for body_id in contact_set:
            _push_string(data, body_id)
    data.append(int(bool(measurement.get("runtime_contact_ambiguous", False))))
    data.append(int(bool(measurement.get("saturated", False))))
    _push_string(data, measurement.get("origin"))
    return _sha256(data)


def _push_transform_values(data: bytearray, transform: Any) -> None:
    _require(isinstance(transform, dict), "transform must be an object")
    rotation = transform.get("rotation_to_support")
    _require(isinstance(rotation, list) and len(rotation) == 3, "rotation must be 3 by 3")
    for row in rotation:
        _push_f64s(data, row, 3)
    _push_f64s(data, transform.get("translation_to_support_m"), 3)


def _validate_transform(transform: Any) -> None:
    _require(isinstance(transform, dict), "transform must be an object")
    rotation = transform.get("rotation_to_support")
    translation = transform.get("translation_to_support_m")
    _require(isinstance(rotation, list) and len(rotation) == 3, "rotation must be 3 by 3")
    for row in rotation:
        _push_f64s(bytearray(), row, 3)
    _push_f64s(bytearray(), translation, 3)
    for row in range(3):
        for column in range(3):
            dot = sum(float(rotation[row][axis]) * float(rotation[column][axis]) for axis in range(3))
            expected = 1.0 if row == column else 0.0
            _require(abs(dot - expected) <= ROTATION_TOLERANCE, "invalid rotation matrix")
    determinant = (
        float(rotation[0][0])
        * (float(rotation[1][1]) * float(rotation[2][2]) - float(rotation[1][2]) * float(rotation[2][1]))
        - float(rotation[0][1])
        * (float(rotation[1][0]) * float(rotation[2][2]) - float(rotation[1][2]) * float(rotation[2][0]))
        + float(rotation[0][2])
        * (float(rotation[1][0]) * float(rotation[2][1]) - float(rotation[1][1]) * float(rotation[2][0]))
    )
    _require(abs(determinant - 1.0) <= ROTATION_TOLERANCE, "invalid rotation determinant")


def _valid_interval(value: Any, name: str) -> list[float]:
    _require(isinstance(value, list) and len(value) == 2, f"{name} must be a two-value interval")
    lower, upper = float(value[0]), float(value[1])
    _require(math.isfinite(lower) and math.isfinite(upper) and lower <= upper, f"invalid {name}")
    return [lower, upper]


def _validate_compensation(compensation: Any) -> None:
    _require(isinstance(compensation, dict), "compensation must be an object")
    bias = compensation.get("bias")
    gravity = compensation.get("gravity_wrench")
    residual = compensation.get("residual_error")
    _push_f64s(bytearray(), bias, 6)
    _push_f64s(bytearray(), gravity, 6)
    _push_f64s(bytearray(), residual, 6)
    _require(all(float(value) >= 0.0 for value in residual), "negative compensation residual")
    data = bytearray(b"realityos.wrench_compensation/1\0")
    _push_f64s(data, bias, 6)
    _push_f64s(data, gravity, 6)
    _push_f64s(data, residual, 6)
    digest = compensation.get("digest")
    _require(_is_sha256(digest) and digest == _sha256(data), "compensation digest mismatch")


def _validate_intended_contact_identity(measurement: dict[str, Any]) -> tuple[str, str, str]:
    intended = measurement.get("intended_contact")
    _require(isinstance(intended, dict), "intended-contact evidence missing")
    epoch = measurement.get("current_observation_epoch")
    digest = measurement.get("current_collision_model_digest")
    _require(isinstance(epoch, str) and epoch.strip(), "observation epoch missing")
    _require(epoch == intended.get("observation_epoch"), "stale observation epoch")
    _require(_is_sha256(digest) and digest == intended.get("collision_model_digest"), "stale collision model")
    tool = intended.get("tool_body_id")
    target = intended.get("target_body_id")
    _require(isinstance(tool, str) and tool.strip(), "tool body missing")
    _require(isinstance(target, str) and target.strip() and target != tool, "target body invalid")
    pairs = intended.get("plausible_contact_sets")
    _require(isinstance(pairs, list) and pairs, "plausible contact sets missing")
    _require(isinstance(pairs[0], list), "invalid intended-contact declaration")
    return epoch, tool, target


def _validate_ambiguous_runtime_contact_measurement(
    measurement: dict[str, Any],
) -> tuple[list[float], bool]:
    """Structural verify for sealed unexpected-contact measurements.

    Matches push_probe semantics: runtime_contact_ambiguous forbids friction
    interval updates. Digests and NoInference(unexpected_contact) are checked;
    slip-onset interval math is not invented.
    """
    _require(measurement.get("origin") == "simulation", "unsupported evidence origin")
    _require(
        measurement.get("runtime_contact_ambiguous", False) is True,
        "expected runtime_contact_ambiguous",
    )
    _require(not measurement.get("saturated", False), "saturated measurement unsupported")
    mass = _valid_interval(measurement.get("mass_kg"), "mass")
    _require(mass[0] > 0.0, "nonpositive mass")
    prior = _valid_interval(measurement.get("prior_support_friction"), "prior friction")
    _require(prior[0] >= 0.0, "negative friction prior")
    _validate_intended_contact_identity(measurement)
    _validate_compensation(measurement.get("compensation"))
    _validate_transform(measurement.get("sensor_to_support"))
    motion = measurement.get("motion")
    _require(isinstance(motion, dict), "motion window missing")
    start_s, end_s = float(motion.get("start_s")), float(motion.get("end_s"))
    _require(start_s >= 0.0 and end_s > start_s, "invalid motion window")
    samples = measurement.get("samples")
    _require(isinstance(samples, list) and samples, "ambiguous-contact measurement lacks samples")
    for sample in samples:
        _require(isinstance(sample, dict), "wrench sample must be an object")
        digest = sample.get("digest")
        _require(_is_sha256(digest), "wrench sample digest missing")
    trace = measurement.get("motion_trace")
    if trace is not None:
        _require(isinstance(trace, dict), "motion_trace must be an object")
        trace_digest = trace.get("digest")
        _require(
            _is_sha256(trace_digest) and trace_digest == _trace_digest(trace),
            "motion trace digest mismatch",
        )
    # has_contact=False: interval replay path is intentionally not taken.
    return prior, False


def _validate_measurement(measurement: Any) -> tuple[list[float], bool]:
    _require(isinstance(measurement, dict), "measurement must be an object")
    _require(measurement.get("origin") == "simulation", "unsupported evidence origin")
    if measurement.get("runtime_contact_ambiguous", False):
        return _validate_ambiguous_runtime_contact_measurement(measurement)
    _require(not measurement.get("saturated", False), "saturated measurement unsupported")

    mass = _valid_interval(measurement.get("mass_kg"), "mass")
    _require(mass[0] > 0.0, "nonpositive mass")
    prior = _valid_interval(measurement.get("prior_support_friction"), "prior friction")
    _require(prior[0] >= 0.0, "negative friction prior")
    epoch, tool, target = _validate_intended_contact_identity(measurement)
    intended = measurement["intended_contact"]
    pairs = intended.get("plausible_contact_sets")
    _require(isinstance(pairs, list) and len(pairs) == 1, "ambiguous intended-contact declaration")
    _require(isinstance(pairs[0], list), "invalid intended-contact declaration")
    _require(sorted(set(pairs[0])) == sorted((tool, target)), "intended-contact declaration mismatch")

    normal = measurement.get("support_normal_world")
    _push_f64s(bytearray(), normal, 3)
    normal_length = math.hypot(float(normal[0]), float(normal[1]), float(normal[2]))
    _require(
        abs(normal_length - 1.0) <= SUPPORT_NORMAL_TOLERANCE
        and abs(float(normal[0])) <= SUPPORT_NORMAL_TOLERANCE
        and abs(float(normal[1])) <= SUPPORT_NORMAL_TOLERANCE
        and abs(float(normal[2]) - 1.0) <= SUPPORT_NORMAL_TOLERANCE,
        "unsupported support normal",
    )
    _validate_compensation(measurement.get("compensation"))
    _validate_transform(measurement.get("sensor_to_support"))

    motion = measurement.get("motion")
    _require(isinstance(motion, dict), "motion window missing")
    start_s, end_s = float(motion.get("start_s")), float(motion.get("end_s"))
    _require(start_s >= 0.0 and end_s > start_s, "invalid motion window")
    displacement = motion.get("displacement_m")
    _push_f64s(bytearray(), displacement, 3)
    speed = float(motion.get("max_speed_m_s"))
    quasi_static = float(motion.get("max_quasi_static_speed_m_s"))
    resolution = float(motion.get("pose_resolution_m"))
    _require(speed >= 0.0 and quasi_static > 0.0 and resolution > 0.0, "invalid motion bounds")
    _require(not motion.get("unresolved_motion_before_force", False), "unbracketed motion unsupported")
    _require(speed <= quasi_static, "dynamic motion unsupported")
    vertical = abs(float(displacement[2]))
    horizontal = math.hypot(float(displacement[0]), float(displacement[1]))
    _require(
        not (
            vertical > resolution
            and vertical > MAX_VERTICAL_TO_HORIZONTAL_PROBE_RATIO * max(horizontal, resolution)
        ),
        "nonplanar motion unsupported",
    )

    now_s = float(measurement.get("now_s"))
    max_age = float(measurement.get("max_sample_age_s"))
    max_gap = float(measurement.get("max_inter_sample_gap_s"))
    max_skew = float(measurement.get("max_sample_motion_skew_s"))
    rise_rate = measurement.get("max_wrench_rise_rate")
    _push_f64s(bytearray(), rise_rate, 6)
    _require(
        now_s >= end_s and max_age > 0.0 and max_gap > 0.0 and max_skew >= 0.0
        and all(float(rate) >= 0.0 for rate in rise_rate),
        "invalid timing policy",
    )

    samples = measurement.get("samples")
    _require(isinstance(samples, list) and len(samples) >= 2, "insufficient wrench samples")
    baseline = measurement.get("precontact_baseline")
    _require(isinstance(baseline, dict), "precontact baseline missing")
    _validate_wrench_sample(baseline)
    first = samples[0]
    _require(
        baseline.get("origin") == measurement["origin"]
        and baseline.get("transform_epoch") == epoch
        and baseline.get("sensor_id") == first.get("sensor_id")
        and baseline.get("sensor_frame") == first.get("sensor_frame")
        and baseline.get("units") == first.get("units")
        and baseline.get("clock_domain") == first.get("clock_domain")
        and baseline.get("sequence") < first.get("sequence")
        and baseline.get("capture_s") < first.get("capture_s"),
        "stale precontact baseline",
    )
    _require(float(first["capture_s"]) - float(baseline["capture_s"]) <= max_gap, "precontact gap exceeded")

    previous: dict[str, Any] | None = None
    for sample in samples:
        _validate_wrench_sample(sample)
        _require(sample.get("origin") == measurement["origin"], "sample origin mismatch")
        _require(sample.get("transform_epoch") == epoch, "sample observation epoch mismatch")
        capture_s, receive_s = float(sample["capture_s"]), float(sample["receive_s"])
        _require(start_s <= capture_s <= end_s and receive_s <= now_s, "sample outside measurement window")
        _require(receive_s - capture_s <= max_age, "stale wrench sample")
        if previous is not None:
            _require(
                sample.get("sensor_id") == previous.get("sensor_id")
                and sample.get("sensor_frame") == previous.get("sensor_frame")
                and sample.get("clock_domain") == previous.get("clock_domain")
                and sample.get("sequence") > previous.get("sequence")
                and capture_s > float(previous["capture_s"])
                and receive_s >= float(previous["receive_s"]),
                "wrench sample order mismatch",
            )
            _require(capture_s - float(previous["capture_s"]) <= max_gap, "wrench sample gap exceeded")
        previous = sample
    _require(now_s - float(samples[-1]["receive_s"]) <= max_age, "stale wrench sample")

    free_wrenches = measurement.get("sample_free_space_wrenches", [])
    free_errors = measurement.get("sample_free_space_wrench_error_bounds", [])
    sample_transforms = measurement.get("sample_sensor_transforms", [])
    _require(isinstance(free_wrenches, list) and isinstance(free_errors, list), "free-space arrays invalid")
    _require(
        (not free_wrenches and not free_errors)
        or (len(free_wrenches) == len(samples) and len(free_errors) == len(samples)),
        "free-space arrays do not match samples",
    )
    _require(isinstance(sample_transforms, list), "sample transforms invalid")
    _require(not sample_transforms or len(sample_transforms) == len(samples), "sample transforms do not match samples")
    for values in free_wrenches:
        _push_f64s(bytearray(), values, 6)
    for values in free_errors:
        _push_f64s(bytearray(), values, 6)
        _require(all(float(value) >= 0.0 for value in values), "negative free-space error bound")

    first_motion = motion.get("first_motion_s")
    displacement_xy = math.hypot(float(displacement[0]), float(displacement[1]))
    if first_motion is None:
        _require(displacement_xy <= resolution, "unbracketed motion unsupported")
        mode = "sticking"
    else:
        onset = float(first_motion)
        _require(start_s <= onset <= end_s and displacement_xy > resolution, "invalid slip onset")
        mode = "slip_onset"
    trace = measurement.get("motion_trace")
    has_contact = _validate_motion_trace(trace, measurement)
    _require(trace["clock_domain"] == samples[0]["clock_domain"], "motion and wrench clocks differ")
    if has_contact:
        _require(mode == "slip_onset", "sticking mode is unsupported by reference v1")

    return prior, has_contact


def _force_interval(sample: dict[str, Any], measurement: dict[str, Any], onset_s: float) -> list[float]:
    samples = measurement["samples"]
    index_by_sequence = {sample_item["sequence"]: index for index, sample_item in enumerate(samples)}
    index = index_by_sequence.get(sample["sequence"])
    _require(index is not None, "sample sequence missing from measurement")
    compensation = measurement["compensation"]
    bias = compensation["bias"]
    if measurement.get("sample_free_space_wrenches"):
        gravity = measurement["sample_free_space_wrenches"][index]
        residual = [
            float(compensation["residual_error"][axis])
            + float(measurement["sample_free_space_wrench_error_bounds"][index][axis])
            for axis in range(6)
        ]
    else:
        gravity = compensation["gravity_wrench"]
        residual = compensation["residual_error"]

    transforms = measurement.get("sample_sensor_transforms", [])
    transform = transforms[index] if transforms else measurement["sensor_to_support"]
    _validate_transform(transform)
    skew = abs(float(sample["capture_s"]) - onset_s)
    _require(skew <= float(measurement["max_sample_motion_skew_s"]), "force sample too far from onset")

    sensor_lower: list[float] = []
    sensor_upper: list[float] = []
    for axis in range(6):
        center = float(sample["wrench"][axis]) - float(bias[axis]) - float(gravity[axis])
        error = (
            float(sample["error_bounds"][axis])
            + float(residual[axis])
            + float(measurement["max_wrench_rise_rate"][axis]) * skew
        )
        sensor_lower.append(-(center + error))
        sensor_upper.append(-(center - error))

    support_force_lower: list[float] = []
    support_force_upper: list[float] = []
    rotation = transform["rotation_to_support"]
    for row in range(3):
        lower = 0.0
        upper = 0.0
        for axis, coefficient in enumerate(rotation[row]):
            coefficient = float(coefficient)
            if coefficient >= 0.0:
                lower += coefficient * sensor_lower[axis]
                upper += coefficient * sensor_upper[axis]
            else:
                lower += coefficient * sensor_upper[axis]
                upper += coefficient * sensor_lower[axis]
        support_force_lower.append(lower)
        support_force_upper.append(upper)

    tangential_x = _magnitude_interval(support_force_lower[0], support_force_upper[0])
    tangential_y = _magnitude_interval(support_force_lower[1], support_force_upper[1])
    tangential = [math.hypot(tangential_x[0], tangential_y[0]), math.hypot(tangential_x[1], tangential_y[1])]
    normal = [
        float(measurement["mass_kg"][0]) * STANDARD_GRAVITY_M_S2 - support_force_upper[2],
        float(measurement["mass_kg"][1]) * STANDARD_GRAVITY_M_S2 - support_force_lower[2],
    ]
    _require(normal[0] > 0.0, "nonpositive normal load")
    return [tangential[0], tangential[1], normal[0], normal[1]]


def _magnitude_interval(lower: float, upper: float) -> list[float]:
    maximum = max(abs(lower), abs(upper))
    minimum = 0.0 if lower <= 0.0 <= upper else min(abs(lower), abs(upper))
    return [minimum, maximum]


def _recompute_inference(measurement: dict[str, Any], prior: list[float], has_contact: bool) -> tuple[dict[str, Any], list[float]]:
    if measurement.get("runtime_contact_ambiguous", False):
        # Matches crates/semantics push_probe: ambiguous contact → UnexpectedContact.
        return {"NoInference": "unexpected_contact"}, prior
    if not has_contact:
        return {"NoInference": "no_intended_contact"}, prior

    motion = measurement["motion"]
    _require(motion.get("first_motion_s") is not None, "slip onset required for supported update")
    onset_s = float(motion["first_motion_s"])
    samples = measurement["samples"]
    before = next((sample for sample in reversed(samples) if float(sample["capture_s"]) < onset_s), None)
    after = next((sample for sample in samples if float(sample["capture_s"]) >= onset_s), None)
    _require(before is not None and after is not None, "slip onset not bracketed by wrench samples")
    pre_force = _force_interval(before, measurement, onset_s)
    post_force = _force_interval(after, measurement, onset_s)
    _require(pre_force[2] > 0.0 and post_force[2] > 0.0, "nonpositive normal load")
    _require(pre_force[0] > 0.0 and post_force[1] > 0.0, "unidentifiable friction")

    bounds = [pre_force[0] / pre_force[3], post_force[1] / post_force[2]]
    _require(all(math.isfinite(value) for value in bounds) and bounds[0] >= 0.0, "unidentifiable friction bounds")
    _require(bounds[0] <= bounds[1], "empty friction bounds")
    interval = [max(prior[0], bounds[0]), min(prior[1], bounds[1])]
    # Matches crates/semantics push_probe: empty prior∩measurement →
    # PriorMeasurementIncompatible (NoInference), not a hard verifier crash.
    if interval[0] > interval[1]:
        return {"NoInference": "prior_measurement_incompatible"}, prior
    return {"Updated": {"interval": interval, "mode": "slip_onset"}}, interval


def _assert_interval_equal(actual: Any, expected: list[float], message: str) -> None:
    _require(isinstance(actual, list) and len(actual) == 2, message)
    _require(
        all(abs(float(actual[index]) - expected[index]) <= INTERVAL_TOLERANCE for index in range(2)),
        message,
    )


def _inference_equal(stored: Any, recomputed: dict[str, Any]) -> bool:
    if not isinstance(stored, dict) or set(stored) != set(recomputed):
        return False
    if "NoInference" in recomputed:
        return stored.get("NoInference") == recomputed["NoInference"]
    got = stored.get("Updated")
    want = recomputed["Updated"]
    return (
        isinstance(got, dict)
        and got.get("mode") == want["mode"]
        and isinstance(got.get("interval"), list)
        and len(got["interval"]) == 2
        and all(
            abs(float(got["interval"][index]) - float(want["interval"][index])) <= INTERVAL_TOLERANCE
            for index in range(2)
        )
    )


def _verify_sidecar(sidecar: dict[str, Any]) -> dict[str, Any]:
    _require(isinstance(sidecar, dict), "sidecar must be an object")
    _require(sidecar.get("format") == FORMAT and sidecar.get("schema_version") == 1, "unsupported sidecar format")
    _require(isinstance(sidecar.get("trial_id"), str) and sidecar["trial_id"].strip(), "trial id missing")
    _require(_is_sha256(sidecar.get("trial_digest")), "invalid trial digest")
    records = sidecar.get("measurements")
    _require(isinstance(records, list) and records, "sidecar has no measurements")

    summaries: list[dict[str, Any]] = []
    for index, record in enumerate(records):
        _require(isinstance(record, dict), f"measurement record {index} invalid")
        _require(isinstance(record.get("action_key"), str) and record["action_key"].strip(), "action key missing")
        measurement = record.get("measurement")
        prior, has_contact = _validate_measurement(measurement)
        _require(record.get("prior_support_friction_interval") == prior, "prior interval mismatch")
        expected_digest = _measurement_digest(measurement)
        _require(
            _is_sha256(record.get("measurement_digest"))
            and record["measurement_digest"] == expected_digest,
            "measurement digest mismatch",
        )
        recomputed, posterior = _recompute_inference(measurement, prior, has_contact)
        _require(_inference_equal(record.get("inference"), recomputed), "stored inference mismatch")
        stored_posterior = record.get("posterior_support_friction_interval")
        _assert_interval_equal(stored_posterior, posterior, "stored posterior interval mismatch")

        ambiguous = bool(measurement.get("runtime_contact_ambiguous", False))
        trace = measurement.get("motion_trace")
        first_bracket = None
        if isinstance(trace, dict) and not ambiguous:
            start = trace["start_position_world_m"]
            previous_time = float(trace["start_s"])
            for pose in trace["samples"]:
                dx = float(pose["position_world_m"][0]) - float(start[0])
                dy = float(pose["position_world_m"][1]) - float(start[1])
                if math.hypot(dx, dy) > float(measurement["motion"]["pose_resolution_m"]):
                    first_bracket = [previous_time, float(pose["capture_s"])]
                    break
                previous_time = float(pose["capture_s"])

        if "Updated" in recomputed:
            inference_name = "Updated(SlipOnset)"
            interval = recomputed["Updated"]["interval"]
        elif recomputed.get("NoInference") == "unexpected_contact":
            inference_name = "NoInference(UnexpectedContact)"
            interval = prior
        elif recomputed.get("NoInference") == "prior_measurement_incompatible":
            inference_name = "NoInference(PriorMeasurementIncompatible)"
            interval = prior
        elif recomputed.get("NoInference") == "no_intended_contact":
            inference_name = "NoInference(NoIntendedContact)"
            interval = prior
        else:
            inference_name = f"NoInference({recomputed.get('NoInference')})"
            interval = prior
        pose_count = (
            len(trace["samples"])
            if isinstance(trace, dict) and isinstance(trace.get("samples"), list)
            else 0
        )
        summaries.append(
            {
                "action_key": record["action_key"],
                "measurement_digest": expected_digest,
                "inference": inference_name,
                "prior_support_friction_interval": prior,
                "interval": interval,
                "first_xy_motion_bracket_s": first_bracket,
                "sample_count": len(measurement["samples"]),
                "pose_sample_count": pose_count,
                "motion_trace_status": "verified" if isinstance(trace, dict) else "absent",
                "runtime_contact_ambiguous": ambiguous,
                "interval_replay_supported": not ambiguous,
            }
        )

    ambiguous_only = summaries and all(
        row.get("runtime_contact_ambiguous") for row in summaries
    )
    mixed = any(row.get("runtime_contact_ambiguous") for row in summaries) and not ambiguous_only
    if ambiguous_only:
        status = "PASS_STRUCTURAL_AMBIGUOUS_CONTACT"
    elif mixed:
        status = "PASS_MIXED_AMBIGUOUS_CONTACT"
    else:
        status = "PASS"

    return {
        "format": OUTPUT_FORMAT,
        "schema_version": 1,
        "status": status,
        "trial_id": sidecar["trial_id"],
        "trial_digest": sidecar["trial_digest"],
        "measurement_count": len(summaries),
        "measurements": summaries,
        "interval_replay_supported": status == "PASS",
        "limitations": (
            [
                "runtime_contact_ambiguous measurements refuse friction-interval replay; "
                "structural digests and NoInference(unexpected_contact) are verified only."
            ]
            if status != "PASS"
            else []
        ),
    }


def verify_sidecar(sidecar: dict[str, Any]) -> dict[str, Any]:
    """Validate and independently replay supported records in a v1 sidecar."""
    try:
        return _verify_sidecar(sidecar)
    except ReplayError:
        raise
    except (AttributeError, IndexError, KeyError, TypeError, ValueError, OverflowError, struct.error) as error:
        raise ReplayError(f"invalid sidecar: {error}") from error


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sidecar", type=Path)
    args = parser.parse_args(argv)
    try:
        sidecar = json.loads(args.sidecar.read_text(encoding="utf-8"))
        report = verify_sidecar(sidecar)
    except (OSError, json.JSONDecodeError, ReplayError, KeyError, TypeError, ValueError) as error:
        print(json.dumps({"status": "FAIL", "error": str(error)}, indent=2), file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
