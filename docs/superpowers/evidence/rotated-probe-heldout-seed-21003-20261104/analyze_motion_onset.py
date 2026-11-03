from __future__ import annotations
import json, math, re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
EVIDENCE = Path(__file__).resolve().parent
SIDECAR = EVIDENCE / 'PROBE_MEASUREMENT_SIDECAR.json'
REGISTRY = ROOT / 'robots/benchmarks/manipulation/embodiments.json'
EXECUTION = ROOT / 'crates/verify/src/manipulation_benchmark/execution.rs'
MOTION_THRESHOLD_M = 0.001
FORCE_THRESHOLD_N = 0.5


def planar_distance(position: list[float], origin: list[float]) -> float:
    return math.hypot(position[0] - origin[0], position[1] - origin[1])


def lower_bound_at(measurement: dict, profile: dict, now_s: float) -> tuple[float | None, int | None, float | None]:
    samples = measurement['samples']
    if not (len(samples) == len(measurement['sample_free_space_wrenches'])
            == len(measurement['sample_free_space_wrench_error_bounds'])
            == len(measurement['sample_sensor_transforms'])):
        return None, None, None
    eligible = [(i, sample) for i, sample in enumerate(samples)
                if math.isfinite(sample['capture_s']) and sample['capture_s'] <= now_s]
    if not eligible:
        return None, None, None
    index, sample = max(eligible, key=lambda pair: (pair[1]['capture_s'], pair[1]['sequence']))
    age_s = now_s - sample['capture_s']
    if not math.isfinite(age_s) or age_s < 0.0 or age_s > profile['limit_policy']['max_sample_age_s']:
        return None, index, age_s
    free = measurement['sample_free_space_wrenches'][index]
    rotation = measurement['sample_sensor_transforms'][index]['rotation_to_support']
    sensor_force = [sample['wrench'][axis] - profile['bias'][axis] - free[axis] for axis in range(6)]
    sensor_error = [
        sample['error_bounds'][axis]
        + profile['error_bounds'][axis]
        + profile['free_space_model_error_bounds'][axis]
        + profile['limit_policy']['max_rise_rate'][axis] * age_s
        for axis in range(6)
    ]
    lower_components = []
    for axis in (0, 1):
        center = sum(rotation[axis][source] * sensor_force[source] for source in range(3))
        error = sum(abs(rotation[axis][source]) * sensor_error[source] for source in range(3))
        low, high = center - error, center + error
        lower_components.append(0.0 if low <= 0.0 <= high else min(abs(low), abs(high)))
    return math.hypot(*lower_components), index, age_s


def main() -> None:
    sidecar = json.loads(SIDECAR.read_text(encoding='utf-8'))
    registry = json.loads(REGISTRY.read_text(encoding='utf-8'))
    profile = next(item['simulation_wrench_profile'] for item in registry['registrations']
                   if item['robot_id'] == 'menagerie_iiwa14')
    source = EXECUTION.read_text(encoding='utf-8')
    match = re.search(r'const PROBE_CONTROL_UPDATE_STEPS: u32 = (\d+);', source)
    if not match:
        raise RuntimeError('could not read frozen probe control batch size')
    control_update_steps = int(match.group(1))

    outcomes = []
    for record in sidecar['measurements']:
        measurement = record['measurement']
        trace = measurement['motion_trace']
        poses = trace['samples']
        origin = trace['start_position_world_m']
        crossing = None
        first_supported_force = None
        evaluated = []
        for pose in poses:
            displacement = planar_distance(pose['position_world_m'], origin)
            force, sample_index, age = lower_bound_at(measurement, profile, pose['capture_s'])
            row = {
                'time_s': pose['capture_s'],
                'simulator_step': pose['simulator_step'],
                'wrench_sequence': pose.get('wrench_sequence'),
                'planar_displacement_m': displacement,
                'tangential_force_lower_bound_n': force,
                'matched_sample_index': sample_index,
                'sample_age_s': age,
            }
            evaluated.append(row)
            if first_supported_force is None and force is not None and force >= FORCE_THRESHOLD_N:
                first_supported_force = row
            if crossing is None and displacement > MOTION_THRESHOLD_M:
                crossing = row
        if not poses:
            raise RuntimeError('sidecar motion trace has no pose samples')
        sample_dts = [right['capture_s'] - left['capture_s']
                      for left, right in zip(poses, poses[1:])]
        timestep_s = sorted(sample_dts)[len(sample_dts) // 2]
        start_s = trace['start_s']
        batch_crossing = None
        endpoint_rows = []
        for batch in range(1, len(poses) // control_update_steps + 2):
            target_time = start_s + batch * control_update_steps * timestep_s
            eligible = [row for row in evaluated if row['time_s'] <= target_time + 1e-12]
            if not eligible:
                continue
            endpoint = eligible[-1]
            endpoint = {**endpoint, 'control_batch_index': batch,
                        'nominal_batch_end_s': target_time}
            endpoint_rows.append(endpoint)
            if endpoint['planar_displacement_m'] > MOTION_THRESHOLD_M:
                batch_crossing = endpoint
                break

        outcomes.append({
            'action_key': record['action_key'],
            'measurement_digest': record['measurement_digest'],
            'measurement_inference': record['inference'],
            'motion_audit': measurement['motion'],
            'pose_sample_count': len(poses),
            'wrench_sample_count': len(measurement['samples']),
            'inferred_timestep_s': timestep_s,
            'control_update_steps': control_update_steps,
            'control_update_interval_s': control_update_steps * timestep_s,
            'first_force_lower_bound_at_or_above_0_5_n': first_supported_force,
            'first_pose_sample_over_1_mm': crossing,
            'first_100_step_endpoint_over_1_mm': batch_crossing,
            'force_supported_before_motion_crossing': bool(
                first_supported_force and crossing
                and first_supported_force['time_s'] < crossing['time_s']
            ),
            'force_supported_at_first_motion_crossing': bool(
                crossing and crossing['tangential_force_lower_bound_n'] is not None
                and crossing['tangential_force_lower_bound_n'] >= FORCE_THRESHOLD_N
            ),
            'runtime_batch_endpoint_would_resolve_motion': bool(
                batch_crossing
                and batch_crossing['tangential_force_lower_bound_n'] is not None
                and batch_crossing['tangential_force_lower_bound_n'] >= FORCE_THRESHOLD_N
            ),
            'motion_over_1_mm_and_force_under_0_5_n_at_batch_endpoint': bool(
                batch_crossing
                and (batch_crossing['tangential_force_lower_bound_n'] is None
                     or batch_crossing['tangential_force_lower_bound_n'] < FORCE_THRESHOLD_N)
            ),
            'boundary_samples': [row for row in evaluated if crossing and
                                 abs(row['time_s'] - crossing['time_s']) <= 0.004],
            'batch_endpoint_samples': endpoint_rows[-3:],
        })

    result = {
        'format': 'realityos.rotated_probe_motion_onset_analysis',
        'schema_version': 1,
        'status': 'OFFLINE_RECOMPUTATION_ONLY',
        'trial_id': sidecar['trial_id'],
        'trial_digest': sidecar['trial_digest'],
        'thresholds': {'planar_motion_m': MOTION_THRESHOLD_M,
                       'tangential_force_lower_bound_n': FORCE_THRESHOLD_N},
        'method': 'Recompute the source tangential-force lower bound from each time-aligned pose sample, raw wrench, free-space wrench, sensor transform, uncertainty, rate bound, and sample age; compare with the simulator executor’s 100-step observation cadence.',
        'control_batch_constant_source': 'crates/verify/src/manipulation_benchmark/execution.rs',
        'measurements': outcomes,
        'scope': 'No simulator or control code imported or executed. This analysis is limited to the frozen single-case simulation sidecar and does not establish physical performance.'
    }
    output = EVIDENCE / 'OFFLINE_ANALYSIS.json'
    with output.open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    print(json.dumps({
        'status': result['status'],
        'trial_id': result['trial_id'],
        'measurements': [{
            'action_key': item['action_key'],
            'first_pose_crossing': item['first_pose_sample_over_1_mm'],
            'first_batch_endpoint_crossing': item['first_100_step_endpoint_over_1_mm'],
            'force_supported_before_crossing': item['force_supported_before_motion_crossing'],
            'force_supported_at_crossing': item['force_supported_at_first_motion_crossing'],
            'endpoint_force_below_threshold': item['motion_over_1_mm_and_force_under_0_5_n_at_batch_endpoint'],
        } for item in outcomes]
    }, indent=2))

if __name__ == '__main__':
    main()
