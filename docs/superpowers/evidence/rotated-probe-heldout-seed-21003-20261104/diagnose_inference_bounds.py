import json
import sys
from pathlib import Path
root = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(root / 'crates' / 'verify'))
import probe_interval_reference as ref
sidecar_path = Path(__file__).resolve().parent / 'PROBE_MEASUREMENT_SIDECAR.json'
sidecar = json.loads(sidecar_path.read_text(encoding='utf-8'))
rows = []
for record in sidecar['measurements']:
    measurement = record['measurement']
    motion = measurement['motion']
    onset = float(motion['first_motion_s'])
    samples = measurement['samples']
    before = next(sample for sample in reversed(samples) if float(sample['capture_s']) < onset)
    after = next(sample for sample in samples if float(sample['capture_s']) >= onset)
    pre = ref._force_interval(before, measurement, onset)
    post = ref._force_interval(after, measurement, onset)
    bounds = [pre[0] / pre[3], post[1] / post[2]]
    prior = list(record['prior_support_friction_interval'])
    intersection = [max(prior[0], bounds[0]), min(prior[1], bounds[1])]
    if bounds[0] > bounds[1]:
        result = 'measurement_bounds_empty'
    elif intersection[0] > intersection[1]:
        result = 'measurement_bounds_do_not_overlap_prior'
    else:
        result = 'updated'
    rows.append({
        'action_key': record['action_key'],
        'measurement_digest': record['measurement_digest'],
        'inference': record['inference'],
        'motion_first_motion_s': onset,
        'motion_unresolved_before_force': motion['unresolved_motion_before_force'],
        'prior_support_friction_interval': prior,
        'before_sample': {'sequence': before['sequence'], 'capture_s': before['capture_s'], 'force_interval_n': pre[:2], 'normal_interval_n': pre[2:]},
        'after_sample': {'sequence': after['sequence'], 'capture_s': after['capture_s'], 'force_interval_n': post[:2], 'normal_interval_n': post[2:]},
        'single_measurement_friction_bounds': bounds,
        'prior_intersection': intersection,
        'diagnosis': result,
    })
result = {
    'format': 'realityos.rotated_probe_inference_bound_diagnosis',
    'schema_version': 1,
    'status': 'OFFLINE_RECOMPUTATION_ONLY',
    'trial_id': sidecar['trial_id'],
    'trial_digest': sidecar['trial_digest'],
    'method': 'Independently recompute before/after wrench force and normal intervals at each recorded slip onset using the repository offline reference functions; derive the single-measurement friction bounds and intersect them with the frozen prior. No simulator or control code is executed.',
    'measurements': rows,
}
out = Path(__file__).resolve().parent / 'INFERENCE_BOUND_DIAGNOSIS.json'
with out.open('x', encoding='utf-8', newline='\n') as f:
    json.dump(result, f, indent=2, sort_keys=True)
    f.write('\n')
print(json.dumps(result, indent=2))
