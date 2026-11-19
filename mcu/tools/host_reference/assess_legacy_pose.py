#!/usr/bin/env python3
"""Offline legacy pose assessment; reads saved captures, never a probe.

Run: python tools/assess_legacy_pose.py [--captures PATH] [--output FILE]
Defaults match assess_legacy_imu_timing.py: mcu/build/OdomTune/captures,
with output in legacy-error-assessment/pose-errors.json.

Requires <captures>/physical-creep-01/analysis.json, including drive_zero_hold.
Turn results are recomputed from raw captures; the existing creep analysis is
incorporated as a dependency, not rebuilt by this script. All errors remain
conditional on the stated reference, not calibrated accuracy.
"""
import argparse
import json
import math
from pathlib import Path
import statistics
import analyze_odom_capture as capture

REPO = Path(__file__).resolve().parents[1]


def get(row, path):
    return capture.get(row, path)


def assess(base, name, nominal):
    directory = base / name
    files = sorted(directory.glob('sample-*.ndjson'))
    rows, diagnostics, errors = capture.load(files)
    capture.set_times(rows)
    yaw = [get(rows[0], 's_odom.angle.z')]
    for previous, current in zip(rows, rows[1:]):
        increment = math.remainder(get(current, 's_odom.angle.z') -
                                   get(previous, 's_odom.angle.z'), 2 * math.pi)
        yaw.append(yaw[-1] + increment)
    moving = [i for i, row in enumerate(rows)
              if any(get(row, 's_chassis.kine.control.' + k) != 0 for k in ('vx', 'vy', 'wz'))]
    first, last = moving[0], moving[-1]
    pre_end = max(i for i in range(first) if get(rows[i], 's_fusion.stationary'))
    pre = [i for i, row in enumerate(rows)
           if rows[pre_end]['t'] - 5 <= row['t'] <= rows[pre_end]['t']]
    post = [i for i, row in enumerate(rows) if row['t'] >= rows[-1]['t'] - 5]

    def window(indices):
        return {'indices_inclusive': [indices[0], indices[-1]],
                'elapsed_s': [rows[indices[0]]['t'], rows[indices[-1]]['t']],
                'samples': len(indices),
                'stationary_classified_samples': int(sum(get(rows[i], 's_fusion.stationary') for i in indices)),
                'yaw_mean_rad': statistics.fmean(yaw[i] for i in indices),
                'x_mean_m': statistics.fmean(get(rows[i], 's_odom.odom_interface.x') for i in indices),
                'y_mean_m': statistics.fmean(get(rows[i], 's_odom.odom_interface.y') for i in indices)}

    start, end = window(pre), window(post)
    dx = 1000 * (end['x_mean_m'] - start['x_mean_m'])
    dy = 1000 * (end['y_mean_m'] - start['y_mean_m'])
    path = [(1000 * (get(row, 's_odom.odom_interface.x') - start['x_mean_m']),
             1000 * (get(row, 's_odom.odom_interface.y') - start['y_mean_m']))
            for row in rows[pre_end:]]
    max_index = max(range(len(path)), key=lambda i: math.hypot(*path[i]))
    angle = math.degrees(end['yaw_mean_rad'] - start['yaw_mean_rad'])

    def stopped(lo, hi):
        subset = rows[lo:hi]
        return {'indices_inclusive': [lo, hi - 1],
                'elapsed_s': [subset[0]['t'], subset[-1]['t']],
                'duration_s': subset[-1]['t'] - subset[0]['t'],
                'samples': len(subset),
                'stationary_classified_samples': int(sum(get(row, 's_fusion.stationary') for row in subset)),
                'yaw_change_deg': math.degrees(yaw[hi - 1] - yaw[lo]),
                'estimated_xy_change_mm': [1000 * (get(subset[-1], 's_odom.odom_interface.' + k) -
                                                   get(subset[0], 's_odom.odom_interface.' + k)) for k in ('x', 'y')]}

    attitude = {}
    for axis, label in [('x', 'roll'), ('y', 'pitch')]:
        key = 's_odom.angle.' + axis
        baseline = statistics.fmean(get(rows[i], key) for i in pre)
        final = statistics.fmean(get(rows[i], key) for i in post)
        attitude[label] = {'pre_mean_deg': math.degrees(baseline),
                           'post_mean_deg': math.degrees(final),
                           'change_deg': math.degrees(final - baseline),
                           'max_absolute_change_from_pre_deg': max(abs(math.degrees(get(row, key) - baseline))
                                                                   for row in rows[pre_end:]),
                           'external_reference_available': False}
    observed_keys = set().union(*(row['values'].keys() for row in rows))
    for snapshot in ('start.ndjson', 'end.ndjson'):
        snapshot_rows, _, _ = capture.load([directory / snapshot])
        observed_keys.update(snapshot_rows[0]['values'])
    covariance_keys = sorted(k for k in observed_keys
                             if 'p_data' in k.lower() or 'covariance' in k.lower() or '.filter.P' in k)
    result = {'capture': name, 'source_file_count': len(files), 'samples': len(rows),
              'elf_sha256': sorted({row['elf_sha256'] for row in rows}),
              'read_errors': errors, 'diagnostics': diagnostics,
              'reference': {'nominal_yaw_deg': nominal, 'source': 'operator RC-only return to original line',
                            'visual_alignment_uncertainty_deg': None, 'external_position_reference': None},
              'pre_static_window': start, 'final_static_window': end,
              'yaw': {'unwrapped_difference_deg': angle, 'signed_difference_from_nominal_deg': angle - nominal,
                      'magnitude_difference_deg': abs(angle) - abs(nominal),
                      'magnitude_difference_percent': 100 * (abs(angle) / abs(nominal) - 1)},
              'estimated_position': {'endpoint_x_m': end['x_mean_m'], 'endpoint_y_m': end['y_mean_m'],
                                     'endpoint_change_x_mm': dx, 'endpoint_change_y_mm': dy,
                                     'endpoint_change_norm_mm': math.hypot(dx, dy),
                                     'maximum_distance_from_pre_mm': math.hypot(*path[max_index]),
                                     'maximum_distance_sample_index': pre_end + max_index,
                                     'maximum_absolute_x_change_mm': max(abs(x) for x, y in path),
                                     'maximum_absolute_y_change_mm': max(abs(y) for x, y in path),
                                     'scope': 'All motion, adjustments and subsequent samples from pre-window endpoint',
                                     'physical_error_available': False},
              'observed_roll_pitch': attitude,
              'zero_command_sections': {'before_turn': stopped(0, first), 'after_final_adjustment': stopped(last + 1, len(rows))},
              'covariance_fields_in_capture': covariance_keys}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--captures', type=Path, default=REPO / 'mcu/build/OdomTune/captures')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    base = args.captures
    output = args.output or base / 'legacy-error-assessment/pose-errors.json'
    turns = [assess(base, 'held-steer-positive-01', 360.0),
             assess(base, 'held-steer-negative-01', -360.0)]
    creep_path = base / 'physical-creep-01/analysis.json'
    creep = json.loads(creep_path.read_text(encoding='utf-8'))
    hold = creep['drive_zero_hold']
    report = {
        'scope': 'Historical f4ca9e3f firmware captures; not measurements of any subsequent drive-hold or diagnostic fix',
        'reproduce': {'script': str(Path(__file__).resolve()), 'command': 'python tools/assess_legacy_pose.py [--captures PATH] [--output FILE]',
                      'captures': str(base), 'output': str(output),
                      'required_existing_analysis': str(creep_path),
                      'shared_parser': str(REPO / 'tools/analyze_odom_capture.py')},
        'turns': turns,
        'passive_creep_and_return': {'source_analysis': str(creep_path),
                                    'operator_observation': creep['operator_observation'],
                                    'timeline': creep['timeline'], 'closure': creep['closure'],
                                    'pre_zero_command': creep['pre_return_zero_command'],
                                    'post_zero_command': creep['post_return_zero_command']},
        'zero_drive_hold': {'samples': hold['samples'], 'span_s': hold['span_s'],
                            'stationary_classified_fraction': hold['stationary_classified_fraction'],
                            'stored_fused_yaw_change_deg': hold['stored_fused_yaw_change_deg'],
                            'encoder_fk': hold['encoder_fk'],
                            'encoder_fk_is_chassis_ground_truth': False,
                            'sparse_captured_wheel_wz_integral_deg': hold['sparse_captured_wheel_wz_integral_deg']},
        'covariance_assessment': {
            'P_recorded_in_selected_turn_samples_or_snapshots': any(t['covariance_fields_in_capture'] for t in turns),
            'confidence_calibratable_from_these_captures': False,
            'reasons': ['No P/covariance fields were logged in these selected pose captures, so predicted intervals cannot be checked.',
                        'The visual heading reference has unknown uncertainty; the physical center translation and roll/pitch truth were not measured.',
                        'Only one nominal full turn in each direction is available, with micro-adjustment sampling gaps and directional/model errors.',
                        'Internal P is conditional on the model, chosen Q/R and stationary classification; unmodelled scale, slip, backlash or true slow creep can violate those assumptions.',
                        'Pose P is retained during confirmed static after the consistency fix; a held mean or small velocity P does not establish absolute pose accuracy.']},
        'interpretation_limits': ['Reported turn percentages are conditional discrepancies against operator-aligned nominal turns, not accuracy specifications.',
                                  'Estimated xy closure/max excursion is an estimator output; it is not physical position error because the true center trajectory was not measured.',
                                  'A zero command or stationary classifier output does not prove physical rest; the operator later confirmed passive heading creep.',
                                  'No old-data computation can validate the performance of a drive-hold fix that was not present in the recorded firmware.']}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(output)
    for item in turns:
        p = item['estimated_position']
        print(item['capture'], 'yaw_deg=', item['yaw']['unwrapped_difference_deg'],
              'endpoint_xy_mm=', (p['endpoint_change_x_mm'], p['endpoint_change_y_mm']),
              'net_mm=', p['endpoint_change_norm_mm'], 'max_mm=', p['maximum_distance_from_pre_mm'])


if __name__ == '__main__':
    main()
