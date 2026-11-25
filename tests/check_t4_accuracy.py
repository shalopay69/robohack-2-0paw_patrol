"""Check odometry against a recording from the public Gazebo simulator."""

import argparse
import gzip
import json
import math
from pathlib import Path

import numpy as np

from t4_odometry import heading


def load(path):
    rows = {}
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        for line in stream:
            topic, *values = json.loads(line)
            rows.setdefault(topic, []).append(values)
    return rows


def relative_errors(actual, estimate, begin, end, dimensions):
    distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(actual[:, :dimensions], axis=0), axis=1))]
    errors = []
    left = begin
    while left < end:
        right = int(np.searchsorted(distance, distance[left] + 1.0, side='left'))
        if right > end:
            break
        a = actual[right, :dimensions] - actual[left, :dimensions]
        b = estimate[right, :dimensions] - estimate[left, :dimensions]
        errors.append(float(np.linalg.norm(a - b) / (distance[right] - distance[left])))
        left = right
    if not errors:
        raise ValueError('Recorded movement did not cover a metre')
    return errors


def angle_between(first, last):
    dot = abs(float(np.dot(first, last) / (np.linalg.norm(first) * np.linalg.norm(last))))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def score(rows):
    odom = rows['odom']
    poses = np.array([r[1] for r in odom])
    quaternions = np.array([r[2] for r in odom])
    assert np.isfinite(poses).all() and np.isfinite(quaternions).all()
    np.testing.assert_allclose(np.linalg.norm(quaternions, axis=1), 1.0, atol=1e-5)
    assert all(r[3:] == ['odom', 'trunk'] for r in odom)
    times = np.array([r[0] for r in odom])
    assert np.all(np.diff(times) > 0)
    estimated_rate = (len(times) - 1) / (times[-1] - times[0])
    first_movement = next(time for time, phase in rows['phase'] if phase != 'stand')
    last_stand = next(time for time, phase in reversed(rows['phase']) if phase == 'stand')
    gt = [r for r in rows['gt'] if times[0] <= r[0] <= times[-1]]
    truth_times = np.array([r[0] for r in gt])
    actual = np.array([r[1] for r in gt])
    estimate = np.column_stack([np.interp(truth_times, times, poses[:, axis]) for axis in range(3)])
    begin = int(np.searchsorted(truth_times, first_movement))
    end = int(np.searchsorted(truth_times, last_stand)) - 1
    assert end > begin
    estimate_yaw = heading(quaternions[np.searchsorted(times, truth_times[begin])])
    yaw = heading(gt[begin][2]) - estimate_yaw
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    estimate[:, :2] = estimate[:, :2] @ rotation.T
    planar = relative_errors(actual, estimate, begin, end, 2)
    spatial = relative_errors(actual, estimate, begin, end, 3)
    start = last_stand + 3.0
    assert times[-1] >= start + 10.0
    a, b = np.searchsorted(times, [start, start + 10.0])
    drift = float(np.linalg.norm(poses[b] - poses[a]))
    orientation_drift = angle_between(quaternions[b], quaternions[a])
    answer = {'odom_hz': estimated_rate, 'odom_count': len(times),
              'planar_rpe': float(np.mean(planar)), 'planar_segments': len(planar),
              'spatial_rpe': float(np.mean(spatial)), 'spatial_segments': len(spatial),
              'position_drift_m': drift, 'orientation_drift_deg': orientation_drift}
    if 'tf' in rows:
        tf = {r[0]: (r[1], r[2]) for r in rows['tf']}
        match = [r for r in odom if r[0] in tf]
        assert all((r[1], r[2]) == tf[r[0]] for r in match)
        answer['tf_hz'] = (len(tf) - 1) / (max(tf) - min(tf))
        answer['matched_odom_tf'] = len(match)
        answer['matched_hz'] = len(match) / (times[-1] - times[0])
    return answer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    args = parser.parse_args()
    rows = load(args.recording)
    result = score(rows)
    print(json.dumps(result, indent=2))
    required_rates = (result['odom_hz'], result.get('tf_hz', 0.0), result.get('matched_hz', 0.0))
    return int(min(required_rates) < 20.0 or max(result['planar_rpe'], result['spatial_rpe']) > .20
               or result['position_drift_m'] > .10 or result['orientation_drift_deg'] > 5.0)


if __name__ == '__main__':
    raise SystemExit(main())
