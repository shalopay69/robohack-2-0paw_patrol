import math
import unittest

import numpy as np

from t4_odometry import LIDAR_OFFSET, LidarOdometry, orientation_matrix


ANGLES = np.linspace(-math.pi, math.pi, 720)
ANGLE_STEP = float(ANGLES[1] - ANGLES[0])


def room_scan(position, heading, pitch=0.0):
    c, s = math.cos(heading), math.sin(heading)
    forward = np.cos(ANGLES) * math.cos(pitch)
    lateral = np.sin(ANGLES)
    direction = np.column_stack((c * forward - s * lateral, s * forward + c * lateral))
    offset = LIDAR_OFFSET[0] * math.cos(pitch) + LIDAR_OFFSET[2] * math.sin(pitch)
    laser = np.asarray(position) + offset * np.array([c, s])
    walls = np.where(direction > 0, [4, 2], [-3, -2.5])
    with np.errstate(divide='ignore'):
        distance = np.min((walls - laser) / direction, axis=1)
    return distance


def orientation(angle):
    return np.array([0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)])


def update(odometry, stamp, position, heading):
    return odometry.update(
        stamp, room_scan(position, heading), float(ANGLES[0]), ANGLE_STEP, 0.12, 20.0,
        orientation(heading),
    )


class LidarOdometryTest(unittest.TestCase):
    def test_projects_body_axes_using_ros_quaternion_order(self):
        np.testing.assert_allclose(
            orientation_matrix(orientation(math.pi / 2)) @ [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0], atol=1e-9,
        )

    def test_recovers_translation_while_turning(self):
        estimator = LidarOdometry()
        self.assertTrue(update(estimator, 1.0, [0.0, 0.0], 0.3))
        for n in range(1, 11):
            self.assertTrue(update(estimator, 1.0 + n * 0.05, [n * 0.012, n * 0.004], 0.3 + n * 0.015))
        expected = [0.12 * math.cos(.3) + .04 * math.sin(.3),
                    -.12 * math.sin(.3) + .04 * math.cos(.3)]
        np.testing.assert_allclose(estimator.position, expected, atol=0.008)

    def test_rejects_invalid_scan_and_duplicate_time(self):
        estimator = LidarOdometry()
        self.assertTrue(update(estimator, 1.0, [0.0, 0.0], 0.0))
        self.assertFalse(update(estimator, 1.0, [0.1, 0.0], 0.0))
        self.assertFalse(estimator.update(
            1.05, np.full(len(ANGLES), np.inf), -math.pi, ANGLE_STEP, 0.12, 20.0, orientation(0),
        ))
        np.testing.assert_array_equal(estimator.position, [0.0, 0.0])

    def test_pitch_does_not_produce_false_horizontal_motion(self):
        estimator = LidarOdometry()
        self.assertTrue(update(estimator, 1.0, [0.0, 0.0], 0.0))
        for n in range(1, 9):
            tilt = n * 0.012
            q = [0.0, math.sin(tilt / 2), 0.0, math.cos(tilt / 2)]
            self.assertTrue(estimator.update(
                1.0 + n * 0.05, room_scan([0, 0], 0, tilt), -math.pi, ANGLE_STEP,
                0.12, 20.0, q,
            ))
        np.testing.assert_allclose(estimator.position, [0.0, 0.0], atol=0.008)


if __name__ == '__main__':
    unittest.main()
