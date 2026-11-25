import math
import unittest

import numpy as np

from t3_policy import ObservationHistory, joint_targets


class PolicyInterfaceTests(unittest.TestCase):
    def test_observation_history_honors_joint_order_and_scaling(self):
        history = ObservationHistory()
        q = np.arange(12)
        dq = np.arange(12) * 20
        action = np.arange(12)
        observation = history.append(
            q, dq, [1, 0, 0, 0], [0.4, 0.0, 0.0], action
        ).reshape(30, 70)
        np.testing.assert_array_equal(observation[:-1], 0.0)
        np.testing.assert_allclose(observation[-1, :6], [0, 0, -1, 0.8, 0, 0])
        np.testing.assert_allclose(
            observation[-1, 30:42], [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
        )
        np.testing.assert_array_equal(observation[-1, 42:54], action)

    def test_gravity_is_rotated_into_body_frame(self):
        root = math.sqrt(0.5)
        observation = ObservationHistory().append(
            np.zeros(12), np.zeros(12), [root, root, 0, 0], np.zeros(3), np.zeros(12)
        )
        np.testing.assert_allclose(observation[-70:-67], [0, -1, 0], atol=1e-7)

    def test_output_uses_unitree_order_and_hip_reduction(self):
        targets = joint_targets(np.arange(12, dtype=np.float32))
        np.testing.assert_allclose(
            targets, [0.275, 1.8, -0.25, 0.1, 1.05, -1, 1.025, 3.5, 1.0, 0.85, 2.75, 0.5],
            atol=1e-6,
        )


if __name__ == '__main__':
    unittest.main()
