import unittest

import numpy as np

from t2_safety import CollisionChecker, SafetyBlock


class SafetyBlockTests(unittest.TestCase):
    def setUp(self):
        self.checker = CollisionChecker('/opt/go1/model')
        self.block = SafetyBlock(self.checker)
        self.home = self.checker.home
        self.command = np.zeros((20, 5))
        self.command[:12, 0] = self.home
        self.command[:12, 3] = 20.0
        self.command[:12, 4] = 1.0

    def test_accepts_a_safe_command_and_holds_its_target_after_timeout(self):
        self.command[1, :3] = (self.home[1] + 0.02, 0.1, 0.2)
        self.block.receive(self.command, 1_000_000_000)
        active = self.block.sample(1_470_000_000)
        np.testing.assert_allclose(active, self.command[:12], atol=1e-7)
        hold = self.block.sample(1_489_000_000)
        np.testing.assert_array_equal(hold[:, 0], active[:, 0])
        np.testing.assert_array_equal(hold[:, 1:3], 0.0)
        self.assertTrue(np.all(hold[:, 3:] > 0.0))

    def test_limits_a_large_step_and_stays_within_joint_bounds(self):
        self.command[2, 0] = -20.0
        self.block.receive(self.command, 1)
        active = self.block.sample(2)
        self.assertGreater(active[2, 0], self.home[2] - np.deg2rad(5.0) - 1e-7)
        self.assertLess(active[2, 0], self.home[2] - 0.08)
        self.assertTrue(np.all(active[:, 0] >= self.checker.lower))
        self.assertTrue(np.all(active[:, 0] <= self.checker.upper))

    def test_rejects_the_whole_frame_for_nonfinite_values(self):
        for value in (np.nan, np.inf, -np.inf):
            with self.subTest(value=value):
                self.command[1, 0] = self.home[1] + 0.02
                self.command[18, 2] = value
                self.block.receive(self.command, 1)
                np.testing.assert_array_equal(self.block.sample(2)[:, 0], self.home)

    def test_invalid_frames_do_not_replace_the_active_command_or_stop_the_stream(self):
        self.command[1, 1:3] = (0.1, 0.2)
        self.block.receive(self.command, 0)
        active = self.block.sample(1).copy()

        self.command[1, 0] = np.inf
        self.block.receive(self.command, 400_000_000)
        np.testing.assert_array_equal(self.block.sample(550_000_000), active)
        np.testing.assert_array_equal(self.block.sample(900_000_000)[:, 1:3], 0.0)

    def test_detects_forbidden_self_collision_in_the_public_model(self):
        self.assertTrue(self.checker.is_transition_safe(self.home, self.home))
        rng = np.random.default_rng(47)
        for _ in range(500):
            target = rng.uniform(self.checker.lower, self.checker.upper)
            if not self.checker.is_transition_safe(target, target):
                return
        self.fail('Expected at least one forbidden collision inside joint limits')

    def test_rejects_a_collision_between_safe_endpoints(self):
        start = np.array([
            -0.6596552, 2.2139776, -1.654755, 0.2977706, 1.4387652, -1.733867,
            0.7712194, 4.165555, -1.706531, 0.2136192, 3.9239924, -1.8176542,
        ])
        target = np.array([
            -0.5551704, 2.5303352, -2.36469, 0.1671912, 2.8402204, -1.771164,
            0.8254188, 4.35432, -2.327052, -0.0602316, 3.6851548, -1.9257984,
        ])
        self.assertTrue(self.checker.is_transition_safe(start, start))
        self.assertTrue(self.checker.is_transition_safe(target, target))
        self.assertFalse(self.checker.is_transition_safe(start, target))

        delta = target - start
        step = start + delta * (np.deg2rad(4.0) / np.max(np.abs(delta)))
        self.checker.home = start
        block = SafetyBlock(self.checker)
        self.command[:12, 0] = step
        block.receive(self.command, 1)
        np.testing.assert_array_equal(block.sample(2)[:, 0], start)


if __name__ == '__main__':
    unittest.main()
