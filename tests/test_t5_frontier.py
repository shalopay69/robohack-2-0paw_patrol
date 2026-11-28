import unittest

import numpy as np

from t5_frontier import choose_waypoint


class FrontierTest(unittest.TestCase):
    def test_waypoint_stays_in_reachable_component(self):
        cells = np.full((60, 60), -1, dtype=np.int8)
        cells[21:40, 3:29] = 0
        cells[21:40, 29] = 100
        cells[21:40, 30:57] = 0
        waypoint, final = choose_waypoint(cells, (0, 0), 0.1, (0.6, 3), (5.0, 3), [])
        self.assertFalse(final)
        self.assertLess(waypoint[0], 2.9)
        self.assertGreater(waypoint[0], 1.4)

    def test_reachable_goal_keeps_its_exact_pose(self):
        cells = np.full((60, 60), -1, dtype=np.int8)
        cells[21:40, 3:57] = 0
        waypoint, final = choose_waypoint(cells, (0, 0), 0.1, (0.6, 3), (5.0, 3), [])
        self.assertTrue(final)
        np.testing.assert_allclose(waypoint, (5.0, 3))


if __name__ == '__main__':
    unittest.main()
