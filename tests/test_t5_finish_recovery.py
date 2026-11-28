import unittest

import numpy as np

from t5_frontier import FinishRetryBudget, choose_waypoint
from t5_grid import OccupancyGrid


class FinishRecoveryTest(unittest.TestCase):
    def test_recent_free_rays_correct_only_observed_local_cells(self):
        grid = OccupancyGrid(0.1, (0, 0), (50, 50))
        grid.odds[5, [12, 20, 30]] = 20
        grid.odds[8, 10] = 20
        sensor = np.array([0.55, 0.55])
        ends = np.array([[3.55, 0.55], [2.05, 0.55]])
        hits = np.array([False, True])

        grid.reconcile_free(sensor, [(sensor, ends, hits)] * 10, radius=1.3)

        self.assertEqual(grid.snapshot()[5, 12], 0)
        self.assertEqual(grid.snapshot()[5, 20], 100)
        self.assertEqual(grid.snapshot()[5, 30], 100)
        self.assertEqual(grid.snapshot()[8, 10], 100)

    def test_recovery_can_choose_an_intermediate_goal_with_a_reachable_finish(self):
        cells = np.full((60, 60), -1, dtype=np.int8)
        cells[21:40, 3:57] = 0
        current = np.array([0.6, 3.0])
        target = np.array([5.0, 3.0])

        point, is_final = choose_waypoint(cells, (0, 0), 0.1, current, target, [], recovery=True)

        self.assertFalse(is_final)
        self.assertGreater(np.linalg.norm(point - current), 0.75)
        self.assertLessEqual(np.linalg.norm(point - current), 1.8)

    def test_self_hits_cannot_occupy_cells_inside_the_trunk(self):
        grid = OccupancyGrid(0.06, (0, 0), (50, 50))
        grid.odds[11, 10] = 20
        grid.odds[13, 10] = 20
        grid.odds[16, 10] = 20

        grid.clear_robot_body(np.array([0.63, 0.63]))

        self.assertEqual(grid.snapshot()[11, 10], 0)
        self.assertEqual(grid.snapshot()[13, 10], 100)
        self.assertEqual(grid.snapshot()[16, 10], 100)

    def test_finish_retries_are_renewed_by_progress(self):
        retry = FinishRetryBudget()
        self.assertEqual(retry.on_abort(5.0), 1)
        self.assertEqual(retry.on_abort(3.3), 2)
        self.assertEqual(retry.on_abort(1.0), 1)
        self.assertEqual(retry.on_abort(1.1), 2)
        self.assertIsNone(retry.on_abort(1.0))


if __name__ == '__main__':
    unittest.main()
