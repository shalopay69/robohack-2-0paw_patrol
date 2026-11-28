import unittest

import numpy as np

from t5_grid import OccupancyGrid


class GridTest(unittest.TestCase):
    def test_rays_distinguish_occupied_free_and_unknown_space(self):
        grid = OccupancyGrid(0.1, (0, 0), (50, 50))
        sensor = np.array([0.55, 0.55])
        ends = np.array([[2.55, 0.55], [0.55, 2.55]])
        grid.update(sensor, ends, np.array([True, False]))
        cells = grid.snapshot()
        self.assertEqual(cells[5, 5], 0)
        self.assertEqual(cells[5, 25], 100)
        self.assertEqual(cells[15, 5], 0)
        self.assertEqual(cells[40, 40], -1)

    def test_evidence_saturates_under_repeated_scans(self):
        grid = OccupancyGrid(0.1, (0, 0), (50, 50))
        for _ in range(25):
            grid.update(np.array([0.55, 0.55]), np.array([[2.55, 0.55]]), np.array([True]))
        self.assertEqual(grid.odds.max(), 20)
        self.assertEqual(grid.odds.min(), -12)


if __name__ == '__main__':
    unittest.main()
