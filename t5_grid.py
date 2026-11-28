import numpy as np


class OccupancyGrid:
    """Bounded log-odds grid in map coordinates."""

    def __init__(self, resolution=0.06, origin=(-12.0, -18.0), shape=(600, 900)):
        self.resolution = resolution
        self.origin = np.asarray(origin, dtype=float)
        self.odds = np.zeros(shape, dtype=np.int8)

    def grid_indices(self, points):
        indices = np.floor((points - self.origin) / self.resolution).astype(np.int32)
        height, width = self.odds.shape
        inside = ((indices[..., 0] >= 0) & (indices[..., 0] < width)
                  & (indices[..., 1] >= 0) & (indices[..., 1] < height))
        return (indices[..., 1] * width + indices[..., 0])[inside]

    def update(self, sensor, ends, hits):
        if len(ends) == 0:
            return
        rays = ends - sensor
        distances = np.linalg.norm(rays, axis=1)
        direction = rays / np.maximum(distances[:, None], 1e-9)
        samples = np.arange(0, distances.max(), self.resolution * 0.85)
        free = samples[None, :] < np.maximum(
            0.0, distances - np.where(hits, self.resolution * 1.4, 0.0)
        )[:, None]
        points = sensor + direction[:, None, :] * samples[None, :, None]
        misses = np.unique(self.grid_indices(points[free]))
        occupied = np.unique(self.grid_indices(ends[hits]))
        view = self.odds.ravel()
        view[misses] = np.maximum(-12, view[misses] - 1)
        view[occupied] = np.minimum(20, view[occupied] + 5)

    def snapshot(self):
        return np.where(self.odds >= 3, 100, np.where(self.odds < 0, 0, -1)).astype(np.int8)

    def reconcile_free(self, center, scans, radius=2.5):
        recent = OccupancyGrid(self.resolution, self.origin, self.odds.shape)
        for sensor, ends, hits in scans:
            recent.update(sensor, ends, hits)
        rows, cols = np.nonzero((recent.odds <= -4) & (self.odds > 0))
        xy = self.origin + (np.column_stack((cols, rows)) + 0.5) * self.resolution
        local = np.sum((xy - center) ** 2, axis=1) <= radius**2
        rows, cols = rows[local], cols[local]
        self.odds[rows, cols] = recent.odds[rows, cols]
        return len(rows)

    def clear_robot_body(self, center):
        # Use the central part of the collision footprint to suppress
        # close-range self returns on the global floor map.
        radius = 0.12
        start = np.floor((center - radius - self.origin) / self.resolution).astype(int)
        stop = np.ceil((center + radius - self.origin) / self.resolution).astype(int)
        x0, y0 = np.clip(start, (0, 0), self.odds.shape[::-1])
        x1, y1 = np.clip(stop, (0, 0), self.odds.shape[::-1])
        xs = self.origin[0] + (np.arange(x0, x1) + 0.5) * self.resolution
        ys = self.origin[1] + (np.arange(y0, y1) + 0.5) * self.resolution
        inside = (xs - center[0])**2 + (ys[:, None] - center[1])**2 <= radius**2
        region = self.odds[y0:y1, x0:x1]
        region[inside & (region > 0)] = -4
