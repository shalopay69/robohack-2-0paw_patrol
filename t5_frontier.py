import cv2
import numpy as np


class FinishRetryBudget:
    def __init__(self):
        self.attempts = 0
        self.anchor_distance = None

    def on_abort(self, distance):
        if self.attempts == 2:
            if self.anchor_distance - distance < 0.75:
                return None
            self.attempts = 0
        if self.attempts == 0:
            self.anchor_distance = distance
        self.attempts += 1
        return self.attempts


def choose_waypoint(cells, origin, resolution, position, target, avoid, *, recovery=False):
    """Choose a reachable free goal or a waypoint toward an unexplored region."""
    cv2.setNumThreads(1)
    position, target, origin = map(np.asarray, (position, target, origin))
    height, width = cells.shape
    ix, iy = np.floor((position - origin) / resolution).astype(int)
    if not (0 <= ix < width and 0 <= iy < height):
        return None

    wall_clearance = cv2.distanceTransform((cells != 100).astype(np.uint8), cv2.DIST_L2, 5)
    free = cv2.morphologyEx((cells == 0).astype(np.uint8), cv2.MORPH_CLOSE,
                           np.ones((5, 5), np.uint8))
    traversable = (free > 0) & (wall_clearance * resolution >= 0.25)
    # The lidar is ahead of the trunk. Its origin may have free evidence before the trunk does.
    seed = traversable[max(0, iy - 5):iy + 6, max(0, ix - 5):ix + 6]
    choices = np.argwhere(seed)
    if not len(choices):
        return None
    ys = choices[:, 0] + max(0, iy - 5)
    xs = choices[:, 1] + max(0, ix - 5)
    near = np.argmin((ys - iy) ** 2 + (xs - ix) ** 2)
    reachable = traversable.astype(np.uint8)
    cv2.floodFill(reachable, None, (int(xs[near]), int(ys[near])), 2, flags=4)
    reachable = reachable == 2
    tx, ty = np.floor((target - origin) / resolution).astype(int)
    if not recovery and 0 <= tx < width and 0 <= ty < height and reachable[ty, tx]:
        return target, True

    yy, xx = np.nonzero(reachable & (cells == 0))
    points = origin + np.column_stack((xx + 0.5, yy + 0.5)) * resolution
    step = np.linalg.norm(points - position, axis=1)
    keep = (step >= 0.75) & (step <= (1.8 if recovery else 4.0))
    if recovery:
        keep &= np.linalg.norm(points - target, axis=1) > 0.30
    for entry in avoid:
        keep &= np.linalg.norm(points - entry, axis=1) >= 0.65
    if not np.any(keep):
        return None
    open_distance = cv2.distanceTransform((cells >= 0).astype(np.uint8), cv2.DIST_L2, 5)
    edge = open_distance[yy, xx] * resolution
    clearance = wall_clearance[yy, xx] * resolution
    remaining = np.linalg.norm(points - target, axis=1)
    score = remaining + 0.20 * step - 0.7 * np.minimum(clearance, 0.65)
    if recovery:
        score -= 1.3 * np.minimum(clearance, 0.45)
    score += 0.5 * np.maximum(0, edge - 0.85)
    score[~keep] = np.inf
    return points[int(np.argmin(score))], False
