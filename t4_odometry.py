from collections import deque
from math import atan2, cos, sin

import numpy as np


# Transform trunk -> base_scan in the public Go1 URDF. The axes are parallel.
LIDAR_OFFSET = np.array([0.22, 0.0, 0.11])


def unit_quaternion(value):
    quaternion = np.asarray(value, dtype=float)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        return None
    norm = np.linalg.norm(quaternion)
    return quaternion / norm if 0.5 < norm < 1.5 else None


def orientation_matrix(quaternion):
    x, y, z, w = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def heading(quaternion):
    x, y, z, w = quaternion
    return atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def relative_orientation(quaternion, origin_yaw):
    x, y, z, w = quaternion
    c, s = cos(-origin_yaw / 2), sin(-origin_yaw / 2)
    return np.array([c * x - s * y, c * y + s * x, c * z + s * w, c * w - s * z])


def scan_lines(ranges, angle_min, angle_step, range_min, range_max, rotation):
    limits = (angle_min, angle_step, range_min, range_max)
    if len(ranges) < 24 or angle_step <= 0 or not np.isfinite(limits).all():
        return np.empty((0, 2)), np.empty((0, 2))
    ranges = np.asarray(ranges, dtype=float)
    valid = np.isfinite(ranges) & (ranges > max(0.17, range_min))
    valid &= ranges < min(range_max - 0.01, 12.0)
    angles = angle_min + np.arange(len(ranges)) * angle_step
    distance = np.where(valid, ranges, 0.0)
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    points = (distance[:, None] * directions) @ rotation[:, :2].T
    # Exclude beams likely to hit the ground when the trunk rolls or pitches.
    valid &= points[:, 2] > -0.26
    flat = points[:, :2] + (rotation @ LIDAR_OFFSET)[:2]

    # Nearby rays on a continuous surface provide a local line and its normal.
    half_window = max(2, round(0.035 / angle_step))
    middle = np.arange(half_window, len(flat) - half_window, max(1, round(0.025 / angle_step)))
    before, after = middle - half_window, middle + half_window
    tangent = flat[after] - flat[before]
    lengths = np.linalg.norm(tangent, axis=1)
    normals = np.column_stack((-tangent[:, 1], tangent[:, 0])) / np.maximum(lengths[:, None], 1e-9)
    span = np.maximum(
        np.linalg.norm(flat[middle] - flat[before], axis=1),
        np.linalg.norm(flat[middle] - flat[after], axis=1),
    )
    central_error = np.abs(np.sum((flat[middle] - flat[before]) * normals, axis=1))
    supported = valid[before] & valid[middle] & valid[after]
    supported &= (lengths > 0.035) & (span < 0.53)
    supported &= central_error < 0.025 + 0.035 * lengths
    return flat[middle][supported], normals[supported]


class LidarOdometry:
    """Track horizontal trunk motion against recent scan keyframes."""

    def __init__(self):
        self.position = np.zeros(2)
        self.velocity = np.zeros(2)
        self.origin_yaw = None
        self.last_stamp = None
        self.frames = deque(maxlen=4)
        self.reference = None

    def add_frame(self, points, normals, yaw):
        self.frames.append((points + self.position, normals, self.position.copy(), yaw))
        ordered = list(reversed(self.frames))
        positions = np.vstack([frame[0] for frame in ordered])
        directions = np.vstack([frame[1] for frame in ordered])
        cells = np.floor(positions / 0.05).astype(np.int32)
        _, keep = np.unique(cells, axis=0, return_index=True)
        targets = positions[keep]
        self.reference = targets, directions[keep], np.sum(targets * targets, axis=1)

    def match(self, points, normals, initial):
        targets, target_normals, target_norm2 = self.reference
        estimate = initial.copy()
        for _ in range(6):
            moved = points + estimate
            distance2 = (np.sum(moved * moved, axis=1)[:, None] + target_norm2
                         - 2.0 * (moved @ targets.T))
            nearest = np.argmin(distance2, axis=1)
            facing = np.sum(normals * target_normals[nearest], axis=1)
            residual = np.sum((moved - targets[nearest]) * target_normals[nearest], axis=1)
            close = distance2[np.arange(len(points)), nearest] < 0.20 ** 2
            good = close & (np.abs(facing) > 0.72) & (np.abs(residual) < 0.11)
            if good.sum() < max(28, 0.27 * len(points)):
                return None
            error = residual[good]
            jacobian = target_normals[nearest[good]]
            weights = np.minimum(1.0, 0.021 / np.maximum(np.abs(error), 1e-9))
            weighted = jacobian * weights[:, None]
            normal = weighted.T @ jacobian
            if np.linalg.det(normal) < 0.035 * np.trace(normal) ** 2:
                return None
            step = np.linalg.solve(normal, -weighted.T @ error)
            estimate += step
            if np.linalg.norm(step) < 0.0004:
                break
        if np.median(np.abs(error)) > 0.040:
            return None
        return estimate

    def update(self, stamp, ranges, angle_min, angle_step, range_min, range_max, quaternion):
        if not np.isfinite(stamp) or (self.last_stamp is not None and stamp <= self.last_stamp):
            return False
        quaternion = unit_quaternion(quaternion)
        if quaternion is None:
            return False
        yaw = heading(quaternion)
        initial_yaw = yaw if self.origin_yaw is None else self.origin_yaw
        rotation = orientation_matrix(relative_orientation(quaternion, initial_yaw))
        points, normals = scan_lines(ranges, angle_min, angle_step, range_min, range_max, rotation)
        if len(points) < 35:
            return False
        if self.reference is None:
            self.origin_yaw = initial_yaw
            self.last_stamp = stamp
            self.add_frame(points, normals, yaw)
            return True
        elapsed = stamp - self.last_stamp
        predicted = self.position + self.velocity * min(elapsed, 0.10)
        estimate = self.match(points, normals, predicted)
        if estimate is None or np.linalg.norm(estimate - self.position) > 0.08 + min(elapsed, 0.2) * 0.8:
            return False
        self.velocity = 0.65 * self.velocity + 0.35 * (estimate - self.position) / max(elapsed, 0.025)
        self.position = estimate
        self.last_stamp = stamp
        _, _, anchor, anchor_yaw = self.frames[-1]
        turn = atan2(sin(yaw - anchor_yaw), cos(yaw - anchor_yaw))
        if np.linalg.norm(self.position - anchor) > 0.32 or abs(turn) > 0.38:
            self.add_frame(points, normals, yaw)
        return True
