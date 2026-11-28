from array import array
from collections import deque
from math import cos, sin

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid as MapMessage, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from t4_odometry import LIDAR_OFFSET, heading, orientation_matrix, unit_quaternion
from t5_grid import OccupancyGrid


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def yaw_rotation(angle):
    return np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])


class Mapper(Node):
    def __init__(self):
        super().__init__('go1_t5_mapper', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.grid = OccupancyGrid()
        self.yaw = 0.0
        self.offset = np.zeros(2)
        self.odom = None
        self.initial = None
        self.dirty = False
        self.last_scan = None
        self.recent_scans = deque(maxlen=32)
        live = QoSProfile(depth=4, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map = self.create_publisher(MapMessage, '/map', latched)
        self.obstacles = self.create_publisher(LaserScan, '/scan_filtered', 5)
        self.transforms = StaticTransformBroadcaster(self)
        self.publish_transform()
        self.create_subscription(Odometry, '/odom', self.receive_odom, live)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialize_pose, 5)
        self.create_subscription(LaserScan, '/scan', self.receive_scan, live)
        self.create_service(Trigger, '~/refresh_recent', self.refresh_recent)
        self.create_timer(0.4, self.publish_map)

    def publish_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x, transform.transform.translation.y = map(float, self.offset)
        transform.transform.rotation.z = sin(self.yaw / 2)
        transform.transform.rotation.w = cos(self.yaw / 2)
        self.transforms.sendTransform(transform)

    def receive_odom(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        orientation = unit_quaternion([q.x, q.y, q.z, q.w])
        if orientation is None or not np.isfinite([p.x, p.y]).all():
            return
        self.odom = seconds(message.header.stamp), np.array([p.x, p.y]), orientation
        if self.initial is not None:
            self.apply_initial_pose()

    def initialize_pose(self, message):
        if message.header.frame_id.lstrip('/') not in ('', 'map'):
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        orientation = unit_quaternion([q.x, q.y, q.z, q.w])
        if orientation is None or not np.isfinite([p.x, p.y]).all():
            return
        self.initial = np.array([p.x, p.y]), heading(orientation)
        if self.odom is not None:
            self.apply_initial_pose()

    def apply_initial_pose(self):
        position, yaw = self.initial
        _, previous, orientation = self.odom
        self.yaw = yaw - heading(orientation)
        self.offset = position - yaw_rotation(self.yaw) @ previous
        self.grid.odds.fill(0)
        self.recent_scans.clear()
        self.initial = None
        self.dirty = True
        self.publish_transform()

    def receive_scan(self, message):
        if self.odom is None or message.header.frame_id.lstrip('/') != 'base_scan':
            return
        stamp = seconds(message.header.stamp)
        odom_stamp, position, quaternion = self.odom
        if abs(odom_stamp - stamp) > 0.16:
            return
        rotation = orientation_matrix(quaternion)
        ranges = np.asarray(message.ranges, dtype=float)
        angles = message.angle_min + np.arange(len(ranges)) * message.angle_increment
        directions = np.column_stack((np.cos(angles), np.sin(angles))) @ rotation[:, :2].T
        valid_hit = (np.isfinite(ranges) & (ranges > max(message.range_min, 0.17))
                     & (ranges < message.range_max - 0.02))
        floor = valid_hit & (np.where(valid_hit, ranges, 0.0) * directions[:, 2] < -0.25)
        no_return = np.isposinf(ranges) | (ranges >= message.range_max - 0.02)
        ego = np.isfinite(ranges) & (ranges <= 0.17)
        message.ranges = array('f', np.where(floor | ego, np.nan, ranges).astype(np.float32))
        self.obstacles.publish(message)

        if self.last_scan is not None and stamp - self.last_scan < 0.09:
            return
        self.last_scan = stamp
        indices = np.arange(0, len(ranges), 3)
        good = valid_hit | no_return
        indices = indices[good[indices]]
        max_distance = np.minimum(9.0, 0.25 / np.maximum(-directions[indices, 2], 1e-5))
        length = np.minimum(ranges[indices], max_distance)
        hits = valid_hit[indices] & ~floor[indices] & (ranges[indices] <= max_distance)
        transform = yaw_rotation(self.yaw)
        center = transform @ (position + (rotation @ LIDAR_OFFSET)[:2]) + self.offset
        ends = center + (length[:, None] * directions[indices, :2]) @ transform.T
        self.grid.update(center, ends, hits)
        self.grid.clear_robot_body(transform @ position + self.offset)
        self.recent_scans.append((stamp, center, ends, hits))
        self.dirty = True

    def refresh_recent(self, request, response):
        now = self.get_clock().now().nanoseconds * 1e-9
        scans = [scan[1:] for scan in self.recent_scans if 0 <= now - scan[0] <= 3.0]
        if len(scans) < 4:
            response.message = 'Waiting for recent lidar scans'
            return response
        changed = self.grid.reconcile_free(scans[-1][0], scans)
        self.dirty = True
        self.publish_map()
        response.success = True
        response.message = f'Updated {changed} map cells from recent lidar scans'
        return response

    def publish_map(self):
        if not self.dirty:
            return
        grid = self.grid.snapshot()
        msg = MapMessage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.grid.resolution
        msg.info.height, msg.info.width = grid.shape
        msg.info.origin.position.x, msg.info.origin.position.y = map(float, self.grid.origin)
        msg.info.origin.orientation.w = 1.0
        msg.data = array('b', grid.tobytes())
        self.map.publish(msg)
        self.dirty = False


def main():
    rclpy.init()
    node = Mapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
