from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage

from t4_odometry import LidarOdometry, orientation_matrix, relative_orientation, unit_quaternion


PUBLISH_PERIOD = 0.025
MAX_IMU_SCAN_GAP = 0.045
MAX_VELOCITY_AGE = 0.18
POSE_COVARIANCE = np.diag([0.02, 0.02, 0.10, 0.005, 0.005, 0.005]).ravel().tolist()
TWIST_COVARIANCE = np.diag([0.10, 0.10, 0.25, 0.02, 0.02, 0.02]).ravel().tolist()


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1_000_000_000


def vector(value):
    return value.x, value.y, value.z


class OdometryNode(Node):
    def __init__(self):
        super().__init__('go1_t4_odometry', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.estimator = LidarOdometry()
        self.imu = deque(maxlen=128)
        self.last_publish_ns = -1
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf = self.create_publisher(TFMessage, '/tf', 10)
        sensor_qos = QoSProfile(depth=8, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, '/imu/data', self.receive_imu, sensor_qos)
        self.create_subscription(LaserScan, '/scan', self.receive_scan, sensor_qos)
        self.create_timer(PUBLISH_PERIOD, self.publish_pose)

    def receive_imu(self, message):
        q = message.orientation
        quaternion = unit_quaternion((q.x, q.y, q.z, q.w))
        angular = vector(message.angular_velocity)
        stamp = seconds(message.header.stamp)
        if quaternion is None or not np.isfinite(angular).all():
            return
        if self.imu and stamp <= self.imu[-1][0]:
            return
        self.imu.append((stamp, quaternion, angular))

    def receive_scan(self, message):
        stamp = seconds(message.header.stamp)
        if not self.imu or message.header.frame_id.lstrip('/') != 'base_scan':
            return
        for time, quaternion, _ in reversed(self.imu):
            if time <= stamp:
                break
        else:
            time, quaternion, _ = self.imu[0]
        if abs(stamp - time) <= MAX_IMU_SCAN_GAP:
            self.estimator.update(
                stamp, message.ranges, message.angle_min, message.angle_increment,
                message.range_min, message.range_max, quaternion,
            )

    def publish_pose(self):
        if self.estimator.last_stamp is None:
            return
        now = self.get_clock().now()
        if now.nanoseconds <= self.last_publish_ns:
            return
        now_seconds = now.nanoseconds / 1_000_000_000
        imu_stamp, imu_orientation, angular_velocity = self.imu[-1]
        quaternion = relative_orientation(imu_orientation, self.estimator.origin_yaw)
        odometry = Odometry()
        odometry.header.stamp = now.to_msg()
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'trunk'
        pose = odometry.pose.pose
        pose.position.x, pose.position.y = map(float, self.estimator.position)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, quaternion)
        odometry.pose.covariance = POSE_COVARIANCE
        if 0.0 <= now_seconds - self.estimator.last_stamp <= MAX_VELOCITY_AGE:
            world_velocity = [*self.estimator.velocity, 0.0]
            vx, vy, vz = orientation_matrix(quaternion).T @ world_velocity
            linear = odometry.twist.twist.linear
            linear.x, linear.y, linear.z = float(vx), float(vy), float(vz)
        if 0.0 <= now_seconds - imu_stamp <= MAX_VELOCITY_AGE:
            angular = odometry.twist.twist.angular
            angular.x, angular.y, angular.z = angular_velocity
        odometry.twist.covariance = TWIST_COVARIANCE

        transform = TransformStamped()
        transform.header = odometry.header
        transform.child_frame_id = odometry.child_frame_id
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        self.odom.publish(odometry)
        self.tf.publish(TFMessage(transforms=[transform]))
        self.last_publish_ns = now.nanoseconds


def main():
    rclpy.init()
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
