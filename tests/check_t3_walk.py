"""Publish a local T3 command and measure the public simulator's odometry."""

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_go.msg import LowCmd


class CheckWalking(Node):
    def __init__(self, speed, time_limit):
        super().__init__('check_t3_walking')
        self.velocity = self.create_publisher(Twist, '/cmd_vel', 1)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom/ground_truth', self.measure, qos)
        self.create_subscription(LowCmd, '/lowcmd_bare', self.count_bare, qos)
        self.create_subscription(LowCmd, '/lowcmd', self.count_safe, qos)
        self.command = Twist()
        self.command.linear.x = speed
        self.time_limit = time_limit
        self.bare_messages = 0
        self.safe_messages = 0
        self.start = None
        self.origin = None
        self.forward = None
        self.since_violation = [None] * 3
        self.result = None
        self.create_timer(0.05, self.send_command)

    def count_bare(self, _):
        self.bare_messages += 1

    def count_safe(self, _):
        self.safe_messages += 1

    def send_command(self):
        if self.velocity.get_subscription_count() > 0:
            self.velocity.publish(self.command)

    def measure(self, msg):
        if self.result is not None or self.velocity.get_subscription_count() == 0:
            return
        now = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        position, q = msg.pose.pose.position, msg.pose.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        if self.start is None:
            self.start = now
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            self.forward = math.cos(yaw), math.sin(yaw)
        elapsed = now - self.start
        if self.origin is None and elapsed >= 1.5:
            self.origin = position.x, position.y
        conditions = [position.z < 0.16, abs(roll) > math.radians(50), abs(pitch) > math.radians(50)]
        for index, violation in enumerate(conditions):
            if not violation:
                self.since_violation[index] = None
            elif self.since_violation[index] is None:
                self.since_violation[index] = now
        fell = any(stamp is not None and now - stamp > 0.5 for stamp in self.since_violation)
        distance = 0.0
        if self.origin is not None:
            distance = ((position.x - self.origin[0]) * self.forward[0]
                        + (position.y - self.origin[1]) * self.forward[1])
        if fell or distance >= 3.0 or elapsed >= self.time_limit + 1.5:
            self.result = {
                'speed_mps': self.command.linear.x,
                'timed_sim_seconds': round(max(0.0, elapsed - 1.5), 3),
                'forward_m': round(distance, 3),
                'fell': fell,
                'bare_messages': self.bare_messages,
                'safe_messages': self.safe_messages,
                'success': not fell and distance >= 3.0 and self.bare_messages > 0 and self.safe_messages > 0,
            }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--speed', type=float, default=0.4)
    parser.add_argument('--time-limit', type=float, default=90.0)
    args = parser.parse_args()
    rclpy.init()
    node = CheckWalking(args.speed, args.time_limit)
    try:
        deadline = time.monotonic() + args.time_limit + 25
        while node.result is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.result is None:
            raise SystemExit('No result received from the simulator before the wall-clock deadline.')
        print(json.dumps(node.result), flush=True)
        return 0 if node.result['success'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
