"""Record a local Gazebo square for replay and diagnostics."""

import gzip
import json
from pathlib import Path
import sys
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


def stamp(value):
    return value.sec + value.nanosec / 1e9


def quat(value):
    return [value.x, value.y, value.z, value.w]


class Recorder(Node):
    def __init__(self, output):
        super().__init__('odometry_test_recorder')
        self.output = output
        self.start = None
        self.elapsed = 0.0
        self.last_phase = None
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, '/clock', self.clock, qos)
        self.create_subscription(Odometry, '/odom/ground_truth', self.odom, qos)
        self.create_subscription(Odometry, '/odom', self.estimate, qos)
        self.create_subscription(TFMessage, '/tf', self.tf, qos)
        self.command = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_timer(0.04, self.drive)

    def write(self, *fields):
        self.output.write(json.dumps(fields, separators=(',', ':')) + '\n')

    def clock(self, value):
        time = stamp(value.clock)
        if self.start is None:
            self.start = time
            self.write('start', time)
        self.elapsed = time - self.start

    def record_pose(self, kind, msg):
        pose = msg.pose.pose
        p = pose.position
        self.write(kind, stamp(msg.header.stamp), [p.x,p.y,p.z], quat(pose.orientation),
                   msg.header.frame_id, msg.child_frame_id)

    def odom(self, msg):
        self.record_pose('gt', msg)

    def estimate(self, msg):
        self.record_pose('odom', msg)

    def tf(self, msg):
        for part in msg.transforms:
            if part.header.frame_id == 'odom' and part.child_frame_id == 'trunk':
                q, p = part.transform.rotation, part.transform.translation
                self.write('tf', stamp(part.header.stamp), [p.x,p.y,p.z], quat(q))

    def drive(self):
        if self.start is None:
            return
        command = Twist()
        phase = 'stand'
        moving_time = self.elapsed - 4.0
        turn_time = 1.5707963267948966 / 0.3
        cycle = 8.0 + turn_time
        if 0 <= moving_time < 4 * cycle:
            if moving_time % cycle < 8.0:
                phase = 'line'
                command.linear.x = 0.2
            else:
                phase = 'turn'
                command.angular.z = 0.3
        self.command.publish(command)
        if phase != self.last_phase:
            self.last_phase = phase
            self.write('phase', self.start + self.elapsed, phase)
            self.output.flush()
            print(f'{self.elapsed:.2f} sim s: {phase}', file=sys.stderr, flush=True)


def main():
    rclpy.init()
    with gzip.open(Path(sys.argv[1]), 'wt', encoding='utf-8') as output:
        node = Recorder(output)
        try:
            deadline = monotonic() + 360.0
            while rclpy.ok() and node.elapsed < 72.0 and monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.25)
            node.command.publish(Twist())
            print(f'Recorded {node.elapsed:.2f} simulation seconds', file=sys.stderr)
            return int(node.elapsed < 71.0)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
