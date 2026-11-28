"""Send a local NavigateToPose goal and verify the physical outcome in Gazebo."""

import argparse
import json
import math
from time import monotonic

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def yaw(pose):
    q = pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class CourseTest(Node):
    def __init__(self):
        super().__init__('t5_course_test')
        self.time = 0.0
        self.world_pose = None
        self.estimate = None
        self.markers = set()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, '/clock', self.clock, qos)
        self.create_subscription(Odometry, '/odom/ground_truth', self.truth, qos)
        self.create_subscription(Odometry, '/odom', self.odom, qos)
        self.create_subscription(String, '/aruco/detections', self.detection, 10)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 5)
        self.state = self.create_client(GetState, '/t5_nav2/bt_navigator/get_state')
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def clock(self, msg):
        self.time = msg.clock.sec + msg.clock.nanosec / 1e9

    def truth(self, msg):
        self.world_pose = msg.pose.pose

    def odom(self, msg):
        p = msg.pose.pose.position
        self.estimate = p.x, p.y

    def detection(self, msg):
        self.markers.update(json.loads(msg.data)['ids'])

    def spin(self):
        rclpy.spin_once(self, timeout_sec=0.10)

    def run(self, args):
        deadline = monotonic() + 170
        ready = False
        while monotonic() < deadline:
            self.spin()
            if self.state.service_is_ready() and self.action.server_is_ready():
                future = self.state.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.done() and future.result().current_state.label == 'active':
                    ready = True
                    break
        while ready and (self.world_pose is None or self.estimate is None) and monotonic() < deadline:
            self.spin()
        if not ready or self.world_pose is None or self.estimate is None:
            raise RuntimeError('Nav2 and the local Gazebo measurements never became ready')
        first = self.world_pose
        heading = yaw(first)
        c, s = math.cos(heading), math.sin(heading)
        target_x = first.position.x + args.x * c - args.y * s
        target_y = first.position.y + args.x * s + args.y * c
        target_yaw = heading + args.yaw
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial.pose.pose.orientation.w = 1.0
        self.initial.publish(initial)
        until = self.time + 0.8
        while self.time < until and monotonic() < deadline:
            self.spin()

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = args.x, args.y
        goal.pose.pose.orientation.z = math.sin(args.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(args.yaw / 2)
        future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=12.0)
        if not future.done() or not future.result().accepted:
            raise RuntimeError('NavigateToPose rejected the local goal')
        handle = future.result()
        result = handle.get_result_async()
        start = self.time
        next_report = start
        deadline = monotonic() + args.wall_limit
        while not result.done() and self.time - start < args.sim_limit and monotonic() < deadline:
            self.spin()
            if self.time >= next_report:
                p = self.world_pose.position
                remaining = math.hypot(p.x - target_x, p.y - target_y)
                print(json.dumps({'sim_s': round(self.time - start, 2), 'sim_clock_s': round(self.time, 3),
                                  'gazebo_xy': [round(p.x, 3), round(p.y, 3)],
                                  'odom_xy': [round(v, 3) for v in self.estimate],
                                  'remaining_m': round(remaining, 3), 'ids': sorted(self.markers)}),
                      flush=True)
                next_report = self.time + 3
        status = result.result().status if result.done() else None
        if status is None:
            cancelled = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancelled, timeout_sec=5.0)
        p = self.world_pose.position
        position_error = math.hypot(p.x - target_x, p.y - target_y)
        yaw_error = abs(math.degrees(math.remainder(yaw(self.world_pose) - target_yaw, math.tau)))
        print(json.dumps({'nav2_status': status, 'position_error_m': position_error,
                          'yaw_error_deg': yaw_error, 'ids': sorted(self.markers)}), flush=True)
        return int(status != GoalStatus.STATUS_SUCCEEDED or position_error > 0.5 or yaw_error > 30.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, default=22.7)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--sim-limit', type=float, default=280.0)
    parser.add_argument('--wall-limit', type=float, default=3000.0)
    args = parser.parse_args()
    rclpy.init()
    node = CourseTest()
    try:
        return node.run(args)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
