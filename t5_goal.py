from math import atan2, isfinite, remainder, tau
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from t5_frontier import FinishRetryBudget, choose_waypoint


def yaw(q):
    return atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class GoalServer(Node):
    def __init__(self):
        super().__init__('go1_t5_goal', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.group = ReentrantCallbackGroup()
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.map = None
        self.busy = False
        self.pending = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.receive_map, latched)
        self.client = ActionClient(self, NavigateToPose, '/t5_nav2/navigate_to_pose', callback_group=self.group)
        self.spin_client = ActionClient(self, Spin, '/t5_nav2/spin', callback_group=self.group)
        self.navigator_state = self.create_client(GetState, '/t5_nav2/bt_navigator/get_state')
        self.map_refresh = self.create_client(Trigger, '/go1_t5_mapper/refresh_recent')
        self.state_request = None
        self.server = None
        self.startup = self.create_timer(0.2, self.advertise_when_ready)
        self.explore_tree = str(Path(__file__).with_name('t5_explore.xml'))

    def advertise_when_ready(self):
        if self.map is None or not self.buffer.can_transform('map', 'trunk', Time()):
            return
        if (not self.client.server_is_ready() or not self.spin_client.server_is_ready()
            or not self.navigator_state.service_is_ready()):
            return
        if self.state_request is None:
            self.state_request = self.navigator_state.call_async(GetState.Request())
        elif self.state_request.done():
            state = self.state_request.result().current_state
            self.state_request = None
            if state.id == State.PRIMARY_STATE_ACTIVE:
                self.server = ActionServer(
                    self, NavigateToPose, '/navigate_to_pose', execute_callback=self.execute,
                    goal_callback=self.accept, cancel_callback=self.cancel, callback_group=self.group,
                )
                self.startup.cancel()

    def receive_map(self, message):
        self.map = message

    def accept(self, request):
        p = request.pose.pose.position
        q = request.pose.pose.orientation
        if (self.busy or not self.client.server_is_ready() or self.map is None
            or request.pose.header.frame_id.lstrip('/') != 'map'
            or not all(map(isfinite, (p.x, p.y, q.x, q.y, q.z, q.w)))
            or not 0.5 < q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 1.5):
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def cancel(self, handle):
        if self.pending is not None:
            self.pending.cancel_goal_async()
        return CancelResponse.ACCEPT

    def position(self):
        transform = self.buffer.lookup_transform('map', 'trunk', Time())
        p = transform.transform.translation
        return np.array([p.x, p.y])

    def pick(self, position, target, avoid, recovery=False):
        msg = self.map
        info = msg.info
        origin = info.origin.position.x, info.origin.position.y
        cells = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        return choose_waypoint(cells, origin, info.resolution, position, target, avoid, recovery=recovery)

    async def wait_for_scan(self):
        delay = Future()
        timer = self.create_timer(0.4, lambda: delay.set_result(None))
        try:
            await delay
        finally:
            self.destroy_timer(timer)

    async def align_heading(self, handle, orientation):
        await self.wait_for_scan()
        if handle.is_cancel_requested:
            return False
        current = self.buffer.lookup_transform('map', 'trunk', Time()).transform.rotation
        error = remainder(yaw(orientation) - yaw(current), tau)
        self.get_logger().info(f'Final heading adjustment: {error:.3f} rad')
        if abs(error) <= 0.20:
            return True
        goal = Spin.Goal()
        goal.target_yaw = error
        goal.time_allowance.sec = 20
        self.pending = await self.spin_client.send_goal_async(goal)
        if not self.pending.accepted:
            return False
        if handle.is_cancel_requested:
            await self.pending.cancel_goal_async()
        result = await self.pending.get_result_async()
        self.pending = None
        return result.status == GoalStatus.STATUS_SUCCEEDED

    async def execute(self, handle):
        target_pose = handle.request.pose.pose
        target = np.array([target_pose.position.x, target_pose.position.y])
        avoid = []
        no_waypoint = 0
        retry = FinishRetryBudget()
        recovery_waypoint = False

        def feedback(event):
            message = event.feedback
            try:
                message.distance_remaining = float(np.linalg.norm(self.position() - target))
            except TransformException:
                pass
            handle.publish_feedback(message)

        try:
            for _ in range(45):
                if handle.is_cancel_requested:
                    handle.canceled()
                    return NavigateToPose.Result()
                current = self.position()
                selection = self.pick(current, target, avoid, recovery_waypoint)
                if selection is None:
                    if recovery_waypoint:
                        recovery_waypoint = False
                        await self.wait_for_scan()
                        continue
                    no_waypoint += 1
                    if no_waypoint >= 6:
                        self.get_logger().warn('No reachable navigation waypoint on the current map')
                        break
                    await self.wait_for_scan()
                    continue
                no_waypoint = 0
                point, final = selection
                goal = NavigateToPose.Goal()
                goal.pose.header.frame_id = 'map'
                goal.pose.pose.position.x, goal.pose.pose.position.y = map(float, point)
                if final:
                    goal.pose.pose.orientation = target_pose.orientation
                else:
                    goal.pose.pose.orientation.w = 1.0
                    goal.behavior_tree = self.explore_tree
                self.get_logger().info(f'Nav2 {"goal" if final else "waypoint"} x={point[0]:.2f} y={point[1]:.2f}')
                self.pending = await self.client.send_goal_async(goal, feedback_callback=feedback)
                if handle.is_cancel_requested and self.pending.accepted:
                    await self.pending.cancel_goal_async()
                if not self.pending.accepted:
                    break
                result = await self.pending.get_result_async()
                self.pending = None
                if handle.is_cancel_requested:
                    handle.canceled()
                    return result.result
                if final:
                    aligned = result.status == GoalStatus.STATUS_SUCCEEDED and await self.align_heading(
                        handle, target_pose.orientation)
                    if handle.is_cancel_requested:
                        handle.canceled()
                        return result.result
                    if aligned:
                        handle.succeed()
                        return result.result
                    if result.status != GoalStatus.STATUS_ABORTED:
                        break
                    distance = float(np.linalg.norm(self.position() - target))
                    attempt = retry.on_abort(distance)
                    if attempt is None:
                        break
                    if self.map_refresh.service_is_ready():
                        refresh = await self.map_refresh.call_async(Trigger.Request())
                        self.get_logger().info(refresh.message)
                    recovery_waypoint = attempt == 2
                    self.get_logger().info(f'Replanning final approach after Nav2 abort ({attempt}/2, {distance:.2f} m left)')
                    await self.wait_for_scan()
                    continue
                avoid.append(point.copy())
                recovery_waypoint = False
        except TransformException as error:
            self.get_logger().warn(str(error))
        finally:
            self.busy = False
            self.pending = None
        handle.abort()
        return NavigateToPose.Result()


def main():
    rclpy.init()
    node = GoalServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
