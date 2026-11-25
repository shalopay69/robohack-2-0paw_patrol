from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_go.msg import LowCmd, LowState

from t2_node import SafetyNode
from t3_policy import POLICY_PERIOD, ObservationHistory, WalkingPolicy, joint_targets


POLICY_PERIOD_NS = round(POLICY_PERIOD * 1_000_000_000)
STALE_STATE_NS = 100_000_000


class WalkingNode(Node):
    def __init__(self, policy, home):
        super().__init__('go1_walking', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.policy = policy
        self.home = home
        self.state = None
        self.velocity = np.zeros(3)
        self.history = ObservationHistory()
        self.action = np.zeros(12, dtype=np.float32)
        self.next_inference_ns = None
        self.output = self.create_publisher(LowCmd, '/lowcmd_bare', 1)
        input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LowState, '/lowstate', self.receive_state, input_qos)
        self.create_subscription(Twist, '/cmd_vel', self.receive_velocity, input_qos)
        self.create_timer(0.01, self.publish_command)

    def receive_state(self, message):
        quaternion = np.asarray(message.imu_state.quaternion, dtype=float)
        angles = np.array([(motor.q, motor.dq) for motor in message.motor_state[:12]])
        if not np.isfinite(angles).all() or not np.isfinite(quaternion).all():
            return
        if not 0.5 < np.linalg.norm(quaternion) < 1.5:
            return
        self.state = self.get_clock().now().nanoseconds, quaternion, angles

    def receive_velocity(self, message):
        velocity = np.array([message.linear.x, message.linear.y, message.angular.z])
        if np.isfinite(velocity).all():
            self.velocity = np.clip(velocity, [0.0, -0.6, -0.5], [0.6, 0.6, 0.5])

    def stop_walking(self):
        if self.next_inference_ns is not None:
            self.history = ObservationHistory()
            self.action.fill(0)
            self.next_inference_ns = None

    def publish_command(self):
        now = self.get_clock().now().nanoseconds
        if self.state is None or not 0 <= now - self.state[0] <= STALE_STATE_NS:
            self.stop_walking()
            return
        if np.max(np.abs(self.velocity)) < 1e-4:
            self.stop_walking()
            target, kp, kd = self.home, 40.0, 2.0
        else:
            if self.next_inference_ns is None or now >= self.next_inference_ns:
                _, quaternion, angles = self.state
                observation = self.history.append(
                    angles[:, 0], angles[:, 1], quaternion, self.velocity, self.action
                )
                self.action = np.clip(self.policy.action(observation), -10.0, 10.0)
                if self.next_inference_ns is None or now - self.next_inference_ns >= POLICY_PERIOD_NS:
                    self.next_inference_ns = now
                self.next_inference_ns += POLICY_PERIOD_NS
            target, kp, kd = joint_targets(self.action), 20.0, 0.5
        command = LowCmd()
        for motor, position in zip(command.motor_cmd, target):
            motor.mode = 0x0A
            motor.q = float(position)
            motor.kp = kp
            motor.kd = kd
        self.output.publish(command)


def main():
    policy = WalkingPolicy(Path(__file__).with_name('t3_weights.npz'))
    rclpy.init()
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        safety = SafetyNode()
        nodes.append(safety)
        nodes.append(WalkingNode(policy, safety.block.checker.home))
        for node in nodes:
            executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
