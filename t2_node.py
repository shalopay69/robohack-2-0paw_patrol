import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_go.msg import LowCmd

from t2_safety import PUBLISH_PERIOD_NS, CollisionChecker, SafetyBlock


class SafetyNode(Node):
    def __init__(self):
        super().__init__('go1_safety_block', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.block = SafetyBlock(CollisionChecker('/opt/go1/model'))
        input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.output = self.create_publisher(LowCmd, '/lowcmd', 1)
        self.create_subscription(LowCmd, '/lowcmd_bare', self.receive, input_qos)
        self.create_timer(PUBLISH_PERIOD_NS / 1_000_000_000, self.publish_safe_command)

    def receive(self, message):
        commands = np.array([(motor.q, motor.dq, motor.tau, motor.kp, motor.kd)
                             for motor in message.motor_cmd])
        self.block.receive(commands, self.get_clock().now().nanoseconds)

    def publish_safe_command(self):
        target = self.block.sample(self.get_clock().now().nanoseconds)
        message = LowCmd()
        for motor, values in zip(message.motor_cmd, target):
            motor.mode = 0x0A
            motor.q, motor.dq, motor.tau, motor.kp, motor.kd = map(float, values)
        self.output.publish(message)


def main():
    rclpy.init()
    node = None
    try:
        node = SafetyNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
