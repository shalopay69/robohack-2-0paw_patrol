from collections import deque
from math import isfinite

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


PULSE = 0.27
REST = 0.38
ATTITUDE_WINDOW = 0.17
ATTITUDE_SPAN = 0.12
MAX_GRAVITY_CHANGE_SQUARED = 0.00010
MAX_TILT_SQUARED = 0.00070
COMMAND_AGE = 0.45
IMU_AGE = 0.15


class VelocityGate(Node):
    def __init__(self):
        super().__init__('go1_t5_velocity_gate', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.request = None
        self.request_time = 0.0
        self.imu_time = -IMU_AGE
        self.attitudes = deque()
        self.active_since = None
        self.rest_since = 0.0
        self.output = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/t5_cmd_vel_requested', self.receive_command, 10)
        self.create_subscription(Imu, '/imu/data', self.receive_imu, qos_profile_sensor_data)
        self.create_timer(0.025, self.send)

    def seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def receive_command(self, message):
        real = (message.linear.x, message.linear.y, message.angular.z)
        self.request = message if all(map(isfinite, real)) and max(map(abs, real)) > 0.001 else None
        self.request_time = self.seconds()

    def receive_imu(self, message):
        now = self.seconds()
        q = message.orientation
        if now < self.imu_time or now - self.imu_time > IMU_AGE:
            self.attitudes.clear()
        self.attitudes.append((now, 2 * (q.x*q.z - q.w*q.y), 2 * (q.y*q.z + q.w*q.x)))
        while self.attitudes and now - self.attitudes[0][0] > ATTITUDE_WINDOW:
            self.attitudes.popleft()
        self.imu_time = now

    def is_stable(self, now):
        if not self.attitudes or now - self.attitudes[0][0] < ATTITUDE_SPAN:
            return False
        _, x0, y0 = self.attitudes[0]
        return all(
            x*x + y*y <= MAX_TILT_SQUARED
            and (x-x0)*(x-x0) + (y-y0)*(y-y0) <= MAX_GRAVITY_CHANGE_SQUARED
            for _, x, y in self.attitudes
        )

    def send(self):
        now = self.seconds()
        command = self.request
        command_ready = command is not None and 0.0 <= now - self.request_time <= COMMAND_AGE
        imu_ready = 0.0 <= now - self.imu_time <= IMU_AGE
        if (command_ready and imu_ready and abs(command.linear.x) <= 0.015
            and abs(command.linear.y) <= 0.015 and abs(command.angular.z) >= 0.06):
            self.active_since = None
            self.rest_since = now
            self.output.publish(command)
            return
        if self.active_since is not None and (not command_ready or not imu_ready or now - self.active_since >= PULSE):
            self.active_since = None
            self.rest_since = now
        if (self.active_since is None and command_ready and imu_ready and now - self.rest_since >= REST
            and self.is_stable(now)):
            self.active_since = now
        self.output.publish(command if self.active_since is not None else Twist())


def main():
    rclpy.init()
    node = VelocityGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.output.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
