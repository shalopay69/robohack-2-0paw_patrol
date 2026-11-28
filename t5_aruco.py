import json

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


CAMERAS = ('camera', 'camera_chin', 'camera_left', 'camera_right', 'camera_rearDown')


class Detector:
    def __init__(self):
        cv2.setNumThreads(1)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.minMarkerPerimeterRate = 0.025
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.errorCorrectionRate = 0.5
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

    def detect(self, pixels):
        gray = cv2.cvtColor(pixels, cv2.COLOR_RGB2GRAY)
        _, ids, _ = self.detector.detectMarkers(gray)
        return [] if ids is None else sorted({int(i) for i in ids.ravel()})


class ArucoNode(Node):
    def __init__(self):
        super().__init__('go1_t5_aruco', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.detector = Detector()
        self.publisher = self.create_publisher(String, '/aruco/detections', 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.last_frames = {}
        for camera in CAMERAS:
            self.create_subscription(Image, f'/{camera}/image_raw', self.receive, qos)

    def receive(self, message):
        if (message.encoding != 'rgb8' or message.width <= 0 or message.height <= 0
            or message.step < 3 * message.width or len(message.data) != message.height * message.step):
            return
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = self.get_clock().now().nanoseconds - stamp
        frame = message.header.frame_id
        if age > 350_000_000 or stamp - self.last_frames.get(frame, -1_000_000_000) < 95_000_000:
            return
        self.last_frames[frame] = stamp
        buffer = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        pixels = buffer[:, :message.width * 3].reshape(message.height, message.width, 3)
        ids = self.detector.detect(pixels)
        if ids:
            self.publisher.publish(String(data=json.dumps({'ids': ids}, separators=(',', ':'))))


def main():
    rclpy.init()
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
