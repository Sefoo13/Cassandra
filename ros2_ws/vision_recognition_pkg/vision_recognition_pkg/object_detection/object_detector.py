import rclpy
from rclpy.node import Node
import cv2

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from rclpy.qos import qos_profile_sensor_data
import numpy as np

import os

from ultralytics import YOLO


class ObjectDetector(Node):
    def __init__(self):
        super().__init__("object_detector")
        self.bridge = CvBridge()
        # 📷 Subscription
        self.subscription = self.create_subscription(
            CompressedImage,
            "/camera/realsense2_camera/color/image_raw/compressed",
            self.image_callback,
            qos_profile_sensor_data,
        )
        # 📡 Publisher
        self.publisher = self.create_publisher(
            CompressedImage,
            "/vision/objects", qos_profile_sensor_data
        )
        # 📦 Load YOLO model
        model_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), "..", "models", "yolov8n.pt"
        )
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        self.get_logger().info("Object Detector Started")

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # YOLO detection
            results = self.model(frame)[0]
            # Draw boxes
            annotated_frame = results.plot()
            out_msg = CompressedImage()
            out_msg.data = cv2.imencode(".jpg", annotated_frame)[1].tobytes()
            self.publisher.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"Object detection error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
