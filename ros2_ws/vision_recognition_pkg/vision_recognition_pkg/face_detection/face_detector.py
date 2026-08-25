import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from rclpy.qos import qos_profile_sensor_data

import cv2


class FaceDetector(Node):
    def __init__(self):
        super().__init__("face_detector")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            CompressedImage,
            "/camera/realsense2_camera/color/image_raw/compressed",
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.publisher = self.create_publisher(
            CompressedImage, "/vision/faces1", qos_profile_sensor_data
        )
        cascade_path = (
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
        )
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        self.get_logger().info("Face Detector Started")

    def image_callback(self, msg):
        self.get_logger().info("Frame received")

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(
                gray, scaleFactor=1.3, minNeighbors=5, minSize=(30, 30)
            )
            for x, y, w, h in faces:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            out_msg = CompressedImage()
            out_msg.data = cv2.imencode(".jpg", frame)[1].tobytes()
            self.publisher.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
