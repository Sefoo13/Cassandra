#!/usr/bin/env python3
"""Detect objects in compressed camera frames with Ultralytics YOLO."""

import json
import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

try:
    from ultralytics import YOLO
except ImportError as error:
    YOLO = None
    YOLO_IMPORT_ERROR = error
else:
    YOLO_IMPORT_ERROR = None


class ObjectDetector(Node):
    """Run YOLO on camera frames and publish images plus structured results."""

    def __init__(self):
        super().__init__("object_detector")
        self.declare_parameter(
            "input_topic",
            "/camera/realsense2_camera/color/image_raw/compressed",
        )
        self.declare_parameter("annotated_topic", "/vision/objects")
        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("max_fps", 5.0)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("device", "auto")
        self.declare_parameter("jpeg_quality", 85)

        if YOLO is None:
            raise RuntimeError(
                "Ultralytics is unavailable. Install a Jetson-compatible "
                f"PyTorch build and 'ultralytics': {YOLO_IMPORT_ERROR}"
            )

        input_topic = str(self.get_parameter("input_topic").value)
        annotated_topic = str(self.get_parameter("annotated_topic").value)
        detections_topic = str(self.get_parameter("detections_topic").value)
        self._confidence = min(
            1.0,
            max(0.0, float(self.get_parameter("confidence_threshold").value)),
        )
        self._iou = min(
            1.0,
            max(0.0, float(self.get_parameter("iou_threshold").value)),
        )
        max_fps = max(0.1, float(self.get_parameter("max_fps").value))
        self._minimum_frame_interval = 1.0 / max_fps
        self._image_size = max(32, int(self.get_parameter("image_size").value))
        self._device = str(self.get_parameter("device").value).strip().lower()
        self._jpeg_quality = min(
            100,
            max(1, int(self.get_parameter("jpeg_quality").value)),
        )
        self._last_processed_at = 0.0

        configured_model = str(self.get_parameter("model_path").value).strip()
        model_path = configured_model or os.path.join(
            get_package_share_directory("vision_recognition_pkg"),
            "models",
            "yolov8n.pt",
        )
        if not os.path.isfile(model_path):
            raise RuntimeError(f"YOLO model was not found at {model_path!r}")

        self._annotated_publisher = self.create_publisher(
            CompressedImage,
            annotated_topic,
            qos_profile_sensor_data,
        )
        self._detections_publisher = self.create_publisher(
            String,
            detections_topic,
            10,
        )
        self._subscription = self.create_subscription(
            CompressedImage,
            input_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self._model = YOLO(model_path)
        self.get_logger().info(
            f"Object detector ready: input={input_topic}, max_fps={max_fps:g}, "
            f"confidence={self._confidence:g}, device={self._device}"
        )

    def _image_callback(self, message):
        now = time.monotonic()
        if now - self._last_processed_at < self._minimum_frame_interval:
            return
        self._last_processed_at = now
        started_at = time.monotonic()

        try:
            encoded = np.frombuffer(message.data, dtype=np.uint8)
            frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError("OpenCV could not decode the compressed frame")

            predict_options = {
                "source": frame,
                "conf": self._confidence,
                "iou": self._iou,
                "imgsz": self._image_size,
                "verbose": False,
            }
            if self._device != "auto":
                predict_options["device"] = self._device
            result = self._model.predict(**predict_options)[0]

            detections = self._serialize_detections(result)
            elapsed_ms = (time.monotonic() - started_at) * 1000.0
            self._publish_detections(message, detections, elapsed_ms)
            self._publish_annotated_image(message, result.plot())
        except Exception as error:
            self.get_logger().error(f"Object detection error: {error}")

    @staticmethod
    def _serialize_detections(result):
        detections = []
        names = result.names
        if result.boxes is None:
            return detections

        coordinates = result.boxes.xyxy.detach().cpu().tolist()
        confidences = result.boxes.conf.detach().cpu().tolist()
        classes = result.boxes.cls.detach().cpu().tolist()
        for xyxy, confidence, class_value in zip(
            coordinates,
            confidences,
            classes,
        ):
            class_id = int(class_value)
            if isinstance(names, dict):
                label = str(names.get(class_id, class_id))
            else:
                label = str(names[class_id])
            detections.append(
                {
                    "class_id": class_id,
                    "label": label,
                    "confidence": round(float(confidence), 4),
                    "bbox": {
                        "x1": round(float(xyxy[0]), 1),
                        "y1": round(float(xyxy[1]), 1),
                        "x2": round(float(xyxy[2]), 1),
                        "y2": round(float(xyxy[3]), 1),
                    },
                }
            )
        return detections

    def _publish_detections(self, source_message, detections, elapsed_ms):
        output = String()
        output.data = json.dumps(
            {
                "stamp": {
                    "sec": source_message.header.stamp.sec,
                    "nanosec": source_message.header.stamp.nanosec,
                },
                "frame_id": source_message.header.frame_id,
                "inference_ms": round(elapsed_ms, 1),
                "count": len(detections),
                "detections": detections,
            },
            ensure_ascii=False,
        )
        self._detections_publisher.publish(output)

    def _publish_annotated_image(self, source_message, frame):
        success, encoded = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
        )
        if not success:
            raise RuntimeError("OpenCV could not encode the annotated frame")

        output = CompressedImage()
        output.header = source_message.header
        output.format = "jpeg"
        output.data = encoded.tobytes()
        self._annotated_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObjectDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
