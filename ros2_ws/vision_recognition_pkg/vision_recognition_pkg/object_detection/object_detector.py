#!/usr/bin/env python3
"""Detect objects in compressed camera frames with TensorRT YOLOv5."""

import json
import glob
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

try:
    import pycuda.driver as cuda
    import tensorrt as trt
except ImportError as error:
    cuda = None
    trt = None
    TENSORRT_IMPORT_ERROR = error
else:
    TENSORRT_IMPORT_ERROR = None


COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
]


def class_name(class_id):
    """Return a COCO label for a numeric class identifier."""
    class_id = int(class_id)
    if 0 <= class_id < len(COCO_CLASSES):
        return COCO_CLASSES[class_id]
    return str(class_id)


def intersection_over_union(first, second):
    """Calculate IoU for two xyxy boxes."""
    x1 = max(first[0], second[0])
    y1 = max(first[1], second[1])
    x2 = min(first[2], second[2])
    y2 = min(first[3], second[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    first_area = max(0.0, first[2] - first[0]) * max(
        0.0, first[3] - first[1]
    )
    second_area = max(0.0, second[2] - second[0]) * max(
        0.0, second[3] - second[1]
    )
    return intersection / (first_area + second_area - intersection + 1e-6)


def non_maximum_suppression(detections, iou_threshold):
    """Suppress overlapping detections, independently for every class."""
    remaining = sorted(detections, key=lambda item: item[4], reverse=True)
    selected = []
    while remaining:
        best = remaining.pop(0)
        selected.append(best)
        remaining = [
            candidate
            for candidate in remaining
            if int(candidate[5]) != int(best[5])
            or intersection_over_union(best, candidate) < iou_threshold
        ]
    return selected


def decode_yolov5_output(output, confidence_threshold):
    """Decode raw or postprocessed YOLOv5 TensorRT output."""
    predictions = np.squeeze(np.asarray(output))
    if predictions.ndim != 2 or predictions.shape[1] < 6:
        return []

    if predictions.shape[1] < 85:
        rows = predictions[predictions[:, 4] >= confidence_threshold]
        return [
            [row[0], row[1], row[2], row[3], row[4], int(row[5])]
            for row in rows
        ]

    objectness = predictions[:, 4]
    class_scores = predictions[:, 5:]
    class_ids = np.argmax(class_scores, axis=1)
    scores = objectness * class_scores[np.arange(class_scores.shape[0]), class_ids]
    keep = scores >= confidence_threshold
    if not np.any(keep):
        return []

    boxes = predictions[keep, :4]
    scores = scores[keep]
    class_ids = class_ids[keep]
    x, y, width, height = (
        boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    )
    return np.stack(
        [
            x - width / 2,
            y - height / 2,
            x + width / 2,
            y + height / 2,
            scores,
            class_ids,
        ],
        axis=1,
    ).tolist()


class TensorRTModel:
    """Own a TensorRT engine, CUDA context, and its reusable buffers."""

    def __init__(self, engine_path):
        cuda.init()
        self._lock = threading.Lock()
        self._cuda_context = cuda.Device(0).make_context()
        try:
            self._logger = trt.Logger(trt.Logger.WARNING)
            self._runtime = trt.Runtime(self._logger)
            with open(engine_path, "rb") as engine_file:
                self._engine = self._runtime.deserialize_cuda_engine(
                    engine_file.read()
                )
            if self._engine is None:
                raise RuntimeError(
                    "TensorRT could not deserialize the engine. Rebuild it on "
                    "this Jetson if its TensorRT version is different."
                )
            self._execution_context = self._engine.create_execution_context()
            self._bindings = []
            self._allocate_buffers()
        finally:
            self._cuda_context.pop()

    def _allocate_buffers(self):
        output_count = 0
        for binding in self._engine:
            shape = tuple(self._engine.get_binding_shape(binding))
            if any(dimension < 0 for dimension in shape):
                raise RuntimeError(
                    f"Dynamic TensorRT binding {binding!r} is not supported"
                )
            size = trt.volume(shape)
            dtype = trt.nptype(self._engine.get_binding_dtype(binding))
            host_memory = cuda.pagelocked_empty(size, dtype)
            device_memory = cuda.mem_alloc(host_memory.nbytes)
            self._bindings.append(int(device_memory))
            if self._engine.binding_is_input(binding):
                self._input_host = host_memory
                self._input_device = device_memory
                self.input_shape = shape
            else:
                output_count += 1
                self._output_host = host_memory
                self._output_device = device_memory
                self._output_shape = shape
        if output_count != 1:
            raise RuntimeError(
                f"Expected one TensorRT output binding, found {output_count}"
            )

    def _preprocess(self, frame):
        _, _, input_height, input_width = self.input_shape
        frame_height, frame_width = frame.shape[:2]
        self.ratio = min(
            input_width / frame_width,
            input_height / frame_height,
        )
        resized_width = int(round(frame_width * self.ratio))
        resized_height = int(round(frame_height * self.ratio))
        horizontal_padding = (input_width - resized_width) / 2.0
        vertical_padding = (input_height - resized_height) / 2.0
        left = int(round(horizontal_padding - 0.1))
        right = int(round(horizontal_padding + 0.1))
        top = int(round(vertical_padding - 0.1))
        bottom = int(round(vertical_padding + 0.1))
        resized = cv2.resize(
            frame,
            (resized_width, resized_height),
            interpolation=cv2.INTER_LINEAR,
        )
        padded = cv2.copyMakeBorder(
            resized,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )
        self.padding = (left, top)
        image = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        image = image.astype(np.float32) / 255.0
        image = np.transpose(image, (2, 0, 1))
        return np.ascontiguousarray(image).ravel()

    def infer(self, frame):
        """Run synchronous inference under the CUDA context that owns buffers."""
        with self._lock:
            self._cuda_context.push()
            try:
                np.copyto(self._input_host, self._preprocess(frame))
                cuda.memcpy_htod(self._input_device, self._input_host)
                if not self._execution_context.execute_v2(self._bindings):
                    raise RuntimeError("TensorRT execute_v2 returned false")
                cuda.memcpy_dtoh(self._output_host, self._output_device)
                return self._output_host.reshape(self._output_shape).copy()
            finally:
                self._cuda_context.pop()


class ObjectDetector(Node):
    """Run TensorRT YOLOv5 and publish annotated and structured detections."""

    def __init__(self):
        super().__init__("object_detector")
        self.declare_parameter(
            "input_topic",
            "/camera/realsense2_camera/color/image_raw/compressed",
        )
        self.declare_parameter(
            "depth_topic",
            "/camera/realsense2_camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter("annotated_topic", "/vision/objects")
        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("max_fps", 30.0)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("tracking_iou_threshold", 0.3)
        self.declare_parameter("tracking_max_age_seconds", 1.0)

        if trt is None or cuda is None:
            raise RuntimeError(
                "TensorRT backend is unavailable. The Jetson container must "
                f"provide 'tensorrt' and 'pycuda': {TENSORRT_IMPORT_ERROR}"
            )

        input_topic = str(self.get_parameter("input_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
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
        self._jpeg_quality = min(
            100,
            max(1, int(self.get_parameter("jpeg_quality").value)),
        )
        self._tracking_iou = min(
            1.0,
            max(
                0.0,
                float(self.get_parameter("tracking_iou_threshold").value),
            ),
        )
        self._tracking_max_age = max(
            0.1,
            float(self.get_parameter("tracking_max_age_seconds").value),
        )
        self._last_processed_at = 0.0
        self._last_inference_finished_at = 0.0
        self._processing_fps = 0.0
        self._latest_depth = None
        self._tracks = {}
        self._next_track_id = 1
        self._last_system_stats_at = 0.0
        self._temperature_c = None
        self._gpu_load_percent = None
        self._cpu_load_percent = None
        self._thermal_paths = glob.glob("/sys/class/thermal/thermal_zone*/temp")

        configured_model = str(self.get_parameter("model_path").value).strip()
        model_path = configured_model or os.path.join(
            get_package_share_directory("ros_cassandra_control"),
            "config",
            "yolov5n.engine",
        )
        if not os.path.isfile(model_path):
            raise RuntimeError(f"TensorRT engine was not found at {model_path!r}")

        self._annotated_publisher = self.create_publisher(
            CompressedImage, annotated_topic, qos_profile_sensor_data
        )
        self._detections_publisher = self.create_publisher(
            String, detections_topic, 10
        )
        self._subscription = self.create_subscription(
            CompressedImage,
            input_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self._depth_subscription = self.create_subscription(
            Image,
            depth_topic,
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f"Loading TensorRT engine: {model_path}")
        self._model = TensorRTModel(model_path)
        self.get_logger().info(
            f"Object detector ready: input={input_topic}, depth={depth_topic}, "
            f"max_fps={max_fps:g}, "
            f"confidence={self._confidence:g}, backend=TensorRT"
        )

    def _image_callback(self, message):
        now = time.monotonic()
        if now - self._last_processed_at < self._minimum_frame_interval:
            return
        self._last_processed_at = now
        started_at = time.monotonic()
        try:
            frame = cv2.imdecode(
                np.frombuffer(message.data, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
            if frame is None:
                raise ValueError("OpenCV could not decode the compressed frame")
            output = self._model.infer(frame)
            raw_detections = decode_yolov5_output(output, self._confidence)
            raw_detections = non_maximum_suppression(raw_detections, self._iou)
            detections = self._map_detections(raw_detections, frame.shape)
            self._assign_tracking_ids(detections, finished_at=time.monotonic())
            self._add_spatial_data(detections, frame.shape)
            target_track_id = self._select_target(detections)
            finished_at = time.monotonic()
            elapsed_ms = (finished_at - started_at) * 1000.0
            self._update_processing_fps(finished_at)
            self._update_system_stats(finished_at)
            self._publish_detections(
                message,
                detections,
                elapsed_ms,
                self._processing_fps,
                frame.shape,
                target_track_id,
            )
            self._draw_detections(frame, detections, target_track_id)
            self._draw_hud(frame, detections, elapsed_ms, target_track_id)
            self._publish_annotated_image(message, frame)
        except Exception as error:
            self.get_logger().error(f"Object detection error: {error}")

    def _depth_callback(self, message):
        try:
            if message.encoding in ("16UC1", "mono16"):
                dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
                row_width = message.step // dtype.itemsize
                depth = np.frombuffer(message.data, dtype=dtype).reshape(
                    message.height, row_width
                )[:, : message.width]
                self._latest_depth = depth.astype(np.float32) * 0.001
            elif message.encoding == "32FC1":
                dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
                row_width = message.step // dtype.itemsize
                self._latest_depth = np.frombuffer(
                    message.data, dtype=dtype
                ).reshape(message.height, row_width)[:, : message.width].copy()
            else:
                self.get_logger().warning(
                    f"Unsupported depth encoding: {message.encoding}",
                    throttle_duration_sec=10.0,
                )
        except Exception as error:
            self.get_logger().warning(f"Cannot decode depth frame: {error}")

    def _map_detections(self, raw_detections, frame_shape):
        frame_height, frame_width = frame_shape[:2]
        padding_x, padding_y = self._model.padding
        mapped = []
        for x1, y1, x2, y2, score, class_id in raw_detections:
            x1 = max(0.0, min(frame_width - 1, (x1 - padding_x) / self._model.ratio))
            y1 = max(0.0, min(frame_height - 1, (y1 - padding_y) / self._model.ratio))
            x2 = max(0.0, min(frame_width - 1, (x2 - padding_x) / self._model.ratio))
            y2 = max(0.0, min(frame_height - 1, (y2 - padding_y) / self._model.ratio))
            mapped.append(
                {
                    "class_id": int(class_id),
                    "label": class_name(class_id),
                    "confidence": round(float(score), 4),
                    "bbox": {
                        "x1": round(float(x1), 1),
                        "y1": round(float(y1), 1),
                        "x2": round(float(x2), 1),
                        "y2": round(float(y2), 1),
                    },
                }
            )
        return mapped

    def _assign_tracking_ids(self, detections, finished_at):
        expired = [
            track_id
            for track_id, track in self._tracks.items()
            if finished_at - track["updated_at"] > self._tracking_max_age
        ]
        for track_id in expired:
            del self._tracks[track_id]

        available_tracks = set(self._tracks)
        for detection in sorted(
            detections,
            key=lambda item: item["confidence"],
            reverse=True,
        ):
            box = detection["bbox"]
            coordinates = [box["x1"], box["y1"], box["x2"], box["y2"]]
            best_track_id = None
            best_iou = self._tracking_iou
            for track_id in available_tracks:
                track = self._tracks[track_id]
                if track["class_id"] != detection["class_id"]:
                    continue
                overlap = intersection_over_union(coordinates, track["bbox"])
                if overlap >= best_iou:
                    best_iou = overlap
                    best_track_id = track_id

            if best_track_id is None:
                best_track_id = self._next_track_id
                self._next_track_id += 1
            else:
                available_tracks.remove(best_track_id)

            detection["track_id"] = best_track_id
            self._tracks[best_track_id] = {
                "class_id": detection["class_id"],
                "bbox": coordinates,
                "updated_at": finished_at,
            }

    def _add_spatial_data(self, detections, frame_shape):
        frame_height, frame_width = frame_shape[:2]
        for detection in detections:
            box = detection["bbox"]
            center_x = (box["x1"] + box["x2"]) / 2.0
            center_y = (box["y1"] + box["y2"]) / 2.0
            normalized_x = center_x / max(1.0, frame_width)
            if normalized_x < 0.4:
                direction = "left"
            elif normalized_x > 0.6:
                direction = "right"
            else:
                direction = "center"
            detection["center"] = {
                "x": round(center_x, 1),
                "y": round(center_y, 1),
            }
            detection["direction"] = direction
            detection["distance_m"] = self._measure_distance(
                center_x,
                center_y,
                box,
                frame_width,
                frame_height,
            )

    def _measure_distance(
        self,
        center_x,
        center_y,
        box,
        frame_width,
        frame_height,
    ):
        if self._latest_depth is None:
            return None
        depth_height, depth_width = self._latest_depth.shape[:2]
        scale_x = depth_width / max(1.0, frame_width)
        scale_y = depth_height / max(1.0, frame_height)
        half_width = max(2, int((box["x2"] - box["x1"]) * scale_x * 0.1))
        half_height = max(2, int((box["y2"] - box["y1"]) * scale_y * 0.1))
        depth_x = int(center_x * scale_x)
        depth_y = int(center_y * scale_y)
        x1 = max(0, depth_x - half_width)
        x2 = min(depth_width, depth_x + half_width + 1)
        y1 = max(0, depth_y - half_height)
        y2 = min(depth_height, depth_y + half_height + 1)
        values = self._latest_depth[y1:y2, x1:x2]
        valid = values[np.isfinite(values) & (values > 0.1) & (values < 20.0)]
        if valid.size == 0:
            return None
        return round(float(np.median(valid)), 2)

    @staticmethod
    def _select_target(detections):
        people = [item for item in detections if item["label"] == "person"]
        if not people:
            return None
        people_with_depth = [
            item for item in people if item["distance_m"] is not None
        ]
        if people_with_depth:
            target = min(people_with_depth, key=lambda item: item["distance_m"])
        else:
            target = max(people, key=lambda item: item["confidence"])
        return target["track_id"]

    @staticmethod
    def _draw_detections(frame, detections, target_track_id):
        for detection in detections:
            box = detection["bbox"]
            first = (int(box["x1"]), int(box["y1"]))
            second = (int(box["x2"]), int(box["y2"]))
            is_target = detection["track_id"] == target_track_id
            color = (0, 0, 255) if is_target else (0, 255, 0)
            thickness = 3 if is_target else 2
            distance = detection["distance_m"]
            distance_label = "" if distance is None else f" {distance:.2f}m"
            target_label = " TARGET" if is_target else ""
            cv2.rectangle(frame, first, second, color, thickness)
            cv2.putText(
                frame,
                f"#{detection['track_id']} {detection['label']} "
                f"{detection['confidence']:.2f} {detection['direction']}"
                f"{distance_label}{target_label}",
                (first[0], max(0, first[1] - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                thickness,
            )
            center = detection["center"]
            cv2.circle(
                frame,
                (int(center["x"]), int(center["y"])),
                4,
                color,
                -1,
            )

    def _update_processing_fps(self, finished_at):
        if self._last_inference_finished_at > 0.0:
            interval = finished_at - self._last_inference_finished_at
            if interval > 0.0:
                current_fps = 1.0 / interval
                if self._processing_fps <= 0.0:
                    self._processing_fps = current_fps
                else:
                    self._processing_fps = (
                        0.8 * self._processing_fps + 0.2 * current_fps
                    )
        self._last_inference_finished_at = finished_at

    def _update_system_stats(self, now):
        if now - self._last_system_stats_at < 1.0:
            return
        self._last_system_stats_at = now
        temperatures = []
        for path in self._thermal_paths:
            try:
                with open(path, "r", encoding="ascii") as temperature_file:
                    value = float(temperature_file.read().strip())
                temperatures.append(value / 1000.0 if value > 200.0 else value)
            except (OSError, ValueError):
                continue
        self._temperature_c = max(temperatures) if temperatures else None
        try:
            with open("/sys/devices/gpu.0/load", "r", encoding="ascii") as load_file:
                self._gpu_load_percent = float(load_file.read().strip()) / 10.0
        except (OSError, ValueError):
            self._gpu_load_percent = None
        try:
            cpu_count = max(1, os.cpu_count() or 1)
            self._cpu_load_percent = min(
                999.0, os.getloadavg()[0] * 100.0 / cpu_count
            )
        except OSError:
            self._cpu_load_percent = None

    def _draw_hud(self, frame, detections, inference_ms, target_track_id):
        height, width = frame.shape[:2]
        temperature = (
            "--" if self._temperature_c is None else f"{self._temperature_c:.0f}C"
        )
        gpu_load = (
            "--"
            if self._gpu_load_percent is None
            else f"{self._gpu_load_percent:.0f}%"
        )
        cpu_load = (
            "--"
            if self._cpu_load_percent is None
            else f"{self._cpu_load_percent:.0f}%"
        )
        first_line = (
            f"FPS {self._processing_fps:.1f} | {inference_ms:.0f}ms | "
            f"Objects {len(detections)} | {width}x{height}"
        )
        second_line = (
            f"GPU {gpu_load} | CPU {cpu_load} | Temp {temperature} | "
            f"{time.strftime('%H:%M:%S')}"
        )
        cv2.rectangle(frame, (5, 5), (min(width - 5, 620), 66), (0, 0, 0), -1)
        cv2.putText(
            frame,
            first_line,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            1,
        )
        cv2.putText(
            frame,
            second_line,
            (12, 54),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )
        if not detections:
            text_size = cv2.getTextSize(
                "NO DETECTIONS", cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2
            )[0]
            cv2.putText(
                frame,
                "NO DETECTIONS",
                ((width - text_size[0]) // 2, height // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
            )
        elif target_track_id is not None:
            cv2.putText(
                frame,
                f"TARGET: person #{target_track_id}",
                (12, 88),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )

    def _publish_detections(
        self,
        source_message,
        detections,
        elapsed_ms,
        processing_fps,
        frame_shape,
        target_track_id,
    ):
        output = String()
        output.data = json.dumps(
            {
                "stamp": {
                    "sec": source_message.header.stamp.sec,
                    "nanosec": source_message.header.stamp.nanosec,
                },
                "frame_id": source_message.header.frame_id,
                "inference_ms": round(elapsed_ms, 1),
                "processing_fps": round(processing_fps, 2),
                "resolution": {
                    "width": frame_shape[1],
                    "height": frame_shape[0],
                },
                "jetson": {
                    "temperature_c": self._temperature_c,
                    "gpu_load_percent": self._gpu_load_percent,
                    "cpu_load_percent": self._cpu_load_percent,
                },
                "target_track_id": target_track_id,
                "count": len(detections),
                "detections": detections,
            },
            ensure_ascii=False,
        )
        self._detections_publisher.publish(output)

    def _publish_annotated_image(self, source_message, frame):
        success, encoded = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
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
