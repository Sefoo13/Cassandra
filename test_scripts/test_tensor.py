from flask import Flask, Response
import cv2
import time
import threading
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda

cuda.init()
app = Flask(__name__)

# COCO class names: YOLOv5 returns a numeric class id, so map it to a name
# (id 0 -> "person") for the on-screen label.
COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]


def class_name(class_id):
    cid = int(class_id)
    if 0 <= cid < len(COCO_CLASSES):
        return COCO_CLASSES[cid]
    return str(cid)


# =========================
# CAMERA
# =========================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

prev_time = 0


def iou(box1, box2):
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])

    inter = max(0, x2 - x1) * max(0, y2 - y1)

    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])

    return inter / (area1 + area2 - inter + 1e-6)


# -------------------------
# NMS
# -------------------------
def nms(dets, iou_thres=0.45):
    dets = sorted(dets, key=lambda x: x[4], reverse=True)
    keep = []

    while dets:
        best = dets.pop(0)
        keep.append(best)

        dets = [d for d in dets if iou(best, d) < iou_thres]

    return keep


# -------------------------
# YOLOv5 decode
# -------------------------
def decode_output(outputs, conf_thres=0.25):
    # Vectorized: a per-row Python loop over ~25200 YOLO predictions stalls the
    # Jetson Nano (FPS -> 0). Do the whole decode with numpy instead.
    preds = np.squeeze(np.array(outputs))

    if preds.ndim != 2 or preds.shape[1] < 6:
        return []

    # CASE B: already x1,y1,x2,y2,score,class
    if preds.shape[1] < 85:
        rows = preds[preds[:, 4] >= conf_thres]
        return [
            [r[0], r[1], r[2], r[3], r[4], int(r[5])] for r in rows
        ]

    # CASE A: raw YOLO format (x, y, w, h, obj, cls0..clsN)
    obj = preds[:, 4]
    cls_scores = preds[:, 5:]
    class_ids = np.argmax(cls_scores, axis=1)
    scores = obj * cls_scores[np.arange(cls_scores.shape[0]), class_ids]

    keep = scores >= conf_thres
    if not np.any(keep):
        return []

    boxes = preds[keep, :4]
    scores = scores[keep]
    class_ids = class_ids[keep]

    # xywh -> xyxy
    x, y, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    xyxy = np.stack(
        [x - w / 2, y - h / 2, x + w / 2, y + h / 2, scores, class_ids],
        axis=1,
    )
    return xyxy.tolist()


class TensorRTModel:
    def __init__(self, engine_path):
        self.logger = trt.Logger(trt.Logger.WARNING)

        # A CUDA context must be current BEFORE the engine is deserialized, and
        # the SAME context must own the execution context, the buffers, and
        # every execute_v2() call. Deserializing first (under the driver's
        # implicit primary context) and then creating the execution context
        # under a different make_context() context binds the engine and its
        # kernels to mismatched contexts -> "invalid resource handle" at launch.
        self.device = cuda.Device(0)
        self.cuda_context = self.device.make_context()

        # The context and host/device buffers are shared, so concurrent Flask
        # worker threads must not run infer() at the same time.
        self.lock = threading.Lock()

        runtime = trt.Runtime(self.logger)
        with open(engine_path, "rb") as f:
            self.engine = runtime.deserialize_cuda_engine(f.read())

        self.trt_context = self.engine.create_execution_context()
        self.bindings = []

        for binding in self.engine:
            size = trt.volume(self.engine.get_binding_shape(binding))
            dtype = trt.nptype(self.engine.get_binding_dtype(binding))

            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)

            self.bindings.append(int(device_mem))

            if self.engine.binding_is_input(binding):
                self.input_host = host_mem
                self.input_device = device_mem
                # Keep the input shape (N, C, H, W) for preprocessing.
                self.input_shape = tuple(self.engine.get_binding_shape(binding))
            else:
                self.output_host = host_mem
                self.output_device = device_mem
                # Keep the output shape (e.g. (1, 25200, 85)) so the flat host
                # buffer can be reshaped before decoding -- otherwise decode
                # sees a 1-D array and returns no detections.
                self.output_shape = tuple(self.engine.get_binding_shape(binding))

        # make_context() left the context current on *this* (import) thread.
        # Pop it so worker threads can push it themselves in infer().
        self.cuda_context.pop()

    def preprocess(self, frame):
        # Letterbox: resize keeping aspect ratio and pad to the network size so
        # boxes don't shift. The scale ratio and padding are saved so detections
        # can be mapped back to the original frame.
        _, _, in_h, in_w = self.input_shape
        h0, w0 = frame.shape[:2]

        r = min(in_w / w0, in_h / h0)
        new_w, new_h = int(round(w0 * r)), int(round(h0 * r))
        dw = (in_w - new_w) / 2.0  # padding split between both sides
        dh = (in_h - new_h) / 2.0

        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        padded = cv2.copyMakeBorder(
            resized, top, bottom, left, right,
            cv2.BORDER_CONSTANT, value=(114, 114, 114),
        )

        # Save the mapping for postprocess (single-threaded worker, so safe).
        self.ratio = r
        self.pad = (left, top)

        # YOLOv5 expects RGB, normalized [0, 1], CHW, contiguous float32.
        img = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC -> CHW
        return np.ascontiguousarray(img).ravel()

    def infer(self, frame):
        # Serialize: shared context/buffers are not safe for concurrent use,
        # and the context must be current on the calling thread (Flask is
        # threaded). The lock guards both invariants.
        with self.lock:
            self.cuda_context.push()
            try:
                # copy the actual frame into the pinned input buffer
                np.copyto(self.input_host, self.preprocess(frame))

                # Synchronous execution on the context's default stream. An
                # explicit cross-thread cuda.Stream() handle is what triggered
                # the "invalid resource handle" errors, so we avoid it.
                cuda.memcpy_htod(self.input_device, self.input_host)
                self.trt_context.execute_v2(bindings=self.bindings)
                cuda.memcpy_dtoh(self.output_host, self.output_device)

                # Reshape the flat buffer back to the network's output shape
                # and copy (output_host is reused on the next inference).
                return self.output_host.reshape(self.output_shape).copy()
            finally:
                self.cuda_context.pop()


def run_engine_inference(model, frame):
    outputs = model.infer(frame)
    detections = nms(decode_output(outputs))

    # Undo the letterbox: remove padding, then divide by the scale ratio.
    r = model.ratio
    pad_x, pad_y = model.pad
    h0, w0 = frame.shape[:2]

    for x1, y1, x2, y2, score, class_id in detections:
        x1 = (x1 - pad_x) / r
        y1 = (y1 - pad_y) / r
        x2 = (x2 - pad_x) / r
        y2 = (y2 - pad_y) / r
        # Clamp to the frame bounds.
        p1 = (max(0, int(x1)), max(0, int(y1)))
        p2 = (min(w0 - 1, int(x2)), min(h0 - 1, int(y2)))
        cv2.rectangle(frame, p1, p2, (0, 255, 0), 2)
        cv2.putText(
            frame,
            f"{class_name(class_id)}: {score:.2f}",
            (p1[0], max(0, p1[1] - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )

    return frame


# =========================
# DEDICATED CUDA WORKER
# =========================
# All CUDA work -- context creation, the engine, and every inference -- must
# stay on ONE thread. TensorRT's internal cuDNN/cuBLAS handles are bound to the
# thread/context that created them, so touching them from Flask's per-connection
# worker threads raises "Cuda Driver (invalid resource handle)" (the error seen
# on the Jetson Nano). This thread owns the context for its whole lifetime and
# only publishes finished JPEG frames for the HTTP routes to read.
latest_jpeg = None
latest_lock = threading.Lock()
stop_event = threading.Event()


def inference_worker():
    global latest_jpeg, prev_time

    # Created here so make_context() runs on this thread, not at import time.
    model = TensorRTModel("yolov5n.engine")

    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

        frame = run_engine_inference(model, frame)

        current_time = time.time()
        fps = 1 / (current_time - prev_time) if prev_time else 0
        prev_time = current_time

        cv2.putText(
            frame,
            f"FPS: {int(fps)}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )

        ok, buffer = cv2.imencode(".jpg", frame)
        if not ok:
            continue

        with latest_lock:
            latest_jpeg = buffer.tobytes()


# =========================
# STREAM
# =========================
def generate():
    while True:
        with latest_lock:
            jpeg = latest_jpeg

        if jpeg is None:
            # Worker has not produced a frame yet.
            time.sleep(0.01)
            continue

        yield b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
        time.sleep(0.01)


@app.route("/video")
def video():
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/")
def index():
    return "YOLO TensorRT Stream running /video"


if __name__ == "__main__":
    # Start the single CUDA-owning thread before serving requests.
    worker = threading.Thread(target=inference_worker, daemon=True)
    worker.start()
    app.run(host="0.0.0.0", port=5000, threaded=True)
