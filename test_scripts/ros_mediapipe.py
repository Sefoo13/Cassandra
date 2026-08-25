import tensorrt as trt
import pycuda.driver as cuda
import numpy as np
import cv2
import time


class TRT_YOLO:
    def __init__(self, engine_path="yolov5n.engine", input_shape=(1, 3, 640, 640)):
        self.engine_path = engine_path
        self.input_shape = input_shape

        # -------------------------
        # CUDA INIT (IMPORTANT FIX)
        # -------------------------
        cuda.init()
        self.device = cuda.Device(0)
        self.ctx = self.device.make_context()

        # -------------------------
        # TensorRT init
        # -------------------------
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)

        with open(engine_path, "rb") as f:
            engine_data = f.read()

        self.engine = self.runtime.deserialize_cuda_engine(engine_data)
        if self.engine is None:
            raise RuntimeError("Failed to load engine")

        self.context = self.engine.create_execution_context()

        # -------------------------
        # Allocate buffers
        # -------------------------
        self.inputs = []
        self.outputs = []
        self.bindings = []
        self.stream = cuda.Stream()

        for binding in self.engine:

            size = trt.volume(self.engine.get_binding_shape(binding))
            dtype = trt.nptype(self.engine.get_binding_dtype(binding))

            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)

            self.bindings.append(int(device_mem))

            if self.engine.binding_is_input(binding):
                self.input_host = host_mem
                self.input_device = device_mem
            else:
                self.output_host = host_mem
                self.output_device = device_mem

    def preprocess(self, img):
        img = cv2.resize(img, (self.input_shape[3], self.input_shape[2]))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)
        return img.ravel()

    def infer(self, img):
        # set input shape (IMPORTANT for dynamic engines)
        self.context.set_binding_shape(0, self.input_shape)

        np.copyto(self.input_host, self.preprocess(img))

        # transfer to GPU
        cuda.memcpy_htod_async(self.input_device, self.input_host, self.stream)

        # inference
        self.context.execute_async_v2(
            bindings=self.bindings,
            stream_handle=self.stream.handle
        )

        # transfer back
        cuda.memcpy_dtoh_async(self.output_host, self.output_device, self.stream)

        self.stream.synchronize()

        return self.output_host

    def __del__(self):
        try:
            self.ctx.pop()
        except:
            pass


def main():
    model = TRT_YOLO("yolov5n.engine")

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        output = model.infer(frame)

        print("output shape:", output.shape)

        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
