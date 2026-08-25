from flask import Flask, Response
import cv2
import torch
import time

app = Flask(__name__)

# YOLOv5 MODEL (local .pt)
model = torch.hub.load("", "custom", path="yolov5n.pt", source="local")

model.conf = 0.4  # confidence threshold

# CAMERA
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

prev_time = 0


def generate():
    global prev_time

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # YOLO INFERENCE
        results = model(frame)
        frame = results.render()[0]

        # FPS CALCULATION
        current_time = time.time()

        if prev_time == 0:
            fps = 0
        else:
            fps = 1 / (current_time - prev_time)

        prev_time = current_time

        
        # DRAW FPS ON FRAME
        
        cv2.putText(
            frame,
            f"FPS: {int(fps)}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )

        # ENCODE FRAME
        _, buffer = cv2.imencode(".jpg", frame)
        frame = buffer.tobytes()

        yield b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"


@app.route("/")
def index():
    return "YOLOv5 Stream running. Open /video"


@app.route("/video")
def video():
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)
