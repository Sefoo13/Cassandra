from flask import Flask, Response
import cv2
from fer import FER
import time

app = Flask(__name__)
cap = cv2.VideoCapture(0)

# FER
emotion_detector = FER(mtcnn=False)


def generate_frames():
    prev_time = time.time()

    while True:
        success, frame = cap.read()
        if not success:
            continue

        # FER
        results = emotion_detector.detect_emotions(frame)

        for face in results:
            x, y, w, h = face["box"]

            emotions = face["emotions"]

            emotion = max(emotions, key=emotions.get)
            confidence = emotions[emotion]

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cv2.putText(
                frame,
                f"{emotion} {confidence:.2f}",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

        # FPS
        current_time = time.time()
        fps = 1.0 / (current_time - prev_time)
        prev_time = current_time

        cv2.putText(
            frame,
            f"FPS: {int(fps)}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 0, 0),
            2,
        )

        # JPEG
        ret, buffer = cv2.imencode(".jpg", frame)
        frame_bytes = buffer.tobytes()

        yield (
            b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
        )


@app.route("/")
def index():
    return """
    <html>
      <body>
        <h1>FER Stream</h1>
        <img src="/video_feed">
      </body>
    </html>
    """


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)
