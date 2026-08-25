from flask import Flask, Response
import cv2
import mediapipe as mp

app = Flask(__name__)

cap = cv2.VideoCapture(0)

mp_face = mp.solutions.face_mesh
face_mesh = mp_face.FaceMesh(
    max_num_faces=1, refine_landmarks=True, min_detection_confidence=0.5
)


def gen():
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = face_mesh.process(rgb)

        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                for lm in face_landmarks.landmark:
                    x = int(lm.x * w)
                    y = int(lm.y * h)

                    cv2.circle(frame, (x, y), 1, (0, 255, 0), -1)

        ret, buffer = cv2.imencode(".jpg", frame)
        frame = buffer.tobytes()

        yield b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"


@app.route("/video")
def video():
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


app.run(host="0.0.0.0", port=5000)
