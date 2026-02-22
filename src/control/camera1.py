import cv2
import threading
import time
from flask import Flask, Response

# -----------------------------
# Camera thread (IMPORTANT)
# -----------------------------
class Camera:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2)

        # Low-latency camera settings
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.frame = None
        self.running = True

        t = threading.Thread(target=self.update, daemon=True)
        t.start()

    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame
            else:
                time.sleep(0.01)

    def get_frame(self):
        return self.frame

# -----------------------------
# Flask MJPEG server
# -----------------------------
app = Flask(__name__)
camera = Camera(0)

def generate():
    while True:
        frame = camera.get_frame()
        if frame is None:
            continue

        # JPEG encode (lower quality = lower latency)
        ret, jpeg = cv2.imencode(
            '.jpg',
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, 50]
        )

        if not ret:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' +
               jpeg.tobytes() +
               b'\r\n')


@app.route('/video')
def video():
    return Response(
        generate(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


if __name__ == '__main__':
    print("MJPEG stream started on port 8080")
    app.run(host='0.0.0.0', port=8080, threaded=True)