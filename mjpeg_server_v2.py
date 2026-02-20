#!/usr/bin/env python3
import os
import cv2
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

# Reduce OpenCV logging noise
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

HOST = "0.0.0.0"
PORT = 8080

DEVICE = "/dev/video0"
WIDTH = 640
HEIGHT = 480
FPS = 20
JPEG_QUALITY = 80


# Open and configure camera
cap = cv2.VideoCapture(DEVICE)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

if not cap.isOpened():
    raise RuntimeError(f"Could not open camera {DEVICE}")

print(f"[MJPEG] Camera opened ({WIDTH}x{HEIGHT} @ {FPS} FPS)")


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        # Serve stream from /stream.mjpg (and optionally /)
        if self.path not in ("/", "/stream.mjpg"):
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        try:
            while True:
                ok, frame = cap.read()
                if not ok or frame is None:
                    time.sleep(0.02)
                    continue

                ok, jpg = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
                )
                if not ok:
                    continue

                data = jpg.tobytes()

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode("utf-8"))
                self.wfile.write(data)
                self.wfile.write(b"\r\n")

                time.sleep(1.0 / max(FPS, 1))

        except (BrokenPipeError, ConnectionResetError):
            # Client disconnected
            pass
        except Exception:
            # Keep server alive on unexpected client errors
            pass

    def log_message(self, format, *args):
        # Stop spammy HTTP logs
        return


def main():
    url = f"http://{HOST}:{PORT}/stream.mjpg"
    print(f"[MJPEG] Serving on {url}")
    server = HTTPServer((HOST, PORT), MJPEGHandler)
    try:
        server.serve_forever()
    finally:
        cap.release()


if __name__ == "__main__":
    main()
