#!/usr/bin/env python
from __future__ import print_function

import cv2
import time
import threading
import numpy as np
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer

import rospy
from nav_msgs.msg import OccupancyGrid

HOST = "0.0.0.0"
PORT = 8080
FPS = 5
JPEG_QUALITY = 80

# Output display size for UI
DISPLAY_W = 800
DISPLAY_H = 800

# Extra border around explored map
CROP_MARGIN = 40

map_lock = threading.Lock()
latest_frame = None


def build_zoomed_map(msg):
    w = msg.info.width
    h = msg.info.height
    data = np.array(msg.data, dtype=np.int16).reshape((h, w))

    # Build grayscale occupancy image
    img = np.zeros((h, w), dtype=np.uint8)
    img[data == -1] = 205      # unknown = gray
    img[data == 0] = 255       # free = white
    img[data > 50] = 0         # occupied = black

    # Flip vertically so it looks natural
    img = cv2.flip(img, 0)

    # Find explored cells (not unknown)
    explored = (img != 205)

    if np.any(explored):
        ys, xs = np.where(explored)
        y1 = max(0, ys.min() - CROP_MARGIN)
        y2 = min(img.shape[0], ys.max() + CROP_MARGIN)
        x1 = max(0, xs.min() - CROP_MARGIN)
        x2 = min(img.shape[1], xs.max() + CROP_MARGIN)
        img = img[y1:y2, x1:x2]

    # Convert to BGR for text overlay
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # Resize to UI-friendly size while keeping aspect ratio
    h2, w2 = img.shape[:2]
    scale = min(float(DISPLAY_W) / w2, float(DISPLAY_H) / h2)
    new_w = max(1, int(w2 * scale))
    new_h = max(1, int(h2 * scale))
    img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_NEAREST)

    # Put resized map in center of fixed canvas
    canvas = np.full((DISPLAY_H, DISPLAY_W, 3), 205, dtype=np.uint8)
    xoff = (DISPLAY_W - new_w) // 2
    yoff = (DISPLAY_H - new_h) // 2
    canvas[yoff:yoff + new_h, xoff:xoff + new_w] = img

    cv2.putText(
        canvas,
        "LIVE MAP (AUTO-ZOOM)",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 0, 255),
        2
    )

    return canvas


def map_callback(msg):
    global latest_frame
    frame = build_zoomed_map(msg)

    with map_lock:
        latest_frame = frame


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
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
                with map_lock:
                    frame = None if latest_frame is None else latest_frame.copy()

                if frame is None:
                    frame = np.full((DISPLAY_H, DISPLAY_W, 3), 205, dtype=np.uint8)
                    cv2.putText(
                        frame,
                        "Waiting for /map ...",
                        (220, 400),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        (255, 255, 255),
                        2
                    )

                ok, jpg = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
                )

                if not ok:
                    time.sleep(0.1)
                    continue

                data = jpg.tobytes()

                self.wfile.write("--frame\r\n")
                self.wfile.write("Content-Type: image/jpeg\r\n")
                self.wfile.write("Content-Length: %d\r\n\r\n" % len(data))
                self.wfile.write(data)
                self.wfile.write("\r\n")

                time.sleep(1.0 / max(FPS, 1))

        except Exception:
            pass

    def log_message(self, format, *args):
        return


def server_thread():
    print("[MAP MJPEG] Serving on http://{}:{}/stream.mjpg".format(HOST, PORT))
    server = HTTPServer((HOST, PORT), MJPEGHandler)
    server.serve_forever()


def main():
    rospy.init_node("map_mjpeg_server", anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, map_callback, queue_size=1)
    print("[MAP MJPEG] Waiting for /map...")

    t = threading.Thread(target=server_thread)
    t.daemon = True
    t.start()

    rospy.spin()


if __name__ == "__main__":
    main()
