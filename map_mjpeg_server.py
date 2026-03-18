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

map_lock = threading.Lock()
latest_frame = None


def map_callback(msg):
    global latest_frame

    w = msg.info.width
    h = msg.info.height

    data = np.array(msg.data, dtype=np.int16).reshape((h, w))

    img = np.zeros((h, w), dtype=np.uint8)
    img[data == -1] = 205
    img[data == 0] = 255
    img[data > 50] = 0

    img = cv2.flip(img, 0)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    cv2.putText(
        img,
        "LIVE MAP",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 0, 255),
        2
    )

    with map_lock:
        latest_frame = img


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
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(
                        frame,
                        "Waiting for /map ...",
                        (120, 240),
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
