#!/usr/bin/env python3
import os
import cv2
import time
import threading
import subprocess
import tempfile

from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from urllib.parse import urlparse

# ----------------------------
# ENV / PATH
# ----------------------------
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"
os.chdir("/home/vidura/jetson-inference/data")

import jetson_inference
import jetson_utils

# ----------------------------
# SERVER SETTINGS
# ----------------------------
HOST = "0.0.0.0"
PORT = 8080
DEVICE = "/dev/video0"

# ----------------------------
# VIDEO SETTINGS
# ----------------------------
WIDTH = 480
HEIGHT = 360
FPS = 15
JPEG_QUALITY = 70

# Helps reduce latency on Jetson/OpenCV
FLUSH_OLD_FRAMES = True
FLUSH_GRABS = 4

# ----------------------------
# AI SETTINGS
# ----------------------------
DOG_ID = 18
TEDDY_BEAR_ID = 88
THRESHOLD = 0.35
REQUIRED_CONSECUTIVE_FRAMES = 2
TRIGGER_COOLDOWN = 5.0
DETECT_EVERY_N_FRAMES = 2

# temp image for AI workaround
AI_TMP_PATH = "/dev/shm/petbot_ai_frame.jpg"
AI_TMP_JPEG_QUALITY = 85

# ----------------------------
# GLOBALS
# ----------------------------
cap = None
net = None

frame_lock = threading.Lock()
latest_jpg = None
latest_frame_time = 0.0

detect_enabled = False
detect_count = 0
already_triggered = False
last_trigger_time = 0.0

frame_counter = 0
last_target_present = False
last_confirmed = False
last_detect_label = "none"
last_ai_error = ""
ai_ready = False

# keep last detections so overlay persists between detect cycles
last_all_detections = []

# ----------------------------
# HTTP SERVER
# ----------------------------
class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


# ----------------------------
# ROS
# ----------------------------
def publish_dog_detected():
    try:
        subprocess.Popen(
            [
                "rostopic", "pub", "-1",
                "/dog_detected",
                "std_msgs/Bool",
                "data: true"
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        print("[ROS] Published /dog_detected = True")
    except Exception as e:
        print("[ROS ERROR] Failed to publish /dog_detected = True: {}".format(e))


# ----------------------------
# CAMERA
# ----------------------------
def open_camera():
    global cap

    cap = cv2.VideoCapture(DEVICE)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera {}".format(DEVICE))

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    print("[CAM] Opened {} ({}x{} @ {} FPS)".format(DEVICE, WIDTH, HEIGHT, FPS))


def get_latest_camera_frame():
    if FLUSH_OLD_FRAMES:
        for _ in range(FLUSH_GRABS):
            cap.grab()
        ok, frame = cap.retrieve()
        return ok, frame
    return cap.read()


# ----------------------------
# AI INIT
# ----------------------------
def init_ai():
    global net, ai_ready, last_ai_error

    try:
        net = jetson_inference.detectNet("ssd-mobilenet-v2", threshold=THRESHOLD)
        ai_ready = True
        last_ai_error = ""
        print("[AI] detectNet loaded successfully")
    except Exception as e:
        net = None
        ai_ready = False
        last_ai_error = str(e)
        print("[AI ERROR] Failed to load detectNet: {}".format(e))


def set_detection(enabled):
    global detect_enabled, detect_count, already_triggered, last_trigger_time
    global frame_counter, last_target_present, last_confirmed
    global last_detect_label, last_all_detections

    detect_enabled = enabled
    detect_count = 0
    already_triggered = False
    last_trigger_time = 0.0
    frame_counter = 0
    last_target_present = False
    last_confirmed = False
    last_detect_label = "none"
    last_all_detections = []

    print("[AI] Detection {}".format("ENABLED" if enabled else "DISABLED"))


# ----------------------------
# DRAWING
# ----------------------------
def draw_box_and_label(frame, x1, y1, x2, y2, label, color):
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    cv2.putText(
        frame,
        label,
        (x1, max(20, y1 - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        color,
        2,
        cv2.LINE_AA
    )


# ----------------------------
# AI DETECTION WORKAROUND
# ----------------------------
def run_ai_on_frame_via_tempfile(frame_bgr):
    """
    Workaround for Jetson builds without cudaFromNumpy():
    1. save OpenCV frame to /dev/shm
    2. load with jetson_utils.loadImage()
    3. run detectNet on that image
    """
    global last_ai_error

    try:
        ok = cv2.imwrite(
            AI_TMP_PATH,
            frame_bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), AI_TMP_JPEG_QUALITY]
        )
        if not ok:
            raise RuntimeError("cv2.imwrite failed for AI temp frame")

        cuda_img = jetson_utils.loadImage(AI_TMP_PATH)
        if cuda_img is None:
            raise RuntimeError("jetson_utils.loadImage returned None")

        detections = net.Detect(cuda_img, overlay="none")
        last_ai_error = ""
        return detections

    except Exception as e:
        last_ai_error = str(e)
        raise


# ----------------------------
# FRAME PROCESSING
# ----------------------------
def process_frame(frame_bgr):
    global detect_count, already_triggered, last_trigger_time
    global frame_counter, last_target_present, last_confirmed
    global last_detect_label, last_all_detections, last_ai_error

    out = frame_bgr.copy()

    # Teleop view
    if not detect_enabled:
        cv2.putText(
            out,
            "AI MODE: OFF",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA
        )
        cv2.putText(
            out,
            "TELEOP VIEW",
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
            cv2.LINE_AA
        )
        return out

    # AI ON
    cv2.putText(
        out,
        "AI MODE: ON",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
        cv2.LINE_AA
    )
    cv2.putText(
        out,
        "AUTONOMOUS DETECTION ACTIVE",
        (10, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 0),
        2,
        cv2.LINE_AA
    )

    if not ai_ready or net is None:
        cv2.putText(
            out,
            "AI NOT READY",
            (10, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2,
            cv2.LINE_AA
        )
        return out

    frame_counter += 1
    run_detection = (frame_counter % DETECT_EVERY_N_FRAMES == 0)

    if run_detection:
        try:
            detections = run_ai_on_frame_via_tempfile(frame_bgr)

            target_present = False
            current_target_label = "none"
            all_boxes = []

            print("[AI] detection count = {}".format(len(detections)))

            for d in detections:
                cid = int(d.ClassID)
                conf = float(d.Confidence)
                class_name = net.GetClassDesc(cid)

                x1 = int(d.Left)
                y1 = int(d.Top)
                x2 = int(d.Right)
                y2 = int(d.Bottom)

                is_target = (cid in (DOG_ID, TEDDY_BEAR_ID)) and (conf >= THRESHOLD)
                label = "{} {:.0f}%".format(class_name, conf * 100.0)

                all_boxes.append((x1, y1, x2, y2, label, is_target))

                print("[AI] class={} id={} conf={:.2f}".format(class_name, cid, conf))

                if is_target:
                    target_present = True
                    current_target_label = label
                    print("[TARGET] {} {:.1f}%".format(class_name, conf * 100.0))

            last_all_detections = all_boxes
            last_target_present = target_present
            last_detect_label = current_target_label

            if target_present:
                detect_count += 1
            else:
                detect_count = 0
                already_triggered = False

            confirmed = detect_count >= REQUIRED_CONSECUTIVE_FRAMES
            last_confirmed = confirmed

            now = time.time()
            if confirmed and (not already_triggered) and ((now - last_trigger_time) > TRIGGER_COOLDOWN):
                publish_dog_detected()
                already_triggered = True
                last_trigger_time = now

        except Exception as e:
            print("[AI ERROR] {}".format(e))

    # draw last known detections every frame
    for box in last_all_detections:
        x1, y1, x2, y2, label, is_target = box
        color = (0, 255, 0) if is_target else (0, 0, 255)
        draw_box_and_label(out, x1, y1, x2, y2, label, color)

    cv2.putText(
        out,
        "target={} present={} confirmed={} count={}".format(
            last_detect_label, last_target_present, last_confirmed, detect_count
        ),
        (10, 100),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (255, 255, 255),
        1,
        cv2.LINE_AA
    )

    cv2.putText(
        out,
        "green=dog/teddy  red=other classes",
        (10, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        (255, 255, 255),
        1,
        cv2.LINE_AA
    )

    if last_ai_error:
        cv2.putText(
            out,
            "AI ERR: {}".format(last_ai_error[:55]),
            (10, 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (0, 0, 255),
            1,
            cv2.LINE_AA
        )

    return out


# ----------------------------
# CAPTURE LOOP
# ----------------------------
def capture_loop():
    global latest_jpg, latest_frame_time

    while True:
        ok, frame = get_latest_camera_frame()

        if not ok or frame is None:
            time.sleep(0.005)
            continue

        processed = process_frame(frame)

        ok, jpg = cv2.imencode(
            ".jpg",
            processed,
            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        )
        if not ok:
            continue

        with frame_lock:
            latest_jpg = jpg.tobytes()
            latest_frame_time = time.time()


# ----------------------------
# HTTP HANDLER
# ----------------------------
class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path in ("/", "/stream.mjpg"):
            self.handle_stream()
        elif path == "/detect_on":
            set_detection(True)
            self.send_text("DETECTION ON\n")
        elif path == "/detect_off":
            set_detection(False)
            self.send_text("DETECTION OFF\n")
        elif path == "/status":
            age = time.time() - latest_frame_time if latest_frame_time else -1
            status = (
                "detect_enabled={}\n"
                "ai_ready={}\n"
                "last_ai_error={}\n"
                "frame_age_sec={:.3f}\n"
                "resolution={}x{}\n"
                "fps={}\n"
                "threshold={}\n"
                "detect_every_n_frames={}\n"
                "last_detect_label={}\n"
                "target_present={}\n"
                "confirmed={}\n"
            ).format(
                detect_enabled,
                ai_ready,
                last_ai_error,
                age,
                WIDTH,
                HEIGHT,
                FPS,
                THRESHOLD,
                DETECT_EVERY_N_FRAMES,
                last_detect_label,
                last_target_present,
                last_confirmed
            )
            self.send_text(status)
        else:
            self.send_error(404)

    def handle_stream(self):
        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        last_sent = 0.0
        frame_interval = 1.0 / max(FPS, 1)

        try:
            while True:
                with frame_lock:
                    data = latest_jpg

                if data is None:
                    time.sleep(0.01)
                    continue

                now = time.time()
                if (now - last_sent) < frame_interval:
                    time.sleep(0.001)
                    continue

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(("Content-Length: %d\r\n\r\n" % len(data)).encode("utf-8"))
                self.wfile.write(data)
                self.wfile.write(b"\r\n")

                last_sent = now

        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception:
            pass

    def send_text(self, text):
        data = text.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, format, *args):
        return


# ----------------------------
# MAIN
# ----------------------------
def main():
    # Ensure shared-memory path exists
    shm_dir = os.path.dirname(AI_TMP_PATH)
    if not os.path.isdir(shm_dir):
        raise RuntimeError("/dev/shm not available")

    open_camera()
    init_ai()
    set_detection(False)

    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()

    print("[MJPEG] Stream:  http://{}:{}/stream.mjpg".format(HOST, PORT))
    print("[CTRL ] ON:     http://{}:{}/detect_on".format(HOST, PORT))
    print("[CTRL ] OFF:    http://{}:{}/detect_off".format(HOST, PORT))
    print("[STAT ] Status: http://{}:{}/status".format(HOST, PORT))

    server = ThreadedHTTPServer((HOST, PORT), MJPEGHandler)

    try:
        server.serve_forever()
    finally:
        if cap is not None:
            cap.release()
        try:
            if os.path.exists(AI_TMP_PATH):
                os.remove(AI_TMP_PATH)
        except Exception:
            pass


if __name__ == "__main__":
    main()
