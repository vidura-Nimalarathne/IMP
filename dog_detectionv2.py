#!/usr/bin/env python3
import os
import subprocess
import time

os.chdir("/home/vidura/jetson-inference/data")

import jetson_inference
import jetson_utils

DOG_ID = 18
TEDDY_BEAR_ID = 88
THRESHOLD = 0.5
REQUIRED_CONSECUTIVE_FRAMES = 3
TRIGGER_COOLDOWN = 5.0  # seconds

def publish_dog_detected():
    try:
        subprocess.Popen([
            "rostopic", "pub", "-1",
            "/dog_detected",
            "std_msgs/Bool",
            "data: true"
        ])
        print("[ROS] Published /dog_detected = True")
    except Exception as e:
        print("[ROS ERROR] Failed to publish /dog_detected: {}".format(e))

def main():
    net = jetson_inference.detectNet("ssd-mobilenet-v2", threshold=THRESHOLD)

    DEV = "/dev/video0"
    WIDTH = 1280
    HEIGHT = 720
    FPS = 30

    camera = jetson_utils.videoSource(
        "v4l2://{}".format(DEV),
        argv=[
            "--input-width={}".format(WIDTH),
            "--input-height={}".format(HEIGHT),
            "--input-rate={}".format(FPS),
            "--input-codec=mjpeg",
        ],
    )

    display = jetson_utils.videoOutput("display://0")

    detect_count = 0
    already_triggered = False
    last_trigger_time = 0.0

    while display.IsStreaming():
        img = camera.Capture()
        if img is None:
            continue

        detections = net.Detect(img)

        dog_present = False

        for d in detections:
            cid = int(d.ClassID)
            conf = float(d.Confidence)

            if cid in (DOG_ID, TEDDY_BEAR_ID) and conf >= THRESHOLD:
                dog_present = True
                print("[DETECT] {} {:.1f}%".format(
                    net.GetClassDesc(cid), conf * 100.0
                ))

        if dog_present:
            detect_count += 1
        else:
            detect_count = 0
            already_triggered = False

        confirmed = detect_count >= REQUIRED_CONSECUTIVE_FRAMES

        now = time.time()
        if confirmed and (not already_triggered) and ((now - last_trigger_time) > TRIGGER_COOLDOWN):
            publish_dog_detected()
            already_triggered = True
            last_trigger_time = now

        display.Render(img)
        display.SetStatus(
            "Dog Detector | FPS: {:.1f} | detected: {} | confirmed: {} | count: {}".format(
                net.GetNetworkFPS(), dog_present, confirmed, detect_count
            )
        )

if __name__ == "__main__":
    main()
