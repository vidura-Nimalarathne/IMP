#!/usr/bin/env python
import os
os.chdir("/home/vidura/jetson-inference/data")

import rospy
from std_msgs.msg import Bool

import jetson_inference
import jetson_utils

DOG_ID = 18
TEDDY_BEAR_ID = 88
THRESHOLD = 0.5
REQUIRED_CONSECUTIVE_FRAMES = 3

def main():
    rospy.init_node("dog_detector_node")
    dog_pub = rospy.Publisher("/dog_detected", Bool, queue_size=1)

    net = jetson_inference.detectNet("ssd-mobilenet-v2", threshold=THRESHOLD)

    DEV = "/dev/video0"
    WIDTH = 1280
    HEIGHT = 720
    FPS = 30

    camera = jetson_utils.videoSource(
        f"v4l2://{DEV}",
        argv=[
            f"--input-width={WIDTH}",
            f"--input-height={HEIGHT}",
            f"--input-rate={FPS}",
            "--input-codec=mjpeg",
        ],
    )

    display = jetson_utils.videoOutput("display://0")

    detect_count = 0

    while not rospy.is_shutdown() and display.IsStreaming():
        img = camera.Capture()
        if img is None:
            continue

        detections = net.Detect(img)

        dog_present = False
        for d in detections:
            cid = int(d.ClassID)
            if cid in (DOG_ID, TEDDY_BEAR_ID) and d.Confidence >= THRESHOLD:
                dog_present = True
                print("[DETECT] {} {:.1f}%".format(net.GetClassDesc(cid), d.Confidence * 100.0))

        if dog_present:
            detect_count += 1
        else:
            detect_count = 0

        confirmed = detect_count >= REQUIRED_CONSECUTIVE_FRAMES
        dog_pub.publish(Bool(data=confirmed))

        display.Render(img)
        display.SetStatus(
            "Dog Detector | FPS: {:.1f} | detected: {} | confirmed: {}".format(
                net.GetNetworkFPS(), dog_present, confirmed
            )
        )

if __name__ == "__main__":
    main()
