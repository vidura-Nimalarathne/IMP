import os
os.chdir("/home/vidura/jetson-inference/data")

import jetson_inference
import jetson_utils

DOG_ID = 18
TEDDY_BEAR_ID = 88
THRESHOLD = 0.5

def main():
    net = jetson_inference.detectNet("ssd-mobilenet-v2", threshold=THRESHOLD)

    DEV = "/dev/video0"
    WIDTH = 1280
    HEIGHT = 720
    FPS = 30

    GST_PIPELINE = (
        f"v4l2src device={DEV} ! "
        f"image/jpeg, width={WIDTH}, height={HEIGHT}, framerate={FPS}/1 ! "
        f"jpegdec ! videoconvert ! video/x-raw,format=BGRx ! "
        f"queue ! appsink drop=true max-buffers=1 sync=false"
    )

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

    while display.IsStreaming():
        img = camera.Capture()
        if img is None:
            continue

        detections = net.Detect(img)

        dog_present = False
        for d in detections:
            cid = int(d.ClassID)
            if cid in (DOG_ID, TEDDY_BEAR_ID) and d.Confidence >= THRESHOLD:
                dog_present = True
                print(f"[DETECT] {net.GetClassDesc(cid)} {d.Confidence*100:.1f}%")

        display.Render(img)
        display.SetStatus(
            f"Dog Proxy Detector | FPS: {net.GetNetworkFPS():.1f} | DOG_PRESENT: {dog_present}"
        )

if __name__ == "__main__":
    main()
