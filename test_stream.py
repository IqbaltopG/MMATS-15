import cv2
import time

print("Connecting to downward camera on 5601...")
cap = cv2.VideoCapture("udpsrc port=5601 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("FAILED TO OPEN PORT 5601!")
else:
    print("PORT OPENED! Waiting for frame...")
    start_time = time.time()
    while time.time() - start_time < 5.0:
        ret, frame = cap.read()
        if ret:
            print("FRAME RECEIVED! Saving to test_frame.jpg")
            cv2.imwrite("test_frame.jpg", frame)
            break
        time.sleep(0.1)
    else:
        print("TIMED OUT WAITING FOR FRAME!")
cap.release()
