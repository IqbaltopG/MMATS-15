import cv2
import os
import time
import argparse
import signal
import sys

running = True

def signal_handler(sig, frame):
    global running
    print("\n[!] Ctrl+C detected. Gracefully stopping Data Miner...")
    running = False

def main():
    global running
    signal.signal(signal.SIGINT, signal_handler)

    parser = argparse.ArgumentParser(description="Extract frames from Gazebo UDP streams for SITL YOLO dataset")
    parser.add_argument('--interval', type=float, default=0.5, help='Seconds between frame captures (default: 0.5)')
    parser.add_argument('--out_dir', type=str, default='dataset/raw_frames', help='Output directory for images')
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    # Adding drop-on-latency to pipeline so it doesn't hang forever
    front_cam_gst = 'udpsrc port=5600 ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink sync=false drop=true max-buffers=1'
    down_cam_gst = 'udpsrc port=5601 ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink sync=false drop=true max-buffers=1'

    print("Connecting to Front Camera (UDP 5600) and Down Camera (UDP 5601)...")
    cap_front = cv2.VideoCapture(front_cam_gst, cv2.CAP_GSTREAMER)
    cap_down = cv2.VideoCapture(down_cam_gst, cv2.CAP_GSTREAMER)

    print(f"Starting Data Miner... Saving frames every {args.interval} seconds to {args.out_dir}/")
    print("Press Ctrl+C to stop.")

    last_time = time.time()
    frames_saved = 0

    while running:
        # Pumping OpenCV events allows it to catch interrupts in headless mode
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        ret_f, frame_front = cap_front.read()
        ret_d, frame_down = cap_down.read()

        current_time = time.time()
        if current_time - last_time >= args.interval:
            timestamp = int(current_time * 1000)
            
            saved_any = False
            if ret_f and frame_front is not None:
                cv2.imwrite(os.path.join(args.out_dir, f"front_{timestamp}.jpg"), frame_front)
                saved_any = True
            
            if ret_d and frame_down is not None:
                cv2.imwrite(os.path.join(args.out_dir, f"down_{timestamp}.jpg"), frame_down)
                saved_any = True
            
            if saved_any:
                frames_saved += 1
                if frames_saved % 10 == 0:
                    print(f"Status: {frames_saved} frames saved so far...")
            
            last_time = current_time

    cap_front.release()
    cap_down.release()
    print(f"Total frames saved: {frames_saved}")

if __name__ == '__main__':
    main()
