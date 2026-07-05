import cv2
import socket
import json
import numpy as np
import time
import subprocess
import re
import threading
from ultralytics import YOLO

# CONFIGURATION
UDP_IP = "127.0.0.1"
MODEL_PATH = "/home/ambatron/DRONE/Krti_model.pt" # Menggunakan .pt dulu
CONFIDENCE_THRESHOLD = 0.25

# GStreamer Pipeline
FRONT_CAM_GST = 'udpsrc port=5600 ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink sync=false drop=true max-buffers=1'
DOWN_CAM_GST = 'udpsrc port=5601 ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink sync=false drop=true max-buffers=1'

# UDP CONFIG
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def start_vision_daemon():
    print(f"[VISION] Memulai MATS-15 Vision Daemon (DUAL CAMERA MODE)...")
    
    print(f"[VISION] Loading model: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)
    
    cap_front = cv2.VideoCapture(FRONT_CAM_GST, cv2.CAP_GSTREAMER)
    cap_down = cv2.VideoCapture(DOWN_CAM_GST, cv2.CAP_GSTREAMER)
    
    if not cap_front.isOpened() or not cap_down.isOpened():
        print("[VISION] ERROR: Kamera Gazebo tidak terdeteksi!")
        return

    print(f"[VISION] Mata terbuka. Mengirim data ke {UDP_IP}:{UDP_PORT}...")

    use_front = True # Toggle untuk alternating frames

    try:
        while True:
            # Alternating frames: Frame ganjil = Depan, Genap = Bawah (hemat CPU)
            if use_front:
                ret, frame = cap_front.read()
                cam_name = "front"
            else:
                ret, frame = cap_down.read()
                cam_name = "down"
                
            use_front = not use_front

            if not ret:
                time.sleep(0.05)
                continue
            
            height, width, _ = frame.shape
            frame_center_x = width // 2
            frame_center_y = height // 2

            results = model.predict(source=frame, conf=CONFIDENCE_THRESHOLD, verbose=False)
            
            target_data = {"status": "LOST", "class": "none", "error_x": 0, "error_y": 0, "area": 0, "camera": cam_name}

            if len(results) > 0 and len(results[0].boxes) > 0:
                boxes = results[0].boxes
                best_box = None
                max_area = -1
                
                for box in boxes:
                    bx1, by1, bx2, by2 = box.xyxy[0].cpu().numpy()
                    b_area = (bx2 - bx1) * (by2 - by1)
                    if b_area > max_area:
                        max_area = b_area
                        best_box = box
                
                x1, y1, x2, y2 = best_box.xyxy[0].cpu().numpy()
                cls_id = int(best_box.cls[0].item())
                class_name = model.names[cls_id]
                
                centroid_x = int((x1 + x2) / 2)
                centroid_y = int((y1 + y2) / 2)
                
                error_x = centroid_x - frame_center_x
                error_y = centroid_y - frame_center_y

                target_data = {
                    "status": "LOCKED",
                    "class": class_name,
                    "error_x": int(error_x),
                    "error_y": int(error_y),
                    "area": int(max_area),
                    "camera": cam_name
                }
                
            if target_data["status"] == "LOCKED":
                print(f"[DEBUG VISION] [{cam_name.upper()}] YOLO ngeliat: {target_data['class']} | X={target_data['error_x']}")
                
            message = json.dumps(target_data).encode('utf-8')
            sock.sendto(message, (UDP_IP, UDP_PORT))

    except KeyboardInterrupt:
        print("\n[VISION] Daemon dihentikan.")
    finally:
        cap_front.release()
        cap_down.release()
        sock.close()

def read_gazebo_lidar(topic, side):
    """
    Meniru sensor LiDAR fisik yang dicolok ke Raspberry Pi.
    Baca dari Gazebo, kirim lewat UDP ke Autopilot.
    """
    print(f"[VISION DAEMON] Memulai Lidar Bridge untuk {topic}...")
    try:
        process = subprocess.Popen(['gz', 'topic', '-e', '-t', topic], stdout=subprocess.PIPE, text=True)
        for line in process.stdout:
            match = re.search(r'ranges:\s*([\d\.]+)', line)
            if match:
                dist = float(match.group(1))
                if dist > 5.0: dist = 5.0
                data = {"camera": "lidar", "side": side, "range": dist}
                sock.sendto(json.dumps(data).encode('utf-8'), (UDP_IP, UDP_PORT))
    except Exception as e:
        print(f"[VISION DAEMON] Error LiDAR {topic}: {e}")

if __name__ == "__main__":
    # Start Lidar Threads (Background)
    threading.Thread(target=read_gazebo_lidar, args=('/lidar_left/scan', 'left'), daemon=True).start()
    threading.Thread(target=read_gazebo_lidar, args=('/lidar_right/scan', 'right'), daemon=True).start()
    
    # Start Vision Camera (Foreground)
    start_vision_daemon()
