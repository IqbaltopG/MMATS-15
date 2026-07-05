import subprocess
import re
import socket
import json
import threading
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def read_lidar(topic, side):
    print(f"[LIDAR BRIDGE] Starting Gazebo subscriber for {topic}...")
    try:
        process = subprocess.Popen(['gz', 'topic', '-e', '-t', topic], stdout=subprocess.PIPE, text=True)
        for line in process.stdout:
            match = re.search(r'ranges:\s*([\d\.]+)', line)
            if match:
                current_range = float(match.group(1))
                if current_range > 5.0: current_range = 5.0
                
                data = {"camera": "lidar", "side": side, "range": current_range}
                sock.sendto(json.dumps(data).encode('utf-8'), (UDP_IP, UDP_PORT))
    except Exception as e:
        print(f"[LIDAR BRIDGE] Error on {topic}: {e}")

if __name__ == "__main__":
    t1 = threading.Thread(target=read_lidar, args=('/lidar_left/scan', 'left'), daemon=True)
    t2 = threading.Thread(target=read_lidar, args=('/lidar_right/scan', 'right'), daemon=True)
    t1.start()
    t2.start()
    
    print("[LIDAR BRIDGE] Running in background. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("[LIDAR BRIDGE] Shutting down.")
