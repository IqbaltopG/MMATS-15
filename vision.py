import cv2
import numpy as np

class TargetTracker:
    def __init__(self):
        # TODO: Masukin nilai HSV terbaik dari hasil tuning lu
        self.lower_red = np.array([0, 100, 100]) 
        self.upper_red = np.array([10, 255, 255])
        
        # Kamera (0 = Webcam bawaan, ganti kalau pake input stream Gazebo)
        self.cap = cv2.VideoCapture(0)

    def scan_target(self):
        ret, frame = self.cap.read()
        if not ret:
            return False
            
        frame = cv2.resize(frame, (320, 240))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
        
        # Hitung area putih (pixel merah yang terdeteksi)
        white_pixels = cv2.countNonZero(mask)
        
        # Kalau pixel merahnya banyak, berarti ketemu
        if white_pixels > 500: 
            return True
        return False
        
    def release_camera(self):
        self.cap.release()