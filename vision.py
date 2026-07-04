import cv2
import os
import threading
import time
import numpy as np

class MataDrone:
    def __init__(self, camera_index=0, front_stream="udp://@127.0.0.1:5600", down_stream="udp://@127.0.0.1:5601"):
        # Initialize dual-camera setup
        self.front_stream = front_stream
        self.down_stream = down_stream
        self.cap_front = None
        self.cap_down = None
        self.is_running = False
        self.thread = None
        
        # Thread-safe State Variables
        self.active_camera = "DOWNWARD"
        self.current_mode = "IDLE"
        self.target_detected = False
        self.error_x = 0.0
        self.error_y = 0.0
        
        # Blind Spot Compensation (Tracking Memory)
        self.memory_timeout = 2.5  # seconds
        self.last_seen_time = 0.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0

    def start(self):
        self.is_running = True
        self.thread = threading.Thread(target=self._process_frames, daemon=True)
        self.thread.start()
        print("[+] Thread-Safe Vision Module: ONLINE.")

    def _process_frames(self):
        """ Background thread strictly for OpenCV tasks """
        print("[*] Vision Module: Opening UDP streams via GStreamer...")
        
        front_pipeline = "udpsrc port=5600 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink"
        down_pipeline = "udpsrc port=5601 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink"
        
        print(f"[*] Front Pipeline: {front_pipeline}")
        print(f"[*] Down Pipeline: {down_pipeline}")

        self.cap_front = cv2.VideoCapture()
        self.cap_front.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 2000)
        self.cap_front.open(front_pipeline, cv2.CAP_GSTREAMER)

        self.cap_down = cv2.VideoCapture()
        self.cap_down.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 2000)
        self.cap_down.open(down_pipeline, cv2.CAP_GSTREAMER)

        self.front_pipeline_str = front_pipeline
        self.down_pipeline_str = down_pipeline

        if not self.cap_front.isOpened():
            print("[-] WARNING: Front Camera UDP Port not ready yet. Will retry...")
        if not self.cap_down.isOpened():
            print("[-] WARNING: Downward Camera UDP Port not ready yet. Will retry...")
            
        print("[+] Vision Module: UDP Stream monitor started.")

        previous_mode = self.current_mode
        frame_count = 0
        fps_start_time = time.time()

        while self.is_running:
            # Explicitly select active camera stream controlled by main.py
            ret, frame = False, None
            if self.active_camera == "FRONT":
                if not self.cap_front.isOpened():
                    self.cap_front.open(self.front_pipeline_str, cv2.CAP_GSTREAMER)
                if self.cap_front.isOpened():
                    ret, frame = self.cap_front.read()
            elif self.active_camera == "DOWNWARD":
                if not self.cap_down.isOpened():
                    self.cap_down.open(self.down_pipeline_str, cv2.CAP_GSTREAMER)
                if self.cap_down.isOpened():
                    ret, frame = self.cap_down.read()
            
            if not ret or frame is None:
                time.sleep(0.5) # Wait a bit before retrying to prevent CPU spam
                continue

            raw_detected = False
            raw_err_x, raw_err_y = 0.0, 0.0
            
            height, width = frame.shape[:2]
            center_x, center_y = width // 2, height // 2
            mask = None

            if self.current_mode == "CARI_ARUCO":
                # Check multiple dictionaries to be robust against different simulator models
                dicts_to_try = [
                    cv2.aruco.DICT_5X5_250,
                    cv2.aruco.DICT_4X4_250,
                    cv2.aruco.DICT_ARUCO_ORIGINAL,
                    cv2.aruco.DICT_6X6_250
                ]
                aruco_params = getattr(cv2.aruco, 'DetectorParameters_create', cv2.aruco.DetectorParameters)()
                
                for dict_enum in dicts_to_try:
                    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_enum)
                    corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
                    
                    if ids is not None and len(ids) > 0:
                        corner = corners[0][0]
                        cx = int(np.mean(corner[:, 0]))
                        cy = int(np.mean(corner[:, 1]))
                        
                        raw_detected = True
                        raw_err_x = float(cx - center_x)
                        raw_err_y = float(center_y - cy)
                        w = np.max(corner[:, 0]) - np.min(corner[:, 0])
                        h = np.max(corner[:, 1]) - np.min(corner[:, 1])
                        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                        break  # Found it, stop trying other dictionaries
            elif self.current_mode == "ALIGN_DOUBLE_GATE":
                # Strict Orange/Yellow Gate Frame (12-30) to ignore the Red Box
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, np.array([12, 80, 80]), np.array([30, 255, 255]))
            elif self.current_mode == "CARI_BOX":
                # Red Dropzone Box (Combining 2 ranges for wrap-around hue)
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
                mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
                mask = cv2.bitwise_or(mask1, mask2)
            elif self.current_mode == "LINE_TRACKING":
                # Black Line
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 50]))

            if mask is not None:
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    largest_contour = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(largest_contour)

                    if area > 1000:
                        M = cv2.moments(largest_contour)
                        x, y, w, h = cv2.boundingRect(largest_contour)
                        
                        # Shape filter for Double Gate: The banner is a long horizontal line (width > height)
                        # If width is less than height, it's probably the ArUco pad or a glitch.
                        if self.current_mode == "ALIGN_DOUBLE_GATE" and w < (h * 1.5):
                            pass # Ignore it, not the gate banner
                        elif M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            raw_detected = True
                            raw_err_x = float(cx - center_x)
                            
                            # Offset Y for Double Gate so drone flies UNDER the banner
                            if self.current_mode == "ALIGN_DOUBLE_GATE":
                                target_y = 80  # Keep the banner at the top of the screen
                            else:
                                target_y = center_y
                                
                            raw_err_y = float(target_y - cy)

            # --- Memory / Blind Spot Compensation Logic ---
            current_time = time.time()
            
            if raw_detected:
                self.target_detected = True
                self.error_x = raw_err_x
                self.error_y = raw_err_y
                self.box_w = w
                self.box_h = h
                
                # Update Memory
                self.last_seen_time = current_time
                self.last_error_x = raw_err_x
                self.last_error_y = raw_err_y
            else:
                # If target lost, retain old trajectory if within timeout period
                if current_time - self.last_seen_time < self.memory_timeout:
                    self.target_detected = True  # Artificially keep tracking alive
                    self.error_x = self.last_error_x
                    self.error_y = self.last_error_y
                else:
                    self.target_detected = False
                    self.error_x, self.error_y = 0.0, 0.0

            # --- Debugging Pipeline ---
            # cv2.imshow("Original Downward", frame)
            # if mask is not None:
            #     cv2.imshow("HSV Mask", mask)
            # cv2.waitKey(1)
            
            frame_count += 1
            if frame_count % 30 == 0:
                elapsed = time.time() - fps_start_time
                fps = 30 / elapsed
                # Only print FPS periodically so it doesn't spam the console too badly
                print(f"[VISION] Processing FPS: {fps:.1f} | Mode: {self.current_mode}")
                fps_start_time = time.time()

            time.sleep(1 / 30)  # Throttle to ~30 FPS

    def stop(self):
        self.is_running = False
        if self.thread is not None:
            self.thread.join()
        if self.cap_front is not None and self.cap_front.isOpened():
            self.cap_front.release()
        if self.cap_down is not None and self.cap_down.isOpened():
            self.cap_down.release()
        print("[-] Vision Module: OFFLINE.")