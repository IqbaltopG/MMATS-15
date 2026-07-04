import cv2
import numpy as np
import subprocess
import shlex

class TargetTracker:
    def __init__(self, debug_mode=True):
        self.debug_mode = debug_mode
        
        # Rentang HSV untuk warna merah, disesuaikan untuk kondisi pencahayaan Gazebo.
        self.lower_red_1 = np.array([0, 50, 30]) 
        self.upper_red_1 = np.array([15, 255, 255])
        self.lower_red_2 = np.array([155, 50, 30])
        self.upper_red_2 = np.array([180, 255, 255])

        print("[*] Menjalankan FFmpeg untuk menangkap stream video...")
        
        # Perintah FFmpeg untuk menangkap stream UDP dari Gazebo.
        # - loglevel error: Hanya tampilkan error kritis.
        # - overrun_nonfatal=1: Mencegah FFmpeg berhenti jika buffer penuh.
        # - fifo_size: Menaikkan ukuran buffer input UDP.
        cmd = (
            "ffmpeg -loglevel error "
            "-i udp://127.0.0.1:5600?overrun_nonfatal=1&fifo_size=500000 "
            "-f image2pipe -pix_fmt bgr24 -vcodec rawvideo -s 320x240 -an -sn -"
        )
        
        # Eksekusi FFmpeg sebagai proses background
        self.pipe = subprocess.Popen(
            shlex.split(cmd), 
            stdout=subprocess.PIPE, 
            bufsize=10**8
        )
        
        print("[+] Modul vision online (via FFmpeg)!")

    def scan_target(self):
        # Membaca frame mentah dari pipe FFmpeg.
        raw_image = self.pipe.stdout.read(320 * 240 * 3)
        
        if not raw_image or len(raw_image) != (320 * 240 * 3):
            print("[-] Peringatan: Frame video kosong. Cek stream dari Gazebo di port UDP 5600.")
            return False
            
        frame = np.frombuffer(raw_image, dtype='uint8').reshape((240, 320, 3))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Membuat mask gabungan untuk dua rentang HSV merah.
        mask1 = cv2.inRange(hsv, self.lower_red_1, self.upper_red_1)
        mask2 = cv2.inRange(hsv, self.lower_red_2, self.upper_red_2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Menghitung jumlah piksel yang cocok dengan warna merah.
        white_pixels = cv2.countNonZero(mask)
        
        if self.debug_mode:
            cv2.imshow("Mata Drone (Raw)", frame)
            cv2.imshow("Masking (Putih = Merah)", mask)
            cv2.waitKey(1)
        
        if white_pixels > 0:
            print(f"[*] Piksel merah terdeteksi: {white_pixels}")

        # Threshold untuk menganggap target "terkunci".
        if white_pixels > 200:
            return True
        return False
        
    def release_camera(self):
        print("[*] Menghentikan proses FFmpeg...")
        self.pipe.terminate()
        if self.debug_mode:
            cv2.destroyAllWindows()