import asyncio
import json
import math
from mavsdk import System
import flight

# GLOBAL STATE DARI UDP
TARGET_STATE_FRONT = {"status": "LOST", "class": "none", "error_x": 0, "error_y": 0, "area": 0}
TARGET_STATE_DOWN = {"status": "LOST", "class": "none", "error_x": 0, "error_y": 0, "area": 0}

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

class UDPReceiverProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data, addr):
        global TARGET_STATE_FRONT, TARGET_STATE_DOWN
        try:
            message = data.decode('utf-8')
            parsed = json.loads(message)
            if parsed.get("camera") == "down":
                TARGET_STATE_DOWN.update(parsed)
            else:
                TARGET_STATE_FRONT.update(parsed)
        except Exception as e:
            pass

async def start_udp_server():
    loop = asyncio.get_running_loop()
    transport, protocol = await loop.create_datagram_endpoint(
        lambda: UDPReceiverProtocol(),
        local_addr=(UDP_IP, UDP_PORT)
    )
    return transport

async def run_mission():
    drone = System()
    print("[AUTOPILOT] Menyambung ke Drone (SITL)...")
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[AUTOPILOT] Drone Terkoneksi!")
            break

    print("[AUTOPILOT] Memulai Smart Takeoff...")
    await flight.arm_and_takeoff(drone, altitude_m=1.5)

    print("[AUTOPILOT] Hovering di 1.5m selama 5 detik untuk Stabilisasi...")
    await flight.send_body_velocity(drone, 0.0, 0.0, 0.0, 0.0)
    await asyncio.sleep(5)

    # STATE MACHINE VARIABLES
    state_phase = "CENTERING_GATE_1" # Langsung tembak lurus nyari Gate 1!
    timeout_counter = 0
    has_seen_target = False
    last_front_err_y = 0
    last_front_area = 0
    
    # PID Constants
    kp_yaw = 0.005
    kp_up = 0.005

    while True:
        front_status = TARGET_STATE_FRONT.get("status", "LOST")
        front_class = TARGET_STATE_FRONT.get("class", "none")
        front_err_x = TARGET_STATE_FRONT.get("error_x", 0)
        front_err_y = TARGET_STATE_FRONT.get("error_y", 0)

        front_area = TARGET_STATE_FRONT.get("area", 0)
        front_confident = TARGET_STATE_FRONT.get("confident", TARGET_STATE_FRONT.get("confidence", 0.0))

        down_status = TARGET_STATE_DOWN.get("status", "LOST")
        down_confident = TARGET_STATE_DOWN.get("confident", TARGET_STATE_DOWN.get("confidence", 0.0))
        down_class = TARGET_STATE_DOWN.get("class", "none")
        down_err_x = TARGET_STATE_DOWN.get("error_x", 0)
        down_err_y = TARGET_STATE_DOWN.get("error_y", 0)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        if state_phase == "CENTERING_GATE_1":
            if front_status == "LOCKED" and front_class == "Single Gate":
                has_seen_target = True
                timeout_counter = 0
                last_front_area = front_area
                last_front_err_y = front_err_y
                yaw_cmd = front_err_x * kp_yaw
                # ALGORITMA NENGAHIN DULU: Cuma cek Kiri/Kanan, abaikan Naik/Turun karena Pitch bikin gambar goyang!
                if abs(front_err_x) > 40:
                    fwd_cmd = 0.0
                    up_cmd = (front_err_y + 150) * kp_up
                else:
                    fwd_cmd = 0.8
                    up_cmd = 0.0
                
                print(f"[AUTOPILOT] [GATE 1] Fwd: {fwd_cmd}, Strafe X: {yaw_cmd:.2f}, Z: {up_cmd:.2f}")
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=yaw_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
            else:
                if has_seen_target:
                    if last_front_err_y < 20 and last_front_area > 8000:
                        timeout_counter += 1
                    else:
                        print(f"[AUTOPILOT] Gawang 1 hilang/flicker (Area: {last_front_area}). Hovering...")
                        timeout_counter = 0
                
                if timeout_counter == 0:
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                elif timeout_counter < 150:
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                elif timeout_counter < 180:
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=-0.6, yaw_deg_s=0.0)
                else:
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                
                if timeout_counter > 200: # 3 detik blind forward untuk nembus
                    print("[AUTOPILOT] Gate 1 terlewati! Beralih nyari Aruco 1...")
                    state_phase = "FIND_ARUCO_1"
                    timeout_counter = 0
                    has_seen_target = False


        # ---------------------------------------------------------
        # PHASE 3: FIND_ARUCO_1 (Mencari WP1 / Pad kuning)
        # ---------------------------------------------------------
        elif state_phase == "FIND_ARUCO_1":
            if down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area"]:
                print("[AUTOPILOT] Aruco 1 (Marker/Area) Terlihat di Kamera Bawah! Memulai Precision Centering...")
                state_phase = "CENTER_ARUCO_1"
                timeout_counter = 0
            elif front_status == "LOCKED" and front_class == "Aruco Area":
                yaw_cmd = front_err_x * kp_yaw
                await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=yaw_cmd)
            else:
                await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)

        # ---------------------------------------------------------
        # PHASE 3B: CENTER_ARUCO_1 (Precision Hover)
        # ---------------------------------------------------------
        elif state_phase == "CENTER_ARUCO_1":
            if down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area"]:
                fwd_cmd = -down_err_y * kp_yaw
                strafe_cmd = down_err_x * kp_yaw
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=0.0, yaw_deg_s=0.0)
                
                if abs(down_err_x) < 80 and abs(down_err_y) < 80:
                    timeout_counter += 1
                    if timeout_counter > 5: # Stabil di tengah selama 0.5 detik
                        print("[AUTOPILOT] Presisi WP1 Tercapai! Mengikuti Straight Line menuju WP2...")
                        state_phase = "FOLLOW_LINE_TO_WP2"
                        timeout_counter = 0
                        has_seen_target = False
                else:
                    timeout_counter = 0
            else:
                await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)

        # ---------------------------------------------------------
        # PHASE 4: FOLLOW_LINE_TO_WP2 (Murni Ngikutin Garis)
        # ---------------------------------------------------------
        elif state_phase == "FOLLOW_LINE_TO_WP2":
            fwd_cmd = 1.0
            yaw_cmd = 0.0
            strafe_cmd = 0.0
            
            if front_status == "LOCKED" and front_class == "Straight Line":
                yaw_cmd = front_err_x * kp_yaw
                
            if down_status == "LOCKED" and down_class == "Straight Line":
                strafe_cmd = down_err_x * kp_yaw
                timeout_counter = 0 # Reset timeout kalau masih liat garis
                has_seen_target = True # Tandai bahwa kita udah berhasil nangkep garis
            else:
                if has_seen_target: # Cuma ngitung timeout hilang JIKA sebelumnya udah dapet garis
                    timeout_counter += 1
                
            if timeout_counter > 100: # Garis hilang selama ~10 detik nyata (RTF 40% = ~4 detik sim)
                print("[AUTOPILOT] Ujung Garis WP2 tercapai! Beralih mencari Aruco 2...")
                state_phase = "FIND_ARUCO_2"
                timeout_counter = 0
                has_seen_target = False
            
            await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=0.0, yaw_deg_s=yaw_cmd)

        # ---------------------------------------------------------
        # PHASE 4A: FIND_ARUCO_2 (Mencari Aruco setelah garis habis)
        # ---------------------------------------------------------
        elif state_phase == "FIND_ARUCO_2":
            if down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area"]:
                print("[AUTOPILOT] WP2 (Aruco 2) Terlihat! Memulai Centering...")
                state_phase = "CENTER_ARUCO_2"
                timeout_counter = 0
            elif front_status == "LOCKED" and front_class == "Aruco Area":
                yaw_cmd = front_err_x * kp_yaw
                await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=yaw_cmd)
            else:
                await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)

        # ---------------------------------------------------------
        # PHASE 4B: CENTER_ARUCO_2 (Precision Hover)
        # ---------------------------------------------------------
        elif state_phase == "CENTER_ARUCO_2":
            if down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area"]:
                fwd_cmd = -down_err_y * kp_yaw
                strafe_cmd = down_err_x * kp_yaw
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=0.0, yaw_deg_s=0.0)
                
                if abs(down_err_x) < 80 and abs(down_err_y) < 80:
                    timeout_counter += 1
                    if timeout_counter > 5:
                        print("[AUTOPILOT] Presisi WP2 Tercapai! Mutar kiri nyari Triple Gate 1...")
                        state_phase = "YAW_LEFT_TRIPLE_1"
                        timeout_counter = 0
                        has_seen_target = False

                else:
                    timeout_counter = 0
            else:
                await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)

        # ---------------------------------------------------------
        # PHASE 4C: YAW_LEFT_TRIPLE_1 (Belok Kiri Nyari Triple Gate)
        # ---------------------------------------------------------
        elif state_phase == "YAW_LEFT_TRIPLE_1":
            timeout_counter += 1
            await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=-15.0)
            if front_status == "LOCKED" and front_class == "Tripple Gate" and front_confident > 0.5:
                print("[AUTOPILOT] Triple Gate 1 Terkunci! Meluncur maju...")
                state_phase = "FIND_TRIPLE_GATE_1"
                timeout_counter = 0
            elif timeout_counter > 150: # RTF 40% x 15s = 6s sim. 6s x 15 deg/s = MENTOK 90 DERAJAT!
                print("[AUTOPILOT] Mentok 90 derajat! Berhenti muter buat jaga-jaga.")
                state_phase = "FIND_TRIPLE_GATE_1"
                timeout_counter = 0

        # ---------------------------------------------------------
        # PHASE 5: FIND_TRIPLE_GATE_1 (Habis WP2, masuk lorong 2 meter)
        # ---------------------------------------------------------
        elif state_phase == "FIND_TRIPLE_GATE_1":
            if front_status == "LOCKED" and front_class == "Tripple Gate":
                has_seen_target = True
                timeout_counter = 0
                last_front_area = front_area
                last_front_err_y = front_err_y
                yaw_cmd = front_err_x * kp_yaw
                if abs(front_err_x) > 40:
                    fwd_cmd = 0.0
                    up_cmd = (front_err_y + 20) * kp_up
                else:
                    fwd_cmd = 0.8
                    up_cmd = 0.0
                    
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
            else:
                if has_seen_target:
                    # Cek ukuran area terakhir buat ngebedain "flicker dari jauh" vs "masuk lorong"
                    if last_front_area > 10000:
                        # Udah lumayan deket, fix ini masuk lorong. Gas buta!
                        timeout_counter += 1
                    else:
                        # Masih jauh banget (area < 10000), ini cuma YOLO nge-blur. Hover tungguin!
                        print(f"[AUTOPILOT] Triple Gate 1 hilang/flicker dari jauh (Area: {last_front_area}). Hovering...")
                        timeout_counter = 0
                
                if timeout_counter == 0:
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                elif timeout_counter < 150:
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                elif timeout_counter < 180:
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=-0.6, yaw_deg_s=0.0)
                else:
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                
                if timeout_counter > 200: # 3 detik blind forward untuk nembus lorong Triple Gate
                    print("[AUTOPILOT] Keluar dari Triple Gate 1! Nyari Red Box buat Drop...")
                    state_phase = "FIND_RED_BOX"
                    timeout_counter = 0
                    has_seen_target = False


        # ---------------------------------------------------------
        # PHASE 6: FIND_RED_BOX (Medkit Drop)
        # ---------------------------------------------------------
        elif state_phase == "FIND_RED_BOX":
            # 1. Prioritas Utama: Kamera Bawah nyari Kotak Merah untuk Drop Presisi
            if down_status == "LOCKED" and down_class == "RedDrop Box":
                # Pake kp yang jauh lebih kecil (0.0015) biar alus
                fwd_cmd = -down_err_y * 0.0015
                strafe_cmd = down_err_x * 0.0015
                
                # Hard limit kecepatan maksimal cuma 0.2 m/s biar ngeremnya dapet dan nggak bablas (overshoot)
                fwd_cmd = max(-0.2, min(fwd_cmd, 0.2))
                strafe_cmd = max(-0.2, min(strafe_cmd, 0.2))
                
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=0.0, yaw_deg_s=0.0)
                
                if abs(down_err_x) < 20 and abs(down_err_y) < 20:
                    timeout_counter += 1
                    if timeout_counter > 15:
                        print("[AUTOPILOT] BERADA PRESISI DI ATAS RED BOX! DROPPING MEDKIT!!!")
                        await asyncio.sleep(2)
                        print("[AUTOPILOT] Medkit Dropped. Yaw Kanan nyari Triple Gate 2...")
                        state_phase = "YAW_RIGHT_TRIPLE_2"
                        timeout_counter = 0
                else:
                    timeout_counter = 0
            # 2. Prioritas Kedua: Kamera Depan liat Red Box dari jauh, arahkan yaw ke sana
            elif front_status == "LOCKED" and front_class == "RedDrop Box":
                yaw_cmd = front_err_x * kp_yaw
                await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=yaw_cmd)
            # 3. Kalau blank semua, maju pelan
            else:
                await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)

        # ---------------------------------------------------------
        # PHASE 7: YAW_RIGHT_TRIPLE_2 (Yaw 90 derajat kanan)
        # ---------------------------------------------------------
        elif state_phase == "YAW_RIGHT_TRIPLE_2":
            timeout_counter += 1
            await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=15.0)
            if front_status == "LOCKED" and front_class == "Tripple Gate" and front_confident > 0.5:
                print(f"[AUTOPILOT] Triple Gate 2 Ditemukan! (Conf: {front_confident:.2f})")
                state_phase = "TRIPLE_GATE_2"
                timeout_counter = 0
            elif timeout_counter > 150: # RTF 40% x 15s = 6s sim. 6s x 15 deg/s = MENTOK 90 DERAJAT!
                print("[AUTOPILOT] Mentok 90 derajat! Berhenti muter buat jaga-jaga.")
                state_phase = "TRIPLE_GATE_2"
                timeout_counter = 0

        # ---------------------------------------------------------
        # PHASE 8: TRIPLE_GATE_2 (Lewati lorong ke-2 menuju WP4)
        # ---------------------------------------------------------
        elif state_phase == "TRIPLE_GATE_2":
            if front_status == "LOCKED" and front_class == "Tripple Gate":
                has_seen_target = True
                timeout_counter = 0
                last_front_area = front_area
                last_front_err_y = front_err_y
                yaw_cmd = front_err_x * kp_yaw
                if abs(front_err_x) > 40:
                    fwd_cmd = 0.0
                    up_cmd = (front_err_y + 20) * kp_up
                else:
                    fwd_cmd = 0.8
                    up_cmd = 0.0
                    
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
            else:
                if has_seen_target:
                    # Cek ukuran area terakhir buat ngebedain "flicker dari jauh" vs "masuk lorong"
                    if last_front_area > 10000:
                        # Udah lumayan deket, fix ini masuk lorong. Gas buta!
                        timeout_counter += 1
                    else:
                        # Masih jauh banget (area < 10000), ini cuma YOLO nge-blur. Hover tungguin!
                        print(f"[AUTOPILOT] Triple Gate 1 hilang/flicker dari jauh (Area: {last_front_area}). Hovering...")
                        timeout_counter = 0
                
                if timeout_counter == 0:
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                elif timeout_counter < 150:
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                elif timeout_counter < 180:
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=-0.6, yaw_deg_s=0.0)
                else:
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                
                if timeout_counter > 200: # 3 detik blind forward untuk nembus lorong
                    print("[AUTOPILOT] Lolos Triple Gate 2! Mencari Aruco 3...")
                    state_phase = "FIND_ARUCO_3"
                    timeout_counter = 0
                    has_seen_target = False


        # ---------------------------------------------------------
        # PHASE 9: FIND_ARUCO_3 (Mencari belokan kiri)
        # ---------------------------------------------------------
        elif state_phase == "FIND_ARUCO_3":
            if down_status == "LOCKED" and down_class == "Aruco":
                print("[AUTOPILOT] Aruco 3 Ketemu di kamera bawah! Belok kiri...")
                state_phase = "TURN_ARUCO_3"
                timeout_counter = 0
            elif front_status == "LOCKED" and front_class == "Aruco Area":
                # Kunci target dari jauh pakai kamera depan biar gak nyasar/meleset
                strafe_cmd = front_err_x * kp_yaw
                await flight.send_body_velocity(drone, forward_m_s=1.0, right_m_s=strafe_cmd, down_m_s=0.0, yaw_deg_s=0.0)
            else:
                await flight.send_body_velocity(drone, forward_m_s=1.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)

        # ---------------------------------------------------------
        # PHASE 10: TURN_ARUCO_3 (Belok Kiri 90 derajat)
        # ---------------------------------------------------------
        elif state_phase == "TURN_ARUCO_3":
            timeout_counter += 1
            # Belok kiri (yaw_deg_s negatif)
            await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=-30.0)
            
            # Kunci Matematis: Fallback Memory. Turn slowly until we see the gate.
            if front_status == "LOCKED" and front_class == "Single Gate" and front_area > 5000 and front_err_x < 0:
                print(f"[AUTOPILOT] Final Gate 1 Terkunci di Kiri (Area: {front_area}, ErrX: {front_err_x})! Meluncur maju...")
                state_phase = "FIND_FINAL_GATE_1"
                timeout_counter = 0
            
            if timeout_counter > 50: # ~5 detik (Maksimum turning limit)
                print("[AUTOPILOT] Timeout belok! Mencari gawang final 1 secara manual...")
                state_phase = "FIND_FINAL_GATE_1"
                timeout_counter = 0

        # ---------------------------------------------------------
        # PHASE 11: FIND_FINAL_GATE_1 (Mencari Single Gate pertama setelah WP3)
        # ---------------------------------------------------------
        elif state_phase == "FIND_FINAL_GATE_1":
            if front_status == "LOCKED" and front_class == "Single Gate":
                print("[AUTOPILOT] Final Gate 1 Ditemukan! Proses Nembus...")
                state_phase = "PASS_FINAL_GATE_1"
                timeout_counter = 0
                has_seen_target = False
            else:
                await flight.send_body_velocity(drone, forward_m_s=1.5, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                
        # ---------------------------------------------------------
        # PHASE 12: PASS_FINAL_GATE_1 (Tembus Single Gate 1)
        # ---------------------------------------------------------
        elif state_phase == "PASS_FINAL_GATE_1":
            if front_status == "LOCKED" and front_class == "Single Gate" and timeout_counter == 0:
                has_seen_target = True
                timeout_counter = 0
                last_front_area = front_area
                last_front_err_y = front_err_y
                last_front_err_x = front_err_x
                yaw_cmd = front_err_x * kp_yaw
                if not altitude_locked:
                    fwd_cmd = 0.0
                    up_cmd = (front_err_y + 150) * kp_up
                    up_cmd = max(-0.8, min(0.6, up_cmd))
                    if abs(front_err_x) < 20 and abs(front_err_y + 150) < 20:
                        altitude_locked = True
                        print("[AUTOPILOT] Altitude Locked! Punching forward...")
                else:
                    if abs(front_err_x) > 20:
                        fwd_cmd = 0.0
                    else:
                        fwd_cmd = 0.8
                    up_cmd = 0.0
                print(f"[AUTOPILOT] [FINAL GATE 1] Fwd: {fwd_cmd}, Strafe X: {yaw_cmd:.2f}, Z: {up_cmd:.2f}, Lock: {altitude_locked}")
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
            else:
                if has_seen_target:
                    if (last_front_err_y < 20 and abs(last_front_err_x) < 30 and last_front_area > 5000) or last_front_area > 200000 or timeout_counter > 0:
                        if timeout_counter == 0:
                            blind_start_x = DRONE_X
                            blind_start_y = DRONE_Y
                        timeout_counter += 1
                    else:
                        print(f"[AUTOPILOT] Final Gate 1 hilang/flicker (Area: {last_front_area}, ErrX: {last_front_err_x}). Hovering...")
                        timeout_counter = 0
                
                dist_flown = math.sqrt((DRONE_X - blind_start_x)**2 + (DRONE_Y - blind_start_y)**2) if timeout_counter > 0 else 0
                
                if timeout_counter == 0:
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                elif timeout_counter < 30: # DROP INTO THE OPENING
                    if timeout_counter % 10 == 0:
                        print(f"[AUTOPILOT] [FINAL GATE 1] Strafe Down untuk hindari top bar! (Tick: {timeout_counter}/30)")
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.5, yaw_deg_s=0.0)
                elif dist_flown < 2.5: # PUNCH THROUGH
                    if timeout_counter % 10 == 0:
                        print(f"[AUTOPILOT] [FINAL GATE 1] Punching blind! INS Jarak: {dist_flown:.2f}/2.5m")
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                else:
                    print(f"[AUTOPILOT] Lolos Final Gate 1 (Jarak INS: {dist_flown:.2f}m)! Mencari Final Gate 2...")
                    state_phase = "FIND_FINAL_GATE_2"
                    timeout_counter = 0
                    has_seen_target = False

        # ---------------------------------------------------------
        # PHASE 13: FIND_FINAL_GATE_2 (Mencari Single Gate kedua)
        # ---------------------------------------------------------
        elif state_phase == "FIND_FINAL_GATE_2":
            if front_status == "LOCKED" and front_class == "Single Gate":
                print("[AUTOPILOT] Final Gate 2 Ditemukan! Proses Nembus...")
                state_phase = "PASS_FINAL_GATE_2"
                timeout_counter = 0
                has_seen_target = False
            else:
                timeout_counter += 1
                climb_cmd = -0.3 if DRONE_Z > -1.0 else 0.0
                await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                
        # ---------------------------------------------------------
        # PHASE 14: PASS_FINAL_GATE_2 (Tembus Single Gate 2)
        # ---------------------------------------------------------
        elif state_phase == "PASS_FINAL_GATE_2":
            if front_status == "LOCKED" and front_class == "Single Gate" and timeout_counter == 0:
                has_seen_target = True
                timeout_counter = 0
                last_front_area = front_area
                last_front_err_y = front_err_y
                last_front_err_x = front_err_x
                yaw_cmd = front_err_x * kp_yaw
                if not altitude_locked:
                    fwd_cmd = 0.0
                    up_cmd = (front_err_y + 150) * kp_up
                    up_cmd = max(-0.8, min(0.6, up_cmd))
                    if abs(front_err_x) < 20 and abs(front_err_y + 150) < 20:
                        altitude_locked = True
                        print("[AUTOPILOT] Altitude Locked! Punching forward...")
                else:
                    if abs(front_err_x) > 20:
                        fwd_cmd = 0.0
                    else:
                        fwd_cmd = 0.8
                    up_cmd = 0.0
                print(f"[AUTOPILOT] [FINAL GATE 2] Fwd: {fwd_cmd}, Strafe X: {yaw_cmd:.2f}, Z: {up_cmd:.2f}, Lock: {altitude_locked}")
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
            else:
                if has_seen_target:
                    if (last_front_err_y < 20 and abs(last_front_err_x) < 30 and last_front_area > 5000) or last_front_area > 200000 or timeout_counter > 0:
                        if timeout_counter == 0:
                            blind_start_x = DRONE_X
                            blind_start_y = DRONE_Y
                        timeout_counter += 1
                    else:
                        print(f"[AUTOPILOT] Final Gate 2 hilang/flicker (Area: {last_front_area}, ErrX: {last_front_err_x}). Hovering...")
                        timeout_counter = 0
                
                dist_flown = math.sqrt((DRONE_X - blind_start_x)**2 + (DRONE_Y - blind_start_y)**2) if timeout_counter > 0 else 0
                
                if timeout_counter == 0:
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                elif timeout_counter < 30: 
                    if timeout_counter % 10 == 0:
                        print(f"[AUTOPILOT] [FINAL GATE 2] Strafe Down untuk hindari top bar! (Tick: {timeout_counter}/30)")
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.5, yaw_deg_s=0.0)
                elif dist_flown < 3.2: 
                    if timeout_counter % 10 == 0:
                        print(f"[AUTOPILOT] [FINAL GATE 2] Punching blind! INS Jarak: {dist_flown:.2f}/3.2m")
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                elif dist_flown < 4.2: 
                    if timeout_counter % 10 == 0:
                        print(f"[AUTOPILOT] [FINAL GATE 2] Clear! Climbing back up for Landing Pad! Jarak: {dist_flown:.2f}/4.2m")
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=-0.8, yaw_deg_s=0.0)
                else:
                    print(f"[AUTOPILOT] Lolos Final Gate 2 (Jarak INS: {dist_flown:.2f}m)! Mencari Landing Pad...")
                    state_phase = "FIND_LANDING_PAD"
                    timeout_counter = 0
                    has_seen_target = False

        # ---------------------------------------------------------
        # PHASE 15: FIND_LANDING_PAD (Mencari pendaratan akhir)
        # ---------------------------------------------------------
        elif state_phase == "FIND_LANDING_PAD":
            if down_status == "LOCKED" and down_class == "Landing path":
                print("[AUTOPILOT] Landing Pad Terlihat! Memulai Precision Landing...")
                state_phase = "PRECISION_LANDING"
                timeout_counter = 0
            elif front_status == "LOCKED" and front_class == "Landing path":
                yaw_cmd = front_err_x * kp_yaw
                await flight.send_body_velocity(drone, forward_m_s=1.5, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=yaw_cmd)
            else:
                await flight.send_body_velocity(drone, forward_m_s=1.5, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)

        # ---------------------------------------------------------
        # PHASE 10: PRECISION_LANDING (Turun pelan sambil centering)
        # ---------------------------------------------------------
        elif state_phase == "PRECISION_LANDING":
            if down_status == "LOCKED" and (down_class == "Landing path" or down_class == "Aruco"):
                fwd_cmd = -down_err_y * 0.0015
                strafe_cmd = down_err_x * 0.0015
                
                print(f"[AUTOPILOT] [LANDING] Fwd: {fwd_cmd:.2f}, Strafe: {strafe_cmd:.2f}, Tick: {timeout_counter}/5")
                # Turun pelan-pelan (0.3 m/s) sambil centering
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=0.3, yaw_deg_s=0.0)
                
                if abs(down_err_x) < 80 and abs(down_err_y) < 80:
                    timeout_counter += 1
                    if timeout_counter > 5:
                        print("[AUTOPILOT] Mendarat sempurna di titik tengah!")
                        await drone.action.land()
                        print("[AUTOPILOT] Menunggu 8 detik buat pendaratan fisik sebelum Auto-Reset...")
                        await asyncio.sleep(8)
                        import os
                        os.system("./respawn.sh")
                        break
                else:
                    timeout_counter = 0
            else:
                # Kalo target ilang dari kamera bawah pas lagi nunduk, hover sambil turun pelan
                await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.5, yaw_deg_s=0.0)

        await asyncio.sleep(0.1) # Loop jalan 10 Hz

async def main():
    transport = await start_udp_server()
    try:
        await run_mission()
    finally:
        transport.close()

if __name__ == "__main__":
    asyncio.run(main())
