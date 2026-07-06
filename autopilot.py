import asyncio
import json
import math
from utils import clamp, calculate_distance, get_stutter_creep_speed
from mavsdk import System
import flight
from flight import get_distance_sensor_stream
from comms import state, telemetry_task, start_udp_server

async def run_mission():
    drone = System()
    print("[AUTOPILOT] Menyambung ke Drone (SITL)...")
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[AUTOPILOT] Drone Terkoneksi!")
            break
    
    print("[AUTOPILOT] Starting Telemetry Task...")
    asyncio.create_task(telemetry_task(drone))

    print("[AUTOPILOT] Memulai Smart Takeoff...")
    await flight.arm_and_takeoff(drone, altitude_m=1.5)

    print("[AUTOPILOT] Hovering di 1.5m selama 5 detik untuk Stabilisasi...")
    await flight.send_body_velocity(drone, 0.0, 0.0, 0.0, 0.0)
    await asyncio.sleep(5)

    # STATE MACHINE VARIABLES
    state_phase = "CENTERING_GATE_1" # Langsung tembak lurus nyari Gate 1!
    timeout_counter = 0
    has_seen_target = False
    last_front_err_x = 0
    last_front_err_y = 0
    last_front_area = 0
    altitude_locked = False
    blind_start_x = 0.0
    blind_start_y = 0.0
    landing_ticks = 0
    
    # PID Constants
    kp_yaw = 0.005
    kp_up = 0.005

    while True:
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        if state_phase == "CENTERING_GATE_1":
            if front_status == "LOCKED" and front_class == "Single Gate":
                has_seen_target = True
                timeout_counter = 0
                last_front_area = front_area
                last_front_err_x = front_err_x
                last_front_err_y = front_err_y
                
                # Logic: Mendekat dulu, baru Strafe untuk centering X dan Y (hindari top bar)
                if front_area < 25000:
                    fwd_cmd = 0.8
                    strafe_cmd = front_err_x * kp_yaw
                    z_err = -0.8 - state.z
                    up_cmd = clamp(z_err * 0.5, -0.5, 0.5) # Active Z=0.8m Lock
                else:
                    if not altitude_locked:
                        # Belum nge-lock, kita hover dan presisi-kan
                        strafe_cmd = front_err_x * kp_yaw
                        up_cmd = (front_err_y + 150) * kp_up
                        up_cmd = clamp(up_cmd, -0.6, 0.6)
                        fwd_cmd = 0.0 # Berhenti maju buat nunggu stabil (anti-banteng)
                        
                        # Tolerance dilebarkan sedikit (30 pixel) biar ga nyangkut infinite loop
                        if abs(front_err_x) < 30 and abs(front_err_y + 150) < 30:
                            altitude_locked = True
                            print("[AUTOPILOT] [GATE 1] Centered! ALTITUDE LOCKED. Going Pitbull...")
                    else:
                        # UDAH LOCK! Bodo amat sama error Y (Karna kalau maju drone nunduk dan bikin ilusi error Y)
                        z_err = -0.8 - state.z

                        up_cmd = clamp(z_err * 0.5, -0.5, 0.5) # Active Z=0.8m Lock
                        strafe_cmd = front_err_x * kp_yaw
                        if abs(front_err_x) > 40:
                            fwd_cmd = 0.0 # Kalo melenceng X-nya aja baru ngerem
                        else:
                            fwd_cmd = 0.8
                            
                        # ANTI-DRIFT: Kalau udah terlalu deket (Bounding Box nutupin layar), 
                        # jangan ngelakuin micro-correction nyamping karena pixel error-nya nggak akurat
                        if front_area > 100000:
                            strafe_cmd = 0.0
                        
                    print(f"[AUTOPILOT] [GATE 1] Centering (Area: {front_area}). Strafe: {strafe_cmd:.2f}, Z: {up_cmd:.2f}, Lock: {altitude_locked}")

                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
            else:
                if has_seen_target:
                    # Syarat blind-punch: Harus udah deket banget (Area gede) atau bener-bener di tengah sebelum hilang
                    if (last_front_err_y < 20 and abs(last_front_err_x) < 30 and last_front_area > 20000) or last_front_area > 150000 or timeout_counter > 0:
                        if timeout_counter == 0:
                            blind_start_x = state.x
                            blind_start_y = state.y
                        timeout_counter += 1
                    else:
                        print(f"[AUTOPILOT] Gawang 1 hilang dari jauh (Area: {last_front_area}, ErrX: {last_front_err_x}). Hovering...")
                        timeout_counter = 0
                
                dist_flown = math.sqrt((state.x - blind_start_x)**2 + (state.y - blind_start_y)**2) if timeout_counter > 0 else 0
                
                if timeout_counter == 0:
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                elif dist_flown < 3.2: # PUNCH THROUGH INS
                    z_err = -0.8 - state.z
                    up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                    
                    if timeout_counter % 10 == 0:
                        print(f"[AUTOPILOT] [GATE 1] Punching blind! INS Jarak: {dist_flown:.2f}/3.2m, Z: {up_cmd:.2f}")
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
                else:
                    print(f"[AUTOPILOT] Lolos Gate 1 (Jarak INS: {dist_flown:.2f}m)! Mencari Aruco 1...")
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
                has_seen_target = False
            else:
                # Kembali ke ketinggian 1.5m (state.z = -1.5)
                z_err = -1.5 - state.z
                climb_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                
                if front_status == "LOCKED" and front_class == "Aruco Area":
                    blind_start_x = state.x
                    blind_start_y = state.y
                    has_seen_target = True
                    timeout_counter = 0
                    last_front_err_x = front_err_x
                    yaw_cmd = front_err_x * kp_yaw
                    
                    if abs(front_err_x) > 40:
                        # Target off-center! Hover and rotate to face it before pushing forward.
                        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=yaw_cmd)
                    else:
                        await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=yaw_cmd)
                elif has_seen_target:
                    # FALLBACK MEMORY: Masuk blind spot antara kamera depan dan bawah
                    dist_flown = math.sqrt((state.x - blind_start_x)**2 + (state.y - blind_start_y)**2)
                    timeout_counter += 1
                    
                    if dist_flown > 2.5:
                        timeout_counter = 500 # Latch reverse state
                        
                    if timeout_counter >= 500:
                        print(f"[AUTOPILOT] Kebablasan ArUco 1 di blind spot! Terbang mundur...")
                        await flight.send_body_velocity(drone, forward_m_s=-0.3, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                    else:
                        # Stutter creep to level pitch and scan straight down
                        fwd_creep = 0.3 if timeout_counter % 20 < 10 else 0.0
                        await flight.send_body_velocity(drone, forward_m_s=fwd_creep, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                else:
                    await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)

        # ---------------------------------------------------------
        # PHASE 3B: CENTER_ARUCO_1 (Precision Hover)
        # ---------------------------------------------------------
        elif state_phase == "CENTER_ARUCO_1":
            if down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area"]:
                has_seen_target = True
                last_down_err_x = down_err_x
                last_down_err_y = down_err_y
                
                fwd_cmd = -down_err_y * 0.0015
                strafe_cmd = down_err_x * 0.0015
                
                # Limit speed so it doesn't overshoot without gimbal
                fwd_cmd = clamp(fwd_cmd, -0.2, 0.2)
                strafe_cmd = clamp(strafe_cmd, -0.2, 0.2)
                
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=0.0, yaw_deg_s=0.0)
                
                if abs(down_err_x) < 80 and abs(down_err_y) < 80:
                    timeout_counter += 1
                    
                    # Fast completion if inner marker is seen, slow fallback if only outer area is seen
                    completion_threshold = 50 if down_class == "Aruco" else 150
                    
                    if timeout_counter > completion_threshold: 
                        print("[AUTOPILOT] Presisi WP1 Tercapai! Mengikuti Straight Line menuju WP2...")
                        state_phase = "FOLLOW_LINE_TO_WP2"
                        timeout_counter = 0
                        has_seen_target = False
                else:
                    timeout_counter = 0
            else:
                timeout_counter += 1
                if timeout_counter > 50:
                    print("[AUTOPILOT] WP1 Hilang! Kembali ke FIND_ARUCO_1...")
                    state_phase = "FIND_ARUCO_1"
                    timeout_counter = 0
                    blind_start_x = state.x
                    blind_start_y = state.y
                elif has_seen_target:
                    # FALLBACK MEMORY: Rebound brake
                    fwd_cmd = clamp(-last_down_err_y * 0.0015, -0.2, 0.2)
                    strafe_cmd = clamp(last_down_err_x * 0.0015, -0.2, 0.2)
                    await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=global_climb_cmd, yaw_deg_s=0.0)
                else:
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=0.0)

        # ---------------------------------------------------------
        # PHASE 4: FOLLOW_LINE_TO_WP2 (Murni Ngikutin Garis)
        # ---------------------------------------------------------
        elif state_phase == "FOLLOW_LINE_TO_WP2":
            fwd_cmd = 1.0
            yaw_cmd = 0.0
            strafe_cmd = 0.0
            
            # EARLY EXIT: Jika sudah melihat Aruco 2, langsung stop ikuti garis
            # ONLY early exit if we have seen the straight line first (meaning we left Aruco 1)
            if has_seen_target and down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area"]:
                print("[AUTOPILOT] WP2 (Aruco 2) Terlihat di Bawah! Langsung Centering...")
                state_phase = "CENTER_ARUCO_2"
                timeout_counter = 0
                has_seen_target = False
                continue # Skip the rest of the loop to enter new phase immediately
                
            elif has_seen_target and front_status == "LOCKED" and front_class == "Aruco Area":
                print("[AUTOPILOT] Aruco 2 Terlihat di Depan! Beralih mencari Aruco 2...")
                state_phase = "FIND_ARUCO_2"
                timeout_counter = 0
                has_seen_target = False
                continue
            
            if front_status == "LOCKED" and front_class == "Straight Line":
                yaw_cmd = front_err_x * kp_yaw
                
            if down_status == "LOCKED" and down_class == "Straight Line":
                strafe_cmd = down_err_x * kp_yaw
                timeout_counter = 0 # Reset timeout kalau masih liat garis
                has_seen_target = True # Tandai bahwa kita udah berhasil nangkep garis
            else:
                if has_seen_target: # Cuma ngitung timeout hilang JIKA sebelumnya udah dapet garis
                    timeout_counter += 1
                
            if timeout_counter > 150: # Garis hilang (RTF 30% scale)
                print("[AUTOPILOT] Ujung Garis WP2 tercapai! Beralih mencari Aruco 2...")
                state_phase = "FIND_ARUCO_2"
                timeout_counter = 0
                has_seen_target = False
            
            await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=global_climb_cmd, yaw_deg_s=yaw_cmd)

        # ---------------------------------------------------------
        # PHASE 4A: FIND_ARUCO_2 (Mencari Aruco setelah garis habis)
        # ---------------------------------------------------------
        elif state_phase == "FIND_ARUCO_2":
            if down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area"]:
                print("[AUTOPILOT] WP2 (Aruco 2) Terlihat! Memulai Centering...")
                state_phase = "CENTER_ARUCO_2"
                timeout_counter = 0
                has_seen_target = False
            else:
                if front_status == "LOCKED" and front_class == "Aruco Area":
                    blind_start_x = state.x
                    blind_start_y = state.y
                    has_seen_target = True
                    timeout_counter = 0
                    last_front_err_x = front_err_x
                    yaw_cmd = front_err_x * kp_yaw
                    
                    if abs(front_err_x) > 40:
                        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=yaw_cmd)
                    else:
                        await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=yaw_cmd)
                elif has_seen_target:
                    # FALLBACK MEMORY: Blind spot creep
                    dist_flown = math.sqrt((state.x - blind_start_x)**2 + (state.y - blind_start_y)**2)
                    timeout_counter += 1
                    
                    if dist_flown > 2.5:
                        timeout_counter = 500 # Latch reverse state
                        
                    if timeout_counter >= 500:
                        print(f"[AUTOPILOT] Kebablasan ArUco 2 di blind spot! Terbang mundur...")
                        await flight.send_body_velocity(drone, forward_m_s=-0.3, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=0.0)
                    else:
                        # Stutter creep to level pitch and scan straight down
                        fwd_creep = 0.3 if timeout_counter % 20 < 10 else 0.0
                        await flight.send_body_velocity(drone, forward_m_s=fwd_creep, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=0.0)
                else:
                    await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=0.0)

        # ---------------------------------------------------------
        # PHASE 4B: CENTER_ARUCO_2 (Precision Hover)
        # ---------------------------------------------------------
        elif state_phase == "CENTER_ARUCO_2":
            if down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area"]:
                has_seen_target = True
                last_down_err_x = down_err_x
                last_down_err_y = down_err_y
                
                fwd_cmd = -down_err_y * 0.0015
                strafe_cmd = down_err_x * 0.0015
                
                fwd_cmd = clamp(fwd_cmd, -0.2, 0.2)
                strafe_cmd = clamp(strafe_cmd, -0.2, 0.2)
                
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=global_climb_cmd, yaw_deg_s=0.0)
                
                if abs(down_err_x) < 80 and abs(down_err_y) < 80:
                    timeout_counter += 1
                    
                    # Fast completion if inner marker is seen, slow fallback if only outer area is seen
                    completion_threshold = 50 if down_class == "Aruco" else 150
                    
                    if timeout_counter > completion_threshold:
                        print("[AUTOPILOT] Presisi WP2 Tercapai! Mutar kiri nyari Triple Gate 1...")
                        state_phase = "YAW_LEFT_TRIPLE_1"
                        timeout_counter = 0
                        has_seen_target = False
                else:
                    timeout_counter = 0
            else:
                timeout_counter += 1
                if timeout_counter > 50:
                    print("[AUTOPILOT] WP2 Hilang! Kembali ke FIND_ARUCO_2...")
                    state_phase = "FIND_ARUCO_2"
                    timeout_counter = 0
                    blind_start_x = state.x
                    blind_start_y = state.y
                elif has_seen_target:
                    # FALLBACK MEMORY: Rebound brake
                    fwd_cmd = clamp(-last_down_err_y * 0.0015, -0.2, 0.2)
                    strafe_cmd = clamp(last_down_err_x * 0.0015, -0.2, 0.2)
                    await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=global_climb_cmd, yaw_deg_s=0.0)
                else:
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=0.0)

        # ---------------------------------------------------------
        # PHASE 4C: YAW_LEFT_TRIPLE_1 (Belok Kiri Nyari Triple Gate)
        # ---------------------------------------------------------
        elif state_phase == "YAW_LEFT_TRIPLE_1":
            timeout_counter += 1
            await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=-15.0)
            if front_status == "LOCKED" and front_class == "Tripple Gate" and front_area > 10000:
                print(f"[AUTOPILOT] Triple Gate terlihat (Area: {front_area})! Memulai approach...")
                state_phase = "FIND_TRIPLE_GATE_1"
                timeout_counter = 0

        # ---------------------------------------------------------
        # PHASE 5: FIND_TRIPLE_GATE_1 (Habis WP2, masuk lorong 2 meter)
        # ---------------------------------------------------------
        elif state_phase == "FIND_TRIPLE_GATE_1":
            if front_status == "LOCKED" and front_class == "Tripple Gate":
                has_seen_target = True
                last_front_area = front_area
                last_front_err_y = front_err_y
                last_front_err_x = front_err_x
                yaw_cmd = front_err_x * kp_yaw
                
                if front_area < 12000:
                    fwd_cmd = 0.8
                    z_err = -0.8 - state.z
                    up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                else:
                    if abs(front_err_x) > 40:
                        fwd_cmd = 0.0
                        up_cmd = (front_err_y + 20) * kp_up
                    else:
                        fwd_cmd = 0.8
                        z_err = -0.8 - state.z
                        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                    
                print(f"[AUTOPILOT] [TRIPLE GATE 1] Centering (Area: {front_area}). Fwd: {fwd_cmd}, Yaw: {yaw_cmd:.2f}, Z: {up_cmd:.2f}")
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
            else:
                if has_seen_target:
                    if last_front_area > 10000 and abs(last_front_err_x) < 100:
                        print("[AUTOPILOT] Memasuki Lorong Triple Gate 1! Berpindah ke PUNCH_TRIPLE_GATE_1")
                        state_phase = "PUNCH_TRIPLE_GATE_1"
                        blind_start_x = state.x
                        blind_start_y = state.y
                        continue
                    else:
                        print(f"[AUTOPILOT] Triple Gate 1 hilang dari jauh (Area: {last_front_area}). Fallback Hover & Yaw!")
                
                # Hover in place while rotating to find the gate
                mem_yaw = last_front_err_x * kp_yaw
                mem_yaw = clamp(mem_yaw, -15.0, 15.0)
                await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=mem_yaw)

        elif state_phase == "PUNCH_TRIPLE_GATE_1":
            dist_flown = math.sqrt((state.x - blind_start_x)**2 + (state.y - blind_start_y)**2)
            if dist_flown < 3.8: # PUNCH THROUGH TUNNEL
                strafe_cmd = 0.0
                if state.lidar_left < 4.9 or state.lidar_right < 4.9:
                    strafe_cmd = (state.lidar_right - state.lidar_left) * 0.05
                z_err = -0.8 - state.z
                up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                
                print(f"[AUTOPILOT] [TRIPLE GATE 1] Blind Punch INS! Jarak: {dist_flown:.2f}/3.8m, Lidar: {strafe_cmd:.2f}")
                await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
            else:
                print("[AUTOPILOT] Keluar dari Triple Gate 1! Nyari Red Box buat Drop...")
                state_phase = "FIND_DROPBOX"
                has_seen_target = False


        # ---------------------------------------------------------
        # PHASE 6: FIND_DROPBOX (Mencari Red Drop Box)
        # ---------------------------------------------------------
        elif state_phase == "FIND_DROPBOX":
            if down_status == "LOCKED" and down_class in ["Red Drop Box", "RedDrop Box"]:
                print("[AUTOPILOT] Red Drop Box Terlihat di Kamera Bawah! AUTO-STOP & Memulai Centering...")
                state_phase = "CENTER_DROPBOX"
                timeout_counter = 0
                has_seen_target = False
            else:
                # P-Controller untuk CLIMB ke -1.5 meter secara instan setelah keluar lorong
                z_err = -1.5 - state.z
                climb_cmd = clamp(z_err * 0.8, -0.8, 0.5) # Agresif naik

                # Rule 4: Flat Object Camera Handoff
                # Kalo drop box kelihatan di kamera depan, steer ke sana
                if front_status == "LOCKED" and front_class in ["Red Drop Box", "RedDrop Box"]:
                    blind_start_x = state.x
                    blind_start_y = state.y
                    has_seen_target = True
                    timeout_counter = 0
                    last_front_err_x = front_err_x
                    yaw_cmd = front_err_x * kp_yaw
                    
                    if abs(front_err_x) > 40:
                        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=yaw_cmd)
                    else:
                        await flight.send_body_velocity(drone, forward_m_s=0.6, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=yaw_cmd)
                elif has_seen_target:
                    # FALLBACK MEMORY: Masuk blind spot antara kamera depan dan bawah. Creep pelan pakai memori yaw.
                    dist_flown = math.sqrt((state.x - blind_start_x)**2 + (state.y - blind_start_y)**2)
                    timeout_counter += 1
                    
                    if dist_flown > 2.5:
                        timeout_counter = 500 # Latch reverse state
                        
                    if timeout_counter >= 500:
                        print(f"[AUTOPILOT] Kebablasan Drop Box di blind spot! Terbang mundur...")
                        await flight.send_body_velocity(drone, forward_m_s=-0.3, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                    else:
                        # Stutter creep to level pitch and scan straight down
                        fwd_creep = 0.3 if timeout_counter % 20 < 10 else 0.0
                        await flight.send_body_velocity(drone, forward_m_s=fwd_creep, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                else:
                    # Belum keliatan, jalan lurus pelan sambil nanjak ke ketinggian operasi
                    await flight.send_body_velocity(drone, forward_m_s=0.6, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                    
        # ---------------------------------------------------------
        # PHASE 6.5: CENTER_DROPBOX (Mensejajarkan Drone dengan Drop Box)
        # ---------------------------------------------------------
        elif state_phase == "CENTER_DROPBOX":
            if down_status == "LOCKED" and down_class in ["Red Drop Box", "RedDrop Box", "Aruco Area"]:
                has_seen_target = True
                last_down_err_x = down_err_x
                last_down_err_y = down_err_y
                
                # Active Auto-Stop Braking (Gentle for Gimbal-less)
                fwd_cmd = -down_err_y * 0.0015
                strafe_cmd = down_err_x * 0.0015
                
                # Limit speed so it brakes instead of overshooting
                fwd_cmd = clamp(fwd_cmd, -0.2, 0.2)
                strafe_cmd = clamp(strafe_cmd, -0.2, 0.2)
                
                # Z dijaga ketat di -1.5
                z_err = -1.5 - state.z
                up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                
                print(f"[AUTOPILOT] [DROP BOX] Centering (X:{down_err_x}, Y:{down_err_y}). Fwd:{fwd_cmd:.2f}, Strafe:{strafe_cmd:.2f}, Z:{up_cmd:.2f}")
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
                
                if abs(down_err_x) < 20 and abs(down_err_y) < 20:
                    if down_class in ["Red Drop Box", "RedDrop Box"]:
                        timeout_counter += 1
                        if timeout_counter > 100:
                            print("[AUTOPILOT] Medkit Dropped. Yaw Kanan nyari Triple Gate 2...")
                            state_phase = "YAW_RIGHT_TRIPLE_2"
                            timeout_counter = 0
                            has_seen_target = False
                    else:
                        timeout_counter = 0
                else:
                    timeout_counter = 0
            else:
                timeout_counter += 1
                if timeout_counter > 50:
                    print("[AUTOPILOT] Drop Box Hilang dari kamera bawah! Kembali ke FIND_DROPBOX...")
                    state_phase = "FIND_DROPBOX"
                    timeout_counter = 0
                    blind_start_x = state.x
                    blind_start_y = state.y
                elif has_seen_target:
                    # FALLBACK MEMORY DOWN CAMERA: Terbang balik ke kordinat terakhir kali keliatan!
                    fwd_cmd = clamp(-last_down_err_y * 0.0015, -0.2, 0.2)
                    strafe_cmd = clamp(last_down_err_x * 0.0015, -0.2, 0.2)
                    z_err = -1.5 - state.z
                    up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                    print(f"[AUTOPILOT] Drop Box Flicker! Terbang balik pake memori kamera bawah... Fwd: {fwd_cmd:.2f}")
                    await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
                else:
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)

        # ---------------------------------------------------------
        # PHASE 7: YAW_RIGHT_TRIPLE_2 (Yaw Kanan ke Triple Gate 2)
        # ---------------------------------------------------------
        elif state_phase == "YAW_RIGHT_TRIPLE_2":
            timeout_counter += 1
            await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=15.0)
            if front_status == "LOCKED" and front_class == "Tripple Gate" and front_area > 10000:
                print(f"[AUTOPILOT] Triple Gate 2 terlihat (Area: {front_area})! Memulai approach...")
                state_phase = "TRIPLE_GATE_2"
                timeout_counter = 0

        # ---------------------------------------------------------
        # PHASE 8: TRIPLE_GATE_2 (Lewati lorong ke-2 menuju WP4)
        # ---------------------------------------------------------
        elif state_phase == "TRIPLE_GATE_2":
            if front_status == "LOCKED" and front_class == "Tripple Gate":
                has_seen_target = True
                last_front_area = front_area
                last_front_err_y = front_err_y
                last_front_err_x = front_err_x
                yaw_cmd = front_err_x * kp_yaw
                
                if front_area < 12000:
                    fwd_cmd = 0.8
                    z_err = -0.8 - state.z
                    up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                else:
                    if abs(front_err_x) > 40:
                        fwd_cmd = 0.0
                        up_cmd = (front_err_y + 20) * kp_up
                    else:
                        fwd_cmd = 0.8
                        z_err = -0.8 - state.z
                        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                        
                print(f"[AUTOPILOT] [TRIPLE GATE 2] Centering (Area: {front_area}). Fwd: {fwd_cmd}, Yaw: {yaw_cmd:.2f}, Z: {up_cmd:.2f}")
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
            else:
                if has_seen_target:
                    if last_front_area > 10000 and abs(last_front_err_x) < 100:
                        print("[AUTOPILOT] Memasuki Lorong Triple Gate 2! Berpindah ke PUNCH_TRIPLE_GATE_2")
                        state_phase = "PUNCH_TRIPLE_GATE_2"
                        blind_start_x = state.x
                        blind_start_y = state.y
                        continue
                    else:
                        print(f"[AUTOPILOT] Triple Gate 2 hilang dari jauh (Area: {last_front_area}). Fallback Hover & Yaw!")
                
                mem_yaw = last_front_err_x * kp_yaw
                mem_yaw = clamp(mem_yaw, -15.0, 15.0)
                await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=mem_yaw)

        elif state_phase == "PUNCH_TRIPLE_GATE_2":
            dist_flown = math.sqrt((state.x - blind_start_x)**2 + (state.y - blind_start_y)**2)
            if dist_flown < 3.8:
                strafe_cmd = 0.0
                if state.lidar_left < 4.9 or state.lidar_right < 4.9:
                    strafe_cmd = (state.lidar_right - state.lidar_left) * 0.05
                z_err = -0.8 - state.z
                up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                
                print(f"[AUTOPILOT] [TRIPLE GATE 2] Blind Punch INS! Jarak: {dist_flown:.2f}/3.8m, Lidar: {strafe_cmd:.2f}")
                await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
            else:
                print("[AUTOPILOT] Lolos Triple Gate 2! Mencari Aruco 3...")
                state_phase = "FIND_ARUCO_3"
                has_seen_target = False


        # ---------------------------------------------------------
        # PHASE 9: FIND_ARUCO_3 (Mencari belokan kiri)
        # ---------------------------------------------------------
        elif state_phase == "FIND_ARUCO_3":
            if down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area"]:
                print("[AUTOPILOT] Aruco 3 Terlihat di Kamera Bawah! AUTO-STOP & Memulai Centering...")
                state_phase = "CENTER_ARUCO_3"
                timeout_counter = 0
                has_seen_target = False
            else:
                z_err = -1.5 - state.z
                climb_cmd = clamp(z_err * 0.8, -0.8, 0.5)
                
                if front_status == "LOCKED" and front_class in ["Aruco", "Aruco Area"]:
                    blind_start_x = state.x
                    blind_start_y = state.y
                    has_seen_target = True
                    timeout_counter = 0
                    yaw_cmd = front_err_x * kp_yaw
                    
                    if abs(front_err_x) > 40:
                        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=yaw_cmd)
                    else:
                        await flight.send_body_velocity(drone, forward_m_s=0.6, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=yaw_cmd)
                elif has_seen_target:
                    dist_flown = math.sqrt((state.x - blind_start_x)**2 + (state.y - blind_start_y)**2)
                    timeout_counter += 1
                    
                    if dist_flown > 2.5:
                        timeout_counter = 500
                        
                    if timeout_counter >= 500:
                        print(f"[AUTOPILOT] Kebablasan WP3 di blind spot! Terbang mundur...")
                        await flight.send_body_velocity(drone, forward_m_s=-0.3, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                    else:
                        fwd_creep = 0.3 if timeout_counter % 20 < 10 else 0.0
                        await flight.send_body_velocity(drone, forward_m_s=fwd_creep, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                else:
                    await flight.send_body_velocity(drone, forward_m_s=0.6, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)

        # ---------------------------------------------------------
        # PHASE 9.5: CENTER_ARUCO_3 (Mensejajarkan Drone dengan WP3)
        # ---------------------------------------------------------
        elif state_phase == "CENTER_ARUCO_3":
            if down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area"]:
                has_seen_target = True
                last_down_err_x = down_err_x
                last_down_err_y = down_err_y
                
                fwd_cmd = -down_err_y * 0.0015
                strafe_cmd = down_err_x * 0.0015
                fwd_cmd = clamp(fwd_cmd, -0.2, 0.2)
                strafe_cmd = clamp(strafe_cmd, -0.2, 0.2)
                z_err = -1.5 - state.z
                up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
                
                if abs(down_err_x) < 20 and abs(down_err_y) < 20:
                    timeout_counter += 1
                    completion_threshold = 50 if down_class == "Aruco" else 150
                    if timeout_counter > completion_threshold:
                        print("[AUTOPILOT] Presisi WP3 Tercapai! Mutar kiri nyari Final Gate 1...")
                        state_phase = "TURN_ARUCO_3"
                        timeout_counter = 0
                        has_seen_target = False
                else:
                    timeout_counter = 0
            else:
                timeout_counter += 1
                if timeout_counter > 50:
                    print("[AUTOPILOT] WP3 Hilang! Kembali ke FIND_ARUCO_3...")
                    state_phase = "FIND_ARUCO_3"
                    timeout_counter = 0
                    blind_start_x = state.x
                    blind_start_y = state.y
                elif has_seen_target:
                    fwd_cmd = clamp(-last_down_err_y * 0.0015, -0.2, 0.2)
                    strafe_cmd = clamp(last_down_err_x * 0.0015, -0.2, 0.2)
                    z_err = -1.5 - state.z
                    up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                    await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)

        # ---------------------------------------------------------
        # PHASE 10: TURN_ARUCO_3 (Belok Kiri 90 derajat)
        # ---------------------------------------------------------
        elif state_phase == "TURN_ARUCO_3":
            timeout_counter += 1
            # Belok kiri (yaw_deg_s negatif)
            await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=-30.0)
            
            # Kunci Matematis: Area filter > 12000 to ensure we lock the CLOSEST gate (Final Gate 1), not Final Gate 2 in the background.
            if front_status == "LOCKED" and front_class == "Single Gate" and front_area > 12000 and front_err_x < 0:
                print(f"[AUTOPILOT] Final Gate 1 Terkunci di Kiri (Area: {front_area})! Meluncur maju...")
                state_phase = "FIND_FINAL_GATE_1"
                timeout_counter = 0
            
            if timeout_counter > 120: # ~5 detik (Maksimum turning limit)
                print("[AUTOPILOT] Timeout belok! Mencari gawang final 1 secara manual...")
                state_phase = "FIND_FINAL_GATE_1"
                timeout_counter = 0

        # ---------------------------------------------------------
        # PHASE 11: FIND_FINAL_GATE_1 (Mencari Single Gate pertama setelah WP3)
        # ---------------------------------------------------------
        elif state_phase == "FIND_FINAL_GATE_1":
            if front_status == "LOCKED" and front_class == "Single Gate" and front_area > 12000:
                print("[AUTOPILOT] Final Gate 1 (Closest) Ditemukan! Proses Nembus...")
                state_phase = "PASS_FINAL_GATE_1"
                timeout_counter = 0
                has_seen_target = False
            elif front_status == "LOCKED" and front_class == "Single Gate":
                # Keliatan tapi area kecil (mungkin gate 2). Arahkan yaw aja pelan-pelan.
                yaw_cmd = front_err_x * kp_yaw
                await flight.send_body_velocity(drone, forward_m_s=1.5, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=yaw_cmd)
            else:
                await flight.send_body_velocity(drone, forward_m_s=1.5, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                
        # ---------------------------------------------------------
        # PHASE 12: PASS_FINAL_GATE_1 (Tembus Single Gate 1)
        # ---------------------------------------------------------
        elif state_phase == "PASS_FINAL_GATE_1":
            if front_status == "LOCKED" and front_class == "Single Gate":
                has_seen_target = True
                timeout_counter = 0
                last_front_area = front_area
                last_front_err_x = front_err_x
                last_front_err_y = front_err_y
                
                # Logic: Mendekat dulu, baru Strafe & Yaw untuk centering X (drift alignment)
                if front_area < 25000:
                    fwd_cmd = 0.8
                    strafe_cmd = front_err_x * kp_yaw
                    strafe_cmd = clamp(strafe_cmd, -0.15, 0.15) # ANTI-DRIFT: Clamp ketat
                    yaw_cmd = front_err_x * 0.08 # Rotasi pelan ke target
                    yaw_cmd = clamp(yaw_cmd, -10.0, 10.0)
                    
                    z_err = -0.8 - state.z
                    up_cmd = clamp(z_err * 0.5, -0.5, 0.5) # Active Z=0.8m Lock
                else:
                    if not altitude_locked:
                        strafe_cmd = front_err_x * kp_yaw
                        strafe_cmd = clamp(strafe_cmd, -0.15, 0.15)
                        yaw_cmd = front_err_x * 0.08
                        yaw_cmd = clamp(yaw_cmd, -10.0, 10.0)
                        
                        # Turun ke bawah top bar
                        up_cmd = (front_err_y + 150) * kp_up
                        up_cmd = clamp(up_cmd, -0.6, 0.6)
                        fwd_cmd = 0.0
                        
                        if abs(front_err_x) < 30 and abs(front_err_y + 150) < 30:
                            altitude_locked = True
                            print("[AUTOPILOT] [FINAL GATE 1] Centered! ALTITUDE LOCKED. Going Pitbull...")
                    else:
                        z_err = -0.8 - state.z
                        up_cmd = clamp(z_err * 0.5, -0.5, 0.5) # Active Z=0.8m Lock
                        
                        strafe_cmd = front_err_x * kp_yaw
                        strafe_cmd = clamp(strafe_cmd, -0.15, 0.15) # ANTI-DRIFT: Clamp ketat
                        yaw_cmd = front_err_x * 0.08
                        yaw_cmd = clamp(yaw_cmd, -10.0, 10.0)
                        
                        if abs(front_err_x) > 40:
                            fwd_cmd = 0.0
                        else:
                            fwd_cmd = 0.8
                            
                        # ANTI-DRIFT: Kalau udah terlalu deket, tahan lateral & rotasi mendadak
                        if front_area > 100000:
                            strafe_cmd = 0.0
                            yaw_cmd = 0.0
                            fwd_cmd = 0.8 # Paksa maju! Jangan berhenti buat centering kalau udah point-blank
                
                print(f"[AUTOPILOT] [FINAL GATE 1] Centering (Area: {front_area}). Fwd: {fwd_cmd}, Strafe: {strafe_cmd:.2f}, Yaw: {yaw_cmd:.2f}, Z: {up_cmd:.2f}")
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
            else:
                if has_seen_target:
                    if (last_front_err_y < 20 and abs(last_front_err_x) < 30 and last_front_area > 20000) or last_front_area > 150000 or timeout_counter > 0:
                        if timeout_counter == 0:
                            blind_start_x = state.x
                            blind_start_y = state.y
                        timeout_counter += 1
                    else:
                        timeout_counter = 0
                
                dist_flown = math.sqrt((state.x - blind_start_x)**2 + (state.y - blind_start_y)**2) if timeout_counter > 0 else 0
                
                if timeout_counter == 0:
                    # FALLBACK MEMORY: Hover in place while rotating to find the gate!
                    mem_yaw = last_front_err_x * kp_yaw
                    mem_yaw = clamp(mem_yaw, -15.0, 15.0)
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=mem_yaw)
                elif dist_flown < 8.0: # PUNCH THROUGH INS KEDUA GAWANG SEKALIGUS
                    z_err = -0.8 - state.z
                    up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                    print(f"[AUTOPILOT] [FINAL GATE PUNCH] Jarak Tempuh INS: {dist_flown:.2f}m / 8.0m")
                    await flight.send_body_velocity(drone, forward_m_s=1.2, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
                else:
                    print(f"[AUTOPILOT] Lolos Final Gate 1 & 2 Sekaligus (Jarak 8.0m)! Mencari Landing Pad...")
                    state_phase = "FIND_LANDING_PAD"
                    timeout_counter = 0
                    has_seen_target = False

        # ---------------------------------------------------------
        # PHASE 15: FIND_LANDING_PAD (Mencari pendaratan akhir)
        # ---------------------------------------------------------
        elif state_phase == "FIND_LANDING_PAD":
            if down_status == "LOCKED" and down_class in ["Landing path", "Aruco"]:
                print("[AUTOPILOT] Landing Pad Terlihat di Kamera Bawah! AUTO-STOP & Memulai Centering...")
                state_phase = "PRECISION_LANDING"
                timeout_counter = 0
                has_seen_target = False
            else:
                # P-Controller CLIMB ke -1.5m setelah punch through Final Gate 2
                z_err = -1.5 - state.z
                climb_cmd = clamp(z_err * 0.8, -0.8, 0.5)
                
                # Rule 4: Handoff Kamera
                if front_status == "LOCKED" and front_class in ["Landing path", "Aruco"]:
                    blind_start_x = state.x
                    blind_start_y = state.y
                    has_seen_target = True
                    timeout_counter = 0
                    last_front_err_x = front_err_x
                    yaw_cmd = front_err_x * kp_yaw
                    
                    if abs(front_err_x) > 40:
                        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=yaw_cmd)
                    else:
                        await flight.send_body_velocity(drone, forward_m_s=0.6, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=yaw_cmd)
                elif has_seen_target:
                    # FALLBACK MEMORY: Blind spot creep
                    dist_flown = math.sqrt((state.x - blind_start_x)**2 + (state.y - blind_start_y)**2)
                    timeout_counter += 1
                    
                    if dist_flown > 2.5:
                        timeout_counter = 500 # Latch reverse state
                        
                    if timeout_counter >= 500:
                        print(f"[AUTOPILOT] Kebablasan Landing Pad di blind spot! Terbang mundur...")
                        await flight.send_body_velocity(drone, forward_m_s=-0.3, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                    else:
                        # Stutter creep to level pitch and scan straight down
                        fwd_creep = 0.3 if timeout_counter % 20 < 10 else 0.0
                        await flight.send_body_velocity(drone, forward_m_s=fwd_creep, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                else:
                    await flight.send_body_velocity(drone, forward_m_s=0.6, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)

        # ---------------------------------------------------------
        # PHASE 10: PRECISION_LANDING (Turun pelan sambil centering)
        # ---------------------------------------------------------
        elif state_phase == "PRECISION_LANDING":
            if down_status == "LOCKED" and (down_class == "Landing path" or down_class == "Aruco"):
                has_seen_target = True
                last_down_err_x = down_err_x
                last_down_err_y = down_err_y
                
                # Pake 0.0015 buat Active Braking yang gentle
                fwd_cmd = -down_err_y * 0.0015
                strafe_cmd = down_err_x * 0.0015
                
                # Limit speed so it brakes gently instead of overshooting
                fwd_cmd = clamp(fwd_cmd, -0.2, 0.2)
                strafe_cmd = clamp(strafe_cmd, -0.2, 0.2)
                
                print(f"[AUTOPILOT] [LANDING] Fwd: {fwd_cmd:.2f}, Strafe: {strafe_cmd:.2f}, Stable: {landing_ticks}/30, Z: {state.z:.2f}")
                # Turun pelan-pelan (0.3 m/s) sambil centering
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=0.3, yaw_deg_s=0.0)
                
                if abs(down_err_x) < 80 and abs(down_err_y) < 80:
                    landing_ticks += 1
                    if landing_ticks > 30: # Stabil 3 detik nyata (cukup, keburu buta kalau kelamaan)
                        print("[AUTOPILOT] Mendarat sempurna di titik tengah!")
                        await drone.action.land()
                        print("[AUTOPILOT] Menunggu 8 detik buat pendaratan fisik sebelum Auto-Reset...")
                        await asyncio.sleep(8)
                        # import os
                        # os.system("./respawn.sh")
                        break
                else:
                    landing_ticks = 0
                    timeout_counter = 0
            else:
                timeout_counter += 1
                # FORCE LAND: Kalau udah terlalu rendah, kamera bawah pasti buta. Paksa mendarat!
                if state.z > -0.4 and has_seen_target:
                    print(f"[AUTOPILOT] Ketinggian kritis ({state.z:.2f}m)! Kamera bawah buta. FORCE LANDING!")
                    await drone.action.land()
                    print("[AUTOPILOT] Menunggu 8 detik buat pendaratan fisik sebelum Auto-Reset...")
                    await asyncio.sleep(8)
                    # import os
                    # os.system("./respawn.sh")
                    break
                elif timeout_counter > 50:
                    print("[AUTOPILOT] Landing Pad Hilang! Kembali ke FIND_LANDING_PAD...")
                    state_phase = "FIND_LANDING_PAD"
                    timeout_counter = 0
                    landing_ticks = 0
                    blind_start_x = state.x
                    blind_start_y = state.y
                elif has_seen_target:
                    # FALLBACK MEMORY DOWN CAMERA
                    fwd_cmd = clamp(-last_down_err_y * 0.0015, -0.2, 0.2)
                    strafe_cmd = clamp(last_down_err_x * 0.0015, -0.2, 0.2)
                    print(f"[AUTOPILOT] Landing Pad Flicker! Terbang balik pake memori... Fwd: {fwd_cmd:.2f}, Z: {state.z:.2f}")
                    await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=0.3, yaw_deg_s=0.0)
                else:
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
