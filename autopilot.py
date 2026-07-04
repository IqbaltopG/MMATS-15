import asyncio
import json
import math
from mavsdk import System
import flight

# GLOBAL TELEMETRY
DRONE_X = 0.0
DRONE_Y = 0.0
DRONE_Z = 0.0
LIDAR_LEFT_DIST = 5.0
LIDAR_RIGHT_DIST = 5.0

async def telemetry_task(drone):
    global DRONE_X, DRONE_Y, DRONE_Z
    async for pos_vel in drone.telemetry.position_velocity_ned():
        DRONE_X = pos_vel.position.north_m
        DRONE_Y = pos_vel.position.east_m
        DRONE_Z = pos_vel.position.down_m

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

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - DRONE_Z
        global_climb_cmd = max(-0.5, min(0.5, z_err_15 * 0.5))

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
                    z_err = -0.8 - DRONE_Z
                    up_cmd = max(-0.5, min(0.5, z_err * 0.5)) # Active Z=0.8m Lock
                else:
                    if not altitude_locked:
                        # Belum nge-lock, kita hover dan presisi-kan
                        strafe_cmd = front_err_x * kp_yaw
                        up_cmd = (front_err_y + 150) * kp_up
                        up_cmd = max(-0.6, min(0.6, up_cmd))
                        fwd_cmd = 0.0 # Berhenti maju buat nunggu stabil (anti-banteng)
                        
                        # Tolerance dilebarkan sedikit (30 pixel) biar ga nyangkut infinite loop
                        if abs(front_err_x) < 30 and abs(front_err_y + 150) < 30:
                            altitude_locked = True
                            print("[AUTOPILOT] [GATE 1] Centered! ALTITUDE LOCKED. Going Pitbull...")
                    else:
                        # UDAH LOCK! Bodo amat sama error Y (Karna kalau maju drone nunduk dan bikin ilusi error Y)
                        z_err = -0.8 - DRONE_Z

                        up_cmd = max(-0.5, min(0.5, z_err * 0.5)) # Active Z=0.8m Lock
                        strafe_cmd = front_err_x * kp_yaw
                        if abs(front_err_x) > 40:
                            fwd_cmd = 0.0 # Kalo melenceng X-nya aja baru ngerem
                        else:
                            fwd_cmd = 0.8
                        
                    print(f"[AUTOPILOT] [GATE 1] Centering (Area: {front_area}). Strafe: {strafe_cmd:.2f}, Z: {up_cmd:.2f}, Lock: {altitude_locked}")

                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
            else:
                if has_seen_target:
                    # Syarat blind-punch: Harus udah deket banget (Area gede) atau bener-bener di tengah sebelum hilang
                    if (last_front_err_y < 20 and abs(last_front_err_x) < 30 and last_front_area > 20000) or last_front_area > 150000 or timeout_counter > 0:
                        if timeout_counter == 0:
                            blind_start_x = DRONE_X
                            blind_start_y = DRONE_Y
                        timeout_counter += 1
                    else:
                        print(f"[AUTOPILOT] Gawang 1 hilang dari jauh (Area: {last_front_area}, ErrX: {last_front_err_x}). Hovering...")
                        timeout_counter = 0
                
                dist_flown = math.sqrt((DRONE_X - blind_start_x)**2 + (DRONE_Y - blind_start_y)**2) if timeout_counter > 0 else 0
                
                if timeout_counter == 0:
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                elif dist_flown < 3.2: # PUNCH THROUGH INS
                    z_err = -0.8 - DRONE_Z
                    up_cmd = max(-0.5, min(0.5, z_err * 0.5))
                    
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
                # Kembali ke ketinggian 1.5m (DRONE_Z = -1.5)
                z_err = -1.5 - DRONE_Z
                climb_cmd = max(-0.5, min(0.5, z_err * 0.5))
                
                if front_status == "LOCKED" and front_class == "Aruco Area":
                    blind_start_x = DRONE_X
                    blind_start_y = DRONE_Y
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
                    dist_flown = math.sqrt((DRONE_X - blind_start_x)**2 + (DRONE_Y - blind_start_y)**2)
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
                fwd_cmd = max(-0.2, min(0.2, fwd_cmd))
                strafe_cmd = max(-0.2, min(0.2, strafe_cmd))
                
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
                elif has_seen_target:
                    # FALLBACK MEMORY: Rebound brake
                    fwd_cmd = max(-0.2, min(0.2, -last_down_err_y * 0.0015))
                    strafe_cmd = max(-0.2, min(0.2, last_down_err_x * 0.0015))
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
                    blind_start_x = DRONE_X
                    blind_start_y = DRONE_Y
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
                    dist_flown = math.sqrt((DRONE_X - blind_start_x)**2 + (DRONE_Y - blind_start_y)**2)
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
                
                fwd_cmd = max(-0.2, min(0.2, fwd_cmd))
                strafe_cmd = max(-0.2, min(0.2, strafe_cmd))
                
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
                elif has_seen_target:
                    # FALLBACK MEMORY: Rebound brake
                    fwd_cmd = max(-0.2, min(0.2, -last_down_err_y * 0.0015))
                    strafe_cmd = max(-0.2, min(0.2, last_down_err_x * 0.0015))
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
                timeout_counter = 0
                last_front_area = front_area
                last_front_err_y = front_err_y
                last_front_err_x = front_err_x
                yaw_cmd = front_err_x * kp_yaw
                
                # Logic: Kalo masih jauh (Area < 12000), maju lurus aja sambil nge-yaw. Hindari Vertical Centering palsu.
                if front_area < 12000:
                    fwd_cmd = 0.8
                    z_err = -0.8 - DRONE_Z
                    up_cmd = max(-0.5, min(0.5, z_err * 0.5)) # Active Z=0.8m Lock
                else:
                    if abs(front_err_x) > 40:
                        fwd_cmd = 0.0
                        up_cmd = (front_err_y + 20) * kp_up
                    else:
                        fwd_cmd = 0.8
                        z_err = -0.8 - DRONE_Z
                        up_cmd = max(-0.5, min(0.5, z_err * 0.5)) # Active Z=0.8m Lock
                    
                print(f"[AUTOPILOT] [TRIPLE GATE 1] Centering (Area: {front_area}). Fwd: {fwd_cmd}, Yaw: {yaw_cmd:.2f}, Z: {up_cmd:.2f}")
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
            else:
                if has_seen_target:
                    # FALLBACK MEMORY: Cek ukuran area terakhir buat ngebedain "flicker dari jauh" vs "masuk lorong"
                    if last_front_area > 10000:
                        if timeout_counter == 0:
                            blind_start_x = DRONE_X
                            blind_start_y = DRONE_Y
                        timeout_counter += 1
                    else:
                        print(f"[AUTOPILOT] Triple Gate 1 hilang/flicker dari jauh (Area: {last_front_area}). Menggunakan Fallback Memory!")
                        timeout_counter = 0
                
                dist_flown = math.sqrt((DRONE_X - blind_start_x)**2 + (DRONE_Y - blind_start_y)**2) if timeout_counter > 0 else 0
                
                if timeout_counter == 0:
                    # FALLBACK MEMORY: Hover in place while rotating to find the gate!
                    mem_yaw = last_front_err_x * kp_yaw
                    mem_yaw = max(-15.0, min(15.0, mem_yaw)) # Batasi yaw biar ga terlalu agresif
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=mem_yaw)
                elif dist_flown < 3.8: # PUNCH THROUGH TUNNEL (INS Jarak 3.8m)
                    # Repulsion Force Field (LiDAR)
                    strafe_cmd = 0.0
                    if LIDAR_LEFT_DIST < 4.9 or LIDAR_RIGHT_DIST < 4.9:
                        strafe_cmd = (LIDAR_RIGHT_DIST - LIDAR_LEFT_DIST) * 0.05
                    
                    # Barometer / INS Altitude Hold
                    z_err = -0.8 - DRONE_Z
                    up_cmd = max(-0.5, min(0.5, z_err * 0.5))
                    
                    if timeout_counter % 10 == 0:
                        print(f"[AUTOPILOT] [TRIPLE GATE 1] Blind Punch INS! Jarak: {dist_flown:.2f}/3.8m, Lidar Strafe: {strafe_cmd:.2f}, Z: {up_cmd:.2f}")
                        
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
                else:
                    print("[AUTOPILOT] Keluar dari Triple Gate 1! Nyari Red Box buat Drop...")
                    state_phase = "FIND_DROPBOX"
                    timeout_counter = 0
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
                z_err = -1.5 - DRONE_Z
                climb_cmd = max(-0.8, min(0.5, z_err * 0.8)) # Agresif naik

                # Rule 4: Flat Object Camera Handoff
                # Kalo drop box kelihatan di kamera depan, steer ke sana
                if front_status == "LOCKED" and front_class in ["Red Drop Box", "RedDrop Box"]:
                    blind_start_x = DRONE_X
                    blind_start_y = DRONE_Y
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
                    dist_flown = math.sqrt((DRONE_X - blind_start_x)**2 + (DRONE_Y - blind_start_y)**2)
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
                fwd_cmd = down_err_y * 0.0015
                strafe_cmd = down_err_x * 0.0015
                
                # Limit speed so it brakes instead of overshooting
                fwd_cmd = max(-0.2, min(0.2, fwd_cmd))
                strafe_cmd = max(-0.2, min(0.2, strafe_cmd))
                
                # Z dijaga ketat di -1.5
                z_err = -1.5 - DRONE_Z
                up_cmd = max(-0.5, min(0.5, z_err * 0.5))
                
                print(f"[AUTOPILOT] [DROP BOX] Centering (X:{down_err_x}, Y:{down_err_y}). Fwd:{fwd_cmd:.2f}, Strafe:{strafe_cmd:.2f}, Z:{up_cmd:.2f}")
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
                
                if abs(down_err_x) < 20 and abs(down_err_y) < 20:
                    timeout_counter += 1
                    
                    # Fast completion if Red Box is seen, slow fallback if only Aruco Area is seen
                    completion_threshold = 100 if down_class in ["Red Drop Box", "RedDrop Box"] else 200
                    
                    if timeout_counter > completion_threshold:
                        print("[AUTOPILOT] Medkit Dropped. Yaw Kanan nyari Triple Gate 2...")
                        state_phase = "YAW_RIGHT_TRIPLE_2"
                        timeout_counter = 0
                        has_seen_target = False
                else:
                    timeout_counter = 0
            else:
                timeout_counter += 1
                if timeout_counter > 50:
                    print("[AUTOPILOT] Drop Box Hilang dari kamera bawah! Kembali ke FIND_DROPBOX...")
                    state_phase = "FIND_DROPBOX"
                    timeout_counter = 0
                elif has_seen_target:
                    # FALLBACK MEMORY DOWN CAMERA: Terbang balik ke kordinat terakhir kali keliatan!
                    fwd_cmd = max(-0.2, min(0.2, last_down_err_y * 0.0015))
                    strafe_cmd = max(-0.2, min(0.2, last_down_err_x * 0.0015))
                    z_err = -1.5 - DRONE_Z
                    up_cmd = max(-0.5, min(0.5, z_err * 0.5))
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
                timeout_counter = 0
                last_front_area = front_area
                last_front_err_y = front_err_y
                last_front_err_x = front_err_x
                yaw_cmd = front_err_x * kp_yaw
                
                # Logic: Kalo masih jauh (Area < 12000), maju lurus aja sambil nge-yaw. Hindari Vertical Centering palsu.
                if front_area < 12000:
                    fwd_cmd = 0.8
                    z_err = -0.8 - DRONE_Z
                    up_cmd = max(-0.5, min(0.5, z_err * 0.5)) # Active Z=0.8m Lock
                else:
                    if abs(front_err_x) > 40:
                        fwd_cmd = 0.0
                        up_cmd = (front_err_y + 20) * kp_up
                    else:
                        fwd_cmd = 0.8
                        z_err = -0.8 - DRONE_Z
                        up_cmd = max(-0.5, min(0.5, z_err * 0.5)) # Active Z=0.8m Lock
                        
                print(f"[AUTOPILOT] [TRIPLE GATE 2] Centering (Area: {front_area}). Fwd: {fwd_cmd}, Yaw: {yaw_cmd:.2f}, Z: {up_cmd:.2f}")
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
            else:
                if has_seen_target:
                    # FALLBACK MEMORY: Cek ukuran area terakhir buat ngebedain "flicker dari jauh" vs "masuk lorong"
                    if last_front_area > 10000:
                        if timeout_counter == 0:
                            blind_start_x = DRONE_X
                            blind_start_y = DRONE_Y
                        timeout_counter += 1
                    else:
                        print(f"[AUTOPILOT] Triple Gate 2 hilang/flicker dari jauh (Area: {last_front_area}). Menggunakan Fallback Memory!")
                        timeout_counter = 0
                
                dist_flown = math.sqrt((DRONE_X - blind_start_x)**2 + (DRONE_Y - blind_start_y)**2) if timeout_counter > 0 else 0
                
                if timeout_counter == 0:
                    # FALLBACK MEMORY: Hover in place while rotating to find the gate!
                    mem_yaw = last_front_err_x * kp_yaw
                    mem_yaw = max(-15.0, min(15.0, mem_yaw))
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=mem_yaw)
                elif dist_flown < 3.8: # PUNCH THROUGH TUNNEL (INS Jarak 3.8m)
                    # Repulsion Force Field (LiDAR)
                    strafe_cmd = 0.0
                    if LIDAR_LEFT_DIST < 4.9 or LIDAR_RIGHT_DIST < 4.9:
                        strafe_cmd = (LIDAR_RIGHT_DIST - LIDAR_LEFT_DIST) * 0.05
                    
                    # Barometer / INS Altitude Hold
                    z_err = -0.8 - DRONE_Z
                    up_cmd = max(-0.5, min(0.5, z_err * 0.5))
                    
                    if timeout_counter % 10 == 0:
                        print(f"[AUTOPILOT] [TRIPLE GATE 2] Blind Punch INS! Jarak: {dist_flown:.2f}/3.8m, Lidar Strafe: {strafe_cmd:.2f}, Z: {up_cmd:.2f}")
                        
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
                else:
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
                
                # Logic: Mendekat dulu, baru Strafe untuk centering X dan Y (hindari top bar)
                if front_area < 25000:
                    fwd_cmd = 0.8
                    strafe_cmd = front_err_x * kp_yaw
                    z_err = -0.8 - DRONE_Z
                    up_cmd = max(-0.5, min(0.5, z_err * 0.5)) # Active Z=0.8m Lock
                else:
                    if not altitude_locked:
                        strafe_cmd = front_err_x * kp_yaw
                        # Turun ke bawah top bar
                        up_cmd = (front_err_y + 150) * kp_up
                        up_cmd = max(-0.6, min(0.6, up_cmd))
                        fwd_cmd = 0.0
                        
                        if abs(front_err_x) < 30 and abs(front_err_y + 150) < 30:
                            altitude_locked = True
                            print("[AUTOPILOT] [FINAL GATE 1] Centered! ALTITUDE LOCKED. Going Pitbull...")
                    else:
                        z_err = -0.8 - DRONE_Z

                        up_cmd = max(-0.5, min(0.5, z_err * 0.5)) # Active Z=0.8m Lock
                        strafe_cmd = front_err_x * kp_yaw
                        if abs(front_err_x) > 40:
                            fwd_cmd = 0.0
                        else:
                            fwd_cmd = 0.8
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
            else:
                if has_seen_target:
                    if (last_front_err_y < 20 and abs(last_front_err_x) < 30 and last_front_area > 20000) or last_front_area > 150000 or timeout_counter > 0:
                        if timeout_counter == 0:
                            blind_start_x = DRONE_X
                            blind_start_y = DRONE_Y
                        timeout_counter += 1
                    else:
                        timeout_counter = 0
                
                dist_flown = math.sqrt((DRONE_X - blind_start_x)**2 + (DRONE_Y - blind_start_y)**2) if timeout_counter > 0 else 0
                
                if timeout_counter == 0:
                    # FALLBACK MEMORY: Hover in place while rotating to find the gate!
                    mem_yaw = last_front_err_x * kp_yaw
                    mem_yaw = max(-15.0, min(15.0, mem_yaw))
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=mem_yaw)
                elif dist_flown < 2.5: # PUNCH THROUGH INS
                    z_err = -0.8 - DRONE_Z
                    up_cmd = max(-0.5, min(0.5, z_err * 0.5))
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
                else:
                    print(f"[AUTOPILOT] Lolos Final Gate 1! Mencari Final Gate 2...")
                    state_phase = "FIND_FINAL_GATE_2"
                    timeout_counter = 0
                    has_seen_target = False

        # ---------------------------------------------------------
        # PHASE 13: FIND_FINAL_GATE_2 (Mencari Single Gate kedua)
        # ---------------------------------------------------------
        elif state_phase == "FIND_FINAL_GATE_2":
            if front_status == "LOCKED" and front_class == "Single Gate" and front_area > 12000:
                print("[AUTOPILOT] Final Gate 2 Ditemukan! Proses Nembus...")
                state_phase = "PASS_FINAL_GATE_2"
                timeout_counter = 0
                has_seen_target = False
            elif front_status == "LOCKED" and front_class == "Single Gate":
                timeout_counter += 1
                climb_cmd = -0.3 if DRONE_Z > -1.0 else 0.0
                yaw_cmd = front_err_x * kp_yaw
                await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=yaw_cmd)
            else:
                timeout_counter += 1
                climb_cmd = -0.3 if DRONE_Z > -1.0 else 0.0
                await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                
        # ---------------------------------------------------------
        # PHASE 14: PASS_FINAL_GATE_2 (Tembus Single Gate 2)
        # ---------------------------------------------------------
        elif state_phase == "PASS_FINAL_GATE_2":
            if front_status == "LOCKED" and front_class == "Single Gate":
                has_seen_target = True
                timeout_counter = 0
                last_front_area = front_area
                last_front_err_x = front_err_x
                last_front_err_y = front_err_y
                
                if front_area < 25000:
                    fwd_cmd = 0.8
                    strafe_cmd = front_err_x * kp_yaw
                    z_err = -0.8 - DRONE_Z
                    up_cmd = max(-0.5, min(0.5, z_err * 0.5)) # Active Z=0.8m Lock
                else:
                    if not altitude_locked:
                        strafe_cmd = front_err_x * kp_yaw
                        up_cmd = (front_err_y + 150) * kp_up
                        up_cmd = max(-0.6, min(0.6, up_cmd))
                        fwd_cmd = 0.0
                        
                        if abs(front_err_x) < 30 and abs(front_err_y + 150) < 30:
                            altitude_locked = True
                            print("[AUTOPILOT] [FINAL GATE 2] Centered! ALTITUDE LOCKED. Going Pitbull...")
                    else:
                        z_err = -0.8 - DRONE_Z

                        up_cmd = max(-0.5, min(0.5, z_err * 0.5)) # Active Z=0.8m Lock
                        strafe_cmd = front_err_x * kp_yaw
                        if abs(front_err_x) > 40:
                            fwd_cmd = 0.0
                        else:
                            fwd_cmd = 0.8
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
            else:
                if has_seen_target:
                    if (last_front_err_y < 20 and abs(last_front_err_x) < 30 and last_front_area > 20000) or last_front_area > 150000 or timeout_counter > 0:
                        if timeout_counter == 0:
                            blind_start_x = DRONE_X
                            blind_start_y = DRONE_Y
                        timeout_counter += 1
                    else:
                        timeout_counter = 0
                
                dist_flown = math.sqrt((DRONE_X - blind_start_x)**2 + (DRONE_Y - blind_start_y)**2) if timeout_counter > 0 else 0
                
                if timeout_counter == 0:
                    # FALLBACK MEMORY: Hover in place while rotating to find the gate!
                    mem_yaw = last_front_err_x * kp_yaw
                    mem_yaw = max(-15.0, min(15.0, mem_yaw))
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=mem_yaw)
                elif dist_flown < 2.5: 
                    z_err = -0.8 - DRONE_Z
                    up_cmd = max(-0.5, min(0.5, z_err * 0.5))
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
                elif dist_flown < 4.2: 
                    z_err = -0.8 - DRONE_Z
                    up_cmd = max(-0.5, min(0.5, z_err * 0.5))
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=up_cmd - 0.8, yaw_deg_s=0.0)
                else:
                    print(f"[AUTOPILOT] Lolos Final Gate 2! Mencari Landing Pad...")
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
                z_err = -1.5 - DRONE_Z
                climb_cmd = max(-0.8, min(0.5, z_err * 0.8))
                
                # Rule 4: Handoff Kamera
                if front_status == "LOCKED" and front_class in ["Landing path", "Aruco"]:
                    blind_start_x = DRONE_X
                    blind_start_y = DRONE_Y
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
                    dist_flown = math.sqrt((DRONE_X - blind_start_x)**2 + (DRONE_Y - blind_start_y)**2)
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
                fwd_cmd = down_err_y * 0.0015
                strafe_cmd = down_err_x * 0.0015
                
                # Limit speed so it brakes gently instead of overshooting
                fwd_cmd = max(-0.2, min(0.2, fwd_cmd))
                strafe_cmd = max(-0.2, min(0.2, strafe_cmd))
                
                print(f"[AUTOPILOT] [LANDING] Fwd: {fwd_cmd:.2f}, Strafe: {strafe_cmd:.2f}, Tick: {timeout_counter}/50")
                # Turun pelan-pelan (0.3 m/s) sambil centering
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=0.3, yaw_deg_s=0.0)
                
                if abs(down_err_x) < 80 and abs(down_err_y) < 80:
                    timeout_counter += 1
                    if timeout_counter > 100: # Stabil 10 detik nyata (RTF 30%)
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
                timeout_counter += 1
                if timeout_counter > 50:
                    print("[AUTOPILOT] Landing Pad Hilang! Hovering di tempat...")
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
                elif has_seen_target:
                    # FALLBACK MEMORY DOWN CAMERA
                    fwd_cmd = max(-0.2, min(0.2, last_down_err_y * 0.0015))
                    strafe_cmd = max(-0.2, min(0.2, last_down_err_x * 0.0015))
                    print(f"[AUTOPILOT] Landing Pad Flicker! Terbang balik pake memori... Fwd: {fwd_cmd:.2f}")
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
