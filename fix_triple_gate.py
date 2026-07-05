import re

with open('/home/ambatron/DRONE/autopilot.py', 'r') as f:
    code = f.read()

# Fix FIND_TRIPLE_GATE_1
find_tg1_old = """        elif state_phase == "FIND_TRIPLE_GATE_1":
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
                    # Harus udah deket (Area > 10000) DAN udah di tengah (X error < 30) biar nggak nge-punch pas miring
                    if last_front_area > 10000 and abs(last_front_err_x) < 30:
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
                    has_seen_target = False"""

find_tg1_new = """        elif state_phase == "FIND_TRIPLE_GATE_1":
            if front_status == "LOCKED" and front_class == "Tripple Gate":
                has_seen_target = True
                last_front_area = front_area
                last_front_err_y = front_err_y
                last_front_err_x = front_err_x
                yaw_cmd = front_err_x * kp_yaw
                
                if front_area < 12000:
                    fwd_cmd = 0.8
                    z_err = -0.8 - DRONE_Z
                    up_cmd = max(-0.5, min(0.5, z_err * 0.5))
                else:
                    if abs(front_err_x) > 40:
                        fwd_cmd = 0.0
                        up_cmd = (front_err_y + 20) * kp_up
                    else:
                        fwd_cmd = 0.8
                        z_err = -0.8 - DRONE_Z
                        up_cmd = max(-0.5, min(0.5, z_err * 0.5))
                    
                print(f"[AUTOPILOT] [TRIPLE GATE 1] Centering (Area: {front_area}). Fwd: {fwd_cmd}, Yaw: {yaw_cmd:.2f}, Z: {up_cmd:.2f}")
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
            else:
                if has_seen_target:
                    if last_front_area > 10000 and abs(last_front_err_x) < 30:
                        print("[AUTOPILOT] Memasuki Lorong Triple Gate 1! Berpindah ke PUNCH_TRIPLE_GATE_1")
                        state_phase = "PUNCH_TRIPLE_GATE_1"
                        blind_start_x = DRONE_X
                        blind_start_y = DRONE_Y
                        continue
                    else:
                        print(f"[AUTOPILOT] Triple Gate 1 hilang dari jauh (Area: {last_front_area}). Fallback Hover & Yaw!")
                
                # Hover in place while rotating to find the gate
                mem_yaw = last_front_err_x * kp_yaw
                mem_yaw = max(-15.0, min(15.0, mem_yaw))
                await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=mem_yaw)

        elif state_phase == "PUNCH_TRIPLE_GATE_1":
            dist_flown = math.sqrt((DRONE_X - blind_start_x)**2 + (DRONE_Y - blind_start_y)**2)
            if dist_flown < 3.8: # PUNCH THROUGH TUNNEL
                strafe_cmd = 0.0
                if LIDAR_LEFT_DIST < 4.9 or LIDAR_RIGHT_DIST < 4.9:
                    strafe_cmd = (LIDAR_RIGHT_DIST - LIDAR_LEFT_DIST) * 0.05
                z_err = -0.8 - DRONE_Z
                up_cmd = max(-0.5, min(0.5, z_err * 0.5))
                
                print(f"[AUTOPILOT] [TRIPLE GATE 1] Blind Punch INS! Jarak: {dist_flown:.2f}/3.8m, Lidar: {strafe_cmd:.2f}")
                await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
            else:
                print("[AUTOPILOT] Keluar dari Triple Gate 1! Nyari Red Box buat Drop...")
                state_phase = "FIND_DROPBOX"
                has_seen_target = False"""

code = code.replace(find_tg1_old, find_tg1_new)

# Fix TRIPLE_GATE_2
find_tg2_old = """        elif state_phase == "TRIPLE_GATE_2":
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
                    # Harus udah deket (Area > 10000) DAN udah di tengah (X error < 30) biar nggak nge-punch pas miring
                    if last_front_area > 10000 and abs(last_front_err_x) < 30:
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
                    has_seen_target = False"""

find_tg2_new = """        elif state_phase == "TRIPLE_GATE_2":
            if front_status == "LOCKED" and front_class == "Tripple Gate":
                has_seen_target = True
                last_front_area = front_area
                last_front_err_y = front_err_y
                last_front_err_x = front_err_x
                yaw_cmd = front_err_x * kp_yaw
                
                if front_area < 12000:
                    fwd_cmd = 0.8
                    z_err = -0.8 - DRONE_Z
                    up_cmd = max(-0.5, min(0.5, z_err * 0.5))
                else:
                    if abs(front_err_x) > 40:
                        fwd_cmd = 0.0
                        up_cmd = (front_err_y + 20) * kp_up
                    else:
                        fwd_cmd = 0.8
                        z_err = -0.8 - DRONE_Z
                        up_cmd = max(-0.5, min(0.5, z_err * 0.5))
                        
                print(f"[AUTOPILOT] [TRIPLE GATE 2] Centering (Area: {front_area}). Fwd: {fwd_cmd}, Yaw: {yaw_cmd:.2f}, Z: {up_cmd:.2f}")
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
            else:
                if has_seen_target:
                    if last_front_area > 10000 and abs(last_front_err_x) < 30:
                        print("[AUTOPILOT] Memasuki Lorong Triple Gate 2! Berpindah ke PUNCH_TRIPLE_GATE_2")
                        state_phase = "PUNCH_TRIPLE_GATE_2"
                        blind_start_x = DRONE_X
                        blind_start_y = DRONE_Y
                        continue
                    else:
                        print(f"[AUTOPILOT] Triple Gate 2 hilang dari jauh (Area: {last_front_area}). Fallback Hover & Yaw!")
                
                mem_yaw = last_front_err_x * kp_yaw
                mem_yaw = max(-15.0, min(15.0, mem_yaw))
                await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=mem_yaw)

        elif state_phase == "PUNCH_TRIPLE_GATE_2":
            dist_flown = math.sqrt((DRONE_X - blind_start_x)**2 + (DRONE_Y - blind_start_y)**2)
            if dist_flown < 3.8:
                strafe_cmd = 0.0
                if LIDAR_LEFT_DIST < 4.9 or LIDAR_RIGHT_DIST < 4.9:
                    strafe_cmd = (LIDAR_RIGHT_DIST - LIDAR_LEFT_DIST) * 0.05
                z_err = -0.8 - DRONE_Z
                up_cmd = max(-0.5, min(0.5, z_err * 0.5))
                
                print(f"[AUTOPILOT] [TRIPLE GATE 2] Blind Punch INS! Jarak: {dist_flown:.2f}/3.8m, Lidar: {strafe_cmd:.2f}")
                await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
            else:
                print("[AUTOPILOT] Lolos Triple Gate 2! Mencari Aruco 3...")
                state_phase = "FIND_ARUCO_3"
                has_seen_target = False"""

code = code.replace(find_tg2_old, find_tg2_new)

with open('/home/ambatron/DRONE/autopilot.py', 'w') as f:
    f.write(code)
print("Done refactoring Triple Gates!")
