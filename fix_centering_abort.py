with open("autopilot.py", "r") as f:
    code = f.read()

# Replace all occurrences of aborting back to FIND_* during centering
# We will just change the state_phase reassignment into a pass/hover.

# 1. CENTER_ARUCO_1
block_1_old = """                    if timeout_counter > 50:
                        print("[AUTOPILOT] WP1 Hilang! Kembali ke FIND_ARUCO_1")
                        state_phase = "FIND_ARUCO_1"
                        timeout_counter = 0
                        has_seen_target = False"""
block_1_new = """                    if timeout_counter > 50:
                        print("[AUTOPILOT] WP1 Hilang! Hovering di tempat...")
                        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)"""
code = code.replace(block_1_old, block_1_new)

# 2. CENTER_ARUCO_2
block_2_old = """                    if timeout_counter > 50:
                        print("[AUTOPILOT] WP2 Hilang! Kembali ke FIND_ARUCO_2")
                        state_phase = "FIND_ARUCO_2"
                        timeout_counter = 0
                        has_seen_target = False"""
block_2_new = """                    if timeout_counter > 50:
                        print("[AUTOPILOT] WP2 Hilang! Hovering di tempat...")
                        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=0.0)"""
code = code.replace(block_2_old, block_2_new)

# 3. CENTER_DROPBOX
block_3_old = """                    if timeout_counter > 50:
                        print("[AUTOPILOT] Drop Box Hilang dari kamera bawah! Kembali ke FIND_DROPBOX")
                        state_phase = "FIND_DROPBOX"
                        timeout_counter = 0
                        has_seen_target = False"""
block_3_new = """                    if timeout_counter > 50:
                        print("[AUTOPILOT] Drop Box Hilang dari kamera bawah! Hovering di tempat...")
                        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)"""
code = code.replace(block_3_old, block_3_new)

# 4. PRECISION_LANDING
block_4_old = """                    if timeout_counter > 50:
                        print("[AUTOPILOT] Landing Pad Hilang! Kembali ke FIND_LANDING_PAD")
                        state_phase = "FIND_LANDING_PAD"
                        timeout_counter = 0
                        has_seen_target = False"""
block_4_new = """                    if timeout_counter > 50:
                        print("[AUTOPILOT] Landing Pad Hilang! Hovering di tempat...")
                        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)"""
code = code.replace(block_4_old, block_4_new)

with open("autopilot.py", "w") as f:
    f.write(code)
