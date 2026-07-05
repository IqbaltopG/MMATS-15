with open("autopilot.py", "r") as f:
    code = f.read()

# Restore the mem_yaw calculation that was mistakenly commented out
bad_block = """                    # mem_yaw disabled during blind spot to prevent spiraling
                    mem_yaw = max(-15.0, min(15.0, mem_yaw))"""

good_block = """                    mem_yaw = last_front_err_x * kp_yaw
                    mem_yaw = max(-15.0, min(15.0, mem_yaw))"""

code = code.replace(bad_block, good_block)

# Fix the early exits in FOLLOW_LINE_TO_WP2 to prevent false triggers on Aruco 1
early_exit_down = """            if down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area"]:
                print("[AUTOPILOT] WP2 (Aruco 2) Terlihat di Bawah! Langsung Centering...")"""

early_exit_down_fix = """            # ONLY early exit if we have seen the straight line first (meaning we left Aruco 1)
            if has_seen_target and down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area"]:
                print("[AUTOPILOT] WP2 (Aruco 2) Terlihat di Bawah! Langsung Centering...")"""

early_exit_front = """            elif front_status == "LOCKED" and front_class == "Aruco Area":
                print("[AUTOPILOT] Aruco 2 Terlihat di Depan! Beralih mencari Aruco 2...")"""

early_exit_front_fix = """            elif has_seen_target and front_status == "LOCKED" and front_class == "Aruco Area":
                print("[AUTOPILOT] Aruco 2 Terlihat di Depan! Beralih mencari Aruco 2...")"""

code = code.replace(early_exit_down, early_exit_down_fix)
code = code.replace(early_exit_front, early_exit_front_fix)

with open("autopilot.py", "w") as f:
    f.write(code)
