import re

with open("autopilot.py", "r") as f:
    code = f.read()

# 1. Fix UnboundLocalError for mem_yaw
# Since I commented out `mem_yaw = last_front_err_x * kp_yaw`, I must comment out the clamping too.
code = re.sub(r'(\s+)mem_yaw = max\(-15\.0, min\(15\.0, mem_yaw\)\)', r'\1# mem_yaw = max(-15.0, min(15.0, mem_yaw))', code)

# 2. Fix the Aruco 1 false positive in Early Exit
# We'll require has_seen_target to be True (meaning it has seen the straight line at least once)
early_exit_down = 'if down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area"]:'
early_exit_down_fix = 'if has_seen_target and down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area"]:'

early_exit_front = 'elif front_status == "LOCKED" and front_class == "Aruco Area":'
early_exit_front_fix = 'elif has_seen_target and front_status == "LOCKED" and front_class == "Aruco Area":'

code = code.replace(early_exit_down, early_exit_down_fix)
code = code.replace(early_exit_front, early_exit_front_fix)

# But wait, FIND_ARUCO_1 and others use the same string for down_status check!
# We only want to replace it inside FOLLOW_LINE_TO_WP2.
# Let's revert and do it safely.
