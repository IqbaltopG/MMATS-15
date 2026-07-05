import re

with open("autopilot.py", "r") as f:
    code = f.read()

# 1. Inject global_climb_cmd at the top of the loop
inject_str = """        down_err_x = TARGET_STATE_DOWN.get("error_x", 0)
        down_err_y = TARGET_STATE_DOWN.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - DRONE_Z
        global_climb_cmd = max(-0.5, min(0.5, z_err_15 * 0.5))"""

code = code.replace("""        down_err_x = TARGET_STATE_DOWN.get("error_x", 0)
        down_err_y = TARGET_STATE_DOWN.get("error_y", 0)""", inject_str)

# 2. Extract the block for Phase 4, 4A, 4B (from FOLLOW_LINE_TO_WP2 to the end of CENTER_ARUCO_2)
# We will split the code at "PHASE 4:" and "PHASE 5:"
parts = code.split("# PHASE 5: YAW_LEFT_TRIPLE_1")
if len(parts) == 2:
    pre_phase_5 = parts[0]
    post_phase_5 = parts[1]
    
    parts_2 = pre_phase_5.split("# PHASE 4: FOLLOW_LINE_TO_WP2")
    if len(parts_2) == 2:
        pre_phase_4 = parts_2[0]
        phase_4_block = parts_2[1]
        
        # Replace down_m_s=0.0 with down_m_s=global_climb_cmd inside the block
        phase_4_block = phase_4_block.replace("down_m_s=0.0", "down_m_s=global_climb_cmd")
        
        code = pre_phase_4 + "# PHASE 4: FOLLOW_LINE_TO_WP2" + phase_4_block + "# PHASE 5: YAW_LEFT_TRIPLE_1" + post_phase_5

with open("autopilot.py", "w") as f:
    f.write(code)
