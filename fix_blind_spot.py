import re

with open("autopilot.py", "r") as f:
    code = f.read()

# Replace timeout_counter > 50 with > 80
# And replace yaw_deg_s=mem_yaw with yaw_deg_s=0.0 for the blind spot creep
# We will do this carefully using regex

def fix_blind_spot(match):
    block = match.group(0)
    block = block.replace("timeout_counter > 50", "timeout_counter > 80")
    block = block.replace("yaw_deg_s=mem_yaw", "yaw_deg_s=0.0")
    # Comment out mem_yaw calculation if it exists
    block = re.sub(r'(\s+)mem_yaw = last_front_err_x \* kp_yaw', r'\1# mem_yaw disabled during blind spot to prevent spiraling', block)
    return block

# The blind spot block looks like:
# elif has_seen_target:
#     ...
#     if timeout_counter > 50:
#     ...
#     else:
#         mem_yaw = last_front_err_x * kp_yaw
#         await flight.send_body_velocity(drone, forward_m_s=0.3, right_m_s=0.0, down_m_s=..., yaw_deg_s=mem_yaw)

pattern = r'elif has_seen_target:.*?else:\s+mem_yaw = last_front_err_x \* kp_yaw\s+await flight\.send_body_velocity\([^)]+\)'

code = re.sub(pattern, fix_blind_spot, code, flags=re.DOTALL)

with open("autopilot.py", "w") as f:
    f.write(code)
