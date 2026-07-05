with open("autopilot.py", "r") as f:
    code = f.read()

stutter_logic_1 = """                    else:
                        # Stutter creep to level pitch and scan straight down
                        fwd_creep = 0.3 if timeout_counter % 20 < 10 else 0.0
                        await flight.send_body_velocity(drone, forward_m_s=fwd_creep, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)"""

stutter_logic_2 = """                    else:
                        # mem_yaw disabled during blind spot to prevent spiraling
                        # Stutter creep to level pitch and scan straight down
                        fwd_creep = 0.3 if timeout_counter % 20 < 10 else 0.0
                        await flight.send_body_velocity(drone, forward_m_s=fwd_creep, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=0.0)"""

# FIND_ARUCO_1
code = code.replace("""                    else:
                        await flight.send_body_velocity(drone, forward_m_s=0.3, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)""", stutter_logic_1)

# FIND_ARUCO_2
code = code.replace("""                    else:
                        # mem_yaw disabled during blind spot to prevent spiraling
                        await flight.send_body_velocity(drone, forward_m_s=0.3, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=0.0)""", stutter_logic_2)

# FIND_DROPBOX
code = code.replace("""                    else:
                        # mem_yaw disabled during blind spot to prevent spiraling
                        await flight.send_body_velocity(drone, forward_m_s=0.3, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)""", 
"""                    else:
                        # mem_yaw disabled during blind spot to prevent spiraling
                        # Stutter creep to level pitch and scan straight down
                        fwd_creep = 0.3 if timeout_counter % 20 < 10 else 0.0
                        await flight.send_body_velocity(drone, forward_m_s=fwd_creep, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)""")

# FIND_LANDING_PAD
code = code.replace("""                    else:
                        # mem_yaw disabled during blind spot to prevent spiraling
                        await flight.send_body_velocity(drone, forward_m_s=0.3, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)""", 
"""                    else:
                        # mem_yaw disabled during blind spot to prevent spiraling
                        # Stutter creep to level pitch and scan straight down
                        fwd_creep = 0.3 if timeout_counter % 20 < 10 else 0.0
                        await flight.send_body_velocity(drone, forward_m_s=fwd_creep, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)""")

with open("autopilot.py", "w") as f:
    f.write(code)
