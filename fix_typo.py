import re

with open("autopilot.py", "r") as f:
    code = f.read()

# Revert Hover Climb in else blocks
hover_climb_block = """                    if abs(z_err) > 0.2:
                        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                    else:
                        await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)"""
code = code.replace(hover_climb_block, "                    await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)")

hover_climb_block_6 = """                    # Belum keliatan, HENTIKAN maju jika sedang climb (anti overshoot)!
                    if abs(z_err) > 0.2:
                        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                    else:
                        await flight.send_body_velocity(drone, forward_m_s=0.6, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)"""
code = code.replace(hover_climb_block_6, "                    # Belum keliatan, jalan lurus pelan sambil nanjak ke ketinggian operasi\n                    await flight.send_body_velocity(drone, forward_m_s=0.6, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)")

hover_climb_block_end = """                    if abs(z_err) > 0.2:
                        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                    else:
                        await flight.send_body_velocity(drone, forward_m_s=0.6, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)"""
code = code.replace(hover_climb_block_end, "                    await flight.send_body_velocity(drone, forward_m_s=0.6, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)")

# Fix Typos
code = code.replace('== "Red Drop Box"', 'in ["Red Drop Box", "RedDrop Box"]')

with open("autopilot.py", "w") as f:
    f.write(code)
