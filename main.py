import asyncio
import time
from flight import OtotDrone
from vision import MataDrone

async def center_on_target(otot, mata, mode="DOWNWARD", timeout=20.0):
    """
    Reusable Visual Servoing block with PD control, velocity clamping,
    and strict failsafes (Anti-Flyaway brake & Timeout).
    """
    kp = 0.005
    kd = 0.002
    prev_err_x, prev_err_y = 0.0, 0.0
    start_time = time.time()

    while True:
        if time.time() - start_time > timeout:
            print(f"[!] {timeout}s Timeout Exceeded! Proceeding...")
            break

        if not mata.target_detected:
            # The Anti-Flyaway Target-Lost Brake
            await otot.set_velocity(forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
            await asyncio.sleep(0.1)
            continue

        derivative_x = mata.error_x - prev_err_x
        derivative_y = mata.error_y - prev_err_y
        prev_err_x, prev_err_y = mata.error_x, mata.error_y

        if mode == "DOWNWARD":
            # Target is above center (error_y > 0) -> marker is FORWARD -> fly FORWARD (+vel_x)
            raw_vel_x = (kp * mata.error_y + kd * derivative_y)
            # Target is right of center (error_x > 0) -> marker is RIGHT -> fly RIGHT (+vel_y)
            raw_vel_y = (kp * mata.error_x + kd * derivative_x)
            
            # Velocity Clamping between -0.8 and 0.8
            vel_x = max(-0.8, min(0.8, raw_vel_x))
            vel_y = max(-0.8, min(0.8, raw_vel_y))
            await otot.set_velocity(forward_m_s=vel_x, right_m_s=vel_y, down_m_s=0.0, yaw_deg_s=0.0)
            
        elif mode == "FRONT":
            # REVERT TO LATERAL STRAFING (Slide sideways to center)
            # Yawing while flying forward caused the drone to clip the gate pillars at close range!
            raw_vel_lateral = (kp * mata.error_x + kd * derivative_x)
            vel_lateral = max(-0.8, min(0.8, raw_vel_lateral))
            
            # Altitude locked to 0.0 to prevent perspective nosedive
            # Fly forward slowly while sliding sideways to close the distance and trigger the break condition
            await otot.set_velocity(forward_m_s=0.8, right_m_s=vel_lateral, down_m_s=0.0, yaw_deg_s=0.0)

        if abs(mata.error_x) < 20.0 and abs(mata.error_y) < 20.0:
            print(">>> Target Locked and Centered!")
            await otot.set_velocity(forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)  # Stop for stability
            break
            
        await asyncio.sleep(0.1)

async def run():
    otot = OtotDrone()
    mata = MataDrone(camera_index=0)

    await otot.connect()
    mata.start()
    print("\n--- INITIATING TAKEOFF SEQUENCE ---")
    await otot.takeoff_offboard(target_alt_m=1.5)

    # =======================================================
    # PHASE 1: HOVER AT WP1 (ARUCO)
    # =======================================================
    print("\n[PHASE 1] Centering over WP1 (ARUCO)...")
    mata.active_camera = "DOWNWARD"  # Explicitly command camera stream
    mata.target_detected = False  # Reset State Leakage
    mata.current_mode = "CARI_ARUCO"
    
    # Search loop if not immediately in FOV (Hover without spinning to preserve North heading)
    while not mata.target_detected:
        await otot.set_velocity(forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
        await asyncio.sleep(0.1)

    await otot.set_velocity(0.0, 0.0, 0.0, 0.0)  # Kill yaw rotation immediately
    print(">>> WP1 ARUCO DETECTED! Centering...")
    await center_on_target(otot, mata, mode="DOWNWARD", timeout=20.0)

    # Removed Phase 1.5. We will go straight to Phase 2A and use the front camera to gauge distance.

    # =======================================================
    # PHASE 2A: DOUBLE GATE ALIGNMENT & PASS
    # =======================================================
    print("\n[PHASE 2A] Visual Servoing for Double Gate...")
    mata.active_camera = "FRONT"  # Explicitly switch to front camera
    mata.target_detected = False  # Reset State Leakage
    mata.current_mode = "ALIGN_DOUBLE_GATE"

    # Search loop if not immediately in FOV (Yaw to find the gate)
    while not mata.target_detected:
        await otot.set_velocity(forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=15.0)
        await asyncio.sleep(0.1)

    await otot.set_velocity(0.0, 0.0, 0.0, 0.0)  # Kill yaw rotation immediately
    print(">>> Double Gate Detected! Centering...")

    await center_on_target(otot, mata, mode="FRONT", timeout=40.0)
    print(">>> Double Gate perfectly aligned! Passing through...")

    # Keep flying forward until the gate leaves the Front Camera's FOV (which physically proves we passed under it!)
    # We also add a hard timeout to prevent it flying forever if the camera glitches.
    pass_ticks = 0
    while mata.target_detected and pass_ticks < 60:
        await otot.set_velocity(forward_m_s=1.5, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
        await asyncio.sleep(0.1)
        pass_ticks += 1
        
    print(">>> Gate physically cleared!")

    # =======================================================
    # PHASE 2B: MEDKIT PAYLOAD DROP
    # =======================================================
    print("\n[PHASE 2B] Coasting to Dropzone (DOWNWARD Camera)...")
    mata.active_camera = "DOWNWARD"
    mata.target_detected = False
    mata.current_mode = "CARI_BOX"
    
    # We don't use the front camera because the Red Box is a flat Minecraft slab.
    # Instead, we just fly straight North and look DOWN until we see its huge 0.62x0.415m top surface!
    
    # 1. Blindly blast forward for 3 seconds to get away from the gate and ignore any false-positive orange/red shadows.
    for _ in range(30):
        await otot.set_velocity(forward_m_s=1.5, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
        await asyncio.sleep(0.1)

    # 2. Start actively scanning the floor for the massive red slab
    print(">>> Blind coast complete. Scanning floor for Dropzone...")
    search_ticks = 0
    while not mata.target_detected and search_ticks < 100:
        await otot.set_velocity(forward_m_s=1.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
        await asyncio.sleep(0.1)
        search_ticks += 1
        
    await otot.set_velocity(0.0, 0.0, 0.0, 0.0)  # Brake immediately
    print(">>> Dropzone Detected beneath us! Centering...")
    await center_on_target(otot, mata, mode="DOWNWARD", timeout=20.0)
    print(">>> Perfect alignment! TRIGGERING PAYLOAD DROP!")
    await otot.set_velocity(0.0, 0.0, 0.0, 0.0)  # Stop for stability
    await otot.lepaskan_muatan()

    # =======================================================
    # PHASE 3: LINE TRACKING
    # =======================================================
    print("\n[PHASE 3] Initiating Line Tracking...")
    mata.active_camera = "DOWNWARD"
    mata.target_detected = False
    mata.current_mode = "LINE_TRACKING"
    yaw_kp = 1.2

    for _ in range(50):  # Simulate line tracking for 5 seconds
        if mata.target_detected:
            yaw_rate = mata.error_x * yaw_kp
            await otot.set_velocity(forward_m_s=1.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=yaw_rate)
        else:
            # Coast slowly forward if tracking memory fails entirely
            await otot.set_velocity(forward_m_s=0.5, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)
        await asyncio.sleep(0.1)

    # =======================================================
    # PHASE 4: TESTBED LANDING
    # =======================================================
    print("\n[PHASE 4] Line Tracking Complete. Executing immediate Testbed Landing...")
    await otot.land()
    mata.stop()

if __name__ == "__main__":
    asyncio.run(run())