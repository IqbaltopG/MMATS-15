import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError

async def arm_and_takeoff(drone: System, altitude_m=1.5):
    """
    Takeoff pintar dengan sistem fallback dan pemantauan sensor ketinggian (Barometer/Estimator).
    """
    print("[FLIGHT] Membersihkan parameter bypass lama...")
    
    print("[FLIGHT] Menunggu kesiapan sistem dan Arming Motors...")
    while True:
        try:
            await drone.action.arm()
            print("[FLIGHT] Arming SUKSES!")
            break
        except Exception as e:
            print(f"[FLIGHT] Gagal Arming (EKF belum siap): {e}. Retrying in 1s...")
            await asyncio.sleep(1)

    # Cara 1: Coba Takeoff otomatis bawaan
    print(f"[FLIGHT] Mencoba Auto-Takeoff ke {altitude_m}m...")
    try:
        await drone.action.set_takeoff_altitude(altitude_m)
        await drone.action.takeoff()
    except Exception as e:
        print(f"[FLIGHT] Auto-Takeoff Ditolak ({e}). Beralih ke Manual Offboard Z-Thrust!")
        
        # Mulai setpoint Offboard
        print("[FLIGHT] Menunggu Offboard Mode aktif...")
        while True:
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            try:
                await drone.offboard.start()
                print("[FLIGHT] Offboard Mode AKTIF!")
                break
            except OffboardError as err:
                print(f"[FLIGHT] Offboard ditolak: {err}. Retrying in 1s...")
                await asyncio.sleep(1)
            
        print("[FLIGHT] Memberikan dorongan Z ke atas (-1.5 m/s)...")

    print("[FLIGHT] Memantau sensor ketinggian (Barometer/Lidar)...")
    # Smart Altitude Check: Loop sampai ketinggian tercapai
    async for position in drone.telemetry.position():
        # HARUS DIKIRIM TERUS-MENERUS! Kalau cuma dikali 1x, PX4 Offboard bakal Timeout (0.5s) dan masuk Hold Mode!
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, -1.5, 0.0))
        
        alt = position.relative_altitude_m
        print(f"[FLIGHT] Ketinggian saat ini: {alt:.2f} m")
        if alt >= altitude_m - 0.2: # Toleransi 20cm
            print("[FLIGHT] Ketinggian target berhasil dicapai!")
            break
        await asyncio.sleep(0.1)

    print("[FLIGHT] Mengambil alih dengan Offboard mode (Hover)...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except:
        pass
        
    await asyncio.sleep(2) # Stabilisasi final sebelum mutar 180

async def hover(drone: System):
    """
    Berhenti di udara (Brake / Hover).
    """
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )

async def send_body_velocity(drone: System, forward_m_s, right_m_s, down_m_s, yaw_deg_s):
    """
    Fungsi modular utama untuk bermanuver di Body Frame.
    """
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(forward_m_s, right_m_s, down_m_s, yaw_deg_s)
    )
