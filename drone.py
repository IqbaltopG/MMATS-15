import asyncio
from mavsdk import System
import math
import time

# --- KONFIGURASI TARGET OUTDOOR ---
# Target DPMO (Drop Muatan Outdoor)
titik_target = [
    (-1.259304, 116.862317), # Pojok 1
    (-1.258282, 116.862576), # Pojok 2
]
# Koordinat FLP (Tengah Lapangan) [cite: 116, 117, 118]
koordinat_flp = (-1.258800, 116.862450)

def get_distance_in_meters(coord1, coord2):
    """
    Menghitung jarak antara dua koordinat GPS dalam meter menggunakan formula Haversine.
    Perhitungan ini akurat untuk jarak di permukaan bola seperti Bumi.
    """
    R = 6371000.0  # Radius rata-rata Bumi dalam meter
    lat1, lon1 = coord1
    lat2, lon2 = coord2

    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlat_rad = lat2_rad - lat1_rad
    dlon_rad = math.radians(lon2) - math.radians(lon1)

    a = math.sin(dlat_rad / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon_rad / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c
    return distance

def get_bearing(coord1, coord2):
    """
    Menghitung sudut (yaw/heading) dari koordinat asal ke target.
    Output: Derajat 0-360 (0 = Utara, 90 = Timur, dst).
    """
    lat1, lon1 = coord1
    lat2, lon2 = coord2

    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    lon1_rad = math.radians(lon1)
    lon2_rad = math.radians(lon2)

    dlon_rad = lon2_rad - lon1_rad

    x = math.sin(dlon_rad) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - (math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad))

    initial_bearing = math.atan2(x, y)
    
    # Normalisasi ke format kompas 0-360 derajat
    bearing = (math.degrees(initial_bearing) + 360) % 360
    return bearing

async def lepaskan_muatan(drone, index):
    safe_index = index + 1 
    if safe_index > 6:
         print(f"Index {safe_index} out of bounds, skip actuation")
         return
    print(f"Release muatan ke-{safe_index}... Eksekusi!")
    try:
        await drone.action.set_actuator(safe_index, 1.0) 
        await asyncio.sleep(2) 
        await drone.action.set_actuator(safe_index, -1.0) 
        print("Servo locked kembali.")
    except Exception as e:
        print(f"FAILED to set actuator: {e}")
        print("LANJUT TERBANG, abaikan error aktuator di SITL!")

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate OK")
            break

    print("Mempersiapkan motor... (ARMING)")
    await drone.action.arm()
    print("Mengudara... (TakeOFF)")
    await drone.action.takeoff()
    await asyncio.sleep(15) # Jeda biar stabil di udara

    # ==========================================
    # [PHASE 1] MISI INDOOR (KERANJANG MERAH)
    # ==========================================
    # print("\n--- INITIATING INDOOR MISSION ---")
    
    # # Nanti di sini kita masukin script OpenCV lu.
    # # Buat sekarang, kita asumsikan drone udah disuruh maju 5 meter ke atas keranjang merah.
    # print("[!] Bergerak menuju Keranjang Merah (Simulasi)...")
    # await asyncio.sleep(3) # Pura-puranya lagi jalan ngelewatin lorong L [cite: 141]
    
    # # Jatuhin muatan indoor ke keranjang [cite: 142]
    # print("[!] Target Keranjang Merah Locked!")
    # await lepaskan_muatan(drone, 0) # Index 0 buat servo payload indoor
    
    # print("[!] Bergerak keluar melewati lorong IEG (Indoor Exit Gate)...")
    # await asyncio.sleep(3)
    # ==========================================

    # ==========================================
    # [PHASE 2] MISI OUTDOOR (KUBUS MERAH)
    # ==========================================
    print("\n--- INITIATING OUTDOOR MISSION ---")
    target_ketinggian_terbang_agl = 10.0 # 10 meter [cite: 144]
    
    async for terrain_info in drone.telemetry.position():
        # Dapatkan AMSL tanah dengan mengurangi ketinggian absolut saat ini dengan ketinggian relatif
        ground_amsl = terrain_info.absolute_altitude_m - terrain_info.relative_altitude_m
        break 

    target_alt_amsl = ground_amsl + target_ketinggian_terbang_agl

    # Mulai keliling ke pojok lapangan [cite: 144]
    # Mulai keliling ke pojok lapangan
    for i, (lat, lon) in enumerate(titik_target):
        
        # 1. Ambil posisi saat ini dulu SEBELUM ngegas ke target
        async for pos_awal in drone.telemetry.position():
            current_pos_awal = (pos_awal.latitude_deg, pos_awal.longitude_deg)
            break # Cukup ambil satu sampel data
            
        target_pos = (lat, lon)
        
        # 2. Hitung sudut hadap (Yaw) yang bener
        target_yaw = get_bearing(current_pos_awal, target_pos)
        
        print(f"OTW Target Outdoor {i+1}: {lat}, {lon} | Ketinggian: {target_alt_amsl} AMSL | Hadap: {target_yaw:.1f} derajat")
        
        # 3. Eksekusi dengan Yaw dinamis (Bukan 0.0 lagi!)
        await drone.action.goto_location(lat, lon, target_alt_amsl, target_yaw)
        
        # [WAJIB ADA] Loop penahan: Tunggu sampe jarak ke target < 2 meter
        last_print = 0
        async for position in drone.telemetry.position():
            current_pos = (position.latitude_deg, position.longitude_deg)
            target_pos = (lat, lon)
            distance = get_distance_in_meters(current_pos, target_pos)
            
            if time.time() - last_print >= 1.0:
                print(f"Jarak ke target {i+1}: {distance:.2f} meter")
                last_print = time.time()
                
            if distance < 2.0:
                print(f"[!] Tiba di Target Outdoor {i+1}")
                break # Keluar dari loop penahan ini kalau udah nyampe
        
        # Baru eksekusi payload setelah loop penahan di-break (sudah sampai)
        await lepaskan_muatan(drone, i + 1) 
        
    # MISI SELESAI: Landing di FLP
    print("\n--- MISI BERES: RTB (Return to Base) ---")
    print(f"Kembali ke FLP di {koordinat_flp[0]}, {koordinat_flp[1]}")

    # 1. Ambil posisi saat ini (posisi drop terakhir) sebelum pulang
    async for pos_pulang in drone.telemetry.position():
        current_pos_pulang = (pos_pulang.latitude_deg, pos_pulang.longitude_deg)
        break
        
    # 2. Hitung sudut hadap (Yaw) buat rute pulang
    rtb_yaw = get_bearing(current_pos_pulang, koordinat_flp)

    # 3. Eksekusi RTB dengan hidung ngadep target FLP (Bukan 0.0!)
    await drone.action.goto_location(koordinat_flp[0], koordinat_flp[1], target_alt_amsl, rtb_yaw)

    last_print = 0
    # ... (lanjut ke logic nunggu jarak buat landing kayak biasa)
    async for position in drone.telemetry.position():
        current_pos = (position.latitude_deg, position.longitude_deg)
        distance = get_distance_in_meters(current_pos, koordinat_flp)
        
        if time.time() - last_print >= 1.0:
            print(f"Jarak ke titik landing: {distance:.2f} meter")
            last_print = time.time()
            
        if distance < 1.0: # Radius penerimaan 1 meter
            print("Tiba di atas titik landing. Memulai pendaratan.")
            break
            
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())