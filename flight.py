import os
import asyncio
import math
import time
from mavsdk import System
from dotenv import load_dotenv

class DroneController:
    def __init__(self):
        load_dotenv()
        self.drone = System()
        
        # Load variabel env ke dalam properties class
        self.titik_target = [
            (float(os.getenv("TARGET_1_LAT", 0)), float(os.getenv("TARGET_1_LON", 0))),
            (float(os.getenv("TARGET_2_LAT", 0)), float(os.getenv("TARGET_2_LON", 0))),
        ]
        self.koordinat_flp = (float(os.getenv("FLP_LAT", 0)), float(os.getenv("FLP_LON", 0)))
        self.target_alt_agl = 10.0

    @staticmethod
    def get_distance_in_meters(coord1, coord2):
        R = 6371000.0
        lat1, lon1 = coord1
        lat2, lon2 = coord2
        lat1_rad, lat2_rad = math.radians(lat1), math.radians(lat2)
        dlat_rad = lat2_rad - lat1_rad
        dlon_rad = math.radians(lon2) - math.radians(lon1)
        a = math.sin(dlat_rad / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon_rad / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    @staticmethod
    def get_bearing(coord1, coord2):
        lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
        lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])
        dlon_rad = lon2 - lon1
        x = math.sin(dlon_rad) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dlon_rad))
        initial_bearing = math.atan2(x, y)
        return (math.degrees(initial_bearing) + 360) % 360

    async def connect_and_arm(self):
        await self.drone.connect(system_address="udp://:14540")
        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Drone connected!")
                break

        print("Waiting for global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok:
                print("Global position estimate OK")
                break

        async for home in self.drone.telemetry.home():
            self.koordinat_flp = (home.latitude_deg, home.longitude_deg)
            print(f"[!] Titik Landing disinkronkan: {self.koordinat_flp}")
            break

        print("Mempersiapkan motor... (ARMING)")
        await self.drone.action.arm()
        print("Mengudara... (TakeOFF)")
        await self.drone.action.takeoff()
        await asyncio.sleep(15)

    async def lepaskan_muatan(self, index):
        safe_index = index + 1 
        if safe_index > 6:
            return
        print(f"Release muatan ke-{safe_index}... Eksekusi!")
        try:
            await self.drone.action.set_actuator(safe_index, 1.0) 
            await asyncio.sleep(2) 
            await self.drone.action.set_actuator(safe_index, -1.0) 
        except Exception as e:
            print("Error aktuator diabaikan (normal di SITL).")
        await asyncio.sleep(5)

    async def misi_outdoor(self):
        print("\n--- INITIATING OUTDOOR MISSION ---")
        async for terrain_info in self.drone.telemetry.position():
            ground_amsl = terrain_info.absolute_altitude_m - terrain_info.relative_altitude_m
            break 

        target_alt_amsl = ground_amsl + self.target_alt_agl

        for i, (lat, lon) in enumerate(self.titik_target):
            async for pos_awal in self.drone.telemetry.position():
                current_pos_awal = (pos_awal.latitude_deg, pos_awal.longitude_deg)
                break 
                
            target_pos = (lat, lon)
            target_yaw = self.get_bearing(current_pos_awal, target_pos)
            
            print(f"OTW Target {i+1} | Hadap: {target_yaw:.1f} derajat")
            await self.drone.action.goto_location(lat, lon, target_alt_amsl, target_yaw)
            
            last_print = 0
            async for position in self.drone.telemetry.position():
                current_pos = (position.latitude_deg, position.longitude_deg)
                distance = self.get_distance_in_meters(current_pos, target_pos)
                
                if time.time() - last_print >= 1.0:
                    print(f"Jarak ke target {i+1}: {distance:.2f} meter")
                    last_print = time.time()
                    
                if distance < 2.0:
                    print(f"[!] Tiba di Target Outdoor {i+1}")
                    break 
            
            await self.lepaskan_muatan(i + 1) 

    async def rtb(self):
        print("\n--- MISI BERES: RTB ---")
        async for pos_pulang in self.drone.telemetry.position():
            current_pos_pulang = (pos_pulang.latitude_deg, pos_pulang.longitude_deg)
            ground_amsl = pos_pulang.absolute_altitude_m - pos_pulang.relative_altitude_m
            break
            
        rtb_yaw = self.get_bearing(current_pos_pulang, self.koordinat_flp)
        await self.drone.action.goto_location(self.koordinat_flp[0], self.koordinat_flp[1], ground_amsl + self.target_alt_agl, rtb_yaw)

        async for position in self.drone.telemetry.position():
            current_pos = (position.latitude_deg, position.longitude_deg)
            if self.get_distance_in_meters(current_pos, self.koordinat_flp) < 1.0:
                print("Tiba di atas titik landing. Mendarat.")
                break
                
        await self.drone.action.land()