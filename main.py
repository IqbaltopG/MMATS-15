import asyncio
from flight import DroneController
from vision import TargetTracker

async def run():
    # Instansiasi objek otot dan mata
    otot = DroneController()
    mata = TargetTracker()

    # 1. Takeoff Setup
    await otot.connect_and_arm()

    # ==========================================
    # [PHASE 1] MISI INDOOR (KERANJANG MERAH)
    # ==========================================
    print("\n--- INITIATING INDOOR MISSION ---")
    print("[!] Scanning area untuk Keranjang Merah...")
    
    # Looping buat nge-scan objek merah (Non-blocking loop)
    target_locked = False
    for _ in range(30): # Scan selama 30 iterasi
        if mata.scan_target():
            print("[!] TARGET KERANJANG MERAH LOCKED!")
            target_locked = True
            break
        await asyncio.sleep(0.5) # Kasih napas CPU dan MAVSDK
        
    if target_locked:
        await otot.lepaskan_muatan(0)
    else:
        print("[-] Target tidak ditemukan, skip pelepasan indoor.")

    mata.release_camera() # Matiin kamera biar enteng

    # ==========================================
    # [PHASE 2 & 3] MISI OUTDOOR & RTB
    # ==========================================
    await otot.misi_outdoor()
    await otot.rtb()

if __name__ == "__main__":
    asyncio.run(run())