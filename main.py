import asyncio
from flight import DroneController
from vision import TargetTracker

async def run():
    # Instansiasi objek otot dan mata
    otot = DroneController()

    # 1. Takeoff Setup
    await otot.connect_and_arm()
    # Inisiasi kamera setelah drone mengudara untuk menghindari buffer usang
    mata = TargetTracker()

    # ==========================================
    # [PHASE 1] MISI INDOOR (KERANJANG MERAH)
    # ==========================================
    print("\n--- INITIATING INDOOR MISSION ---")
    print("[!] Scanning area untuk Keranjang Merah...")

    # Melakukan scan untuk target objek merah
    target_locked = False
    for i in range(60):  # Iterasi scan selama ~12 detik
        if mata.scan_target():
            print("[!] TARGET KERANJANG MERAH LOCKED!")
            target_locked = True
            break
        await asyncio.sleep(0.2)  # Jeda non-blocking antar scan
    if target_locked:
        await otot.lepaskan_muatan(0)
    else:
        print("[-] Target tidak ditemukan, skip pelepasan indoor.")

    mata.release_camera()  # Melepaskan sumber daya kamera

    # ==========================================
    # [PHASE 2 & 3] MISI OUTDOOR & RTB
    # ==========================================
    await otot.misi_outdoor()
    await otot.rtb()

if __name__ == "__main__":
    asyncio.run(run())