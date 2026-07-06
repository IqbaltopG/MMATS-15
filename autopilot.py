import asyncio
import json
import math
from utils import clamp, calculate_distance, get_stutter_creep_speed
from mavsdk import System
import flight
from flight import get_distance_sensor_stream
from comms import state, telemetry_task, start_udp_server


async def kill_switch_task(drone):
    from mavsdk.telemetry import FlightMode
    is_offboard = False
    async for flight_mode in drone.telemetry.flight_mode():
        if flight_mode == FlightMode.OFFBOARD:
            is_offboard = True
        elif is_offboard and flight_mode != FlightMode.OFFBOARD:
            print("[KILL SWITCH] MANUAL OVERRIDE DETECTED! (Flight Mode changed from OFFBOARD). Exiting Autopilot...")
            import os
            os._exit(0)

async def run_mission():
    drone = System()
    print("[AUTOPILOT] Menyambung ke Drone (SITL)...")
    await drone.connect(system_address="udp://:14540")

    async for conn_state in drone.core.connection_state():
        if conn_state.is_connected:
            print("[AUTOPILOT] Drone Terkoneksi!")
            break
    
    print("[AUTOPILOT] Starting Telemetry Task...")
    asyncio.create_task(telemetry_task(drone))
    print("[AUTOPILOT] Starting Kill Switch Task (Flight Mode Listener)...")
    asyncio.create_task(kill_switch_task(drone))

    print("[AUTOPILOT] Memulai Smart Takeoff...")
    await flight.arm_and_takeoff(drone, altitude_m=1.5)

    print("[AUTOPILOT] Hovering di 1.5m selama 5 detik untuk Stabilisasi...")
    await flight.send_body_velocity(drone, 0.0, 0.0, 0.0, 0.0)
    await asyncio.sleep(5)


    from states import STATE_REGISTRY, MissionContext

    # MMATS-15 STATE MACHINE (OOP)
    ctx = MissionContext()
    
    while True:
        current_state = STATE_REGISTRY.get(ctx.state_phase)
        if current_state:
            await current_state.execute(drone, ctx)
            if ctx.state_phase == 'DONE':
                break
        else:
            print(f"[AUTOPILOT] UNKNOWN STATE: {ctx.state_phase}")
            break
        await asyncio.sleep(0.1) # Loop jalan 10 Hz

async def main():
    transport = await start_udp_server()
    try:
        await run_mission()
    finally:
        transport.close()

if __name__ == "__main__":
    asyncio.run(main())
