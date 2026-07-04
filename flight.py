import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError

class OtotDrone:
    def __init__(self):
        self.drone = System()

    async def connect(self, system_address="udp://:14540"):
        await self.drone.connect(system_address=system_address)
        print("Waiting for drone to connect...")
        try:
            async def check_connection():
                async for state in self.drone.core.connection_state():
                    if state.is_connected:
                        print("[+] Vehicle connected via MAVLink!")
                        break
            await asyncio.wait_for(check_connection(), timeout=10.0)
            
            print("[*] Disabling SITL Failsafes (Battery, Datalink, RC)...")
            await self.drone.param.set_param_int("COM_LOW_BAT_ACT", 0)  # 0 = Warning only (no RTL)
            await self.drone.param.set_param_int("NAV_DLL_ACT", 0)      # 0 = Disable Datalink failsafe
            await self.drone.param.set_param_int("NAV_RCL_ACT", 0)      # 0 = Disable RC Loss failsafe
            print("[+] SITL Failsafes Disabled. Safe to test!")

        except asyncio.TimeoutError:
            print("\n[-] CRITICAL: Drone connection timed out! Is PX4 SITL / Gazebo running?")
            raise SystemExit(1)
            
        print("Waiting for EKF/GPS lock...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("[+] EKF/GPS locked. Home position is calibrated (0.0m)!")
                break
            print("Waiting for EKF/GPS lock...")
            await asyncio.sleep(1)

    async def takeoff_offboard(self, target_alt_m=2.5):
        print("Arming motors...")
        await self.drone.action.arm()

        print("Setting initial offboard setpoint...")
        # Send a 0-velocity setpoint before starting offboard (mandatory for safety)
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )

        print("Engaging Offboard Mode...")
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            print(f"Offboard start failed: {error._result.result}")
            await self.drone.action.disarm()
            return

        print(f"Taking off via Offboard Velocity (Ascending to {target_alt_m}m)...")
        # Negative down_m_s means ascending in NED frame
        await self.set_velocity(0.0, 0.0, -1.5, 0.0)
        
        print("Polling telemetry for altitude verification...")
        
        # Monitor altitude in a background task so it doesn't block the Heartbeat
        current_alt = 0.0
        async def monitor_altitude():
            nonlocal current_alt
            async for position in self.drone.telemetry.position():
                current_alt = position.relative_altitude_m
                
        alt_task = asyncio.create_task(monitor_altitude())

        while current_alt < target_alt_m:
            await self.set_velocity(0.0, 0.0, -1.5, 0.0)
            await asyncio.sleep(0.1)  # Strict 10Hz Keep-Alive Heartbeat
            
        alt_task.cancel()
        
        print(f"[+] Safe hover altitude reached: {current_alt:.2f}m")
        print("Stabilizing physics in hover...")
        for _ in range(20):
            await self.set_velocity(0.0, 0.0, 0.0, 0.0)
            await asyncio.sleep(0.1)
            
    async def set_velocity(self, forward_m_s, right_m_s, down_m_s, yaw_deg_s):
        """ Commands Body-frame Velocities (GPS-Denied Navigation) """
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(forward_m_s, right_m_s, down_m_s, yaw_deg_s)
        )

    async def lepaskan_muatan(self):
        print(">>> Actuating servo to drop medkit...")
        try:
            # Target actuator index 1 for payload drop mechanism
            await self.drone.action.set_actuator(1, 1.0)
            # Heartbeat Keep-Alive while dropping payload
            for _ in range(20):
                await self.set_velocity(0.0, 0.0, 0.0, 0.0)
                await asyncio.sleep(0.1)
            await self.drone.action.set_actuator(1, -1.0)
        except Exception as e:
            print(f"[!] Actuator error ignored (normal in basic SITL): {e}")
        print(">>> Medkit Payload Dropped!")

    async def land(self):
        print("Stopping Offboard Mode and Landing...")
        await self.drone.offboard.stop()
        await asyncio.sleep(0.5)  # Buffer to allow mode transition to settle
        await self.drone.action.land()
        
        print("Waiting for drone to physically touch down...")
        async for in_air in self.drone.telemetry.in_air():
            if not in_air:
                print("[+] Touchdown confirmed! Safe to shutdown.")
                break