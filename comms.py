import asyncio
import json

class DroneState:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.lidar_left = 5.0
        self.lidar_right = 5.0
        self.target_front = {"status": "LOST", "class": "none", "error_x": 0, "error_y": 0, "area": 0, "confident": 0.0}
        self.target_down = {"status": "LOST", "class": "none", "error_x": 0, "error_y": 0, "area": 0, "confident": 0.0}

state = DroneState()

async def telemetry_task(drone):
    async for pos_vel in drone.telemetry.position_velocity_ned():
        state.x = pos_vel.position.north_m
        state.y = pos_vel.position.east_m
        state.z = pos_vel.position.down_m

class UDPReceiverProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data, addr):
        try:
            message = data.decode('utf-8')
            parsed = json.loads(message)
            
            if parsed.get("camera") == "lidar":
                side = parsed.get("side")
                if side == "left":
                    state.lidar_left = float(parsed.get("range", 5.0))
                elif side == "right":
                    state.lidar_right = float(parsed.get("range", 5.0))
            elif parsed.get("camera") == "down":
                state.target_down.update(parsed)
            else:
                state.target_front.update(parsed)
        except Exception as e:
            pass

async def start_udp_server(ip="127.0.0.1", port=5005):
    loop = asyncio.get_running_loop()
    transport, protocol = await loop.create_datagram_endpoint(
        lambda: UDPReceiverProtocol(),
        local_addr=(ip, port)
    )
    return transport
