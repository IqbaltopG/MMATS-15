import asyncio
import sys
import select
import tty
import termios
from flight import OtotDrone

settings = termios.tcgetattr(sys.stdin)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

async def run():
    otot = OtotDrone()
    await otot.connect()
    
    print("\n===============================")
    print("   KEYBOARD TELEOP OVERRIDE    ")
    print("===============================")
    print("W / S : Pitch (Forward / Back)")
    print("A / D : Roll  (Strafe Left / Right)")
    print("Q / E : Yaw   (Spin Left / Spin Right)")
    print("R / F : Throttle (Altitude Up / Down)")
    print("SPACE : EMERGENCY BRAKE (Hover)")
    print("L     : LAND AND EXIT")
    print("===============================")
    print("Press any key to Takeoff...")
    
    while True:
        if getKey() != '':
            break
            
    print("\nTaking off to 1.5m...")
    await otot.takeoff_offboard(target_alt_m=1.5)
    
    speed = 2.0
    yaw_speed = 60.0
    
    forward = 0.0
    right = 0.0
    down = 0.0
    yaw = 0.0

    try:
        while True:
            key = getKey()
            
            if key == '':
                # FPS Style Auto-Brake (Friction)
                forward *= 0.5
                right *= 0.5
                down *= 0.5
                # Make Yaw stop INSTANTLY like a mouse look in an FPS
                yaw = 0.0
                
                if abs(forward) < 0.1: forward = 0.0
                if abs(right) < 0.1: right = 0.0
                if abs(down) < 0.1: down = 0.0
            else:
                # Drone Controls
                if key == 'w': forward = speed
                elif key == 's': forward = -speed
                elif key == 'a': right = -speed
                elif key == 'd': right = speed
                elif key == 'q': yaw = -yaw_speed
                elif key == 'e': yaw = yaw_speed
                elif key == 'r': down = -speed  # Z is Down, so -Z is Up
                elif key == 'f': down = speed
                
                # Emergency Hover/Brake
                elif key == ' ': 
                    forward = 0.0
                    right = 0.0
                    down = 0.0
                    yaw = 0.0
                    
                # Land
                elif key == 'l':
                    break
                    
            await otot.set_velocity(forward_m_s=forward, right_m_s=right, down_m_s=down, yaw_deg_s=yaw)
            await asyncio.sleep(0.05)
            
    except Exception as e:
        print(f"Teleop Error: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\nLanding...")
        await otot.set_velocity(0.0, 0.0, 0.0, 0.0)
        await otot.land()

if __name__ == "__main__":
    asyncio.run(run())
