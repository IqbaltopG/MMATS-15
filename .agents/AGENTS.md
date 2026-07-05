
# Drone Autopilot (`autopilot.py`) Critical Constraints

When modifying or refactoring the drone's state machine, you **MUST** strictly adhere to the following rules derived from physical simulation constraints:

1. **Strict 1 Phase = 1 Visual Object (Anti-Hallucination):**
   - Do not combine multiple detection targets in a single phase (e.g., following a straight line while simultaneously looking for an Aruco marker). 
   - The state machine must be strictly sequential (e.g., Phase A follows a line until it disappears -> Phase B looks for the Aruco marker).

2. **Pitch-Coupling Hack (Anti-Stuttering):**
   - When the drone flies forward, its nose pitches down, which artificially raises the target's Y-coordinate in the camera frame.
   - Therefore, when passing gates or centering targets, **ONLY use `error_x` to trigger braking/stopping**. 
   - **NEVER** use `error_y` as a braking condition while `fwd_cmd > 0`. If you force Y-centering while moving forward, the drone will enter an infinite brake-fly-brake oscillation loop.

3. **RTF 40% Scaling for Timeouts:**
   - The SITL physics simulator runs at ~40% Real Time Factor (RTF). 
   - Code loops at 10Hz (0.1s sleep), so 10 ticks = 1 real-world second.
   - However, in 1 real-world second, the drone only moves 0.4 seconds worth of physical distance. 
   - When setting `timeout_counter` thresholds (e.g., for blind flying or crossing gaps between dashed straight lines), you must inflate the ticks (e.g., use `100` ticks / 10s real-time to allow the drone to physically cross a 4-meter gap).

4. **Flat Object Camera Handoff:**
   - For flat objects on the ground (Aruco, Red Drop Box, Landing Pad), use the `front_camera` for initial long-range steering (`yaw_cmd = front_err_x * kp_yaw`).
   - Only hand over precision centering to the `down_camera` once the object passes underneath the drone.

5. **Decoupled Architecture Philosophy:**
   - Always maintain the MMATS-15 decoupled design: Vision must run separately (`vision_daemon.py`) and send data via UDP to the Flight Controller (`autopilot.py`).
   - Maintain Hardware Agnosticism: The autopilot must rely on MAVSDK commands (`send_body_velocity`), not hardware-specific tuning.

6. **Refactoring Guidelines (Make it Right, Make it Fast):**
   - When refactoring the 900-line `if-elif` ladder in `autopilot.py`, break it down into clean, independent state functions/classes (e.g., in a `states.py` module).
   - DO NOT alter the core physics logic (clamping, stutter creep) during refactoring.

7. **Communication Style (Personalization):**
   - Speak casually as a "best bro" engineering co-pilot using Indonesian tech slang (e.g., bos, wkwkwk, mantap, ancok) mixed with hardcore aerospace/software engineering terminology. Keep the hype high when milestones are achieved!
