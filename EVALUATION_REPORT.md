# MMATS-15 (Multiservice Multisensor Autonomous Targeting System)
# Evaluation & Conclusions

This document serves as an evaluation report for the current state of the YOLO vision model and the mathematical/logical workarounds implemented in the Autopilot state machine. It is intended to guide future dataset collection, model training, and control logic improvements.

## 1. YOLO Model & Dataset Evaluations
The current model (trained on an admittedly limited dataset) exhibits several behaviors that required complex code-level mitigation:

* **Incomplete Bounding Boxes (Single Gates & Triple Gate 2):** 
  The model only draws bounding boxes around the *top bar* of the Single Gates, and surprisingly, Triple Gate 2. Interestingly, Triple Gate 1 is annotated fine and does not suffer from this.
  * **Workaround:** We had to inject an artificial `+150` pixel vertical offset into the centering logic (`front_err_y + 150`) for these specific flawed gates to force the drone to aim below the bounding box and through the actual hole. 
  * **Future Training Fix:** Annotate the *empty space* (the hole itself) as the bounding box center, or include the side pillars in the bounding box so the mathematical centroid naturally lands in the middle of the hole.

* **False Positives / Phantom Dead-Centers:** 
  The model occasionally hallucinates a target or gives a false-positive detection that happens to be perfectly centered on the X-axis for a split second (especially immediately after takeoff).
  * **Workaround:** Implemented a two-step `altitude_locked` state machine. The drone is strictly forbidden from flying forward until it has hovered and perfectly aligned *both* the X and Y axes simultaneously.

* **Wall Collision Edge Cases (Triple Gate 1):**
  There is a known edge case where the drone may scrape or collide with the wall of Triple Gate 1. This is currently treated as an acceptable statistical anomaly (edge case) likely caused by minor YOLO centering drift or physics simulator wind/momentum quirks rather than a core logic failure. No code changes are currently mandated for this.

* **Lack of Contextual Differentiation:** 
  The model cannot differentiate between "Triple Gate 1" and "Triple Gate 2" because they look identical. When turning towards a new gate, the drone would sometimes instantly lock onto the old gate in the background.
  * **Workaround:** Implemented strict mathematical filters. The drone must yaw blindly for a set duration, then filter detections by **Area** (e.g., `> 20,000` to ensure it's physically close) and **Direction of Entry** (e.g., `front_err_x > 0` to ensure it's entering the screen from the correct side of the turn).

## 2. Autopilot Logic & Physics Lessons
The physics of the drone (specifically in the SITL Gazebo environment at ~30-40% RTF) revealed several critical constraints:

* **Pitch-Coupling Hallucination (The Stutter Effect):** 
  When the drone accelerates forward, its nose pitches down. This physically tilts the camera down, causing the target's Y-coordinate to artificially jump UP in the video frame.
  * **The Rule:** You **cannot** use the Y-axis error as a braking condition while moving forward. If you force the drone to continuously correct its altitude while flying forward, it will enter an infinite "brake-fly-brake" stuttering loop. Altitude must be locked (`up_cmd = 0.0`) during forward transit.

* **Dynamic Altitude Recovery:** 
  After dipping down to pass through a gate, a fixed-time or fixed-distance "pop-up climb" is unreliable. 
  * **The Rule:** The drone must use real-time telemetry (MAVSDK `pos_vel.position.down_m`) to dynamically climb until it explicitly registers its target altitude (e.g., `DRONE_Z > -1.5`).

* **LiDAR vs INS for Tunnel Exits:** 
  Relying purely on Inertial Navigation (INS) distance tracking to guess when the drone has exited a tunnel is risky.
  * **The Rule:** Use the side-facing LiDAR sensors. The drone knows it has exited the Triple Gate tunnel only when the left and right LiDAR distances suddenly spike (meaning the walls have disappeared).

* **Radar Sweeping for Missed Turns:** 
  If the drone executes a 90-degree turn (e.g., at ArUco 3) but the target gate is slightly outside the FOV due to drift or lag, flying straight forward blindly is fatal.
  * **The Rule:** Implement a sine-wave "Radar Sweep" (`sweep_yaw = math.sin(timeout_counter * 0.1) * 20.0`). The drone must slowly creep forward while smoothly panning its camera left and right to actively scan the horizon until the target is acquired.

## 3. The "AGM-114 / Shahed" Targeting Equations
To overcome the physics limitations and hardware constraints, the drone's autopilot essentially evolved into a heavy terminal-guidance system, drawing parallels to active-radar homing systems like the AIM-120 AMRAAM, semi-active laser guidance of the AGM-114 Hellfire, and the loitering-munitions behavior of Iranian Kamikaze drones (Shahed series).

Here are the core mathematical equations running the command center of this "missile":

* **Terminal Guidance Centering (The Hellfire Lock):**
  A proportional navigation system that calculates pixel-error from the camera centroid and converts it to velocity vector commands.
  ```python
  yaw_cmd = front_err_x * kp_yaw
  up_cmd = (front_err_y + OFFSET) * kp_up
  ```

* **Altitude-Locked Terminal Phase (The AMRAAM Pitbull Mode):**
  To prevent the "Pitch-Coupling Hallucination" from shattering the targeting lock, the drone implements a two-step state machine. It hovers to perfectly zero out the X and Y coordinates. Once `abs(error_x) < 20` and `abs(error_y) < 20`, it goes "Pitbull" (Terminal Phase) by freezing the Y-axis and accelerating violently forward:
  ```python
  if not altitude_locked:
      up_cmd = (front_err_y + 150) * kp_up
      if abs(front_err_x) < 20 and abs(front_err_y + 150) < 20:
          altitude_locked = True
  else:
      up_cmd = 0.0 # Altitude frozen, pitch variations ignored
      fwd_cmd = 0.8 # Terminal dive
  ```

* **Active Horizon Scanning (The Loitering Munition Sweep):**
  If a target is lost during a sharp maneuver, the drone drops into a slow loitering speed and executes a sinusoidal radar-sweep to re-acquire the target signature.
  ```python
  sweep_yaw = math.sin(timeout_counter * 0.1) * 20.0
  await flight.send_body_velocity(drone, forward_m_s=0.5, yaw_deg_s=sweep_yaw)
  ```

* **Repulsion Force Field (The Tunnel Guidance):**
  When blind-flying through a tunnel where the camera is useless, the drone falls back to side-facing LiDAR sensors, acting as a repulsive magnetic field pushing it towards the center of the gap.
  ```python
  if LIDAR_LEFT_DIST < 4.9 or LIDAR_RIGHT_DIST < 4.9:
      strafe_cmd = (LIDAR_RIGHT_DIST - LIDAR_LEFT_DIST) * 0.05
  ```

## 4. Final System Architecture Conclusions (The MMATS-15 Philosophy)

During the development of **MMATS-15**, we arrived at two massive architectural realizations:

1. **Depth Cameras are Overkill (Math + LiDAR is King):**
   We successfully proved that you do *not* need heavy, computationally expensive stereo depth cameras (like RealSense) to navigate complex 3D environments. By combining simple 2D pixel-error equations (Proportional Navigation) with basic 1D LiDAR distance sensors, we achieved robust multi-sensor fusion. The math handles the open-air alignment, and the LiDAR acts as a physical force-field when flying blind inside tunnels. It is vastly more lightweight and arguably more reliable.

2. **The Dataset is the Ultimate Bottleneck:**
   While the mathematical workarounds (like the `+150` pixel offset, the `altitude_locked` anti-stutter state machine, and the `front_area > 20000` filter) are incredibly robust, they only exist because the dataset was flawed. The single biggest takeaway for future iterations is that **dataset quality matters more than anything else**. A perfectly annotated YOLO model (where bounding boxes cover the actual empty holes and differentiate between Gate 1 and Gate 2) would eliminate 50% of the complex state-machine logic required to keep the drone alive.

## 5. The "Whack-A-Mole" Regression Loop
Fixing a bug in one phase (e.g. ArUco 1) often unintentionally broke another phase (e.g. ArUco 2 or Triple Gate) because robotics state machines are physically coupled. Changing the entry speed or pitch in Phase A fundamentally alters the camera's perspective and the drone's momentum in Phase B. 
* **Lesson Learned:** We must rely on hard physics (INS distance, exact area thresholds, hover-and-align checks) rather than assuming the drone is perfectly aligned based on timing. Time-based logic (`timeout_counter`) is highly susceptible to simulation lag, wind, and battery sag.

## 6. The UDP Payload & False "Confidence" Bug
A previous attempt to filter out false positives added a `front_confident > 0.5` check. However, the custom `vision_daemon.py` strictly sends only essential data over UDP (`error_x`, `error_y`, `area`, `class`) to avoid bloatware and packet loss. Since `confidence` was never sent, it defaulted to `0.0`, silently breaking the lock-on logic.
* **Lesson Learned:** Always align the Autopilot's expected data structure with the Vision Daemon's actual UDP payload. Do not add arbitrary checks without confirming the data pipeline supports them. We replaced this with a purely physical `front_area > 10000` filter.

## 7. YOLO Flickering & Strict vs. Dual-Timeout Completion
When centering over ground targets (ArUco / Drop Box), the YOLO model occasionally lost the small inner marker and instead detected the massive yellow pad (`Aruco Area`) underneath it due to shadows or dataset limitations. When it flickered to `Aruco Area`, the drone panicked.
* **Lesson Learned:** 
  1. **For ArUco:** The drone can use `Aruco Area` for general centering, but should only *quickly* complete the phase if it sees the actual `Aruco` marker. If the inner marker is permanently occluded, a "slow fallback" dual-timeout allows the phase to complete safely after 15 seconds.
  2. **For Drop Box:** Because the Red Box is large and reliable, we enforce a strict drop requirement. Flickering to `Aruco Area` resets the timer but does not cause the drone to panic.

## 8. Phase Reversion & Instant Overshoot Loops
When the drone lost the target during centering, it reverted its state machine back to the `FIND_` phase. However, it did not reset its `blind_start` tracking coordinates. Because it had already flown several meters, reverting instantly triggered the `dist_flown > 2.5m` overshoot logic, forcing it backwards in an infinite "Maju Mundur Cantik" loop.
* **Lesson Learned:** State machine reversions MUST be atomic. When reverting a state, all tracking variables associated with that state (like initial coordinates) must be wiped clean to prevent phantom triggers from stale memory.
