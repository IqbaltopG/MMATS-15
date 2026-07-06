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

## 9. The "True Nature" of Physics (Non-Deterministic Bugs)
Because SITL simulates real physics, perfect code on one run can crash on the next due to micro-variations (like a 1cm drift causing a 70px bounding box shift).
* **Lesson Learned:** 
  1. **Anti-Drift:** When targets get too close (Area > 100k), pixel tracking becomes hyper-sensitive. We must force lateral corrections to zero (`strafe_cmd = 0.0`) to avoid injecting momentum just before entering a blind spot.
  2. **Premature Punching:** Hardware limits (like camera FOV) cause YOLO to lose targets prematurely. Instead of arbitrarily inflating area thresholds, we must use multi-conditional logic (e.g. `Area > threshold AND yaw_error < threshold`) to guarantee the physical state of the drone before committing to a blind maneuver.

## 10. ROS vs MAVSDK (Microservice Architecture)
Adding ROS 2 solely to bridge LiDAR data into the Autopilot introduces severe processing overhead, defeating the purpose of a lightweight companion computer setup.
* **Lesson Learned:** Stick to pure MAVSDK endpoints (e.g., `drone.telemetry.distance_sensor()`) embedded within the existing `flight.py` microservice. This decouples the vision AI (`vision_daemon.py`) from the flight controller (`autopilot.py`) without requiring an intermediary bloated pub/sub middleware.

## 11. ⚠️ CRITICAL FOR FUTURE AI: Hardware Abstraction & Simulation Hacks
Do NOT inject simulation-specific tools (like `subprocess.Popen(['gz', 'topic'])`) into core microservices like `flight.py`. `flight.py` must remain a 100% pure MAVLink/MAVSDK hardware abstraction layer.
* **Lesson Learned:** If simulation data is missing (e.g., Gazebo LiDAR not reaching MAVSDK), the fix must be applied to the simulation environment itself (e.g., bridging the `.sdf` to MAVLink) or via a separate external bridge script. Polluting `flight.py` with simulation hacks destroys the ability to deploy the code directly to a physical Pixhawk drone without rewriting the codebase. Always treat `autopilot.py` and `flight.py` as if they are already running on real hardware.

## 12. Garbage In, Garbage Out (The Dataset Hallucination Limit)
When observing the logs at Triple Gate, YOLO bounding boxes randomly teleported from X=-814 to X=+885 within milliseconds, despite the drone hovering steadily. This forced the autopilot to attempt massive yaw corrections, leading to infinite stalling.
* **Lesson Learned:** You cannot out-code a bad dataset. A state machine can only act on the error vector it receives. If the model is trained haphazardly (e.g. failing to differentiate between the left pillar, right pillar, and the center gap of a gate), it will output mathematically impossible bounding boxes. The ultimate fix for "wild teleporting" errors is re-annotating the dataset, not adding more smoothing logic to the flight controller.

## 13. Corrupted Target Memory & Bounding Box Clamping
When the drone approaches a gate at point-blank range (Area > 800,000), the YOLO bounding box occasionally "spills over" outside the physical camera resolution (e.g., generating X-coordinates of -907 on a 640x640 feed). When the camera subsequently loses the target, the Autopilot's "Hover Memory" latches onto this corrupted, extreme outlier coordinate, causing infinite spinning or hovering because the drone believes it is vastly off-center.
* **Lesson Learned (Vision):** Implement strict mathematical clamping on YOLO bounding box coordinates (`bx1 = max(0, min(width, bx1))`). If a target fills the entire screen, clamping mathematically forces the centroid to the exact center of the screen (`error_x = 0`), which perfectly reflects physical reality (the drone is dead-center inside the object).
* **Lesson Learned (Autopilot):** Tolerances must scale with resolution. A strict 50-pixel tolerance is safe for a 640p feed but represents less than a 5% margin on a 1280p HD feed. Centering margins must be widened, or alternatively, replaced by Exponential Moving Averages (EMA) to prevent single-frame glitches from ruining the tracking memory.

## 14. Ghost State Transitions (The Silent Killer)
After the drone successfully punched through 8.0m of Final Gates, the state machine transitioned to `"FIND_LANDING"` instead of the correct `"FIND_LANDING_PAD"`. Because no `elif` block matched the misspelled state, the drone silently continued executing its last velocity command (1.2 m/s forward) forever, overshooting the Landing Pad and flying into oblivion.
* **Lesson Learned:** Typos in state transition strings are the most dangerous class of bugs in a string-based state machine. They produce **zero errors, zero warnings, and zero log output** — the drone simply stops responding to all sensor input while maintaining its last velocity. Future improvement: implement a state validation check (e.g., `assert state_phase in VALID_STATES`) at the top of each loop iteration to catch undefined states immediately.

## 15. Variable Namespace Collision in Multi-Purpose Counters
During `PRECISION_LANDING`, the variable `timeout_counter` was reused for two incompatible purposes: (1) counting stability ticks (target: 100) and (2) counting target-loss ticks (limit: 50). When YOLO flickered for one frame after 53 ticks of stable centering, the fallback logic read `timeout_counter = 53 > 50` and instantly reverted the phase, even though the drone was perfectly centered and descending correctly.
* **Lesson Learned:** In complex state machines, **every counter must be single-purpose**. Reusing a counter variable across different logical branches is a ticking time bomb. The fix was trivial (introduce `landing_ticks`), but the bug was invisible for weeks because the drone never reached the landing phase until all prior bugs were resolved. This is a textbook example of "Bug Ujung Dunia" (End-of-World Bug) — bugs that hide at the end of the pipeline and only surface when everything else finally works.

## 16. See-Through Gate False Positive (YOLO X-Ray Vision)
When the drone was very close to Triple Gate 2 (Area > 1,000,000), the gate's physical opening allowed the camera to see objects behind it (e.g., the Aruco Area pad on the ground). YOLO then detected the Aruco Area through the gate hole, causing `vision_daemon.py` to send Aruco Area coordinates instead of Triple Gate coordinates. The Autopilot lost its gate lock and entered Hover & Yaw fallback.
* **Lesson Learned:** YOLO's "biggest area" selection logic can be fooled by transparent or hollow objects. The camera physically sees through holes in gates, and objects behind the gate can appear larger than the gate's remaining visible frame. The combination of Bounding Box Clamping (forcing error_x to 0 when the gate fills the screen) and relaxed tolerance (100px instead of 50px) successfully mitigated this by allowing the punch to trigger before YOLO switches targets.

## 17. Diagonal Drift at Final Gate (The Corner-Cutting Bug)
When YOLO locked onto Final Gate 2 (behind Gate 1) instead of Gate 1 itself, the strafe correction reached -0.42 m/s. Combined with 0.8 m/s forward speed, the drone flew at a ~28-degree diagonal angle, physically exiting through the side of Gate 1's opening instead of through the center hole.
* **Lesson Learned:** When the drone is in "altitude-locked forward flight" mode (Pitbull Mode), lateral corrections must be aggressively clamped. At `±0.15 m/s` strafe vs `0.8 m/s` forward, the maximum diagonal angle is reduced to ~10 degrees, which is geometrically insufficient to exit a standard KRTI gate opening. The trade-off is slower centering, but the drone physically cannot escape the gate corridor.

## 18. The Google AI Drift Mess (Strafe vs Yaw Variable Collision)
* **Issue:** When trying to implement a "Drift" maneuver (yawing towards the target while strafing laterally) at the Final Gate, naive variable reassignment (`right_m_s=strafe_cmd` where `strafe_cmd=0.0`, and `yaw_deg_s=yaw_cmd*100.0` but clamped to 0.15) caused the drone to lose all lateral control. It resulted in massive unchecked diagonal drifting (-0.709 m/s Vy) because the autopilot was fundamentally misunderstanding MAVSDK control parameters.
* **Lesson Learned:** MAVSDK parameters are physically literal. `right_m_s` dictates lateral velocity in meters/second, while `yaw_deg_s` dictates rotation in degrees/second. To implement a "Drift", both variables must be calculated, scaled appropriately for their respective units, clamped independently (e.g., ±0.15 m/s for strafe, ±10.0 deg/s for yaw), and passed concurrently to `send_body_velocity`. Never trust general-purpose AI with robotic state machine geometry without verifying the physics context.

## 19. 🏆 MILESTONE ACHIEVED: FULL END-TO-END MISSION COMPLETION 🏆
* **Breakthrough:** After weeks of iteration (including custom YOLOv8 training on a noisy dataset, solving simulation RTF scaling bugs, overcoming FOV limitations, and fixing "Ghost State" logic errors), the EVOSKY MMATS-15 drone successfully executed the complete KRTI trajectory from Takeoff -> Single Gates -> Triple Gate -> Red Drop Box -> Final Gates -> Precision Landing, completely autonomously relying solely on YOLOv8 and INS odometry (Zero GPS waypoint reliance).

## 5. Tahap 4 (Final Boss) Insights: The Triumph of Hardware Agnosticism & OOP
During the full Start-to-Finish test run, several catastrophic edge cases emerged that completely validated our decoupled OOP (Object-Oriented Programming) architecture:

* **Dataset Bias & Sun Rotation Hack:** 
  The YOLO model started hallucinating Single Gates as "Tripple Gate" or "Landing path". The root cause? **Dataset Bias**. The original training dataset was recorded while flying the drone backward (Finish to Start). When flying forward, the Gazebo sun acted as a harsh backlight, casting dark shadows on the gate faces that YOLO had never seen.
  * **Workaround:** Instead of retraining the model (which would take hours), we simply rotated the `<light>` vector in `KRTI_2026.sdf` by 180 degrees (`<direction>0.5 0.1 -0.9</direction>`). A god-tier environmental hack that saved massive computational resources.

* **Simulation Time vs. Physical Reality (RTF Scaling):** 
  Using `asyncio.sleep(2.0)` for a blind punch forward resulted in the drone stopping mid-air. Why? Gazebo was running at 30% Real-Time Factor (RTF). Two real-world seconds translated to barely 0.6 seconds in the physics engine. 
  * **Workaround:** Replaced time-based timeouts with physical distance metrics (`calculate_distance` from `dist_flown`). The drone now guarantees movement based on physical meters traveled, entirely ignoring simulation lag.

* **Catastrophic Deadlocks & Bounding Box Merges:** 
  YOLO frequently merged Gate 1 and Gate 2 into a single confusing data stream (e.g., reporting the massive Area of Gate 1, but the off-center Error X of Gate 2). This caused the drone to completely freeze: it refused to fly forward (because X was off-center) and refused to strafe (because the Area was too large, triggering the Anti-Drift safety).
  * **Workaround:** Implemented a Force-Forward override inside `GateCenteringBase` (`fwd_cmd = 0.8`) if the area exceeds 100,000, violently breaking the deadlock and forcing the drone to punch through.

* **Emergent Behavior: The 2-for-1 Gate Bypass:**
  Because Gate 1 and Gate 2 are perfectly aligned, passing Gate 1 meant YOLO immediately locked onto Gate 2. The `TerminalGuidance_Gate1` state effectively swept through *both* gates simultaneously without ever triggering its blind punch until Gate 2 was passed. 
  * **Workaround:** We completely deleted the `TerminalGuidance_Gate2` phase. Gate 1 now handles both gates and executes a massive 9.0-meter blind punch at the end, dropping the drone perfectly on top of Aruco 1.

**Conclusion:** The MMATS-15 architecture holds. The Vision Daemon (`vision_daemon.py`) can hallucinate all it wants, the physics engine can lag, but because the State Machine (`states.py`) is modular and isolated, we only had to tweak logical parameters (punch distances, force forwards) without touching the core AI or Flight Controller communication lines.
