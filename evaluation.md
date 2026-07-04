# EVALUATION REPORT (July 4th/5th 2026 Session)

## 1. The "Whack-A-Mole" Regression Loop
**What Happened:** Fixing a bug in one phase (e.g. ArUco 1) often unintentionally broke another phase (e.g. ArUco 2 or Triple Gate). 
**Why it Happened:** Robotics state machines are physically coupled. Changing the entry speed or pitch in Phase A fundamentally alters the camera's perspective and the drone's momentum in Phase B. 
**Lesson Learned:** We must rely on hard physics (INS distance, exact area thresholds, hover-and-align checks) rather than assuming the drone is perfectly aligned based on timing. Time-based logic (`timeout_counter`) is highly susceptible to simulation lag, wind, and battery sag.

## 2. The UDP Payload & False "Confidence" Bug
**What Happened:** The drone spun endlessly, unable to lock onto the Triple Gate.
**Why it Happened:** A previous attempt to filter out false positives added a `front_confident > 0.5` check. However, the custom `vision_daemon.py` strictly sends only essential data over UDP (`error_x`, `error_y`, `area`, `class`) to avoid bloatware and packet loss. Since `confidence` was never sent, it defaulted to `0.0`, silently breaking the lock-on logic.
**Lesson Learned:** Always align the Autopilot's expected data structure with the Vision Daemon's actual UDP payload. Do not add arbitrary checks without confirming the data pipeline supports them. We replaced this with a purely physical `front_area > 10000` filter, which perfectly suits the lightweight UDP architecture.

## 3. YOLO Flickering & Strict vs. Dual-Timeout Completion
**What Happened:** When centering over ground targets (ArUco / Drop Box), the YOLO model occasionally lost the small inner marker (e.g., QR Code or Red Box) and instead detected the massive yellow pad (`Aruco Area`) underneath it, likely due to shadows or dataset limitations.
**Why it Happened:** Our state machine strictly required the inner marker for centering to proceed. When it flickered to `Aruco Area`, the drone panicked, thought the target was completely lost, and either hovered forever or triggered overshoot fallbacks.
**Lesson Learned:** 
1. **For ArUco:** The drone can use `Aruco Area` for general centering, but should only *quickly* complete the phase if it sees the actual `Aruco` marker. If the inner marker is permanently occluded, a "slow fallback" dual-timeout allows the phase to complete safely after 15 seconds of perfect pad-centering.
2. **For Drop Box:** Because the Red Box is large and reliable, we enforce a strict drop requirement (only drop if `Red Drop Box` is seen). Flickering to `Aruco Area` resets the timer but does not cause the drone to panic or fly away.

## 4. Phase Reversion & Instant Overshoot Loops
**What Happened:** The drone got trapped in an infinite "Maju Mundur Cantik" loop over the Drop Box.
**Why it Happened:** When the drone lost the target during centering, it reverted its state machine back to the `FIND_` phase to search again. However, it did not reset its `blind_start` tracking coordinates. Because it had already flown several meters since it first spotted the target, reverting instantly triggered the `dist_flown > 2.5m` overshoot logic, forcing it backwards.
**Lesson Learned:** State machine reversions MUST be atomic. When reverting a state, all tracking variables associated with that state (like initial coordinates) must be wiped clean to prevent phantom triggers from stale memory.
