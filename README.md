# 🚁 MMATS-15: Microservice Multisensor Autonomous Targetting System

![Python](https://img.shields.io/badge/Python-3.10-blue?style=for-the-badge&logo=python)
![PX4](https://img.shields.io/badge/PX4_Autopilot-SITL-blueviolet?style=for-the-badge)
![MAVSDK](https://img.shields.io/badge/MAVSDK-Enabled-success?style=for-the-badge)
![License](https://img.shields.io/badge/License-Unlicense-black?style=for-the-badge)

## 📌 Executive Summary
Listen up. This repo contains **MMATS-15**, a highly aggressive, brute-force autonomous targeting architecture for precision payload delivery (X500 frame). We didn't build this to look pretty. We built it following the **KISS (Keep It Simple, Stupid)** principle because the hardware we are running on (budget laptops for SITL, Raspberry Pi 5 for Edge) will literally melt if you try to shove a bloated ROS2 framework down its throat. 

This is raw UDP microservices. It's got time-dilation physics compensation, "Tunnel Blind Charge" memory buffers, and zero-cost sensor fusion. The state machine is pure spaghetti `if/else` chaos right now because we are in the "Make It Work" phase, and honestly, we don't care if you judge us. It lands the drone. 

## 🛠️ Tech Stack
* **Core Logic:** Python 3.10, MAVSDK
* **Flight Stack:** PX4 Autopilot
* **Simulation Engine:** Gazebo (gz_x500)
* **Ground Control Station:** QGroundControl (QGC)
* **OS Environment:** Xubuntu / Windows 11 (via WSL2)

---

## 🚀 Environment Setup & Hardware Acceleration (The Nvidia Bypass)
If you're running Gazebo on a Linux machine with hybrid graphics, your OS is probably stupid enough to default to Integrated Graphics. Your system will bottleneck, hang, and crash. 

To force Gazebo to pull from the **Dedicated GPU (Nvidia)** and stop acting like a potato, run this exact command:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json make px4_sitl gz_x500_depth
```

---

## 🐧 Native Linux Setup (Ubuntu 22.04 LTS)
For you bare-metal dual-booters. Run this. Don't skip steps.

### 1. System Provisioning
Execute the official PX4 script to yank Gazebo and the ROS/Toolchain dependencies.

```bash
sudo apt update && sudo apt upgrade -y
wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/ubuntu.sh
bash ubuntu.sh
pip install -r requirements.txt
```
⚠️ **System Halt:** Once the script finishes, **YOU MUST REBOOT YOUR PC**. If you don't reboot, don't cry to us when it breaks.

### 2. Clone & Compile Firmware
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
make px4_sitl gz_x500
```

---

## 🪟 Windows Setup Guide (via WSL2)
**DO NOT USE A VM (VirtualBox/VMware).** If you use a VM, your VRAM will bottleneck into oblivion and your framerate will be measured in Seconds Per Frame. Use **WSL2 (Ubuntu 22.04 LTS)**. 

### 1. Dependency Injection
Open your WSL2 terminal and run:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install git python3-pip python-is-python3 make -y
wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/ubuntu.sh
bash ubuntu.sh
```
⚠️ **System Halt:** **RESTART WSL AFTER THIS FINISHES**. Either close the terminal or run `wsl --shutdown` in Windows CMD. 

### 2. Clone & Compile Firmware
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
make px4_sitl gz_x500
```
*(If Gazebo launches and the X500 drone spawns, you survived compilation. Kill it with `Ctrl+C`).*

---

## 📡 Ground Control Integration (QGC)
For WSL2 users, the simulation runs inside Linux, but you control it from Windows because WSL GUI performance is a joke.

1. Download **QGroundControl** for Windows from the [Official Site](https://docs.qgroundcontrol.com/).
2. **Network Bypass:** When Windows Defender pops up, allow **Private & Public Networks**. If you misclick this, QGC will be completely blind and drop all MAVLink packets. 
3. Run `make px4_sitl gz_x500` in WSL. Open QGC on Windows. It will auto-connect.

---

## ⚠️ Rules of Engagement (READ BEFORE ASKING STUPID QUESTIONS)
Before you dump a stack trace in the group chat, use your brain:

1. **"My Gazebo screen is black / UI is glitching!"** -> Update your GPU drivers. WSLg renders directly through the Host GPU driver. 
2. **"QGC won't connect!"** -> Fix your Windows Firewall. 99% of you blocked the UDP packets.
3. **"Make failed during compile!"** -> Scroll up and find the first RED text. Don't just read "Failed" at the very bottom like an idiot. Read the actual error log.

> **RTFM (Read The Fucking Manual):** These instructions are tested. Execute them exactly as written.

---

## 👁️ The MMATS-15 Architecture

### 1. Zero-Cost Sensor Fusion (LiDAR + Bounding Box Area)
Processing 3D point clouds from a depth camera uses way too much CPU and network I/O. We threw that out. Instead, MMATS-15 extracts the pixel area of the YOLO bounding box `(bx2 - bx1) * (by2 - by1)` as a dirt-cheap pseudo-depth metric. On the physical drone, we slam this together with a downward LiDAR for 2-way verification. It’s cheap, it’s fast, and it works.

### 2. The Microservice Split
We don't cram neural network inference and flight controls into the same loop. `vision_daemon.py` aggressively processes frames and blasts UDP packets containing bounding boxes to `autopilot.py`. This ensures the flight controller loop runs at a strict 10Hz without waiting for YOLO to finish thinking. If you put them in the same file, the drone would have a stroke.

### 3. Simulation Time-Dilation (RTF Scaling Hack)
If you run Gazebo SITL on a potato laptop, the physics engine will choke and run at **30-40% Real-Time Factor (RTF)**. That means 10 real seconds is only 3-4 physical seconds in the simulator.

Because of this, the `timeout_counter` thresholds in our `autopilot.py` state machine are **massively inflated**. If we didn't do this, the drone would time out and abort before it physically crossed a blind spot.

**WARNING:** If you have a beefy PC that actually hits 100% RTF, or when we flash this to the real drone, you **MUST SCALE DOWN THE TIMEOUTS**. If you run this 30% RTF code in the real world, the drone is going to hang in the air for 15 seconds waiting for a timeout. Adjust your shit before you fly IRL.

### 4. "Stutter Creep" Physics Hack
We use raw physics to solve our camera FOV problems. When the drone flies forward, the nose pitches down, aiming the downward camera backward (creating a massive blind spot). Instead of praying it sees the ArUco pad, we implemented a "stutter creep." The drone flies forward for 1 second, then slams the brakes for 1 second. The braking violently levels the pitch, forcing the downward camera to point perfectly vertical like a spotlight, guaranteeing we sweep the floor. It's brute-force engineering.
