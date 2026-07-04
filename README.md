# 🚁 MMATS-15: Microservice Multisensor Autonomous Targetting System

![Python](https://img.shields.io/badge/Python-3.10-blue?style=for-the-badge&logo=python)
![PX4](https://img.shields.io/badge/PX4_Autopilot-SITL-blueviolet?style=for-the-badge)
![MAVSDK](https://img.shields.io/badge/MAVSDK-Enabled-success?style=for-the-badge)
![License](https://img.shields.io/badge/License-Unlicense-black?style=for-the-badge)

## 📌 Executive Summary
This repository contains **MMATS-15**, a missile-inspired, hyper-optimized autonomous targeting architecture for precision payload delivery (X500 frame). Built heavily on the **KISS (Keep It Simple, Stupid)** principle, it strips away bloated frameworks in favor of raw UDP microservices. Tested within the **PX4 SITL** and **Gazebo** environments, the system features time-dilation physics compensation, "Tunnel Blind Charge" memory buffers, and zero-cost sensor fusion designed specifically to run flawlessly on resource-constrained edge hardware (like the Raspberry Pi 5).

## 🛠️ Arsenal & Tech Stack
* **Core Logic:** Python 3.10, MAVSDK
* **Flight Stack:** PX4 Autopilot
* **Simulation Engine:** Gazebo (gz_x500)
* **Ground Control Station:** QGroundControl (QGC)
* **OS Environment:** Xubuntu / Windows 11 (via WSL2)

---

## 🚀 Environment Setup & Hardware Acceleration (The Nvidia Bypass)
Running Gazebo simulations on Linux machines with hybrid graphics often leads to VRAM bottlenecks and system hangs due to the OS defaulting to Integrated Graphics. 

To ensure Gazebo pulls resources directly from the **Dedicated GPU (Nvidia)** and prevents *kernel panics* or memory overloads, I use a specific hardware profiling bypass.

**To initiate the PX4 SITL simulation with Nvidia Offloading, execute the following command:**

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json make px4_sitl gz_x500_depth
```

---

## 🐧 Native Linux Setup (Ubuntu 22.04 LTS)
Instruksi ini khusus untuk pengguna bare-metal Linux (Dual Boot / Single OS).

### 1. System Provisioning
Eksekusi *script* instalasi resmi dari PX4 untuk menarik Gazebo dan seluruh *dependency* ROS/Toolchain.

```bash
sudo apt update && sudo apt upgrade -y
wget [https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/ubuntu.sh](https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/ubuntu.sh)
bash ubuntu.sh
pip install -r requirements.txt
```
⚠️ **System Halt:** Setelah *script* selesai, **WAJIB REBOOT PC KALIAN**.

### 2. Clone & Compile Firmware
Tarik source code dan jalankan kompilasi awal.

```bash
git clone [https://github.com/PX4/PX4-Autopilot.git](https://github.com/PX4/PX4-Autopilot.git) --recursive
cd PX4-Autopilot
make px4_sitl gz_x500
```

---

## 🪟 Windows Setup Guide (via WSL2)
**JANGAN GUNAKAN VM (VirtualBox/VMware)**. Gunakan **WSL2 (Ubuntu 22.04 LTS)** di Windows untuk menghindari VRAM bottleneck yang ekstrem.

### 1. Dependency Injection
Buka terminal WSL2 (Ubuntu) dan jalankan command berikut:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install git python3-pip python-is-python3 make -y
wget [https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/ubuntu.sh](https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/ubuntu.sh)
bash ubuntu.sh
```
⚠️ **System Halt:** Setelah *script* selesai, **WAJIB RESTART WSL**. Tutup terminal atau jalankan `wsl --shutdown` di CMD Windows.

### 2. Clone & Compile Firmware
Tarik *source code* PX4 dan lakukan kompilasi untuk target SITL (X500).

```bash
git clone [https://github.com/PX4/PX4-Autopilot.git](https://github.com/PX4/PX4-Autopilot.git) --recursive
cd PX4-Autopilot
make px4_sitl gz_x500
```
*(Jika terminal menampilkan Gazebo dan model drone X500 muncul, kompilasi sukses. Tutup dengan `Ctrl+C`).*

---

## 📡 Ground Control Integration (QGC)
Untuk *user* WSL2, simulasi berjalan di dalam Linux, namun visualisasi dan kontrol MAVLink ditarik ke OS Host (Windows) untuk performa UI yang maksimal.

1. Download dan Install **QGroundControl** untuk Windows dari [Official Site](https://docs.qgroundcontrol.com/).
2. **Network Bypass:** Saat pertama kali dibuka, Windows Defender akan menahan koneksi. Pastikan checklist **Private & Public Networks** diizinkan. Jika tidak, QGC akan buta jaringan dan paket MAVLink akan ter-drop.
3. Jalankan `make px4_sitl gz_x500` di WSL/Terminal, lalu buka QGC. Koneksi UDP akan otomatis terjalin (*"Vehicle Ready"*).

---

## ⚠️ Rules of Engagement (READ BEFORE ASKING)
Sebelum mengirim *error log* ke grup, pastikan kalian sudah melakukan audit mandiri:

1. **"Gazebo saya layarnya hitam / UI nge-glitch!"** -> Update *driver* VGA Host (Windows/Linux) kalian. WSLg merender GUI langsung melalui *driver* GPU Host.
2. **"QGC tidak connect ke simulasi!"** -> Cek konfigurasi Firewall. 99% kasus koneksi gagal karena *packet drop* di *firewall*.
3. **"Error saat kompilasi / Make failed!"** -> Scroll terminal ke atas, cari baris MERAH pertama yang muncul. Jangan hanya melihat pesan *"Failed"* di akhir. Baca *log*-nya.

> **RTFM (Read The Fucking Manual):** The instructions above are precise and tested. Execute them exactly as written.

---

## 👁️ MMATS-15 (Microservice Multisensor Autonomous Targetting System)
Inspired by the decoupled, high-speed guidance systems of the AGM-114 and AIM-9 missiles, **MMATS-15** is our hyper-optimized visual targeting architecture. Built heavily on the **KISS (Keep It Simple, Stupid)** principle, it strips away bloated frameworks (like ROS2) in favor of raw UDP microservices.

The system incorporates a highly optimized **YOLOv8 Nano** object detection model (`Krti_model.pt`) designed for edge CPU inference (Raspberry Pi 5). Because we are constrained by hardware (budget laptops & edge CPUs), MMATS-15 is "Optimized by Design."

### 1. Zero-Cost Sensor Fusion (LiDAR + Bounding Box Area)
To completely bypass the heavy network I/O and CPU bottleneck of processing 3D point clouds from a depth camera, MMATS-15 extracts the area of the YOLO bounding box `(bx2 - bx1) * (by2 - by1)` as a fast pseudo-depth metric. On the physical drone, this will be cross-referenced with a cheap downward LiDAR for 2-way verification, creating a zero-latency, aerospace-grade redundancy system.

### 2. Data Scraping (`data_collector.py`)
Bypasses manual screenshotting by hooking directly into the drone's GStreamer UDP feed (`udpsrc port=5600`). Automatically scrapes high-fidelity training frames during manual flight to bridge the domain gap between Gazebo SITL and cheap real-world camera lenses.

### 3. Manual Keyboard Teleop (`teleop.py`)
A custom PyGame/MAVSDK script that bypasses QGroundControl to eliminate UDP port conflicts. Allows for precise, game-like manual flight control (`W A S D`, `Arrows`) to generate diverse datasets that mathematically harden the neural network against failure states.

### 4. Hardware-In-the-Loop Integration (`vision_test.py`)
A standalone vision integration test that subscribes to the live GStreamer feed, runs the YOLO AI natively, and extracts exact X/Y centroid coordinates. Allows developers to manually fly the drone via `teleop.py` while visually confirming bounding box lock-ons via a live OpenCV dashboard.

### 5. Simulation Time-Dilation (RTF Scaling)
If you're running Gazebo SITL on a potato laptop, the physics engine is going to choke and run at a **30-40% Real-Time Factor (RTF)**. That means 10 seconds of Python code execution translates to only 3 seconds of actual drone movement. It's flying in extreme slow-motion.

Because of this hardware bottleneck, the `timeout_counter` thresholds in `autopilot.py` have been massively **inflated**. If we didn't do this, the code would time out and reverse before the drone physically finishes crossing a camera blind spot or stopping over a pad. 

**WARNING:** If you are running this on a beefy desktop that hits 100% RTF, or if you deploy this code to the actual physical drone in the real world, you **MUST scale down the timeouts** in `autopilot.py`. If you don't, your drone is going to hang in the air for 15 seconds like an idiot waiting for a timeout that's scaled for a lagging simulator. Adjust your code before you fly IRL.
