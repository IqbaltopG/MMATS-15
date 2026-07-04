# 🚁 Autonomous Quadcopter System - Precision Payload Delivery

![Python](https://img.shields.io/badge/Python-3.10-blue?style=for-the-badge&logo=python)
![PX4](https://img.shields.io/badge/PX4_Autopilot-SITL-blueviolet?style=for-the-badge)
![MAVSDK](https://img.shields.io/badge/MAVSDK-Enabled-success?style=for-the-badge)
![License](https://img.shields.io/badge/License-Unlicense-black?style=for-the-badge)

## 📌 Executive Summary
This repository contains the foundational architecture for an autonomous quadcopter system (X500 frame) designed for precision pathfinding and automated payload delivery. Built utilizing **MAVSDK-Python** and tested within the **PX4 SITL (Software In The Loop)** and **Gazebo** environments, the system emphasizes high availability, dynamic yaw (bearing) calculations, and macro-level mission execution.

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
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia make px4_sitl gz_x500
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
