# 🚁 Autonomous Quadcopter System - Precision Payload Delivery

![Python](https://img.shields.io/badge/Python-3.10-blue?style=for-the-badge&logo=python)
![PX4](https://img.shields.io/badge/PX4_Autopilot-SITL-blueviolet?style=for-the-badge)
![MAVSDK](https://img.shields.io/badge/MAVSDK-Enabled-success?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)

## 📌 Executive Summary
This repository contains the foundational architecture for an autonomous quadcopter system (X500 frame) designed for precision pathfinding and automated payload delivery. Built utilizing **MAVSDK-Python** and tested within the **PX4 SITL (Software In The Loop)** and **Gazebo** environments, the system emphasizes high availability, dynamic yaw (bearing) calculations, and macro-level mission execution.

## 🛠️ Arsenal & Tech Stack
* **Core Logic:** Python 3.10, MAVSDK
* **Flight Stack:** PX4 Autopilot
* **Simulation Engine:** Gazebo (gz_x500)
* **Ground Control Station:** QGroundControl (QGC)
* **OS Environment:** Xubuntu

---

## 🚀 Environment Setup & Hardware Acceleration (The Nvidia Bypass)
Running Gazebo simulations on Linux machines with hybrid graphics often leads to VRAM bottlenecks and system hangs due to the OS defaulting to Integrated Graphics. 

To ensure Gazebo pulls resources directly from the **Dedicated GPU (Nvidia)** and prevents *kernel panics* or memory overloads, I use a specific hardware profiling bypass.

**To initiate the PX4 SITL simulation with Nvidia Offloading, execute the following command:**

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia make px4_sitl gz_x500
