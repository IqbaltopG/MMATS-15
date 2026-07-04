#!/bin/bash
# Script untuk Reset / Respawn Drone ke titik awal tanpa harus restart Gazebo

echo "[+] Mengirim sinyal Reset World ke Gazebo..."
gz service -s /world/KRTI_2026/control \
--reqtype gz.msgs.WorldControl \
--reptype gz.msgs.Boolean \
--timeout 2000 \
--req 'reset: {all: true}'

if [ $? -eq 0 ]; then
    echo "[+] Drone berhasil di-respawn ke titik start!"
    echo "[!] Tunggu sekitar 5 detik biar otak PX4 (EKF) reset, terus jalanin ulang python autopilot.py"
else
    echo "[-] Gagal nge-reset Gazebo. Pastikan Gazebo lagi jalan."
fi
