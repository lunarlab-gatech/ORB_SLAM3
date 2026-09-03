#!/bin/bash

while true; do
    if grep -q "Shutdown" /home/sgarimella34/slam/orbslam3_ws/rgbd_inertial_trimmed.log 2>/dev/null; then
        echo "✅ IMU v4 COMPLETED at $(date +%H:%M:%S)!"
        tail -20 /home/sgarimella34/slam/orbslam3_ws/rgbd_inertial_trimmed.log
        break
    fi
    
    progress=$(tail -1 /home/sgarimella34/slam/orbslam3_ws/rgbd_inertial_trimmed.log 2>/dev/null | grep -o "[0-9.]*%" | tail -1)
    echo "[$(date +%H:%M:%S)] IMU v4 Progress: ${progress:-Loading...}"
    
    sleep 120
done
