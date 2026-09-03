#!/bin/bash

while true; do
    clear
    echo "================================================"
    echo "  ORB-SLAM3 Experiment Monitor - $(date +%H:%M:%S)"
    echo "================================================"
    echo ""
    
    # RGB-D SuperFine
    if [ -f rgbd_superfine.log ]; then
        sf=$(tail -1 rgbd_superfine.log 2>/dev/null | grep -o "[0-9.]*%" | tail -1)
        echo "SuperFine (1.08, 12lvl):  ${sf:-Loading...}"
    fi
    
    # RGB-D UltraFine
    if [ -f rgbd_ultrafine.log ]; then
        uf=$(tail -1 rgbd_ultrafine.log 2>/dev/null | grep -o "[0-9.]*%" | tail -1)
        echo "UltraFine (1.05, 15lvl):  ${uf:-Loading...}"
    fi
    
    # IMU Rotation
    if [ -f rgbd_inertial_rotation.log ]; then
        imu=$(tail -1 rgbd_inertial_rotation.log 2>/dev/null | grep -o "[0-9.]*%" | tail -1)
        echo "IMU Rotation (200Hz):     ${imu:-Loading...}"
    fi
    
    echo ""
    echo "Best so far: Scale Focus = 6.86m RMSE"
    echo "Target: < 3m RMSE"
    echo ""
    echo "Press Ctrl+C to exit"
    
    # Check if all completed
    sf_done=$(grep -q "median tracking time" rgbd_superfine.log 2>/dev/null && echo "yes" || echo "no")
    uf_done=$(grep -q "median tracking time" rgbd_ultrafine.log 2>/dev/null && echo "yes" || echo "no")
    imu_done=$(grep -q "median tracking time" rgbd_inertial_rotation.log 2>/dev/null && echo "yes" || echo "no")
    
    if [ "$sf_done" = "yes" ] && [ "$uf_done" = "yes" ] && [ "$imu_done" = "yes" ]; then
        echo ""
        echo "ALL EXPERIMENTS COMPLETED!"
        break
    fi
    
    sleep 60
done
