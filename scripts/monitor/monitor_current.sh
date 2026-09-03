#!/bin/bash

while true; do
    clear
    echo "================================================"
    echo "  Experiment Monitor - $(date +%H:%M:%S)"
    echo "================================================"
    echo ""
    
    # SuperFine rerun
    if [ -f rgbd_superfine_rerun.log ]; then
        sf=$(tail -1 rgbd_superfine_rerun.log 2>/dev/null | grep -o "[0-9.]*%" | tail -1)
        echo "SuperFine (1.08, 12 levels):       ${sf:-Loading...}"
        if grep -q "Saving camera trajectory" rgbd_superfine_rerun.log 2>/dev/null; then
            echo "  ✅ COMPLETED - saved to CameraTrajectory.txt"
        fi
    fi
    
    # IMU v2
    if [ -f rgbd_inertial_rotation_v2.log ]; then
        imu=$(tail -1 rgbd_inertial_rotation_v2.log 2>/dev/null | grep -o "[0-9.]*%" | tail -1)
        echo "IMU Rotation v2 (200Hz):           ${imu:-Loading...}"
        if grep -q "Saving camera trajectory" rgbd_inertial_rotation_v2.log 2>/dev/null; then
            echo "  ✅ COMPLETED - saved to imu_rotation_v2_traj.txt"
        fi
    fi
    
    echo ""
    echo "Best RGB-D so far: Scale Focus = 6.86m RMSE"
    echo "Target: < 3m RMSE"
    echo ""
    
    # Check if both completed
    sf_done=$(grep -q "Saving camera trajectory" rgbd_superfine_rerun.log 2>/dev/null && echo "yes" || echo "no")
    imu_done=$(grep -q "Saving camera trajectory" rgbd_inertial_rotation_v2.log 2>/dev/null && echo "yes" || echo "no")
    
    if [ "$sf_done" = "yes" ] && [ "$imu_done" = "yes" ]; then
        echo "🎉 ALL EXPERIMENTS COMPLETED!"
        break
    fi
    
    sleep 120
done
