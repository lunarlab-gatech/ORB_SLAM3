#!/bin/bash

echo "=== Drone2 RGB-D Scale Focus - NO LOOP CLOSURE Monitor ==="
echo ""
echo "Dataset: Drone2 (4.70s to 892.60s, 17,759 frames)"
echo "Config: Scale Focus + Loop Closure DISABLED"
echo ""

# Check if process is running
if ps aux | grep -q "[r]gbd_airsim.*drone2_nolc"; then
    echo "Status: RUNNING"
    echo ""

    # Show process info
    echo "Process Info:"
    ps aux | grep "[r]gbd_airsim.*drone2_nolc" | awk '{printf "  CPU: %s%% | Memory: %s MB | Runtime: %s\n", $3, int($6/1024), $10}'
    echo ""

    # Check log for latest frame
    if [ -f "/home/sgarimella34/slam/orbslam3_ws/drone2_nolc_scalefocus_trimmed.log" ]; then
        echo "Latest Progress:"
        tail -5 /home/sgarimella34/slam/orbslam3_ws/drone2_nolc_scalefocus_trimmed.log | grep "Processing frame" | tail -1 | sed 's/^/  /'
    fi
    echo ""
else
    echo "Status: NOT RUNNING (completed or stopped)"
    echo ""

    # Check for final output files
    echo "Output Files:"
    cd /home/sgarimella34/slam/orbslam3_ws
    if [ -f "drone2_nolc_scalefocus_result.txt" ]; then
        count=$(wc -l < drone2_nolc_scalefocus_result.txt)
        echo "  ✓ drone2_nolc_scalefocus_result.txt: $count poses"
    fi
    if [ -f "drone2_nolc_scalefocus_kf_traj.txt" ]; then
        count=$(wc -l < drone2_nolc_scalefocus_kf_traj.txt)
        echo "  ✓ drone2_nolc_scalefocus_kf_traj.txt: $count keyframes"
    fi
    echo ""

    # Show final log output
    if [ -f "drone2_nolc_scalefocus_trimmed.log" ]; then
        echo "Final Log Output:"
        tail -15 drone2_nolc_scalefocus_trimmed.log | sed 's/^/  /'
    fi
fi

echo ""
echo "=== End Monitor ==="
