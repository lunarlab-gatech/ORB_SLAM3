#!/bin/bash

echo "=== Drone2 RGB-D Scale Focus Experiment Monitor ==="
echo ""
echo "Dataset: Drone2 (4.70s to 892.60s, 17,759 frames)"
echo "Config: Scale Focus (3000 features, scaleFactor=1.1, nLevels=10)"
echo ""

# Check if process is running
if ps aux | grep -q "[r]gbd_airsim.*drone2_scale_focus_trimmed"; then
    echo "Status: RUNNING"
    echo ""

    # Show process info
    echo "Process Info:"
    ps aux | grep "[r]gbd_airsim.*drone2_scale_focus_trimmed" | awk '{printf "  CPU: %s%% | Memory: %s MB | Runtime: %s\n", $3, int($6/1024), $10}'
    echo ""

    # Check for trajectory backup files
    echo "Progress Backups:"
    if [ -d "/home/sgarimella34/slam/orbslam3_ws" ]; then
        cd /home/sgarimella34/slam/orbslam3_ws
        backup_files=$(ls -t drone2_scale_focus_trimmed_* 2>/dev/null | head -5)
        if [ -n "$backup_files" ]; then
            for file in $backup_files; do
                count=$(wc -l < "$file")
                timestamp=$(stat -c %y "$file" | cut -d'.' -f1)
                progress=$(echo "scale=2; $count / 177.59" | bc)
                echo "  $file: $count poses ($progress%%) - $timestamp"
            done
        else
            echo "  No backups yet (processing initial frames)"
        fi
    fi
    echo ""

    # Show log file if it exists
    if [ -f "/home/sgarimella34/slam/orbslam3_ws/drone2_rgbd_scalefocus_trimmed.log" ]; then
        echo "Latest Log Output:"
        tail -10 /home/sgarimella34/slam/orbslam3_ws/drone2_rgbd_scalefocus_trimmed.log | sed 's/^/  /'
    fi
else
    echo "Status: NOT RUNNING (completed or stopped)"
    echo ""

    # Check for final output files
    echo "Output Files:"
    cd /home/sgarimella34/slam/orbslam3_ws
    if [ -f "CameraTrajectory.txt" ]; then
        count=$(wc -l < CameraTrajectory.txt)
        echo "  ✓ CameraTrajectory.txt: $count poses"
    fi
    if [ -f "KeyFrameTrajectory.txt" ]; then
        count=$(wc -l < KeyFrameTrajectory.txt)
        echo "  ✓ KeyFrameTrajectory.txt: $count keyframes"
    fi
    echo ""

    # Show final log output
    if [ -f "drone2_rgbd_scalefocus_trimmed.log" ]; then
        echo "Final Log Output:"
        tail -20 drone2_rgbd_scalefocus_trimmed.log | sed 's/^/  /'
    fi
fi

echo ""
echo "=== End Monitor ==="
