#!/bin/bash
# Monitor Stereo Scale Focus Experiment

echo "========================================"
echo "  Stereo Scale Focus Monitor"
echo "========================================"
echo ""

# Count total frames
TOTAL_FRAMES=$(wc -l < /home/sgarimella34/multi-robot-coordination/hercules_datasets/ausenv_roadseq/Drone1/timestamps.txt)
echo "Total frames: $TOTAL_FRAMES"
echo ""

# Monitor loop
while true; do
    clear
    echo "========================================"
    echo "  Stereo Scale Focus - Live Progress"
    echo "========================================"
    echo ""

    # Check if process is still running
    if pgrep -f "stereo_airsim.*stereo_scale_focus" > /dev/null; then
        echo "Status: RUNNING ✓"

        # Check latest backup to estimate progress
        LATEST_BACKUP=$(ls -t /home/sgarimella34/slam/orbslam3_ws/stereo_scale_focus_backup_frame*_traj.txt 2>/dev/null | head -1)
        if [ -n "$LATEST_BACKUP" ]; then
            FRAME=$(echo "$LATEST_BACKUP" | grep -oP 'frame\K[0-9]+')
            PROGRESS=$(echo "scale=2; $FRAME * 100 / $TOTAL_FRAMES" | bc)
            echo "Progress: $FRAME / $TOTAL_FRAMES frames (${PROGRESS}%)"
            echo "Last backup: $(basename $LATEST_BACKUP)"
        else
            echo "Progress: Loading vocabulary..."
        fi
    else
        echo "Status: COMPLETED or STOPPED"

        # Check for final trajectory
        if [ -f "/home/sgarimella34/slam/orbslam3_ws/stereo_scale_focus_traj.txt" ]; then
            echo ""
            echo "Final trajectory saved!"
            echo "  - stereo_scale_focus_traj.txt"
            echo "  - stereo_scale_focus_kf_traj.txt"
            echo ""
            echo "Run comparison with:"
            echo "  cd ~/slam/orbslam3_ws"
            echo "  python3 scripts/analyze/compare_trajectory_simple.py \\"
            echo "    ~/multi-robot-coordination/hercules_datasets/ausenv_roadseq/Drone1/pose_world_frame.txt \\"
            echo "    stereo_scale_focus_traj.txt"
        fi
        break
    fi

    echo ""
    echo "Recent log output:"
    echo "----------------------------------------"
    tail -20 /tmp/stereo_scale_focus.log 2>/dev/null | grep -E "Processing|frame|Loading|SLAM" || echo "No log output yet..."
    echo "----------------------------------------"
    echo ""
    echo "Press Ctrl+C to exit monitor (experiment continues)"

    sleep 10
done
