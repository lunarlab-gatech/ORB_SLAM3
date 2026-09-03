#!/bin/bash

# Auto-monitor all running experiments and analyze when complete

while true; do
    # Count running processes
    RUNNING=$(ps aux | grep "root.*rgbd" | grep -v grep | wc -l)

    if [ $RUNNING -eq 0 ]; then
        echo "========================================"
        echo "ALL EXPERIMENTS COMPLETED at $(date)"
        echo "========================================"

        # Find all trajectory files
        echo -e "\n📊 ANALYZING RESULTS..."

        GT="/home/sgarimella34/multi-robot-coordination/hercules_datasets/ausenv_roadseq/Drone1/pose_world_frame.txt"

        # Check each trajectory
        declare -A RESULTS

        for log in rgbd_*.log; do
            name=$(basename $log .log)

            # Check if trajectory was saved
            if grep -q "Saving camera trajectory" $log; then
                # Extract trajectory filename
                traj=$(grep "Saving camera trajectory" $log | awk '{print $NF}' | tr -d '.')

                if [ -f "$traj" ]; then
                    # Run comparison
                    result=$(python3 "$(dirname "$0")/../analyze/compare_trajectory_simple.py" $GT $traj 2>/dev/null | grep "RMSE:" | awk '{print $2}')
                    scale=$(python3 "$(dirname "$0")/../analyze/compare_trajectory_simple.py" $GT $traj 2>/dev/null | grep "Scale:" | awk '{print $2}')

                    if [ -n "$result" ]; then
                        RESULTS["$name"]="$result $scale"
                        echo "✅ $name: RMSE=${result}m, Scale=${scale}x"
                    fi
                fi
            else
                echo "❌ $name: No trajectory saved (likely failed)"
            fi
        done

        # Find best result
        echo -e "\n🏆 BEST RESULT:"
        for key in "${!RESULTS[@]}"; do
            echo "$key: ${RESULTS[$key]}"
        done | sort -t' ' -k2 -n | head -1

        break
    fi

    # Still running - show progress
    echo "[$(date +%H:%M:%S)] $RUNNING experiments still running..."
    tail -1 rgbd_tuned_v2.log 2>/dev/null | grep -o "Processing.*"
    tail -1 rgbd_tuned_v3.log 2>/dev/null | grep -o "Processing.*"
    tail -1 rgbd_inertial_inverted.log 2>/dev/null | grep -o "Processing.*"

    sleep 120  # Check every 2 minutes
done
