#!/bin/bash
# Run ORB-SLAM3 RGB-D-Inertial mode on Drone1 dataset

echo "Starting ORB-SLAM3 RGB-D-Inertial for Drone1 dataset..."
echo "This will show the Pangolin 3D viewer with real-time SLAM visualization"
echo ""

xhost +si:localuser:root

docker run --rm -it --net=host \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/slam/orbslam3_ws/src":/workspaces/src:rw \
  -v "$HOME/slam/orbslam3_ws/data":/data:rw \
  -v "$HOME/hercules_datasets":/datasets:rw \
  orbslam3-ubuntu22 \
  bash -c "cd /workspaces/src/ORB_SLAM3 && ./Examples/RGB-D-Inertial/rgbd_inertial_airsim Vocabulary/ORBvoc.txt /datasets/ausenv_roadseq_2ugvuav/Drone1_RGBD_Inertial.yaml /datasets/ausenv_roadseq_2ugvuav/Drone1/rgb_stereo_left /datasets/ausenv_roadseq_2ugvuav/Drone1/depth /datasets/ausenv_roadseq_2ugvuav/Drone1/imu.txt drone1_trajectory"
