# ORB-SLAM3 with AirSim Dataset - Complete Setup Guide

This document contains all instructions to replicate the ORB-SLAM3 setup with AirSim drone datasets.

## Overview

- **Goal**: Run ORB-SLAM3 on AirSim drone dataset (RGB-D and RGB-D-Inertial modes)
- **Dataset location**: `~/hercules_datasets/ausenv_roadseq_2ugvuav/Drone1`
- **Best result achieved**: RGB-D only mode with 3000 features = **8.9m RMSE** (1.16x scale)

## Directory Structure

```
~/slam/orbslam3_ws/
├── docker/
│   ├── Dockerfile
│   ├── build_image.sh          # Build the image, tagged orbslam3-ubuntu22
│   ├── run_container.sh        # Create+start the persistent "orbslam3" container
│   └── enter_container.sh      # Attach a shell to the running "orbslam3" container
├── src/
│   └── ORB_SLAM3/
│       ├── Examples/RGB-D/rgbd_airsim.cc          # Custom RGB-D loader
│       ├── Examples/RGB-D-Inertial/rgbd_inertial_airsim.cc  # Custom RGB-D-Inertial loader
│       └── CMakeLists.txt                          # Modified to include custom loaders
├── compare_trajectory_simple.py                    # Trajectory comparison script
└── CLAUDE.md                                       # This file

~/hercules_datasets/ausenv_roadseq_2ugvuav/
├── Drone1/
│   ├── rgb_stereo_left/                           # RGB images (*.png)
│   ├── depth/                                      # Depth as .npy files (meters)
│   ├── depth_png16/                               # Converted 16-bit PNG depth (millimeters) - MUST CREATE
│   ├── synthetic_imu_9axis_200Hz.txt              # IMU data: t ax ay az gx gy gz ...
│   ├── pose_world_frame.txt                       # Ground truth trajectory
│   └── timestamps.txt                             # Frame timestamps - MUST CREATE
├── Drone1_RGBD.yaml                               # RGB-D config
└── Drone1_RGBD_Inertial.yaml                      # RGB-D-Inertial config
```

---

## Step 1: Create Docker Image

### 1.1 Dockerfile

**Use [`docker/Dockerfile`](docker/Dockerfile) in this repo as the source of truth** — it
installs build tooling, OpenCV/Eigen/Boost, and Pangolin v0.8. (An earlier, now-outdated
version of this Dockerfile used to be inlined here; it drifted out of sync with the real
file as dependencies got added, so it's been removed to avoid the same thing happening again
— always check `docker/Dockerfile` directly rather than trusting a copy in prose.)

### 1.2 Build Docker Image

```bash
cd ~/slam/orbslam3_ws/docker
./build_image.sh
```

This tags the image `orbslam3-ubuntu22` (build args set the in-container user's
UID/GID to match the host user, so files written into the mounted repo/datasets
aren't owned by root).

### 1.3 Start the Container

```bash
./run_container.sh
```

Creates and attaches to a persistent container named `orbslam3`, with the repo
mounted at `~/orb_slam3_ws` and the dataset drive at `~/data` (both inside the
container, under the container user's home). Leave this shell
attached, or detach and reattach later — from another terminal, `./enter_container.sh`
opens an additional shell in the same running container (starting it first if it
had been stopped).

---

## Step 2: Clone and Modify ORB-SLAM3

### 2.1 Clone ORB-SLAM3

```bash
cd ~/slam/orbslam3_ws/src
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
```

### 2.2 Modify CMakeLists.txt

Change C++ standard from C++11 to C++14 (required for Pangolin v0.8).

Find this line in `CMakeLists.txt`:
```cmake
set(CMAKE_CXX_STANDARD 11)
```

Change to:
```cmake
set(CMAKE_CXX_STANDARD 14)
```

Also add these custom executables at the end of CMakeLists.txt (before the final closing):

```cmake
# Custom AirSim RGB-D example (no IMU)
add_executable(rgbd_airsim
        Examples/RGB-D/rgbd_airsim.cc)
target_link_libraries(rgbd_airsim ${PROJECT_NAME})

# Custom AirSim RGB-D-Inertial example
add_executable(rgbd_inertial_airsim
        Examples/RGB-D-Inertial/rgbd_inertial_airsim.cc)
target_link_libraries(rgbd_inertial_airsim ${PROJECT_NAME})
```

### 2.3 Create Custom RGB-D Loader

Create `src/ORB_SLAM3/Examples/RGB-D/rgbd_airsim.cc`:

```cpp
/**
* Custom RGB-D loader for AirSim datasets (no IMU)
* Viewer disabled for headless operation
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<opencv2/core/core.hpp>
#include<System.h>

using namespace std;

void LoadImages(const string &strPathRGB, const string &strPathDepth, const string &strPathTimes,
                vector<string> &vstrImageRGB, vector<string> &vstrImageDepth, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    if(argc < 5)
    {
        cerr << endl << "Usage: ./rgbd_airsim path_to_vocabulary path_to_settings path_to_rgb path_to_depth (trajectory_file_name)" << endl;
        return 1;
    }

    string file_name;
    if (argc == 6)
    {
        file_name = string(argv[5]);
    }

    vector<string> vstrImageRGB;
    vector<string> vstrImageDepth;
    vector<double> vTimestamps;

    string pathRGB = string(argv[3]);
    string pathDepth = string(argv[4]);
    // Use depth_png16 for 16-bit depth images in millimeters
    if (pathDepth.find("/depth") != string::npos) {
        size_t pos = pathDepth.find("/depth");
        pathDepth = pathDepth.substr(0, pos) + "/depth_png16";
        cout << "Using 16-bit depth from: " << pathDepth << endl;
    }
    string pathTimes = pathRGB.substr(0, pathRGB.find_last_of("/")) + "/timestamps.txt";

    LoadImages(pathRGB, pathDepth, pathTimes, vstrImageRGB, vstrImageDepth, vTimestamps);

    int nImages = vstrImageRGB.size();

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    cout << "Loaded " << nImages << " images" << endl;

    // Create SLAM system (viewer disabled for headless mode)
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD, false, 0, file_name);

    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cv::Mat imRGB, imDepth;

    for(int ni=0; ni<nImages; ni++)
    {
        imRGB = cv::imread(vstrImageRGB[ni], cv::IMREAD_UNCHANGED);
        imDepth = cv::imread(vstrImageDepth[ni], cv::IMREAD_UNCHANGED);

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load RGB image at: " << vstrImageRGB[ni] << endl;
            return 1;
        }

        if(imDepth.empty())
        {
            cerr << endl << "Failed to load depth image at: " << vstrImageDepth[ni] << endl;
            return 1;
        }

        double tframe = vTimestamps[ni];

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        SLAM.TrackRGBD(imRGB, imDepth, tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack[ni] = ttrack;

        // Progress output every 10 frames
        if(ni % 10 == 0 || ni < 5)
            cout << "Processing frame " << ni << "/" << nImages << " (" << (100.0*ni/nImages) << "%)" << endl;

        // Process at maximum speed (no sleep)
    }

    SLAM.Shutdown();

    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathRGB, const string &strPathDepth, const string &strPathTimes,
                vector<string> &vstrImageRGB, vector<string> &vstrImageDepth, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(50000);
    vstrImageRGB.reserve(50000);
    vstrImageDepth.reserve(50000);

    while(!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if(!s.empty())
        {
            double t = stod(s);
            vTimeStamps.push_back(t);
            vstrImageRGB.push_back(strPathRGB + "/" + s + ".png");
            vstrImageDepth.push_back(strPathDepth + "/" + s + ".png");
        }
    }
}
```

### 2.4 Create Custom RGB-D-Inertial Loader

Create `src/ORB_SLAM3/Examples/RGB-D-Inertial/rgbd_inertial_airsim.cc`:

```cpp
/**
* Custom RGB-D-Inertial loader for AirSim datasets
* IMU format: timestamp ax ay az gx gy gz (space-separated)
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include <ctime>
#include <sstream>

#include <opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"

using namespace std;

void LoadImages(const string &strPathRGB, const string &strPathDepth, const string &strPathTimes,
                vector<string> &vstrImageRGB, vector<string> &vstrImageDepth, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

int main(int argc, char **argv)
{
    if(argc < 6)
    {
        cerr << endl << "Usage: ./rgbd_inertial_airsim path_to_vocabulary path_to_settings path_to_rgb path_to_depth path_to_imu (trajectory_file_name)" << endl;
        return 1;
    }

    string file_name;
    if (argc == 7)
    {
        file_name = string(argv[6]);
    }

    vector<string> vstrImageRGB;
    vector<string> vstrImageDepth;
    vector<double> vTimestampsCam;
    vector<cv::Point3f> vAcc, vGyro;
    vector<double> vTimestampsImu;

    string pathRGB = string(argv[3]);
    string pathDepth = string(argv[4]);
    string pathImu = string(argv[5]);

    // Use depth_png16 for 16-bit depth images in millimeters
    if (pathDepth.find("/depth") != string::npos) {
        size_t pos = pathDepth.find("/depth");
        pathDepth = pathDepth.substr(0, pos) + "/depth_png16";
        cout << "Using 16-bit depth from: " << pathDepth << endl;
    }

    string pathTimes = pathRGB.substr(0, pathRGB.find_last_of("/")) + "/timestamps.txt";

    cout << "Loading images..." << endl;
    LoadImages(pathRGB, pathDepth, pathTimes, vstrImageRGB, vstrImageDepth, vTimestampsCam);
    cout << "Loaded " << vstrImageRGB.size() << " images" << endl;

    cout << "Loading IMU..." << endl;
    LoadIMU(pathImu, vTimestampsImu, vAcc, vGyro);
    cout << "Loaded " << vTimestampsImu.size() << " IMU measurements" << endl;

    int nImages = vstrImageRGB.size();
    int nImu = vTimestampsImu.size();

    if((nImages<=0)||(nImu<=0))
    {
        cerr << "ERROR: Failed to load images or IMU" << endl;
        return 1;
    }

    int first_imu = 0;
    while(vTimestampsImu[first_imu]<=vTimestampsCam[0])
        first_imu++;
    first_imu--;

    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Images: " << nImages << endl;
    cout << "IMU: " << nImu << endl;
    cout.precision(17);

    // Create SLAM system (viewer disabled for headless mode)
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_RGBD, false, 0, file_name);

    cv::Mat imRGB, imDepth;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    for(int ni=0; ni<nImages; ni++)
    {
        imRGB = cv::imread(vstrImageRGB[ni], cv::IMREAD_UNCHANGED);
        imDepth = cv::imread(vstrImageDepth[ni], cv::IMREAD_UNCHANGED);

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load RGB image at: " << vstrImageRGB[ni] << endl;
            return 1;
        }

        if(imDepth.empty())
        {
            cerr << endl << "Failed to load depth image at: " << vstrImageDepth[ni] << endl;
            return 1;
        }

        double tframe = vTimestampsCam[ni];

        vImuMeas.clear();

        if(ni>0)
        {
            while(vTimestampsImu[first_imu]<=vTimestampsCam[ni])
            {
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[first_imu].x, vAcc[first_imu].y, vAcc[first_imu].z,
                                                         vGyro[first_imu].x, vGyro[first_imu].y, vGyro[first_imu].z,
                                                         vTimestampsImu[first_imu]));
                first_imu++;
            }
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        SLAM.TrackRGBD(imRGB, imDepth, tframe, vImuMeas);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Progress output every 10 frames
        if(ni % 10 == 0 || ni < 5)
            cout << "Processing frame " << ni << "/" << nImages << " (" << (100.0*ni/nImages) << "%)" << endl;

        // Process at maximum speed (no sleep)
    }

    SLAM.Shutdown();

    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    if (!file_name.empty())
    {
        SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strPathRGB, const string &strPathDepth, const string &strPathTimes,
                vector<string> &vstrImageRGB, vector<string> &vstrImageDepth, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(50000);
    vstrImageRGB.reserve(50000);
    vstrImageDepth.reserve(50000);

    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            double t = stod(s);
            vTimeStamps.push_back(t);
            vstrImageRGB.push_back(strPathRGB + "/" + s + ".png");
            vstrImageDepth.push_back(strPathDepth + "/" + s + ".png");
        }
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(500000);
    vAcc.reserve(500000);
    vGyro.reserve(500000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;

            // Parse space-separated values: timestamp ax ay az gx gy gz (synthetic IMU format)
            istringstream iss(s);
            iss >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5] >> data[6];

            vTimeStamps.push_back(data[0]);
            vAcc.push_back(cv::Point3f(data[1], data[2], data[3]));
            vGyro.push_back(cv::Point3f(data[4], data[5], data[6]));
        }
    }
}
```

### 2.5 Build ORB-SLAM3 in Docker

With the container from Step 1.3 running (repo mounted at `~/orb_slam3_ws`):

```bash
cd ~/slam/orbslam3_ws/docker
./enter_container.sh
```

Then, inside the container, fetch the ORB vocabulary (`build.sh` expects
`Vocabulary/ORBvoc.txt.tar.gz` to already exist and will fail partway through otherwise):

```bash
cd ~/orb_slam3_ws
./scripts/setup/fetch_orb_vocabulary.sh
```

Then build:

```bash
cd ~/orb_slam3_ws/src/ORB_SLAM3
./build.sh
```

This will take several minutes. It builds:
- Thirdparty libraries (DBoW2, g2o, Sophus)
- ORB_SLAM3 main library
- All examples including our custom loaders

---

## Step 3: Prepare Dataset

### 3.1 Create timestamps.txt

The timestamps file must be **numerically sorted** (not alphabetically):

```bash
cd ~/hercules_datasets/ausenv_roadseq_2ugvuav/Drone1
ls rgb_stereo_left/*.png | sed 's/rgb_stereo_left\///' | sed 's/\.png//' | sort -n > timestamps.txt
```

Verify the first few lines:
```bash
head -5 timestamps.txt
# Should show: 0.050000, 0.100000, 0.150000, etc.
```

### 3.2 Convert Depth Images to 16-bit PNG

AirSim stores depth as `.npy` files in **meters**. ORB-SLAM3 expects 16-bit PNG in **millimeters**.

Create Python script `convert_depth.py`:

```python
import numpy as np
import cv2
from pathlib import Path
from tqdm import tqdm

npy_dir = Path('/home/YOUR_USERNAME/hercules_datasets/ausenv_roadseq_2ugvuav/Drone1/depth')
output_dir = Path('/home/YOUR_USERNAME/hercules_datasets/ausenv_roadseq_2ugvuav/Drone1/depth_png16')
output_dir.mkdir(exist_ok=True)

npy_files = sorted(npy_dir.glob('*.npy'))

print(f"Converting {len(npy_files)} depth files from .npy (meters) to 16-bit PNG (millimeters)...")
for npy_file in tqdm(npy_files):
    depth_m = np.load(npy_file)
    depth_mm = depth_m * 1000.0  # meters to mm
    depth_mm[depth_mm > 65000] = 0  # Clip far/invalid depths
    depth_uint16 = depth_mm.astype(np.uint16)

    output_file = output_dir / f"{npy_file.stem}.png"
    cv2.imwrite(str(output_file), depth_uint16)

print("Conversion complete!")
```

Run it:
```bash
pip install numpy opencv-python tqdm
python3 convert_depth.py
```

---

## Step 4: Create Configuration Files

### 4.1 RGB-D Configuration (Drone1_RGBD.yaml)

Create `~/hercules_datasets/ausenv_roadseq_2ugvuav/Drone1_RGBD.yaml`:

```yaml
%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters (AirSim Drone1 - RGB-D only, no IMU)
#--------------------------------------------------------------------------------------------
File.version: "1.0"

Camera.type: "PinHole"

# Camera calibration parameters (752x480, 90 degree FOV)
Camera1.fx: 376.0
Camera1.fy: 376.0
Camera1.cx: 376.0
Camera1.cy: 240.0

# Distortion parameters
Camera1.k1: 0.0
Camera1.k2: 0.0
Camera1.p1: 0.0
Camera1.p2: 0.0

# Camera resolution
Camera.width: 752
Camera.height: 480

# Camera frames per second
Camera.fps: 20

# Color order of the images (0: BGR, 1: RGB)
Camera.RGB: 1

# Close/Far threshold
Stereo.ThDepth: 40.0
Stereo.b: 0.0745

# Depth map values factor (1000 = millimeters to meters)
RGBD.DepthMapFactor: 1000.0

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------
# ORB Extractor: Number of features per image (3000 for best accuracy)
ORBextractor.nFeatures: 3000

# ORB Extractor: Scale factor between levels in the scale pyramid
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
ORBextractor.iniThFAST: 15
ORBextractor.minThFAST: 5

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
```

### 4.2 RGB-D-Inertial Configuration (Drone1_RGBD_Inertial.yaml)

Create `~/hercules_datasets/ausenv_roadseq_2ugvuav/Drone1_RGBD_Inertial.yaml`:

```yaml
%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters (AirSim Drone1 - RGB-D-Inertial)
#--------------------------------------------------------------------------------------------
File.version: "1.0"

Camera.type: "PinHole"

# Camera calibration parameters (752x480, 90 degree FOV)
Camera1.fx: 376.0
Camera1.fy: 376.0
Camera1.cx: 376.0
Camera1.cy: 240.0

# Distortion parameters
Camera1.k1: 0.0
Camera1.k2: 0.0
Camera1.p1: 0.0
Camera1.p2: 0.0

# Camera resolution
Camera.width: 752
Camera.height: 480

# Camera frames per second
Camera.fps: 20

# Color order of the images (0: BGR, 1: RGB)
Camera.RGB: 1

# Close/Far threshold
Stereo.ThDepth: 40.0
Stereo.b: 0.0745

# Depth map values factor (1000 = millimeters to meters)
RGBD.DepthMapFactor: 1000.0

# Transformation from body-frame (IMU) to camera
# Adjust this based on your actual IMU-camera setup
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0]

# Do not insert KFs when recently lost
IMU.InsertKFsWhenLost: 0

# IMU noise parameters (near-zero for synthetic noiseless IMU)
IMU.NoiseGyro: 0.0001
IMU.NoiseAcc: 0.001
IMU.GyroWalk: 0.00001
IMU.AccWalk: 0.00001
IMU.Frequency: 200.0

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------
# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 3000

# ORB Extractor: Scale factor between levels in the scale pyramid
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
ORBextractor.iniThFAST: 15
ORBextractor.minThFAST: 5

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
```

---

## Step 5: Create Trajectory Comparison Script

Create `~/slam/orbslam3_ws/compare_trajectory_simple.py`:

```python
#!/usr/bin/env python3
"""
Simple trajectory comparison script for ORB-SLAM3 output vs ground truth.
Computes ATE (Absolute Trajectory Error) with SE(3) alignment.
"""

import numpy as np
import sys

def load_tum_trajectory(filepath):
    """Load trajectory in TUM format: timestamp tx ty tz qx qy qz qw"""
    data = np.loadtxt(filepath)
    timestamps = data[:, 0]
    positions = data[:, 1:4]
    quaternions = data[:, 4:8]
    return timestamps, positions, quaternions

def align_trajectories(gt_pos, est_pos):
    """Align estimated trajectory to ground truth using Umeyama alignment with scale."""
    # Center the trajectories
    gt_mean = np.mean(gt_pos, axis=0)
    est_mean = np.mean(est_pos, axis=0)

    gt_centered = gt_pos - gt_mean
    est_centered = est_pos - est_mean

    # Compute scale
    gt_var = np.sum(gt_centered ** 2)
    est_var = np.sum(est_centered ** 2)
    scale = np.sqrt(gt_var / est_var)

    # Compute rotation using SVD
    H = est_centered.T @ gt_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Ensure proper rotation (det = 1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Compute translation
    t = gt_mean - scale * (R @ est_mean)

    # Apply transformation
    est_aligned = scale * (est_pos @ R.T) + t

    return est_aligned, scale, R, t

def compute_ate(gt_pos, est_pos):
    """Compute Absolute Trajectory Error."""
    errors = np.linalg.norm(gt_pos - est_pos, axis=1)
    rmse = np.sqrt(np.mean(errors ** 2))
    mean = np.mean(errors)
    median = np.median(errors)
    std = np.std(errors)
    min_err = np.min(errors)
    max_err = np.max(errors)
    return rmse, mean, median, std, min_err, max_err, errors

def main():
    if len(sys.argv) < 3:
        print("Usage: python compare_trajectory_simple.py <ground_truth.txt> <estimated.txt>")
        sys.exit(1)

    gt_file = sys.argv[1]
    est_file = sys.argv[2]

    print("Loading trajectories...")
    gt_ts, gt_pos, gt_quat = load_tum_trajectory(gt_file)
    est_ts, est_pos, est_quat = load_tum_trajectory(est_file)

    print(f"Ground truth: {len(gt_ts)} poses")
    print(f"Estimated: {len(est_ts)} poses")

    # Synchronize by timestamp (find matching timestamps)
    gt_dict = {round(t, 3): i for i, t in enumerate(gt_ts)}
    matched_gt = []
    matched_est = []

    for i, t in enumerate(est_ts):
        t_rounded = round(t, 3)
        if t_rounded in gt_dict:
            matched_gt.append(gt_pos[gt_dict[t_rounded]])
            matched_est.append(est_pos[i])

    matched_gt = np.array(matched_gt)
    matched_est = np.array(matched_est)

    print(f"Synchronized: {len(matched_gt)} poses")

    if len(matched_gt) < 3:
        print("ERROR: Not enough matching timestamps!")
        sys.exit(1)

    print("\nAligning trajectories...")
    est_aligned, scale, R, t = align_trajectories(matched_gt, matched_est)

    rmse, mean, median, std, min_err, max_err, errors = compute_ate(matched_gt, est_aligned)

    print("\n" + "=" * 60)
    print("ABSOLUTE TRAJECTORY ERROR (ATE)")
    print("=" * 60)
    print(f"RMSE:   {rmse:.6f} m")
    print(f"Mean:   {mean:.6f} m")
    print(f"Median: {median:.6f} m")
    print(f"Std:    {std:.6f} m")
    print(f"Min:    {min_err:.6f} m")
    print(f"Max:    {max_err:.6f} m")
    print(f"\nScale:  {scale:.6f}")
    print("=" * 60)

    # Save plot if matplotlib available
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 2, figsize=(14, 5))

        # XY plot
        axes[0].plot(matched_gt[:, 0], matched_gt[:, 1], 'b-', label='Ground Truth', linewidth=1)
        axes[0].plot(est_aligned[:, 0], est_aligned[:, 1], 'r-', label='Estimated (aligned)', linewidth=1)
        axes[0].set_xlabel('X (m)')
        axes[0].set_ylabel('Y (m)')
        axes[0].set_title('Trajectory Comparison (XY plane)')
        axes[0].legend()
        axes[0].axis('equal')
        axes[0].grid(True)

        # Error over time
        axes[1].plot(errors, 'g-', linewidth=0.5)
        axes[1].axhline(y=rmse, color='r', linestyle='--', label=f'RMSE: {rmse:.2f}m')
        axes[1].set_xlabel('Frame')
        axes[1].set_ylabel('Error (m)')
        axes[1].set_title('Position Error Over Time')
        axes[1].legend()
        axes[1].grid(True)

        plt.tight_layout()
        plt.savefig('trajectory_comparison.png', dpi=150)
        print("\nSaved plot to: trajectory_comparison.png")
    except ImportError:
        print("\nMatplotlib not available, skipping plot generation")

if __name__ == "__main__":
    main()
```

---

## Step 6: Run ORB-SLAM3

Both commands below run inside the container (`cd ~/slam/orbslam3_ws/docker && ./enter_container.sh`),
with the repo at `~/orb_slam3_ws` and the dataset drive at `~/data`:

### 6.1 Run RGB-D Only Mode (RECOMMENDED - Best Results)

```bash
cd ~/orb_slam3_ws
./src/ORB_SLAM3/Examples/RGB-D/rgbd_airsim \
  ./src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  ~/data/ausenv_roadseq_2ugvuav/Drone1_RGBD.yaml \
  ~/data/ausenv_roadseq_2ugvuav/Drone1/rgb_stereo_left \
  ~/data/ausenv_roadseq_2ugvuav/Drone1/depth \
  output_trajectory
```

### 6.2 Run RGB-D-Inertial Mode

```bash
cd ~/orb_slam3_ws
./src/ORB_SLAM3/Examples/RGB-D-Inertial/rgbd_inertial_airsim \
  ./src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  ~/data/ausenv_roadseq_2ugvuav/Drone1_RGBD_Inertial.yaml \
  ~/data/ausenv_roadseq_2ugvuav/Drone1/rgb_stereo_left \
  ~/data/ausenv_roadseq_2ugvuav/Drone1/depth \
  ~/data/ausenv_roadseq_2ugvuav/Drone1/synthetic_imu_9axis_200Hz.txt \
  output_trajectory
```

### 6.3 Compare Results

After processing completes:

```bash
python3 compare_trajectory_simple.py \
  ~/hercules_datasets/ausenv_roadseq_2ugvuav/Drone1/pose_world_frame.txt \
  CameraTrajectory.txt
```

---

## Results Summary

| Mode | Features | RMSE | Scale | Notes |
|------|----------|------|-------|-------|
| RGB-D only | 1250 | 13.1m | 1.11x | Baseline |
| RGB-D only | 3000 | **8.9m** | 1.16x | **Best result** |
| RGB-D-Inertial | 2000 | 90.7m | 0.0006x | Scale collapse |
| RGB-D-Inertial | 5000 | 114m | 0.00004x | IMU issues |

**Recommendation**: Use RGB-D only mode with 3000 features for best results.

---

## Troubleshooting

### "Frame with timestamp older than previous frame"
- timestamps.txt is sorted alphabetically instead of numerically
- Fix: `ls rgb_stereo_left/*.png | sed 's/rgb_stereo_left\///' | sed 's/\.png//' | sort -n > timestamps.txt`

### "not enough acceleration"
- IMU initialization needs motion/rotation
- Drone starts moving around 2 seconds into the dataset
- This warning is normal at the start

### "Failed to open X display"
- Viewer is enabled but no display available
- The custom loaders have viewer disabled by default

### Scale is way off (0.001 or 400+)
- Depth images not converted properly
- Check that depth_png16/ contains 16-bit PNG files
- Verify RGBD.DepthMapFactor is 1000.0

### RGB-D-Inertial worse than RGB-D only
- IMU-camera transformation matrix might be incorrect
- IMU noise parameters might need tuning
- For synthetic noiseless IMU, use very low noise values

---

## Data Format Reference

### timestamps.txt
```
0.050000
0.100000
0.150000
...
```

### synthetic_imu_9axis_200Hz.txt
```
timestamp ax ay az gx gy gz [qx qy qz qw]
0.050000 -0.000000 -0.000000 -9.806650 0.000000 -0.000000 -0.000000 ...
```

### pose_world_frame.txt (Ground Truth)
```
timestamp tx ty tz qx qy qz qw
0.050000 -142.200000 -96.700000 1.290274 1.000000 0.000000 0.000000 0.000000
```

### CameraTrajectory.txt (Output)
```
timestamp tx ty tz qx qy qz qw
0.050000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000
```
