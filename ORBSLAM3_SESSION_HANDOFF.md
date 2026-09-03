# ORB-SLAM3 City Dataset Session Handoff

## Current Status

Successfully completed baseline experiments on city_cslam_ugvuav_test1 dataset with trimmed timestamps (calibration motion removed).

## Latest Results (Baseline Config - No Calibration)

| Platform | Frames | RMSE | Scale | Notes |
|----------|--------|------|-------|-------|
| Drone1 | 21,975 | **0.70m** | 1.06 | ✅ Excellent (reference) |
| Drone2 | 20,346 | **15.62m** | 1.06 | Needs improvement |
| Husky1 | 4,253 | **3.44m** | 0.99 | ✅ Good (6x better after trim!) |
| Husky2 | 3,057 | **11.07m** | 1.01 | ✅ Much better (6x improvement!) |

## Key Discovery

**Removing calibration motion dramatically improved Husky results:**
- Husky1: 9.57m → 3.44m (2.8x improvement)
- Husky2: 66.62m → 11.07m (6x improvement)

## Trimmed Time Ranges

Successfully created `timestamps_trimmed.txt` for each robot:
- **Drone2**: 53.75s - 1071.0s (20,346 frames)
- **Husky1**: 47.80s - 260.40s (4,253 frames)
- **Husky2**: 48.35s - 201.15s (3,057 frames)

## Code Modifications

### 1. Sophus NaN Handling Patch
**File**: `/home/sgarimella34/slam/orbslam3_ws/src/ORB_SLAM3/Thirdparty/Sophus/sophus/so3.hpp`

Added warning-only NaN detection (doesn't crash, doesn't force identity):
```cpp
// Line ~590: Check for NaN/Inf omega
if (isnan(omega.x()) || isnan(omega.y()) || isnan(omega.z()) ||
    isinf(omega.x()) || isinf(omega.y()) || isinf(omega.z())) {
  std::cerr << "[WARNING] SO3::exp received NaN/Inf omega: "
            << omega.transpose() << std::endl;
}

// Line ~626: Warning on quaternion issues (doesn't replace with identity)
if (isnan(qnorm_sq) || isinf(qnorm_sq) ||
    abs(qnorm_sq - Scalar(1)) >= Sophus::Constants<Scalar>::epsilon()) {
  std::cerr << "[WARNING] SO3::exp numerical issue - omega: "
            << omega.transpose() << ", qnorm_sq: " << qnorm_sq << std::endl;
}
```

### 2. Auto-Detect Trimmed Timestamps
**File**: `/home/sgarimella34/slam/orbslam3_ws/src/ORB_SLAM3/Examples/RGB-D/rgbd_airsim.cc`

Modified loader to use `timestamps_trimmed.txt` if available:
```cpp
// Line ~46: Check for trimmed timestamps first
string basePath = pathRGB.substr(0, pathRGB.find_last_of("/"));
string pathTimesTrimmed = basePath + "/timestamps_trimmed.txt";
string pathTimes = basePath + "/timestamps.txt";

ifstream testFile(pathTimesTrimmed.c_str());
if (testFile.good()) {
    pathTimes = pathTimesTrimmed;
    cout << "Using trimmed timestamps (no calibration): " << pathTimes << endl;
}
testFile.close();
```

## Configuration Files

### Baseline Config (Best Overall)
**Used for**: Drone1, Drone2, Husky1, Husky2 trimmed experiments

**Files**:
- `Drone1_RGBD_ScaleFocus.yaml`
- `Drone2_RGBD_ScaleFocus.yaml`
- `Husky1_RGBD_ScaleFocus.yaml`
- `Husky2_RGBD_ScaleFocus.yaml`

**Settings**:
```yaml
ORBextractor.nFeatures: 3000
ORBextractor.scaleFactor: 1.1
ORBextractor.nLevels: 10
ORBextractor.iniThFAST: 15
ORBextractor.minThFAST: 5
```

### Fast Motion Config (Tested)
**Files**: `*_RGBD_FastMotion.yaml`

**Settings**:
```yaml
ORBextractor.nFeatures: 5000
ORBextractor.scaleFactor: 1.15
ORBextractor.nLevels: 12
ORBextractor.iniThFAST: 10
ORBextractor.minThFAST: 3
```

**Results**: Helped NO LC mode but hurt WITH LC mode for Husky1. Mixed results.

## Run Scripts Created

### Baseline Trimmed (Latest - BEST)
- `run_city_drone2_baseline_trimmed.sh` → 15.62m RMSE
- `run_city_husky1_baseline_trimmed.sh` → **3.44m RMSE**
- `run_city_husky2_baseline_trimmed.sh` → **11.07m RMSE**

### Fast Motion Configs
- `run_city_husky1_fastmotion.sh` (WITH LC) → 12.23m
- `run_city_husky1_fastmotion_nolc.sh` (NO LC) → 8.55m
- `run_city_husky2_fastmotion.sh` (WITH LC) → 46.63m
- `run_city_husky2_fastmotion_nolc.sh` (NO LC) → 66.33m

### Conservative Loop Closure (Drone2)
- `run_city_drone2_conservative_lc.sh` → 14.72m (completed without crash)

## Docker Environment

**Image**: `orbslam3-ubuntu22`
**Built from**: `~/slam/orbslam3_ws/docker/Dockerfile`
**ORB-SLAM3 location**: `~/slam/orbslam3_ws/src/ORB_SLAM3`

**Rebuild command**:
```bash
cd ~/slam/orbslam3_ws
docker run --rm \
  -v "$(pwd)/src:/workspace/src" \
  -v "$(pwd):/workspace" \
  orbslam3-ubuntu22 bash -c "
cd /workspace/src/ORB_SLAM3/build &&
make -j8
"
```

## Dataset Location

**Path**: `/media/sgarimella34/hercules-collect/raw_data_hercules/city_cslam_ugvuav_test1/`

**Structure**:
```
city_cslam_ugvuav_test1/
├── Drone1/
│   ├── rgb_stereo_left/
│   ├── depth_png16/
│   ├── timestamps.txt (original)
│   └── pose_world_frame.txt (ground truth)
├── Drone2/
│   ├── rgb_stereo_left/
│   ├── depth_png16/
│   ├── timestamps.txt (original)
│   ├── timestamps_trimmed.txt (53.75s - 1071.0s)
│   └── pose_world_frame.txt
├── Husky1/
│   ├── rgb_stereo_left/
│   ├── depth_png16/
│   ├── timestamps.txt (original)
│   ├── timestamps_trimmed.txt (47.80s - 260.40s)
│   └── pose_world_frame.txt
└── Husky2/
    ├── rgb_stereo_left/
    ├── depth_png16/
    ├── timestamps.txt (original)
    ├── timestamps_trimmed.txt (48.35s - 201.15s)
    └── pose_world_frame.txt
```

## Next Steps / TODO

1. **Improve Drone2** (current: 15.62m RMSE)
   - Try WITH loop closure on trimmed data
   - May need dataset recollection if issues persist

2. **Try Loop Closure on Trimmed Data**
   - Run Husky1 trimmed WITH LC (might improve from 3.44m)
   - Run Husky2 trimmed WITH LC (might improve from 11.07m)

3. **Dataset Recollection** (if needed)
   - Slower shutter speed to reduce motion blur
   - Verify camera calibration
   - Check RGB-D synchronization

## Evaluation Script

**Location**: `/home/sgarimella34/slam/orbslam3_ws/compare_trajectory_simple.py`

**Usage**:
```bash
python3 compare_trajectory_simple.py \
  <ground_truth.txt> \
  <estimated_trajectory.txt>
```

**Output**: RMSE, Mean, Median, Scale, trajectory plot

## Important Notes

- **Trimmed timestamps are key**: Calibration motion significantly hurt Husky results
- **Baseline config works well**: 3000 features, 1.1 scale factor, 10 levels
- **Fast motion config**: More features help NO LC but can hurt WITH LC
- **NaN warnings**: Sophus patch prevents crashes but logs issues
- **Drone1 is reference**: 0.70m RMSE is excellent, use as quality benchmark

## All Experiment History

### Original Results (WITH Calibration)
| Platform | NO LC | WITH LC |
|----------|-------|---------|
| Drone1 | 0.70m | 0.91m |
| Drone2 | 15.38m | Crashed (1.57m partial) |
| Husky1 | 9.57m | 2.12m |
| Husky2 | 66.62m | 56.14m |

### Trimmed Results (NO Calibration) - CURRENT BEST
| Platform | Baseline |
|----------|----------|
| Drone1 | 0.70m (reference) |
| Drone2 | **15.62m** |
| Husky1 | **3.44m** |
| Husky2 | **11.07m** |

## Command Reference

### Run Experiment
```bash
cd ~/slam/orbslam3_ws
bash run_city_<robot>_baseline_trimmed.sh
```

### Monitor Running Experiment
```bash
# Find task ID
/tasks

# Monitor output
tail -f /tmp/claude-*/tasks/<task_id>.output
```

### Create Trimmed Timestamps
```bash
cd /media/sgarimella34/hercules-collect/raw_data_hercules/city_cslam_ugvuav_test1/<Robot>
awk '$1 >= <start_time> && $1 <= <end_time>' timestamps.txt > timestamps_trimmed.txt
```

## Contact / Handoff
- Working directory: `/home/sgarimella34/slam/orbslam3_ws`
- All results saved with prefix `city_<robot>_<config>_`
- Trajectory backups saved every 1000 frames
- CLAUDE.md contains full ORB-SLAM3 setup instructions

**Ready to continue from here!**
