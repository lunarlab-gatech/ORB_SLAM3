# Complete Loop Closure Study - ORB-SLAM3 on AirSim Datasets

**Date:** 2026-02-02
**Study:** Impact of Loop Closure on RGB-D SLAM Performance in Synthetic Environments

---

## Executive Summary

This study investigated the impact of loop closure on ORB-SLAM3 RGB-D performance across four AirSim synthetic datasets (Drone1, Drone2, Husky1, Husky2). **Key finding: Loop closure degrades performance in 3 out of 4 datasets**, with the best overall result achieved by Husky1 WITHOUT loop closure at **1.06m RMSE**.

---

## Configuration

### ORB-SLAM3 Parameters (Scale Focus)
- **Features per image:** 3000
- **Scale factor:** 1.1
- **Pyramid levels:** 10
- **FAST thresholds:** 15 (initial), 5 (minimum)

### Camera Parameters
- **Resolution:** 752x480
- **FOV:** 90°
- **Intrinsics:** fx=fy=376.0, cx=376.0, cy=240.0
- **Distortion:** Zero (synthetic camera)

### Depth Processing
- **Input format:** .npy files (meters, float32)
- **Output format:** 16-bit PNG (millimeters, uint16)
- **Depth factor:** 1000.0 (mm to meters)

### Evaluation Metric
- **Method:** Absolute Trajectory Error (ATE)
- **Alignment:** SE(3) Umeyama with scale
- **Format:** TUM trajectory format

---

## Complete Results

### Dataset 1: Drone1

**Time Range:** 7.0s to 1025.5s (20,371 frames)

| Configuration | RMSE | Mean | Median | Scale | Keyframes | Result |
|---------------|------|------|--------|-------|-----------|--------|
| WITHOUT LC | **4.10m** | 3.27m | 2.63m | 1.007 | 1,879 | ✓ BEST |
| WITH LC | 6.71m | 6.30m | 5.57m | 1.004 | 1,832 | 64% worse |

**Verdict:** Loop closure degrades performance by 1.6x

---

### Dataset 2: Drone2

**Time Range:** 4.7s to 892.6s (17,759 frames)

| Configuration | RMSE | Mean | Median | Scale | Keyframes | Result |
|---------------|------|------|--------|-------|-----------|--------|
| WITHOUT LC | **4.12m** | 3.51m | 3.46m | 1.001 | 1,991 | ✓ BEST |
| WITH LC | 37.85m | 37.81m | 38.51m | 0.994 | 1,881 | 819% worse |

**Verdict:** Loop closure causes catastrophic failure (9.2x degradation)

---

### Dataset 3: Husky1

**Time Range:** 1.5s to 1125.0s (22,471 frames)

| Configuration | RMSE | Mean | Median | Scale | Keyframes | Result |
|---------------|------|------|--------|-------|-----------|--------|
| WITHOUT LC | **1.06m** | 0.90m | 0.69m | 0.998 | 3,480 | ✓ BEST OVERALL |
| WITH LC | 1.82m | 1.48m | 1.07m | 1.000 | 3,443 | 72% worse |

**Verdict:** Loop closure degrades performance by 1.7x
**Note:** Best overall result across all experiments

---

### Dataset 4: Husky2

**Time Range:** 1.5s to 1118.8s (22,347 frames)

| Configuration | RMSE | Mean | Median | Scale | Keyframes | Result |
|---------------|------|------|--------|-------|-----------|--------|
| WITHOUT LC | 1.72m | 1.49m | 1.42m | 0.996 | 3,402 | - |
| WITH LC | **1.19m** | 1.14m | 1.22m | 0.999 | 3,405 | ✓ BEST (31% better) |

**Verdict:** Loop closure improves performance by 1.4x
**Note:** Only dataset where loop closure helps

---

## Summary Statistics

| Dataset | WITHOUT LC | WITH LC | LC Impact | Winner |
|---------|-----------|---------|-----------|--------|
| Drone1 | 4.10m | 6.71m | +64% worse | NO LC |
| Drone2 | 4.12m | 37.85m | +819% worse | NO LC |
| Husky1 | 1.06m | 1.82m | +72% worse | NO LC |
| Husky2 | 1.72m | 1.19m | -31% better | WITH LC |

**Overall Best:** Husky1 WITHOUT loop closure (1.06m RMSE)

---

## Key Findings

### 1. Loop Closure Generally Harmful
- 3 out of 4 datasets show degraded performance with loop closure
- Degradation ranges from 1.6x (Drone1) to 9.2x (Drone2)
- Drone2 shows catastrophic failure with loop closure enabled

### 2. Perceptual Aliasing Hypothesis
Loop closure failures likely caused by:
- Repetitive synthetic textures leading to false loop detections
- Incorrect pose corrections from spurious loop matches
- Trajectory "jumps" at incorrectly detected revisited locations

### 3. Husky2 Exception
Husky2 is the only dataset where loop closure helps:
- May have genuinely revisited locations with distinct features
- Better trajectory characteristics for loop detection
- Suggests loop closure can work when conditions are favorable

### 4. Scale Estimation Robust
- All experiments achieve near-perfect scale (0.994-1.007)
- Scale estimation unaffected by loop closure setting
- RGB-D depth information provides strong scale constraints

### 5. Husky Platforms Outperform Drones
- Husky1 and Husky2 achieve better accuracy (1.06-1.72m) than Drone1/Drone2 (4.10-4.12m)
- Ground-based platforms may have more stable trajectories
- Lower altitude provides richer depth information

---

## Recommendations

### For Synthetic AirSim Datasets
1. **Disable loop closure by default** using `loopClosing: 0` in config files
2. Only enable loop closure if trajectory validation shows improvement
3. Monitor for trajectory "jumps" indicating false loop detections

### For Real-World Datasets
1. Test both configurations on representative data
2. Loop closure more likely beneficial in real-world scenarios
3. Genuine loop closures provide drift correction

### Configuration Files
All configuration files created:
- `Drone1_RGBD_ScaleFocus_NoLC.yaml`
- `Drone2_RGBD_ScaleFocus_NoLC.yaml`
- `Husky1_RGBD_ScaleFocus_NoLC.yaml`
- `Husky2_RGBD_ScaleFocus_NoLC.yaml`
- `Husky1_RGBD_ScaleFocus.yaml`
- `Husky2_RGBD_ScaleFocus.yaml`

### Run Scripts
- `run_husky1_no_lc.sh` / `run_husky1_with_lc.sh`
- `run_husky2_no_lc.sh` / `run_husky2_with_lc.sh`

---

## Result Files

### Drone1
- `drone1_nolc_traj_traj.txt` (20,371 poses, 1,879 KF)
- Visualization: `drone1_nolc_trajectory_comparison.png`
- RMSE: 4.10m

### Drone2
- `drone2_nolc_traj_traj.txt` (17,759 poses, 1,991 KF)
- Visualization: `drone2_nolc_trajectory_comparison.png`
- RMSE: 4.12m

### Husky1
- `husky1_nolc_scalefocus_result.txt` (22,471 poses, 3,480 KF)
- `husky1_lc_scalefocus_result.txt` (22,471 poses, 3,443 KF)
- Visualizations: `husky1_nolc_trajectory_comparison.png`, `husky1_lc_trajectory_comparison.png`
- RMSE: 1.06m (NO LC), 1.82m (WITH LC)

### Husky2
- `husky2_nolc_scalefocus_result.txt` (22,347 poses, 3,402 KF)
- `husky2_lc_scalefocus_result.txt` (22,347 poses, 3,405 KF)
- Visualizations: `husky2_nolc_trajectory_comparison.png`, `husky2_lc_trajectory_comparison.png`
- RMSE: 1.72m (NO LC), 1.19m (WITH LC)

---

## Technical Implementation

### Loop Closure Control
ORB-SLAM3 supports loop closure control via config parameter:
```yaml
# Disable loop closure
loopClosing: 0
```

Implementation in `System.cc:101-106`:
```cpp
node = fsSettings["loopClosing"];
bool activeLC = true;
if(!node.empty())
{
    activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0;
}
```

### Depth Conversion
Python script to convert AirSim depth from .npy to 16-bit PNG:
```python
depth_m = np.load(npy_file)
depth_mm = depth_m * 1000.0  # meters to mm
depth_mm[depth_mm > 65000] = 0  # Clip far/invalid depths
depth_uint16 = depth_mm.astype(np.uint16)
cv2.imwrite(str(output_file), depth_uint16)
```

### Trajectory Comparison
SE(3) Umeyama alignment with scale:
- Align estimated trajectory to ground truth
- Compute rotation, translation, and scale
- Calculate ATE (Absolute Trajectory Error)
- Generate XY trajectory plots and error over time

---

## Archived Results

All previous results preserved in `results_archive/` directory:
- Drone1 and Drone2 initial experiments
- Detailed analysis documents
- Loop closure investigation findings

---

## Conclusions

1. **Loop closure should be disabled** for most AirSim synthetic datasets to avoid false detections
2. **Husky1 WITHOUT loop closure** achieved the best result (1.06m RMSE)
3. **Scale estimation is robust** regardless of loop closure setting
4. **Husky2 shows loop closure can help** when conditions are favorable
5. **Ground-based platforms (Husky)** achieve better accuracy than aerial platforms (Drone)

This comprehensive study demonstrates the importance of evaluating loop closure on synthetic data, where perceptual aliasing can lead to spurious matches and degraded performance.

---

**Study conducted using:**
- ORB-SLAM3 (RGB-D mode)
- Docker container (orbslam3-ubuntu22)
- AirSim synthetic datasets
- Scale Focus configuration (3000 features, 1.1 scale factor, 10 levels)
