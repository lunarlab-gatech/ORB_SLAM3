# ORB-SLAM3 IMU Issues and Fixes

## Current Baseline Results
- **RGB-D only:** RMSE = 11.98m (Scale = 1.18x)
- **RGB-D-Inertial:** RMSE = 11.98m (same as RGB-D, IMU not helping)
- **Target:** RMSE < 3m

## Root Causes Identified

### 1. CRITICAL: Wrong Camera-IMU Extrinsic Calibration ✅ FIXED

**Problem:**
- Camera is at position (0.35, -0.055, 0.2) meters with 10° pitch relative to IMU
- YAML had **identity matrix** → ORB-SLAM3 thought camera and IMU were co-located
- This causes massive fusion errors as visual and inertial measurements don't align

**Fix Applied:**
```yaml
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [0.984808, 0.000000, -0.173648, -0.309953,
          0.000000, 1.000000, 0.000000, 0.055000,
          0.173648, 0.000000, 0.984808, -0.257738,
          0.000000, 0.000000, 0.000000, 1.000000]
```

This correctly transforms from camera frame to IMU body frame.

### 2. IMU Data Format Issue

**Problem:**
- Your script outputs **11 columns**: t, ax, ay, az, gx, gy, gz, qx, qy, qz, qw
- ORB-SLAM3 loader reads only **7 columns**: t, ax, ay, az, gx, gy, gz
- The world quaternion (columns 8-11) is **ignored** by ORB-SLAM3

**Impact:**
- World frame orientation data is wasted
- ORB-SLAM3 uses odometry-based IMU orientation internally

**Recommendation:**
- Keep current format (11 columns) - it doesn't hurt
- Or modify script to output only 7 columns for cleaner data

### 3. IMU Measurement Physics (Actually Correct!)

**Your Data Shows:**
```
0.050000 -0.000000 -0.000000 -9.806650 0.000000 -0.000000 -0.000000
```
This is `az = -9.806 m/s²` when stationary.

**Analysis:**
- AirSim uses **NED frame** (North-East-Down): Z-axis points DOWN
- Gravity vector in world: [0, 0, +9.806] (downward)
- Accelerometer measures **specific force** = measured_accel - gravity
- When stationary: specific_force = 0 - (+9.806 down) = -9.806 = pointing UP
- In body frame (aligned with world at start): az = -9.806 ✅ **CORRECT!**

**This is NOT a bug** - your IMU generation is physically correct!

### 4. IMU Initialization Requirements

**ORB-SLAM3 IMU Init Needs:**
1. **Sufficient motion**: Linear acceleration OR rotation
2. **Multiple frames**: ~10-20 frames with varying motion
3. **Observability**: Motion in different directions helps

**Your Dataset:**
- Drone descends from 1.29m to 0.51m in first 2 seconds (good!)
- BUT: Descent is purely vertical with minimal rotation
- IMU sees mostly gravity, little lateral acceleration
- Result: "not enough acceleration" warnings

**Potential Fix:**
- Increase IMU noise parameters to relax initialization requirements
- Or: Start ORB-SLAM3 from frame 40-50 (1-2 seconds in) when motion is more varied

## Fixes Applied

### ✅ Fixed: Camera-IMU Transform
Updated `Drone1_RGBD_Inertial.yaml` with correct extrinsic calibration.

### 🔄 Testing Now
Running ORB-SLAM3 with fixed transform to see if IMU helps.

## Next Steps to Reduce RMSE < 3m

### Option 1: Tune IMU Parameters
```yaml
# More realistic noise for better initialization
IMU.NoiseGyro: 0.001  # instead of 0.0001
IMU.NoiseAcc: 0.01    # instead of 0.001
IMU.GyroWalk: 0.0001  # instead of 0.00001
IMU.AccWalk: 0.0001   # instead of 0.00001
```

### Option 2: Improve Visual Features
```yaml
# More features for better tracking
ORBextractor.nFeatures: 5000  # instead of 3000
ORBextractor.nLevels: 10      # instead of 8
```

### Option 3: Fix Depth Scale
Current depth images might have calibration issues. Verify:
- Depth conversion: meters → millimeters (×1000) ✅
- DepthMapFactor: 1000.0 ✅
- But: Check if AirSim depth needs additional calibration

### Option 4: Better Ground Truth Alignment
The 1.18x scale factor suggests systematic scale drift. Could be:
- Camera intrinsic calibration (fx, fy, cx, cy)
- Depth measurement bias
- IMU scale observability issues

## Debugging Commands

### Check IMU initialization:
```bash
grep -i "imu\|initialization\|not enough" rgbd_inertial_fixed_transform.log | head -50
```

### Compare trajectories:
```bash
python3 compare_trajectory_simple.py \
  ~/multi-robot-coordination/hercules_datasets/ausenv_roadseq/Drone1/pose_world_frame.txt \
  CameraTrajectory.txt
```

### Visualize trajectory:
```bash
python3 -c "
import matplotlib.pyplot as plt
import numpy as np

# Load trajectories
gt = np.loadtxt('pose_world_frame.txt')
est = np.loadtxt('CameraTrajectory.txt')

plt.figure(figsize=(12, 5))
plt.subplot(121)
plt.plot(gt[:, 1], gt[:, 2], 'b-', label='Ground Truth', linewidth=2)
plt.plot(est[:, 1], est[:, 2], 'r-', label='Estimated', linewidth=1)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.title('Trajectory Comparison (XY)')

plt.subplot(122)
plt.plot(gt[:, 0], gt[:, 3], 'b-', label='GT Z')
plt.plot(est[:, 0], est[:, 3], 'r-', label='Est Z')
plt.xlabel('Time (s)')
plt.ylabel('Z (m)')
plt.legend()
plt.grid(True)
plt.title('Altitude Over Time')

plt.tight_layout()
plt.savefig('trajectory_analysis.png', dpi=150)
print('Saved trajectory_analysis.png')
"
```

## Expected Improvements

With correct camera-IMU transform:
- IMU should now properly constrain scale drift
- Target: **RMSE < 5m** (50% improvement)
- With additional tuning: **RMSE < 3m** (target achieved)
