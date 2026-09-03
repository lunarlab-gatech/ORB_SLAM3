# ORB-SLAM3 RGB-D Baseline Results

**Config**: ScaleFocus baseline (nFeatures=3000, scaleFactor=1.1, nLevels=10, iniThFAST=15, minThFAST=5)
**Evaluation**: ATE RMSE with SE(3) Umeyama alignment (with scale)
**Camera**: 752x480, 90° FOV, fx=fy=376.0, cx=376.0, cy=240.0
**Depth**: 16-bit PNG (millimeters), DepthMapFactor=1000.0

---

## 1. Around Sequence (`ausenv_aroundshortseq_2ugvuav`)

**Dataset**: `/media/sgarimella34/hercules-collect/raw_data_hercules/ausenv_aroundshortseq_2ugvuav/`

**Trimmed timestamp ranges**:
- Drone1: 1.50s – 1121.80s
- Drone2: 4.70s – 1193.80s
- Husky1: 1.95s – 772.15s
- Husky2: 2.15s – 741.45s

**Note**: Two trials were run; best RMSE per robot per LC mode is reported below.

| Robot | Frames | Traj Length | WITH LC (RMSE) | %Traj | NO LC (RMSE) | %Traj | Better |
|-------|--------|-------------|----------------|-------|--------------|-------|--------|
| Drone1 | 22,407 | 887m | 4.34m | 0.49% | **4.16m** | 0.47% | NO LC |
| Drone2 | 23,783 | 945m | **24.13m** | 2.55% | 23.14m | 2.45% | NO LC |
| Husky1 | 15,405 | 791m | 5.40m | 0.68% | **3.53m** | 0.45% | NO LC |
| Husky2 | 14,787 | 758m | **10.56m** | 1.39% | 10.88m | 1.43% | WITH LC |

### Around Sequence - Trial 1 vs Trial 2 Detail

| Robot | Mode | Trial 1 RMSE | Trial 2 RMSE | Best |
|-------|------|-------------|-------------|------|
| Drone1 | LC | **4.34m** | 5.27m | Trial 1 |
| Drone1 | NoLC | 4.28m | **4.16m** | Trial 2 |
| Drone2 | LC | 35.01m | **24.13m** | Trial 2 |
| Drone2 | NoLC | **23.14m** | 40.26m | Trial 1 |
| Husky1 | LC | 8.44m | **5.40m** | Trial 2 |
| Husky1 | NoLC | 5.27m | **3.53m** | Trial 2 |
| Husky2 | LC | 12.35m | **10.56m** | Trial 2 |
| Husky2 | NoLC | **10.88m** | 14.73m | Trial 1 |

---

## 2. Road Sequence (`ausenv_roadseq_2ugvuav`)

**Dataset**: `/media/sgarimella34/hercules-collect/raw_data_hercules/ausenv_roadseq_2ugvuav/`

| Robot | Frames | Traj Length | WITH LC (RMSE) | %Traj | NO LC (RMSE) | %Traj | Better |
|-------|--------|-------------|----------------|-------|--------------|-------|--------|
| Drone1 | 20,567 | 905m | 6.71m | 0.74% | **4.10m** | 0.45% | NO LC |
| Drone2 | 17,759 | 783m | 37.85m | 4.83% | **4.12m** | 0.53% | NO LC |
| Husky1 | 22,471 | 754m | 1.82m | 0.24% | **1.06m** | 0.14% | NO LC |
| Husky2 | 22,347 | 750m | **1.19m** | 0.16% | 1.72m | 0.23% | WITH LC |

---

## 3. City Sequence (best of `city_cslam_ugvuav_test1` + `city_test2`)

**Datasets**:
- Test1: `/media/sgarimella34/hercules-collect/raw_data_hercules/city_cslam_ugvuav_test1/`
- Test2: `/media/sgarimella34/SSD2/raw_data_hercules/city_test2/`

**Source selection**: Drone1 only exists in test1. For Drone2, Husky1, and Husky2, test2 produced better results and is used below.

| Robot | Frames | Traj Length | WITH LC (RMSE) | %Traj | NO LC (RMSE) | %Traj | Better |
|-------|--------|-------------|----------------|-------|--------------|-------|--------|
| Drone1 | 21,975 | 734m | 0.91m | 0.12% | **0.70m** | 0.10% | NO LC |
| Drone2 | 23,754 | 638m | **7.15m** | 1.12% | 8.64m | 1.35% | WITH LC |
| Husky1 | 7,631 | 596m | **1.03m** | 0.17% | 1.27m | 0.21% | WITH LC |
| Husky2 | 7,780 | 610m | 1.66m | 0.27% | **0.68m** | 0.11% | NO LC |

### City Test1 vs Test2 Detail

| Robot | Test1 LC | Test1 NoLC | Test2 LC | Test2 NoLC |
|-------|----------|------------|----------|------------|
| Drone1 | 0.91m | 0.70m | N/A | N/A |
| Drone2 | crashed (NaN) | 15.38m | **7.15m** | **8.64m** |
| Husky1 | 2.12m | 9.57m | **1.03m** | **1.27m** |
| Husky2 | 56.14m | 66.62m | **1.66m** | **0.68m** |

---

## 4. Forest Sequence (`customforest_test1`)

**Dataset**: `/media/sgarimella34/hercules-collect/raw_data_hercules/customforest_test1/`

**Trimmed timestamp ranges**:
- Drone1: 30.45s – 898.10s
- Drone2: 34.50s – 906.85s
- Husky1: 37.05s – 768.90s (capped at data end; user requested 906.85s)
- Husky2: 36.60s – 762.35s

| Robot | Frames | Traj Length | WITH LC (RMSE) | %Traj | NO LC (RMSE) | %Traj | Better |
|-------|--------|-------------|----------------|-------|--------------|-------|--------|
| Drone1 | 17,354 | 515m | 0.59m | 0.11% | **0.41m** | 0.08% | NO LC |
| Drone2 | 17,448 | 518m | 23.96m | 4.63% | **0.60m** | 0.12% | NO LC |
| Husky1 | 14,638 | 359m | **0.46m** | 0.13% | 0.52m | 0.14% | WITH LC |
| Husky2 | 14,516 | 486m | **0.56m** | 0.11% | 0.67m | 0.14% | WITH LC |

---

## Overall Best Per Robot (across all sequences)

| Robot | Best RMSE | %Traj | Sequence | LC Mode |
|-------|-----------|-------|----------|---------|
| Drone1 | **0.41m** | 0.08% | Forest | NO LC |
| Drone2 | **0.60m** | 0.12% | Forest | NO LC |
| Husky1 | **0.46m** | 0.13% | Forest | WITH LC |
| Husky2 | **0.56m** | 0.11% | Forest | WITH LC |

---

## Key Observations

- **Forest sequence produced the best results across all robots**: All 4 robots achieved sub-1m RMSE in the forest, with the best being Drone1 NoLC at 0.41m (0.08% drift). The dense forest environment provides rich visual features and close-range depth.
- **Loop closure is not always beneficial**: Out of 16 robot-sequence combinations, NO LC wins 10 times (63%). LC helps when the trajectory revisits the same area. In several cases, LC introduces false loop closures that significantly increase error (e.g., Road Drone2: 37.85m with LC vs 4.12m without; Forest Drone2: 23.96m with LC vs 0.60m without).
- **Ground robots (Huskys) generally outperform drones in open environments**: Huskys achieve sub-2m RMSE in most sequences due to close-range depth from ground level. Drones struggle with far-range depth in open environments (Around, Road) but perform well in feature-rich environments (Forest, City).
- **ORB-SLAM3 is non-deterministic**: Results vary between trials due to random feature selection and parallel threading. Drone2 Around showed the most variance (23m vs 40m NoLC between trials).
- **Drift rate**: Best runs achieve 0.08–0.14% drift relative to trajectory length. Worst cases (Drone2 Around) reach 2.5%.

---

## Trajectory Comparison Plots

All best plots are saved in `best_plots/` with naming: `{sequence}_{robot}_{lc/nolc}.png`

### Around Sequence
| | WITH LC | NO LC |
|---|---------|-------|
| Drone1 | `best_plots/around_drone1_lc.png` | `best_plots/around_drone1_nolc.png` |
| Drone2 | `best_plots/around_drone2_lc.png` | `best_plots/around_drone2_nolc.png` |
| Husky1 | `best_plots/around_husky1_lc.png` | `best_plots/around_husky1_nolc.png` |
| Husky2 | `best_plots/around_husky2_lc.png` | `best_plots/around_husky2_nolc.png` |

### Road Sequence
| | WITH LC | NO LC |
|---|---------|-------|
| Drone1 | `best_plots/road_drone1_lc.png` | `best_plots/road_drone1_nolc.png` |
| Drone2 | `best_plots/road_drone2_lc.png` | `best_plots/road_drone2_nolc.png` |
| Husky1 | `best_plots/road_husky1_lc.png` | `best_plots/road_husky1_nolc.png` |
| Husky2 | `best_plots/road_husky2_lc.png` | `best_plots/road_husky2_nolc.png` |

### City Sequence
| | WITH LC | NO LC |
|---|---------|-------|
| Drone1 | `best_plots/city_drone1_lc.png` | `best_plots/city_drone1_nolc.png` |
| Drone2 | `best_plots/city_drone2_lc.png` | `best_plots/city_drone2_nolc.png` |
| Husky1 | `best_plots/city_husky1_lc.png` | `best_plots/city_husky1_nolc.png` |
| Husky2 | `best_plots/city_husky2_lc.png` | `best_plots/city_husky2_nolc.png` |

### Forest Sequence
| | WITH LC | NO LC |
|---|---------|-------|
| Drone1 | `best_plots/forest_drone1_lc.png` | `best_plots/forest_drone1_nolc.png` |
| Drone2 | `best_plots/forest_drone2_lc.png` | `best_plots/forest_drone2_nolc.png` |
| Husky1 | `best_plots/forest_husky1_lc.png` | `best_plots/forest_husky1_nolc.png` |
| Husky2 | `best_plots/forest_husky2_lc.png` | `best_plots/forest_husky2_nolc.png` |

### Trial-Specific Plots (Around Sequence)
All trial-specific plots are in the workspace root:
- `around_{robot}_{lc/nolc}_trial1_trajectory_comparison.png`
- `around_{robot}_{lc/nolc}_trial2_trajectory_comparison.png`
