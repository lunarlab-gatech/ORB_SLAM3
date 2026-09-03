# AroundShortSeq Loop Closure Study Results

**Dataset**: ausenv_aroundshortseq_2ugvuav
**Date**: February 2, 2026
**Configuration**: Scale Focus (3000 features, scaleFactor=1.1, nLevels=10)

## Summary Table

| Platform | Frames | NO LC RMSE | NO LC Scale | WITH LC RMSE | WITH LC Scale | LC Effect |
|----------|--------|------------|-------------|--------------|---------------|-----------|
| Drone1   | 22,416 | 4.28m      | 1.23        | 4.34m        | 1.20          | ~same     |
| Drone2   | 23,783 | 23.14m     | 1.31        | 35.01m       | 1.34          | worse     |
| Husky1   | 15,412 | 5.27m      | 1.01        | 8.44m        | 1.02          | worse     |
| Husky2   | 14,787 | 10.88m     | 1.00        | 12.35m       | 1.01          | worse     |

## Detailed Results

### Drone1
**Time range**: 1.05s to 1121.80s (22,416 frames)

**WITHOUT Loop Closure**:
```
RMSE:   4.277759 m
Mean:   3.303879 m
Median: 2.560782 m
Std:    2.717278 m
Min:    0.540199 m
Max:    12.223027 m
Scale:  1.226199
```

**WITH Loop Closure**:
```
RMSE:   4.340709 m
Mean:   3.381031 m
Median: 2.640727 m
Std:    2.722202 m
Min:    0.278313 m
Max:    11.877304 m
Scale:  1.198039
```

### Drone2
**Time range**: 4.70s to 1193.80s (23,783 frames)

**WITHOUT Loop Closure**:
```
RMSE:   23.137429 m
Mean:   22.439295 m
Median: 23.089186 m
Std:    5.640802 m
Min:    4.947975 m
Max:    33.186454 m
Scale:  1.312305
```

**WITH Loop Closure**:
```
RMSE:   35.008103 m
Mean:   34.220404 m
Median: 35.702886 m
Std:    7.384528 m
Min:    7.024796 m
Max:    43.764563 m
Scale:  1.337404
```

### Husky1
**Time range**: 1.95s to 772.5s (15,412 frames)

**WITHOUT Loop Closure**:
```
RMSE:   5.269110 m
Mean:   4.202103 m
Median: 2.911161 m
Std:    3.178971 m
Min:    0.800858 m
Max:    11.593800 m
Scale:  1.007919
```

**WITH Loop Closure**:
```
RMSE:   8.436028 m
Mean:   6.664969 m
Median: 4.549856 m
Std:    5.171532 m
Min:    1.305257 m
Max:    19.604688 m
Scale:  1.018234
```

### Husky2
**Time range**: 2.15s to 741.45s (14,787 frames)

**WITHOUT Loop Closure**:
```
RMSE:   10.876740 m
Mean:   7.729109 m
Median: 4.986980 m
Std:    7.652735 m
Min:    0.504220 m
Max:    35.028975 m
Scale:  1.003763
```

**WITH Loop Closure**:
```
RMSE:   12.348770 m
Mean:   8.780383 m
Median: 5.353915 m
Std:    8.683145 m
Min:    0.782328 m
Max:    40.687391 m
Scale:  1.006641
```

## Key Observations

1. **Husky robots have excellent scale estimation** (~1.0) - ground vehicles perform significantly better for scale than drones

2. **Loop closure consistently made results WORSE** across all platforms on this dataset - this suggests either:
   - False loop closures are being detected
   - The environment lacks sufficient visual overlap for reliable loop closure
   - The trajectory doesn't revisit previous locations

3. **Drone1 performed best overall** with ~4.3m RMSE regardless of loop closure setting

4. **Drone2 had the worst performance** with 23-35m RMSE - possibly due to different flight patterns or visual conditions

5. **Best configuration**: NO Loop Closure for all platforms on this dataset

## Output Files

All output files use `aroundshort_` prefix:
- `aroundshort_drone1_nolc_result.txt`, `aroundshort_drone1_lc_result.txt`
- `aroundshort_drone2_nolc_result.txt`, `aroundshort_drone2_lc_result.txt`
- `aroundshort_husky1_nolc_result.txt`, `aroundshort_husky1_lc_result.txt`
- `aroundshort_husky2_nolc_result.txt`, `aroundshort_husky2_lc_result.txt`

Trajectory comparison plots:
- `aroundshort_drone1_nolc_trajectory_comparison.png`, `aroundshort_drone1_lc_trajectory_comparison.png`
- `aroundshort_drone2_nolc_trajectory_comparison.png`, `aroundshort_drone2_lc_trajectory_comparison.png`
- `aroundshort_husky1_nolc_trajectory_comparison.png`, `aroundshort_husky1_lc_trajectory_comparison.png`
- `aroundshort_husky2_nolc_trajectory_comparison.png`, `aroundshort_husky2_lc_trajectory_comparison.png`
