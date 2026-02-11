# MOLA-SLAM Tuning Guide

## Configuration File

```
~/MOLA-SLAM/mola_ws/install/mola_lidar_odometry/share/mola_lidar_odometry/pipelines/lidar3d-katana.yaml
```

**Important:** Do NOT use `lidar3d-gicp-katana.yaml` - it crashes with `Matcher_Cov2Cov` assertion error when the local map accumulates 2+ keyframes.

## Key Parameters

### ICP Quality Threshold

```yaml
min_icp_goodness: ${MOLA_MINIMUM_ICP_QUALITY|0.92}
```

| Value | Effect |
|-------|--------|
| 0.80 | Permissive - accepts more matches, may drift |
| 0.90 | Balanced - good for most environments |
| 0.95 | Strict - rejects questionable matches |

### Adaptive Threshold (Matching Distance)

```yaml
adaptive_threshold:
  enabled: true
  initial_sigma: 0.3
  min_motion: 0.3
  maximum_sigma: 0.8       # CRITICAL - max matching distance [m]
  icp_quality_controller_setpoint: 0.90
```

Keep `maximum_sigma` between 0.5-1.0m for indoor environments.

### ICP Iterations

```yaml
params:
  maxIterations: ${MOLA_MAX_ICP_ITERATIONS|50}
  minAbsStep_trans: 5e-4
  minAbsStep_rot: 5e-5
```

| Iterations | Effect |
|------------|--------|
| 30 | Faster, may not fully converge |
| 50 | Balanced (recommended) |
| 80+ | More accurate, slower |

### Robust Kernel (Outlier Rejection)

```yaml
solvers:
  - class: mp2p_icp::Solver_GaussNewton
    params:
      maxIterations: 2
      robustKernel: "RobustKernel::GemanMcClure"
      robustKernelParam: 4.0  # Lower = stricter outlier rejection
```

| robustKernelParam | Effect |
|-------------------|--------|
| 6.0 | Permissive |
| 4.0 | Balanced (recommended) |
| 3.0 | Strict - aggressively rejects outliers |

### Keyframe Settings

```yaml
local_map_updates:
  min_translation_between_keyframes: 0.15  # [m]
  min_rotation_between_keyframes: 3.0       # [deg]
  max_distance_to_keep_keyframes: 0         # 0 = keep all
```

### Map Density Control

```yaml
# In observation_pipeline_t:
absolute_minimum_sensor_range: 2.0    # Indoor use (default 5.0 too far)

# In HashedVoxelPointCloud (local map):
max_points_per_voxel: 5               # Limits density in revisited areas (default 20)
min_distance_between_points: 0.05     # 5cm minimum spacing (default 0)
remove_voxels_farther_than: 0         # 0 = map grows forever
```

## Tuning Workflow

1. Edit config:
   ```bash
   nano ~/MOLA-SLAM/mola_ws/install/mola_lidar_odometry/share/mola_lidar_odometry/pipelines/lidar3d-katana.yaml
   ```

2. Restart MOLA:
   ```bash
   pkill -9 -f mola-cli
   bash ~/hexplorer/scripts/start_mola_slam.sh
   ```

3. Monitor:
   ```bash
   ros2 topic hz /lidar_odometry/pose
   ```

## Common Issues and Fixes

### Map Jumps/Displacement
- Increase `min_icp_goodness` to 0.92-0.95
- Decrease `maximum_sigma` to 0.5-0.8
- Move robot slower

### Map Drift Over Time
- Increase `maxIterations` to 60-80
- Decrease `robustKernelParam` to 3.0-4.0
- Add more keyframes (lower `min_translation_between_keyframes`)

### Map Too Dense Over Time
- Decrease `max_points_per_voxel` (default 20, currently 5)
- Increase `min_distance_between_points` (currently 0.05m)

### Features Disappearing
- Set `max_distance_to_keep_keyframes: 0` (keep all)
- Set `remove_frames_farther_than: 0` in localmap insertOpts

### Slow Performance
- Reduce `maxIterations` to 30-40
- Use `--no-rviz` flag

### MOLA Crashes
- **GTSAM IndeterminantLinearSystemException**: Ensure `use_state_estimator:=False` in launch
- **Matcher_Cov2Cov assertion**: Use `lidar3d-katana.yaml`, NOT `lidar3d-gicp-katana.yaml`

## RViz Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/lidar_odometry/pose` | Odometry | Robot pose (arrows) |
| `/livox/lidar_filtered` | PointCloud2 | Current LiDAR scan (QoS: Best Effort) |
| `/lidar_odometry/localmap_points` | PointCloud2 | Accumulated map (use FlatColor) |

Set point cloud **Style** to `Points`, **Size (Pixels)** to `1` for performance.

## Limitations

MOLA-LO is **odometry**, not full SLAM:
- No loop closure detection
- Accumulated drift when returning to visited areas

## Current Tuned Parameters (2026-02-11)

```yaml
min_icp_goodness: 0.92
maximum_sigma: 0.8
maxIterations: 50
robustKernelParam: 4.0
min_translation_between_keyframes: 0.15
min_rotation_between_keyframes: 3.0
max_distance_to_keep_keyframes: 0
absolute_minimum_sensor_range: 2.0
max_points_per_voxel: 5
min_distance_between_points: 0.05
pipeline: lidar3d-katana.yaml
use_state_estimator: False
```
