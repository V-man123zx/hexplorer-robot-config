# MOLA-SLAM Tuning Guide for Hexplorer Robot

## Overview

MOLA (Modular Object Localization and Mapping Architecture) provides LiDAR-only odometry using GICP (Generalized ICP) scan matching. This guide covers tuning parameters for optimal performance on the Hexplorer robot with Livox Mid360 LiDAR.

## Quick Start

```bash
# Start MOLA SLAM with RViz
bash ~/hexplorer/scripts/start_mola_slam.sh

# Start without RViz (headless)
bash ~/hexplorer/scripts/start_mola_slam.sh --no-rviz

# Start with MOLA's built-in GUI
bash ~/hexplorer/scripts/start_mola_slam.sh --gui
```

## Architecture

```
Jetson (192.168.1.20)                    Mini PC (192.168.1.10)
========================                  ========================

livox_lidar_node                         livox_tcp_receiver.py
  └─ /livox_Lidar_node/sn153/              └─ /livox/lidar
     xyz/pointcloud                              │
         │                                       ▼
         │                               filterpass.py
         ▼                                 └─ /livox/lidar_filtered
  livox_tcp_bridge.py ───TCP:9998───►            │
                                                 ▼
                                          MOLA LidarOdometry
                                            ├─ /state_estimator/pose
                                            ├─ /tf (map→odom→base_link)
                                            └─ /lidar_odometry/localmap_points
```

## Configuration File

**Location:**
```
~/MOLA-SLAM/mola_ws/install/mola_lidar_odometry/share/mola_lidar_odometry/pipelines/lidar3d-gicp-katana.yaml
```

## Key Parameters

### ICP Quality Threshold

```yaml
# Line ~68: Minimum ICP quality to accept a match
min_icp_goodness: ${MOLA_MINIMUM_ICP_QUALITY|0.92}
```

| Value | Effect |
|-------|--------|
| 0.80 | Permissive - accepts more matches, may drift |
| 0.90 | Balanced - good for most environments |
| 0.95 | Strict - rejects questionable matches |

**Recommendation:** Start at 0.90, increase if seeing jumps.

### Adaptive Threshold (Matching Distance)

```yaml
# Lines ~70-78: Controls how far apart points can be matched
adaptive_threshold:
  enabled: true
  initial_sigma: 0.3      # Initial matching distance [m]
  min_motion: 0.3         # Minimum motion threshold [m]
  maximum_sigma: 0.8      # Maximum matching distance [m] - CRITICAL
  icp_quality_controller_setpoint: 0.90
```

| Parameter | Effect |
|-----------|--------|
| `maximum_sigma` | Lower = stricter matching, less drift, may fail in feature-poor areas |
| `initial_sigma` | Starting point for adaptive threshold |
| `icp_quality_controller_setpoint` | Target ICP quality for adaptation |

**Recommendation:** Keep `maximum_sigma` between 0.5-1.0m for indoor environments.

### ICP Iterations

```yaml
# Line ~193: Number of ICP iterations
params:
  maxIterations: ${MOLA_MAX_ICP_ITERATIONS|50}
  minAbsStep_trans: 5e-4   # Convergence threshold (translation)
  minAbsStep_rot: 5e-5     # Convergence threshold (rotation)
```

| Iterations | Effect |
|------------|--------|
| 30 | Faster, may not fully converge |
| 50 | Balanced (recommended) |
| 80+ | More accurate, slower |

### Robust Kernel (Outlier Rejection)

```yaml
# Lines ~204-210: Outlier handling
solvers:
  - class: mp2p_icp::Solver_GaussNewton
    params:
      maxIterations: 2
      robustKernel: "RobustKernel::GemanMcClure"
      robustKernelParam: 4.0  # Lower = stricter outlier rejection
```

| robustKernelParam | Effect |
|-------------------|--------|
| 6.0 | Permissive - keeps more correspondences |
| 4.0 | Balanced (recommended) |
| 3.0 | Strict - aggressively rejects outliers |

### Keyframe Settings

```yaml
# Lines ~58-65: When to add new keyframes
local_map_updates:
  min_translation_between_keyframes: 0.15  # [m]
  min_rotation_between_keyframes: 3.0       # [deg]
  max_distance_to_keep_keyframes: 0         # 0 = keep all
```

| Parameter | Effect |
|-----------|--------|
| `min_translation_between_keyframes` | Lower = more keyframes, better coverage |
| `max_distance_to_keep_keyframes` | 0 = keep all; >0 = sliding window |

## Tuning Workflow

### 1. Edit Configuration
```bash
nano ~/MOLA-SLAM/mola_ws/install/mola_lidar_odometry/share/mola_lidar_odometry/pipelines/lidar3d-gicp-katana.yaml
```

### 2. Restart MOLA
```bash
# Kill current instance
pkill -9 -f mola-cli

# Restart
source /opt/ros/humble/setup.bash
source ~/MOLA-SLAM/mola_ws/install/setup.bash
ros2 launch mola_lidar_odometry ros2-lidar-odometry-katana.launch.py \
    lidar_topic_name:=/livox/lidar_filtered \
    ignore_lidar_pose_from_tf:=true \
    use_rviz:=false \
    use_mola_gui:=false &
```

### 3. Monitor Performance
```bash
# Check ICP quality (should be > 0.90)
ros2 topic echo /mola_diagnostics/lidar_odom/status --once

# Check pose output rate
ros2 topic hz /state_estimator/pose
```

## Common Issues and Fixes

### Map Jumps/Displacement
**Symptoms:** Map suddenly shifts, creating duplicate features
**Fixes:**
- Increase `min_icp_goodness` to 0.92-0.95
- Decrease `maximum_sigma` to 0.5-0.8
- Move robot slower

### Map Drift Over Time
**Symptoms:** Map gradually rotates or translates
**Fixes:**
- Increase `maxIterations` to 60-80
- Decrease `robustKernelParam` to 3.0-4.0
- Add more keyframes (lower `min_translation_between_keyframes`)

### Features Disappearing
**Symptoms:** Parts of map fade away
**Fixes:**
- Set `max_distance_to_keep_keyframes: 0` (keep all)
- Set `remove_frames_farther_than: 0` in localmap insertOpts

### Slow Performance
**Symptoms:** High latency, dropped frames
**Fixes:**
- Reduce `maxIterations` to 30-40
- Reduce visualization point limits
- Use `--no-rviz` flag

## RViz Visualization

### Topics to Add
| Topic | Type | Description |
|-------|------|-------------|
| `/state_estimator/pose` | Odometry | Robot pose (working) |
| `/livox/lidar_filtered` | PointCloud2 | Current LiDAR scan |
| `/lidar_odometry/localmap_points` | PointCloud2 | Accumulated map |

### Optimal RViz Settings for Performance
- Set point cloud **Style** to `Points` (not Squares)
- Set **Size (Pixels)** to `1`
- Disable unnecessary displays

## Limitations

MOLA-LO is **odometry**, not full SLAM:
- No loop closure detection
- Accumulated drift when returning to visited areas
- For loop closure, consider RTAB-Map or Cartographer

## Current Tuned Parameters (2026-02-10)

```yaml
min_icp_goodness: 0.92
adaptive_threshold:
  initial_sigma: 0.3
  min_motion: 0.3
  maximum_sigma: 0.8
  icp_quality_controller_setpoint: 0.90
maxIterations: 50
minAbsStep_trans: 5e-4
minAbsStep_rot: 5e-5
robustKernelParam: 4.0
min_translation_between_keyframes: 0.15
min_rotation_between_keyframes: 3.0
max_distance_to_keep_keyframes: 0
```
