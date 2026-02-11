# MOLA-SLAM Setup

**Purpose:** LiDAR-only SLAM/odometry for the Hexplorer robot. Replaces Fast-LIO (which had IMU drift issues).

**Repository:** https://github.com/Whan000/MOLA-SLAM
**Workspace:** `/home/robot/MOLA-SLAM/mola_ws/`

## Architecture

```
Jetson (192.168.1.20)                    Mini PC (192.168.1.10)
livox_lidar_node   }                     livox_tcp_receiver.py
livox_tcp_bridge.py} --TCP:9998-->          /livox/lidar
  (auto-start via systemd)                      |
                                          filterpass.py
                                            /livox/lidar_filtered
                                                |
                                          MOLA LidarOdometry (lidar3d-katana pipeline)
                                            /lidar_odometry/pose  <-- USE THIS
                                            /tf (map->odom->base_link)
                                            /lidar_odometry/localmap_points
```

Jetson LiDAR services (driver + TCP bridge) run as a **systemd service** (`jetson-lidar.service`) that auto-starts at boot. Mini PC scripts check if services are running before starting them.

## Quick Start

```bash
bash ~/hexplorer/scripts/start_mola_slam.sh
```

| Flag | Description |
|------|-------------|
| `--no-rviz` | Disable RViz |
| `--gui` | Enable MOLA GUI |

Press `Ctrl+C` to stop all processes. Jetson LiDAR services keep running (persistent).

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/livox/lidar` | PointCloud2 | Raw LiDAR from TCP receiver |
| `/livox/lidar_filtered` | PointCloud2 | After FilterPass |
| `/lidar_odometry/pose` | Odometry | Robot pose estimate (USE THIS) |
| `/lidar_odometry/localmap_points` | PointCloud2 | Accumulated map |
| `/tf` | TF | map->odom->base_link transforms |

**Note:** State estimator is disabled (`use_state_estimator:=False`) because GTSAM's `StateEstimationSmoother` crashes under CPU load with `IndeterminantLinearSystemException`. `StateEstimationSimple` is used instead, publishing on `/lidar_odometry/pose`.

## Pipeline

Uses `lidar3d-katana.yaml` (NOT `lidar3d-gicp-katana.yaml` which crashes with `Matcher_Cov2Cov` assertion error).

**Config file:**
```
~/MOLA-SLAM/mola_ws/install/mola_lidar_odometry/share/mola_lidar_odometry/pipelines/lidar3d-katana.yaml
```

Key density-control parameters:
- `absolute_minimum_sensor_range`: 2.0m (for indoor use)
- `max_points_per_voxel`: 5 (limits density in revisited areas)
- `min_distance_between_points`: 0.05m (5cm minimum spacing)
- `remove_voxels_farther_than`: 0 (map grows forever)

## TF Tree

```
map -> odom -> base_link -> livox_frame
```

## Map Operations

**Save map:**
```bash
ros2 service call /map_save mola_msgs/srv/MapSave \
  "map_path: '/home/robot/mola_maps/my_map'"
```

**Auto-save on shutdown:** `final_map.simplemap` and `estimated_trajectory.tum` in current directory.

**Localize in existing map:**
```bash
ros2 launch mola_bringup mola_localize_launch.py
```

## Source Environment

```bash
source /opt/ros/humble/setup.bash
source ~/MOLA-SLAM/mola_ws/install/setup.bash
```

## Key Packages

| Package | Purpose |
|---------|---------|
| mola_bringup | Launch files (mola_slam_launch.py, filterpass.py) |
| mola_lidar_odometry | Main LO algorithm |
| mp2p_icp | ICP implementation |
| mola_launcher | mola-cli executable |
| mrpt_ros_bridge | MRPT/ROS2 bridge |

## Jetson Service Management

LiDAR services auto-start via systemd on Jetson boot:
```bash
# Check status
ssh robot@192.168.1.20 'systemctl status jetson-lidar.service'

# Manual restart
ssh robot@192.168.1.20 'sudo systemctl restart jetson-lidar.service'

# View logs
ssh robot@192.168.1.20 'cat /tmp/livox_driver.log'
ssh robot@192.168.1.20 'cat /tmp/livox_bridge.log'
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| No /livox/lidar_filtered | Check FilterPass running; verify `/livox/lidar` has data |
| No /livox/lidar | Check Jetson: `pgrep -f livox_lidar_node` and `pgrep -f livox_tcp_bridge` |
| MOLA not starting | Ensure workspace sourced: `source ~/MOLA-SLAM/mola_ws/install/setup.bash` |
| MOLA crashes (GTSAM) | Ensure `use_state_estimator:=False` in launch command |
| High CPU | Use `--no-gui` and/or `--no-rviz` |
| Duplicate Jetson processes | `killall -9 python3 livox_lidar_node` on Jetson, then reboot |
| Build errors after updates | Clean rebuild: `rm -rf build/ install/ log/` then `colcon build` |

## Build Note

Do NOT install `ros-humble-mrpt-*` apt packages - they conflict with the source build.
