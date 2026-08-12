# Fast-LIO2 Setup (2026-03-24)

**Purpose:** LiDAR-inertial odometry for the Hexplorer. Current system — replaced MOLA,
which was LiDAR-only and produced scan-matching glitches driven by the hexapod's gait
oscillation. Fusing the Mid360's IMU removes them.

**Upstream:** https://github.com/hku-mars/FAST_LIO
**Workspace:** `~/fastlio_ws/` (not vendored in this repo)
**Config:** `~/fastlio_ws/src/FAST_LIO/config/hexplorer_mid360.yaml`

## Architecture

```
Jetson (192.168.1.20)                     Mini PC (192.168.1.10)

livox_lidar_node   } --TCP:9998-->        livox_tcp_receiver.py --> /livox/lidar
livox_tcp_bridge.py}
imu_tcp_bridge.py    --TCP:9995-->        imu_tcp_receiver.py   --> /livox/imu
  (both auto-start via jetson_services.sh)                              |
                                                                        v
                                                   Fast-LIO2 (fastlio_mapping)
                                                     iKD-tree map, error-state EKF
                                                                        |
                                                     /Odometry (camera_init -> body)
                                                                        |
                                                          odom_relay.py
                                                     /lidar_odometry/pose  <-- USE THIS
                                                     /tf (odom -> base_link)
```

Fast-LIO2 does its own voxel filtering, so the `filterpass.py` stage MOLA needed is gone.

## Run

```bash
source /opt/ros/humble/setup.bash
source ~/fastlio_ws/install/setup.bash

bash ~/hexplorer/scripts/start_fastlio.sh          # no RViz
bash ~/hexplorer/scripts/start_fastlio.sh --rviz   # RViz, config/fastlio.rviz
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/lidar_odometry/pose` | Odometry | Remapped pose, `odom -> base_link` — the one to subscribe to |
| `/Odometry` | Odometry | Raw Fast-LIO2 output, `camera_init -> body` |
| `/cloud_registered` | PointCloud2 | Current scan registered into the map |
| `/Laser_map` | PointCloud2 | Accumulated map |
| `/path` | Path | Trajectory |

## Frames

`odom_relay.py` exists because Fast-LIO2 names its frames `camera_init` and `body`, and
everything downstream (`smart_follower.py`, `object_searcher.py`) expects
`odom` / `base_link`. The relay rewrites the header, republishes, and broadcasts the TF.

Static transforms set up by the launch script:

| Parent | Child | Offset |
|--------|-------|--------|
| `odom` | `camera_init` | identity (same origin) |
| `base_link` | `livox_frame` | x 0.3, z 0.2 |

MOLA published pose on the same `/lidar_odometry/pose` topic, which is why the switch
needed no changes in the navigation nodes.

## Known issues

- **LiDAR reaches the node at ~4 Hz, not the nominal 10.** The throttle is in the TCP
  bridge chain — the Jetson publishes ~6.6 Hz and ~4 Hz arrives. This caps odometry rate
  for Fast-LIO2 and anything else consuming `/livox/lidar`. Measured during the EllipseLIO
  evaluation, see `../ellipselio/logs/run_2026-06-18/ANALYSIS.md`.
- **The bridge ships XYZI only, with no per-point timestamps.** Deskew is therefore
  approximate. Emitting per-point `timestamp` at the Jetson publisher would fix it and is
  the prerequisite for a fair comparison against other LIO packages.

## Legacy

MOLA remains available at `scripts/start_mola_slam_legacy.sh`; see `MOLA_SLAM_SETUP_LOG.md`
and `MOLA_SLAM_TUNING.md`.
