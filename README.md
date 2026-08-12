# Hexplorer Robot Config
Current connect ip with router: 192.168.8.123
Password of minipc: 123

Sensing, odometry and autonomy software for a Dobot Hexplorer hexapod running ROS 2 Humble,
with a Livox Mid360 LiDAR and a RealSense D435 on a Jetson Orin Nano.

This is the live configuration for one physical robot, published as a reference rather than
as a reusable package. Paths, IP addresses and the two-machine split are specific to this
build. Most of it ports to a similar rig with small edits.

## Hardware

| Machine | Address | Role |
|---------|---------|------|
| Intel mini PC | 192.168.1.10 | ROS 2 master, robot controller, odometry, navigation |
| Jetson Orin Nano | 192.168.1.20 | Camera and LiDAR capture, YOLO inference |
| Livox Mid360 | 192.168.1.153 | 3D LiDAR + IMU |
| RealSense D435 | USB (Jetson) | Colour + depth |

The two machines exchange sensor data over plain TCP bridges instead of DDS. Cross-machine
DDS discovery was unreliable here, and LiDAR traffic on the shared link starved the robot's
own UDP motor commands, which drops it into safety damping mid-walk. Each sensor gets a
small publisher on the Jetson and a receiver on the mini PC that republishes locally.

```
Mini PC                                Jetson Orin Nano
depth_bridge_receiver.py  <--TCP:9999--  realsense_depth_tcp_publisher.py
livox_tcp_receiver.py     <--TCP:9998--  livox_tcp_bridge.py
imu_tcp_receiver.py       <--TCP:9995--  imu_tcp_bridge.py
detection_receiver.py     <--TCP:9997--  jetson_object_tracker.py
                          <--TCP:9996--  (annotated image stream)
```

The RealSense can only be opened by one process, so depth and object tracking are mutually
exclusive: whichever script you launch stops the other one on the Jetson.

## Layout

```
hexplorer/
├── sensors/      Camera publishers and viewers
├── tracking/     YOLO / YOLO-World detection and object following
├── navigation/   Obstacle avoidance, object search, dances
├── bridges/      TCP sensor bridges and the odometry frame relay
├── voice/        ElevenLabs voice control
├── ellipselio/   EllipseLIO odometry evaluation (patch, launcher, run logs)
├── config/       RViz configs
├── docs/         Setup logs, per-subsystem
└── scripts/      Launch scripts
```

## Setup

Requires ROS 2 Humble on Ubuntu 22.04, plus `sshpass` on the mini PC and `ultralytics` on
the Jetson. The RealSense stack is pinned to **pyrealsense2 2.55 built from source** —
2.56.4 throws "RGB modules inconsistency" on this camera. Launch scripts need
`LD_LIBRARY_PATH=/usr/local/lib` for that build to be picked up.

Fast-LIO2 lives in a separate workspace (`~/fastlio_ws`) and is not vendored here.

Clone to `~/hexplorer` on the mini PC — the scripts refer to each other by that path:

```bash
git clone https://github.com/V-man123zx/hexplorer-robot-config.git
ln -s "$PWD/hexplorer-robot-config/hexplorer" ~/hexplorer

cp ~/hexplorer/.env.example ~/hexplorer/.env
# set JETSON_PASS in ~/hexplorer/.env
```

Every launch script sources `scripts/common.sh`, which reads that `.env` and refuses to
start if `JETSON_PASS` is empty. 

## Running

```bash
bash ~/hexplorer/scripts/start_sensor_demo.sh            # camera + LiDAR + tracking
bash ~/hexplorer/scripts/start_fastlio.sh                # LiDAR-inertial odometry
bash ~/hexplorer/scripts/start_object_tracking.sh        # robot follows a person
bash ~/hexplorer/scripts/start_obstacle_avoidance.sh     # autonomous walking
bash ~/hexplorer/scripts/start_voice_demo.sh             # voice control
```

Add `--rviz` to most of them. Detection target and mode are environment variables:

```bash
TARGET=bottle bash ~/hexplorer/scripts/start_object_tracking.sh
DETECT_MODE=yolo-world TARGET="yellow ball" bash ~/hexplorer/scripts/start_object_tracking.sh
```

Full command reference: [hexplorer/README.md](hexplorer/README.md).

## Odometry

**Fast-LIO2 is the current system** — LiDAR and IMU fused through an EKF, publishing
`/lidar_odometry/pose` and the `odom -> base_link` transform. It replaced MOLA, which was
LiDAR-only and produced scan-matching glitches driven by the hexapod's gait oscillation.
MOLA is kept as a fallback at `scripts/start_mola_slam_legacy.sh`; its docs are marked
legacy. [EllipseLIO](hexplorer/ellipselio/) was evaluated separately in June 2026 — it runs
real-time on this platform, but the sensor pipeline caps it at 4 Hz.

## Status

| Component | State |
|-----------|-------|
| Sensor bridges (camera, LiDAR, IMU) | Working, in daily use |
| Fast-LIO2 odometry | Working |
| Object tracking and following | Working |
| Obstacle avoidance | Working |
| Voice control | Working; needs an RDP session for audio, no headless audio yet |
| Object search | Logic tested Feb 2026, not re-tested since the Fast-LIO2 switch |
| MOLA odometry | Legacy, superseded |

## Documentation

Setup logs for each subsystem are in [hexplorer/docs/](hexplorer/docs/). Start with
[HEXPLORER_CONTROL.md](hexplorer/docs/HEXPLORER_CONTROL.md) for the robot's state machine
and control topics — the state transitions are not obvious and commands have to be
published continuously at ~20 Hz or the controller ignores them.

## Licence

MIT, see [LICENSE](LICENSE). Third-party components (Fast-LIO2, MOLA, EllipseLIO,
Ultralytics) keep their own licences; the EllipseLIO patch in
[hexplorer/ellipselio/code.patch](hexplorer/ellipselio/code.patch) applies to upstream
sources and is covered by upstream's terms.
