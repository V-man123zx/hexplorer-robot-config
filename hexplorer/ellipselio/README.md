# EllipseLIO on Hexplorer

Integration of [EllipseLIO](https://github.com/v4rl-ucy/ellipselio) (adaptive
LiDAR-inertial odometry) on the Hexplorer hexapod with the Livox Mid360. This folder
is the versioned record; the live workspace is at `~/ellipselio_ws` (not committed —
contains colcon build/install artifacts).

## Reproduce
```bash
mkdir -p ~/ellipselio_ws/src && cd ~/ellipselio_ws/src
git clone https://github.com/v4rl-ucy/ellipselio.git
cd ellipselio
git apply ~/hexplorer/ellipselio/code.patch      # Hexplorer/Humble compatibility patch
cd ~/ellipselio_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ellipselio --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Then copy `start_ellipselio.sh` to `~/ellipselio_ws/` and run it (see `NOTES.md`).

## What the patch does (`code.patch`)
1. **CMakeLists.txt** — guard the Jazzy-only `USE_SCOPED_HEADER_INSTALL_DIR`
   ament_auto_package option behind `if(ROS_JAZZY)` so it builds on ROS 2 Humble.
2. **src/lidar_processing.cpp** (Livox `SetPoint`) — only use the per-point timestamp
   when present (>1e9); the robot's TCP bridge ships XYZI only, so timestamp=0 placed
   every scan at the epoch → "Lidar start before IMU start" → EKF never initialised.
3. **src/lidar_processing.cpp** (`PointCloudHandler`) — for Livox, synthesize an
   intra-frame time spread (backward over 1/rate s by point index) so scans have a
   realistic non-zero duration. Deskew is approximate (bridge provides no real per-point times).

## Files
- `code.patch` — the compatibility patch (apply to a fresh upstream clone).
- `start_ellipselio.sh` — self-sufficient live launcher (brings up Livox bridges, RMW=cyclonedds, OMP cap, optional RViz).
- `NOTES.md` — integration notes, topics, caveats.
- `logs/run_2026-06-18/` — first live run (robot stood + followed a person). See `ANALYSIS.md`.

## Headline result (2026-06-18 run)
EllipseLIO runs live and confident, real-time (16 ms/scan). The bottleneck is the
sensor pipeline: LiDAR reaches the node at only ~4 Hz (vs 10 nominal). Vertical drift
~0.34 m mid-walk from the disabled deskew (no per-point timestamps). Full analysis in
`logs/run_2026-06-18/ANALYSIS.md`.
