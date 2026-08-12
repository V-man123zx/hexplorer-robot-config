# Object Search System Setup Log

**Date:** 2026-02-19 (created), 2026-02-25 (tested), 2026-03-24 (moved to Fast-LIO2)
**Status:** Core functionality verified on robot in Feb 2026 against MOLA odometry. The
odometry source has since changed to Fast-LIO2 — same `/lidar_odometry/pose` topic, so no
code changes were needed, but the search has not been re-run end to end since. References
to MOLA below describe the odometry in place at the time of testing.

## Overview

Standalone object search program that systematically scans and navigates to find a target object. Built on proven patterns from `obstacle_avoidance.py` (LiDAR front-cone processing) and `object_follower.py` (detection handling, robot control).

**Not based on** `smart_follower.py` — that system's search behavior and obstacle avoidance were untested and broken.

## Files Created

| File | Lines | Purpose |
|------|-------|---------|
| `navigation/object_searcher.py` | ~1000 | Standalone search node (all logic in one file) |
| `scripts/start_object_search.sh` | ~180 | Launch script (sources MOLA + tracking infra) |
| `config/search_visualization.rviz` | ~140 | RViz config for search visualization |
| `~/start_object_search.sh` | symlink | Convenience symlink |

## Architecture

```
Jetson (192.168.1.20)                    Mini PC (192.168.1.10)
jetson_object_tracker.py  --TCP:9997-->  detection_receiver.py
  (YOLO/YOLO-World/color)                 /object_detection
livox_lidar_node   }                     livox_tcp_receiver.py
livox_tcp_bridge.py} --TCP:9998-->         /livox/lidar
                                               |
                                         filterpass.py
                                           /livox/lidar_filtered
                                               |
                                         MOLA LidarOdometry
                                           /state_estimator/pose
                                               |
                                         object_searcher.py  <-- NEW
                                           Scans + Navigates + Follows
```

## State Machine

```
STANDUP -> SCANNING -> NAVIGATING -> SCANNING -> ... (cycle)
               |             |
             FOUND         FOUND
               |
           APPROACH -> CONFIRMED -> SHUTDOWN
```

### SCANNING
- Rotates 360 degrees in place
- Tracks yaw accumulation via MOLA odometry (not time-based)
- Checks camera for target during rotation
- Timeout: 60s (if obstacles prevent full rotation)

### NAVIGATING
- Moves toward unvisited area using VisitedAreaTracker (0.5m grid)
- 8-direction ray evaluation to pick least-visited direction
- Re-evaluates direction every 3 seconds
- Walks until ~2m from last scan point, then scans again
- Obstacle avoidance: front-cone LiDAR (same as obstacle_avoidance.py)

### FOUND
- Stops motion, centers target in camera frame
- Requires 3 consecutive detection frames to filter false positives

### APPROACH
- Slow forward (0.1 m/s) while keeping target centered
- Stops at confirm distance (default 1500mm)
- Returns to SCANNING if detection lost for >3s

### CONFIRMED
- Logs results: target label, distance, position, scans, area covered, time
- Sits down (or stays standing with --no-sit)

## LiDAR Obstacle Avoidance

Uses the **proven front-cone approach** from `obstacle_avoidance.py`:
- Front cone: 30-degree half-angle from forward
- Height filter: 0.05m to 1.2m (ignore ground and high obstacles)
- Min distance: 0.3m (ignore robot body)
- Distance method: 10th percentile (robust to noise)
- Obstacle side: average Y of close front points (>0.1 = left, <-0.1 = right)
- Back cone: same parameters for safe reversing
- Stop distance: 0.8m (default), slow distance: 1.5m (default)

**Not using** smart_follower's broken 360-degree sector approach.

## VisitedAreaTracker

Simple grid-based coverage tracker (inline in object_searcher.py):
- 0.5m grid cells stored as `set()` of (grid_x, grid_y)
- `get_best_direction()`: casts 8 rays (3m long, 0.25m steps), returns angle with most unvisited cells
- `get_coverage_stats()`: returns cell count and approximate area

## RViz Visualization

| Topic | Type | Display |
|-------|------|---------|
| `/object_searcher/visited_grid` | OccupancyGrid | Map display — visited=white, unvisited=grey |
| `/object_searcher/goal_marker` | Marker (Arrow) | Green=navigating, yellow=scanning, red=obstacle |
| `/object_searcher/path_marker` | Marker (LINE_STRIP) | Cyan line showing search path |
| `/object_searcher/scan_marker` | Marker (CYLINDER) | Yellow disk at current scan location |
| `/object_searcher/state` | String (JSON) | State data for monitoring |

## ROS2 Subscriptions

| Topic | Type | Source |
|-------|------|--------|
| `/object_detection` | String (JSON) | detection_receiver.py |
| `/livox/lidar_filtered` | PointCloud2 | filterpass (primary) |
| `/livox/lidar` | PointCloud2 | livox_tcp_receiver (fallback) |
| `/state_estimator/pose` | Odometry | MOLA (primary) |
| `/odom` | Odometry | fallback |

## CLI Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--search-speed` | 0.15 m/s | Forward speed while navigating |
| `--scan-speed` | 0.15 rad/s | Rotation speed during scans |
| `--navigate-distance` | 2.0 m | Distance between scan points |
| `--stop-distance` | 0.8 m | Obstacle stop distance |
| `--slow-distance` | 1.5 m | Obstacle slow distance |
| `--turn-speed` | 0.1 rad/s | Turn speed for avoidance/centering |
| `--confirm-distance` | 1500 mm | Approach to this distance |
| `--no-approach` | false | Skip approach, confirm on detection |
| `--no-sit` | false | Stay standing after finding target |

## Usage

```bash
# Search for person (default)
bash ~/hexplorer/scripts/start_object_search.sh

# Search for specific object
TARGET=bottle bash ~/hexplorer/scripts/start_object_search.sh

# YOLO-World open vocabulary
DETECT_MODE=yolo-world TARGET="red toolbox" bash ~/hexplorer/scripts/start_object_search.sh

# With RViz visualization
bash ~/hexplorer/scripts/start_object_search.sh --rviz

# No approach (confirm immediately on detection)
bash ~/hexplorer/scripts/start_object_search.sh --no-approach

# Custom parameters
SEARCH_SPEED=0.2 STOP_DISTANCE=1.0 bash ~/hexplorer/scripts/start_object_search.sh
```

## Environment Variables (launch script)

| Variable | Default | Description |
|----------|---------|-------------|
| `DETECT_MODE` | yolo | Detection mode: yolo, yolo-world, color |
| `TARGET` | person | What to search for |
| `SEARCH_SPEED` | 0.15 | Navigation speed (m/s) |
| `SCAN_SPEED` | 0.15 | Scan rotation speed (rad/s) |
| `NAVIGATE_DISTANCE` | 2.0 | Meters between scans |
| `STOP_DISTANCE` | 0.8 | Obstacle stop distance (m) |
| `SLOW_DISTANCE` | 1.5 | Obstacle slow distance (m) |
| `CONFIRM_DISTANCE` | 1500 | Approach distance (mm) |

## What's Proven vs New

**Reused from proven code:**
- LiDAR front-cone processing (obstacle_avoidance.py pattern)
- Stand up / sit down sequences (obstacle_avoidance.py + object_follower.py)
- Detection callback + JSON parsing (object_follower.py)
- MOLA infrastructure launch sequence (start_object_tracking.sh --smart)
- Jetson helpers (common.sh)

**New / untested:**
- Scan-navigate cycle (yaw accumulation, VisitedAreaTracker direction picking)
- FOUND -> APPROACH -> CONFIRMED state flow
- OccupancyGrid visualization publishing
- Goal/path/scan marker publishing
- Back-cone LiDAR check for reversing
- Direction re-evaluation during navigation
- The entire integrated system running together

## Testing Checklist

- [x] Run `start_object_search.sh --rviz` with known object in direct line of sight — found on first scan (0.2s)
- [x] RViz: verify visited grid, goal arrow, path line, scan circle all appear correctly
- [x] Place object around a corner — robot scanned, navigated 2m, scanned again, found target (275s, 5m² covered)
- [x] Place obstacles between robot and object — navigated around them (front cone at 0.6-0.8m)
- [x] Ctrl+C at any time — robot sits down safely
- [x] Check logs for search stats — "2 scans, 5.0m2 covered, 275.7s elapsed"
- [x] Test `--no-approach` flag — confirmed immediately on detection
- [ ] Test `--no-sit` flag
- [ ] Test with YOLO-World mode
- [ ] Test with color mode
- [ ] Verify odometry fallback works if MOLA is unavailable
- [ ] Verify LiDAR fallback works if filterpass is unavailable
- [ ] Long-running search (>5 minutes) — check for memory leaks in path_points or visited set

## Bugs Fixed (2026-02-25)

1. **Odometry topic mismatch**: `object_searcher.py` subscribed to `/state_estimator/pose` but MOLA (with `use_state_estimator:=False`) publishes on `/lidar_odometry/pose`. Fixed by adding `/lidar_odometry/pose` as primary subscription.

## Test Results (2026-02-25)

### Test 1: Person in direct line of sight
- Target: person (YOLO mode)
- Result: CONFIRMED in 0.2s, distance 1448mm
- LiDAR: 1925 pts, front obstacle 0.87m
- Odometry: working via `/lidar_odometry/pose`

### Test 2: Person hidden, full scan-navigate cycle
- Target: person (YOLO mode), --no-approach --rviz
- Scan #1: 360° (6.3 rad) in 45.1s
- Navigate: 2.0m from (-0.1, 0.0) to (-1.8, 1.1), obstacles at 0.6-0.8m
- Scan #2: 360° (6.3 rad) in 45.5s
- Navigate #2: continued, found person at (-2.39, 0.96)
- Total: 2 scans, 5.0 m² covered, 275.7s
- RViz: visited grid (11x8), goal arrow, path line, scan circle all working

## Known Concerns

1. **Scan speed**: 0.15 rad/s works well — full rotation in ~45s
2. **Navigate distance**: 2m between scans works well for room-sized spaces
3. **Obstacle avoidance during scanning**: Robot drifts slightly during rotation (0.3-0.4m) but doesn't collide
4. **VisitedAreaTracker growth**: 18 cells for 5m² search — no issues at this scale
5. **Detection latency**: 3 consecutive frames works — brief false positives (1-2 frames) correctly filtered
6. **MOLA odometry drift**: Noticeable drift during rotation (position shifts by ~0.8m during scan) but acceptable for search
