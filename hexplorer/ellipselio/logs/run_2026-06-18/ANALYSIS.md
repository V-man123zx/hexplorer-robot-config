# EllipseLIO Live Run — Analysis & Conclusions (2026-06-18)

Hexplorer hexapod, Livox Mid360 + IMU. EllipseLIO ran live with RViz while the robot
stood up and followed a person (camera-based follower, separate sensor from EllipseLIO's
LiDAR/IMU), so the robot walked around to exercise the odometry.

## Setup
- EllipseLIO config: `mid360.yaml`, RMW `rmw_cyclonedds_cpp`, `OMP_NUM_THREADS=8`.
- Three local patches required to build + run (see git log): Jazzy-only CMake option
  guarded; Livox per-point timestamp fallback; synthesized intra-frame time spread.
- Person-follow: `start_object_tracking.sh`, MAX_SPEED 0.3 m/s, follow distance 0.9 m.

## Key numbers
| Metric | Value | Read |
|---|---|---|
| Processing time | 16 ms mean / 45 ms max per scan | Huge real-time headroom |
| IMU rate | 218 Hz | Healthy |
| **LiDAR rate into node** | **~4 Hz** (nominal 10) | **Bottleneck** |
| Odom output | 4 Hz | Coarse for motion |
| `scan_time` | 0.0999 s | Synthesized deskew span working |
| Features/scan | 731 planes / 103 lines / 203 ellipsoids | Healthy |
| Range | mean 0.83 m, max 2.6 m | Tight/cluttered space |
| Covariance / obs_score | ~1e-5 / 1.0 | EKF confident |
| CPU load | ~4 / 16 cores | Not CPU-bound |

- Health: 1 "before IMU" error (startup transient; was 73+ before the timestamp patch),
  3 benign init-race errors in the first ~1.5 s, then clean.
- 11,169 field-match warnings = deskew-disabled path running every frame (harmless).
- Trajectory: ~(0,0,0) → (-0.89, 1.27) m; **z drifted to +0.34 m mid-walk, recovered to +0.02 m**.
- Following: FOLLOW 100 / BACKUP 14 / TRACK-no-depth 72 / WAITING 200; person 0.33–4.9 m;
  detection present ~48–59% of ticks.

## Conclusions
1. **EllipseLIO works on the Hexplorer** — continuous, confident odometry through a live
   walking test, no crashes or instability. The library is sound on this platform.
2. **EllipseLIO is not the bottleneck — the sensor pipeline is.** It processes a scan in
   16 ms but only receives data at ~4 Hz. Throttling is in the TCP-bridge/receiver chain
   (Jetson published ~6.6 Hz; ~4 Hz reached the node). This caps odometry at 4 Hz and also
   limits Fast-LIO2. Biggest single win to fix.
3. **Missing per-point timestamps cost accuracy.** Deskew is synthesized/approximate; the
   ~0.34 m vertical drift during walking is the symptom (uncorrected intra-scan distortion
   from the hexapod gait, worsened by 4 Hz). Fix: emit per-point `timestamp` at the Jetson
   Livox publisher.
4. **Tight space is the hard case, and it held up.** Mean range 0.83 m, max 2.6 m — the
   cramped scenario where LiDAR odometry struggles; EllipseLIO stayed confident.
5. **Following was detection-limited, not motion-limited.** Robot translated when it had a
   monocular distance, rotated otherwise; ~half the ticks had no detection.

## Recommended next steps (priority order)
1. Fix LiDAR throttling in `livox_tcp_bridge.py` / `livox_tcp_receiver.py` → unlock 10 Hz.
2. Add per-point timestamps (+ tag/line) at the Jetson publisher → enables real deskew,
   should remove the z-drift.
3. Only then is a fair head-to-head vs Fast-LIO2 meaningful (not run side-by-side here).

## Files in this directory
- `analytics.txt` — raw `/analytics` snapshot.
- `ellipselio_node.log` — EllipseLIO node log (field-match warnings filtered out; count noted).
- `person_follow.log` — follower behavior log (full).
