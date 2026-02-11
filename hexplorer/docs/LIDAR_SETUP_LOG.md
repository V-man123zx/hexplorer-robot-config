# Livox Mid360 LiDAR Setup

## Hardware

- **Model:** Livox Mid360
- **Serial:** 47MCN8F0031553
- **IP Address:** 192.168.1.153 (NOT default 192.168.1.190)
- **Connection:** Ethernet to Jetson Orin Nano

## Architecture

```
Jetson (192.168.1.20)                    Mini PC (192.168.1.10)
livox_lidar_node ─► livox_tcp_bridge.py ─TCP:9998─► livox_tcp_receiver.py ─► /livox/lidar
```

TCP bridge is needed because the Livox driver publishes `custom_msg/msg/LivoxPointcloud` which doesn't transfer cross-machine via DDS and can't be displayed in RViz directly.

## Configuration

**LiDAR IP config file:**
`/home/robot/robot_controller_release/ros2_packages/livox_lidar_node/share/livox_lidar_node/config/lidar_parameters.json`

**Critical:** LiDAR IP must be `192.168.1.153` (not default 192.168.1.190).

## Quick Start

Use the unified launch script:
```bash
bash ~/hexplorer/scripts/start_sensor_demo.sh
```

## Key Files

| File | Machine | Purpose |
|------|---------|---------|
| `~/hexplorer/bridges/livox_tcp_bridge.py` | Jetson | TCP sender |
| `~/hexplorer/bridges/livox_tcp_receiver.py` | Mini PC | TCP receiver, publishes PointCloud2 |

## Camera-LiDAR TF Alignment

The camera optical frame needs rotation to align with LiDAR frame:

**Quaternion:** `qx=-0.5, qy=0.5, qz=-0.5, qw=0.5`

```
map
├── livox_frame (identity transform)
│   └── LiDAR pointcloud (/livox/lidar)
└── camera_depth_optical_frame (rotated)
    └── Camera pointcloud (/camera/points)
```

## Performance

- **Rate:** ~10 Hz
- **Points per frame:** ~14,000-15,000

## Troubleshooting

| Problem | Solution |
|---------|----------|
| LiDAR not detected | `ping 192.168.1.153` from Jetson; verify config IP |
| TCP connection refused | Ensure Livox driver + TCP bridge running on Jetson |
| Pointclouds misaligned | Check camera TF quaternion: `qx=-0.5, qy=0.5, qz=-0.5, qw=0.5` |
| RViz shows 0 messages | Check `ros2 topic hz /livox/lidar`; restart RViz |
