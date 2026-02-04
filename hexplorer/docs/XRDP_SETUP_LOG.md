# XRDP Remote Desktop Setup (2026-01-28)

## Overview
XRDP is configured on the Intel Mini PC to allow remote desktop access from Windows computers over WiFi.

## Connection Details

| Setting | Value |
|---------|-------|
| **WiFi IP Address** | 172.16.151.110 |
| **Protocol** | RDP (port 3389) |
| **Username** | robot |

## How to Connect from Windows

1. Open **Remote Desktop Connection** (mstsc.exe)
2. Enter `172.16.151.110`
3. Click Connect
4. Login with username `robot` and your password

## Network Architecture

The Mini PC has two network interfaces:

| Interface | IP Address | Purpose |
|-----------|------------|---------|
| enp2s0 (Ethernet) | 192.168.1.10 | Robot control network |
| wlo1 (WiFi) | 172.16.151.110 | Remote desktop access |

## Project Impact

Using XRDP over WiFi does **not** interfere with robot operations because:

- Robot control traffic (ROS2, Jetson, LiDAR) uses the **wired ethernet** (192.168.1.x)
- RDP sessions use the **WiFi interface** (172.16.151.x)
- These are physically separate network paths

### Considerations

- Graphics-intensive applications (RViz, camera viewers) may have reduced performance over RDP
- For best visualization performance, use direct display or X11 forwarding for specific apps
- Robot control commands are unaffected by RDP usage

## Service Status

XRDP services are enabled and start automatically on boot:

```bash
# Check status
systemctl status xrdp
systemctl status xrdp-sesman

# Restart if needed
sudo systemctl restart xrdp xrdp-sesman
```

## Troubleshooting

### Cannot connect
1. Verify WiFi IP: `ip addr show wlo1`
2. Check XRDP is running: `systemctl status xrdp`
3. Ensure both computers are on same WiFi network

### Black screen after login
- Log out of any existing local session on the Mini PC
- XRDP cannot share a session with an active local login

### Slow performance
- Close unnecessary applications
- Reduce color depth in RDP client settings
- Use "Experience" settings optimized for slow connections
