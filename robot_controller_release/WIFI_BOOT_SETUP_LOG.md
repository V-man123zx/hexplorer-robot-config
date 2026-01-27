# WiFi Boot Configuration Setup (2026-01-28)

## Overview
Fixed WiFi boot configuration to support "Client mode primary, AP secondary" - the Mini PC connects to external WiFi networks (for RDP access) while also running a WiFi access point for direct robot connections.

## Problem
The original `start_ap.sh` script was disabling WiFi client mode entirely before creating the access point, which broke automatic WiFi client connections on boot.

### Original Script Issues
```bash
sudo nmcli radio wifi off      # <-- This killed client mode
sudo ifconfig wlo1 192.168.12.1  # <-- This replaced any existing IP
```

## Solution
Modified `/home/robot/.config/autostart/scripts/start_ap.sh` to:
1. **Removed** `nmcli radio wifi off` - no longer disables WiFi radio
2. **Changed** IP assignment to use `ip addr add` (adds secondary IP instead of replacing)
3. **Added** 5-second delay to let NetworkManager establish client connection first

### Fixed Script
```bash
#!/bin/bash
# Start WiFi Access Point as secondary mode (client mode primary)
# Modified to NOT disable WiFi radio - preserves NetworkManager client connection

export SUDO_ASKPASS=/home/robot/.config/autostart/scripts/passwd.sh

# Kill any existing hostapd/dhcpd processes
ps -A | grep hostapd | awk '{print $1}' | xargs sudo kill -9 2>/dev/null
ps -A | grep dhcpd | awk '{print $1}' | xargs sudo kill -9 2>/dev/null

# Ensure WiFi is unblocked (do NOT turn off WiFi radio)
sudo rfkill unblock wlan

# Wait for NetworkManager to establish client connection first
sleep 5

# Add AP IP as secondary address (preserves client IP from NetworkManager)
sudo ip addr del 192.168.12.1/24 dev wlo1 2>/dev/null
sudo ip addr add 192.168.12.1/24 dev wlo1

# Start hostapd for access point
sudo hostapd -B /home/robot/.config/autostart/scripts/hostapd.conf

# Start DHCP server for AP clients
sudo chmod 777 /var/lib/dhcp/dhcpd.leases 2>/dev/null
sudo dhcpd
```

## Boot Sequence

| Order | Service | Action |
|-------|---------|--------|
| 1 | `wifi-enable.service` (systemd) | Enables WiFi radio |
| 2 | NetworkManager | Auto-connects to saved WiFi networks |
| 3 | `start_ap.sh` (GNOME autostart) | Adds AP mode as secondary |

## Current Configuration

### WiFi Interface (wlo1)
| Mode | IP Address | Network | Purpose |
|------|------------|---------|---------|
| Client | 172.16.151.110 | GennFlex | RDP access, internet |
| Access Point | 192.168.12.1 | YJ-MiniHexV2-152 | Direct robot connection |

### Access Point Details
- **SSID:** YJ-MiniHexV2-152
- **Password:** 1234abcd
- **Channel:** 11
- **Security:** WPA2-PSK
- **Config file:** `/home/robot/.config/autostart/scripts/hostapd.conf`

## Key Files

| File | Purpose |
|------|---------|
| `/etc/systemd/system/wifi-enable.service` | Enables WiFi at boot |
| `/home/robot/.config/autostart/start_ap.sh.desktop` | GNOME autostart entry |
| `/home/robot/.config/autostart/scripts/start_ap.sh` | AP startup script |
| `/home/robot/.config/autostart/scripts/hostapd.conf` | hostapd configuration |
| `/home/robot/.config/autostart/scripts/passwd.sh` | sudo askpass helper |

## Verification Commands

```bash
# Check both IPs are present
ip addr show wlo1 | grep inet

# Check WiFi client connection
nmcli connection show --active

# Check AP services running
ps aux | grep -E "(hostapd|dhcpd)" | grep -v grep

# Check WiFi radio status
nmcli radio wifi
```

## Limitations

Running concurrent AP and client mode on the same WiFi interface has limitations:
- Both must use the same WiFi channel
- If external network uses a different channel, AP clients may experience interference
- Intel WiFi chipsets handle this with varying success

## Troubleshooting

### Client mode not connecting
1. Check WiFi is enabled: `nmcli radio wifi`
2. Verify saved connection: `nmcli connection show`
3. Check systemd service: `systemctl status wifi-enable.service`

### Access point not working
1. Check hostapd running: `ps aux | grep hostapd`
2. Check AP IP assigned: `ip addr show wlo1 | grep 192.168.12.1`
3. Restart AP: Run `start_ap.sh` again

### Both modes conflicting
- Check channel alignment between AP (channel 11) and external network
- Consider using 5GHz band for client if available
