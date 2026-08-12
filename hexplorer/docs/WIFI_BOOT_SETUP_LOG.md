# WiFi Boot Configuration (2026-01-28)

## Overview
WiFi configured for "Client mode primary, AP secondary" - connects to external networks while also running access point.

## Boot Sequence

| Order | Service | Action |
|-------|---------|--------|
| 1 | `wifi-enable.service` (systemd) | Enables WiFi radio |
| 2 | NetworkManager | Auto-connects to saved WiFi |
| 3 | `start_ap.sh` (GNOME autostart) | Adds AP mode as secondary |

## WiFi Interface (wlo1)

| Mode | IP Address | Network | Purpose |
|------|------------|---------|---------|
| Client | varies | site WiFi | RDP access, internet |
| Access Point | 192.168.12.1 | YJ-MiniHex<unit> | Direct robot connection |

### Access Point Details
- **SSID:** `YJ-MiniHex<unit>` (factory SSID, differs per robot)
- **Password:** set in `hostapd.conf` on the robot, not published here
- **Channel:** 11
- **Security:** WPA2-PSK

The factory AP password is short and shipped identically across units. Change it in
`hostapd.conf` before running the robot anywhere you don't control the radio space.

## Key Files

| File | Purpose |
|------|---------|
| `/etc/systemd/system/wifi-enable.service` | Enables WiFi at boot |
| `/home/robot/.config/autostart/start_ap.sh.desktop` | GNOME autostart entry |
| `/home/robot/.config/autostart/scripts/start_ap.sh` | AP startup script |
| `/home/robot/.config/autostart/scripts/hostapd.conf` | hostapd configuration |
| `/home/robot/.config/autostart/scripts/passwd.sh` | sudo askpass helper |

## Verification

```bash
ip addr show wlo1 | grep inet          # Check both IPs present
nmcli connection show --active          # Check client connection
ps aux | grep -E "(hostapd|dhcpd)"      # Check AP services
nmcli radio wifi                        # Check WiFi radio status
```

## Limitations

- Both AP and client mode share the same WiFi channel
- If external network uses a different channel, AP clients may experience interference

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Client not connecting | Check `nmcli radio wifi`, verify saved connections |
| AP not working | Check `ps aux \| grep hostapd`, verify 192.168.12.1 on wlo1 |
| Both modes conflicting | Check channel alignment (AP uses channel 11) |
