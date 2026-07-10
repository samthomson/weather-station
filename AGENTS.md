# Agent guidance — weather-station

## Flashing ESP32 boards

**Always flash via the `nix run` wrappers; never invoke `esptool` manually** — the wrappers use a pinned esptool with the right flash offsets.

```bash
# Erase NVS + flash (new or recycled board):
nix run .#flash-erase-mvp -- [PORT]

# Flash only (keep existing NVS config):
nix run .#flash-mvp -- [PORT]

# Monitor serial (picocom, 115200 baud; exit with Ctrl-A Ctrl-X):
nix run .#monitor -- [PORT]
```

`PORT` is optional and defaults to `/dev/ttyUSB0`. Variants: `mvp`, `airquality-sps30`, `airquality-sds011` (e.g. `flash-airquality-sps30`).

Every station runs the same firmware; identity lives in NVS (dashboard-configured).

## Confirm which board is connected before flashing

After boot the firmware prints `[wifi] AP SSID: WeatherStation-XXXXXX` every 10 s on serial. Check that `XXXXXX` matches the board entry in `docs/mvp-stations.md` **before** running an upload.

Quick check (passive — does not reset the board):
```bash
python3 -c "
import serial, time
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)  # macOS: /dev/cu.usbserial-*
end = time.time() + 15
while time.time() < end:
    line = ser.readline().decode('utf-8','replace').strip()
    if 'AP SSID' in line: print(line); break
ser.close()
"
```

## Serial port

Linux: `/dev/ttyUSB0` (the default). macOS: `/dev/cu.usbserial-*`. List available ports with:
```bash
ls /dev/ttyUSB* /dev/cu.usb* 2>/dev/null
```

## Board registry

Physical board ↔ firmware env ↔ device_id mapping lives in `docs/mvp-stations.md`. Update it after each flash.

## NVS / config

- First boot seeds NVS from the committed defaults (`include/factory_defaults.h`) — blank identity, MVP sensor set.
- Subsequent boots load from NVS; the defaults file is ignored.
- Factory reset (from dashboard or `nix run .#flash-erase-mvp`) wipes NVS and re-seeds on next boot.

## Project layout

```
src/main.cpp              # sensor reads + Nostr publish
src/config_store.cpp      # NVS-backed user config
src/web_dashboard.cpp     # captive-portal dashboard (embedded HTML)
include/factory_defaults.h # first-boot NVS seed + compile-time sensor set
main/                     # ESP-IDF app entry (CMake component)
flake.nix                 # build env (firmware variants, flash/monitor apps, devShell)
nix/                      # toolchain pin + firmware/variant derivations
sdkconfig.defaults        # ESP-IDF configuration
partitions.csv            # flash partition table
docs/mvp-stations.md      # physical board registry
docs/refactoring-roadmap.md     # future refactoring plan (nix, pure ESP-IDF)
docs/mvp-assembly-guide.html    # wiring / assembly guide
```
