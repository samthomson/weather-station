# Agent guidance — weather-station

## Flashing ESP32 boards

**Always use PlatformIO commands. Never use `esptool.py` directly** — it is not on PATH and its bundled copy (`~/.platformio/packages/tool-esptoolpy/esptool.py`) has missing Python dependencies.

```bash
# Erase NVS + flash (new or recycled board):
pio run -e esp32dev_mvpN -t erase && pio run -e esp32dev_mvpN -t upload

# Flash only (keep existing NVS config):
pio run -e esp32dev_mvpN -t upload

# Monitor serial:
pio device monitor -e esp32dev_mvpN
```

Replace `N` with `1`–`5`.

## Confirm which board is connected before flashing

After boot the firmware prints `[wifi] AP SSID: WeatherStation-XXXXXX` every 10 s on serial. Check that `XXXXXX` matches the board entry in `docs/mvp-stations.md` **before** running an upload.

Quick check (passive — does not reset the board):
```bash
python3 -c "
import serial, time
ser = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=0.5)
end = time.time() + 15
while time.time() < end:
    line = ser.readline().decode('utf-8','replace').strip()
    if 'AP SSID' in line: print(line); break
ser.close()
"
```

## Serial port

Default: `/dev/cu.usbserial-0001`. List available ports with:
```bash
ls /dev/cu.usb*
```

## Board registry

Physical board ↔ firmware env ↔ device_id mapping lives in `docs/mvp-stations.md`. Update it after each flash.

## NVS / config

- First boot seeds NVS from the compile-time secrets file (`include/secrets_station_mvpN.h`).
- Subsequent boots load from NVS; the secrets file is ignored.
- Factory reset (from dashboard or `pio run -t erase`) wipes NVS and re-seeds on next boot.

## Project layout

```
src/main.cpp              # sensor reads + Nostr publish
src/config_store.cpp      # NVS-backed user config
src/web_dashboard.cpp     # captive-portal dashboard (embedded HTML)
include/secrets_station_mvpN.h  # per-board first-boot defaults
platformio.ini            # build envs
docs/mvp-stations.md      # physical board registry
docs/mvp-assembly-guide.html    # wiring / assembly guide
```
