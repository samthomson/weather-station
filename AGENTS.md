# Agent guidance — weather-station

## Git discipline

**Never run `git commit` or `git push` without explicit permission in the current conversation.** Leave changes uncommitted in the working tree for review. This includes chores (docs, registry updates) and "carry-over" work.

**Be concise.** Answer each question in one sentence unless more is genuinely necessary — no preamble, no recaps, no unrequested advice.

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

**Unplug the sensor 5V rail before flashing.** MQ heater boards (minimal air station) draw ~300 mA through the dev board's VIN; that sag corrupts USB-serial flashing (duplicate bytes, checksum errors, sync failures that look like a dead board). Disconnect power to the stripboard/sensors, flash, then reconnect.

Env names: `esp32dev_mvp1`–`mvp7` (weather stations), `esp32dev_aq8` (minimal air station: PMS7003 + MQ-135 + MQ-7), `esp32dev_aq9` (maximal air station: PMS7003 + SCD41 + SGP41 + BME680), `esp32dev_plant10` (plant station: 3x analog soil moisture probes).

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
