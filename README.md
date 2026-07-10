# Nostr Weather Station

ESP32 weather station that posts sensor data to Nostr via WebSocket with BIP-340 Schnorr signatures.

**Viewer app:** [github.com/samthomson/weather](https://github.com/samthomson/weather) — sample app for displaying weather data from Nostr events, deployed at [weather.shakespeare.wtf](https://weather.shakespeare.wtf/)

## Hardware

**Supported Boards:**
- ESP32 Dev Board

**Sensors:**
- DHT11 temp/humidity — also works: DHT22
- PMS5003 particulate matter — reads PM1.0, PM2.5, PM10 | also works: PMS7003
- MQ-135/MQ-7 air quality — uncalibrated gas detection 0-1023 | also works: MQ-2, other MQ series
- SSD1306 OLED display (I2C) - optional

### MVP shopping list

Wall-powered reference build for new prototypes: **ESP32**, particulates, temp/humidity/pressure, daylight, rain — **no MQ gas sensor, no OLED**.

| Qty | Part |
|-----|------|
| 1 | ESP32 dev board (generic ESP32-D0WD dev board, `esp32dev` class) |
| 1 | BME280 breakout (I2C, 3.3 V) — temperature, humidity, barometric pressure |
| 1 | PMS5003 — PM1.0, PM2.5, PM10 (include a cable if the module does not ship with one) |
| 1 | BH1750 breakout (I2C) — ambient light (lux) |
| 1 | MH-RD rain sensor module (resistive panel, analog out) |
| 1 | USB cable — connector must match the board (**Micro-USB** or **USB-C**) |
| 1 | USB wall charger — **5 V**, **≥ 1 A** (2 A is a comfortable margin for ESP32 + PMS fan) |

Hookup wire or Dupont jumpers between the dev board and breakouts are assumed by your enclosure / assembly.

**MVP firmware:** the default build (`include/factory_defaults.h`) already enables `ENABLE_BME280`, `ENABLE_BH1750`, `ENABLE_RAIN`, and `ENABLE_PMS` with `PMS_MODEL "PMS5003"` — no configuration needed.

**MVP ESP32 wiring:** PMS5003 serial RX→**GPIO16**, TX→**GPIO17**. BME280 and BH1750 share **I2C** (**GPIO21**=SDA, **GPIO22**=SCL). Rain sensor analog→**GPIO34**.

Stripboard matrix + cable list: **[docs/mvp-assembly-guide.html](docs/mvp-assembly-guide.html)**. Flash & setup: below.

### MVP wire colours

Use **industry-standard** colours for power rails, and **project theme** colours on the nets with the most jumpers. One dupont colour per stripboard row; label GND / 3V3 / 5V on the board edge.

| Net | Dupont colour | Type | Jumpers |
|-----|---------------|------|---------|
| GND | Black | standard | 5 |
| 3.3 V | Orange | theme + standard for 3V3 | 4 |
| 5 V | Red | standard | 2 |
| I²C SDA | Purple | theme | 3 |
| I²C SCL | White | theme | 3 |
| PMS TX → ESP RX | Green | other | 2 |
| ESP TX → PMS RX | Grey | theme | 2 |
| Rain AO | Blue | other | 2 |
| Rain pad → MH-RD | Blue | pad sense | 2 |

**Theme palette** (UI brand + preferred dupont stock): orange, purple, white, grey. Orange does double duty as 3.3 V (common convention). Green and blue are only used on 2-wire nets so all four theme colours land on the busier rows.

### Pin Connections

**ESP32:**
- DHT11: GPIO4
- PMS5003: TX→GPIO16 (RX2), RX→GPIO17 (TX2)
- MQ sensor: GPIO36 (ADC1_CH0)
- OLED: I2C (GPIO21=SDA, GPIO22=SCL)

## Setup

1. Install [nix](https://nixos.org/download/) (with flakes enabled).
2. Connect the board via USB and flash. For a **brand-new or recycled board**, erase first so NVS starts clean:
   ```bash
   # Erase + flash (new/recycled board — clears all NVS):
   nix run .#flash-erase-mvp -- [PORT]

   # Flash only (re-flash same board, keep NVS config):
   nix run .#flash-mvp -- [PORT]

   # Monitor serial output (picocom, 115200 baud; exit with Ctrl-A Ctrl-X):
   nix run .#monitor -- [PORT]
   ```
   `PORT` is optional and defaults to `/dev/ttyUSB0`.

   To build the firmware without flashing: `nix build .#firmware-mvp` (other variants: `firmware-airquality-sps30`, `firmware-airquality-sds011`).

   Every station runs the **same firmware**; identity (name, WiFi, keys) lives in NVS and is set from the dashboard. Track which board is which in [`docs/mvp-stations.md`](docs/mvp-stations.md).

   **Do not use `esptool.py` directly** — always flash via the `nix run` wrappers, which invoke a pinned esptool with the right flash offsets.

   Default Nostr relay on first boot: `wss://relay.relaying.earth`.

   **Confirm the right board is connected** before flashing. After boot, serial prints `[wifi] AP SSID: WeatherStation-XXXXXX` every 10 s — verify `XXXXXX` matches the board's label in `docs/mvp-stations.md`.

## Configure from your phone (the dashboard)

The station permanently broadcasts its own open WiFi network for configuration. You don't need a router, an app, or an IP address.

1. Plug the station in.
2. On your phone, join the WiFi named **`WeatherStation-XXXXXX`** (`XXXXXX` is the last 6 hex of the board's MAC). It has no password.
3. The "Sign in to network" sheet pops up automatically with the dashboard. (If not, open a browser to `http://192.168.4.1`.)
4. Set your home WiFi, station name, geohash, etc. Toggle the sensors you actually have wired. Hit Save.
5. The station joins your home WiFi and starts publishing to Nostr.

You can rejoin `WeatherStation-XXXXXX` at any time to change anything — including the home WiFi password if it ever changes. Everything you change persists across power loss (stored in flash / NVS).

Once on the home network you can also reach the dashboard at `http://weather.local` (mDNS).

### What's configurable from the dashboard

- **WiFi (home)** — SSID and password, with a built-in network scan.
- **Station identity** — name, description, geohash (with a "use my GPS" button that pulls from the phone), elevation, power source, connectivity type.
- **Nostr** — relay URL (preset dropdown + custom), private key (auto-generated on first boot; reveal / copy / regenerate / paste-an-existing-one).
- **Sensors** — per-sensor on/off toggles for BME280, BH1750, MH-RD rain, PMS5003/PMS7003. Only sensors you actually have wired need to be on.
- **Advanced** — post interval, restart, factory reset.

## Events Published

### kind:16158 (replaceable) — Station Metadata

Describes the station hardware, power, and sensors:

```json
{
  "tags": [
    ["name", "Weather Station 1"],
    ["g", "w5q6u"],
    ["power", "mains"],
    ["connectivity", "wifi"],
    ["sensor", "temp", "DHT11"],
    ["sensor", "pm25", "PMS5003"]
  ]
}
```

**Power types:** Currently `mains` only. Planned: `solar`, `battery`, `solar_battery`, `usb`  
**Connectivity types:** Currently `wifi` only. Planned: `cellular`, `ethernet`, `lora`, `satellite`

### kind:4223 (regular) — Sensor Readings

Each tag: `[sensor_type, value, model]`

```json
{
  "tags": [
    ["t", "weather"],
    ["a", "16158:<pubkey>:"],
    ["temp", "22.5", "DHT11"],
    ["humidity", "65.2", "DHT11"],
    ["pm25", "12", "PMS5003"],
    ["air_quality", "627", "MQ-135"]
  ]
}
```

**Tag Structure:** Third parameter identifies sensor model for cross-station comparison and multi-sensor setups.

**Units:** temp=°C, humidity=%, pm1/pm25/pm10=µg/m³, air_quality=0-1023 (raw analog)

## Project Structure

```
├── src/main.cpp                   # Sensor reads + Nostr publish
├── src/config_store.cpp           # NVS-backed user config
├── src/web_dashboard.cpp          # Captive-portal dashboard
├── include/config_store.h
├── include/web_dashboard.h
├── include/factory_defaults.h     # First-boot NVS seed + compile-time sensor set
├── main/                          # ESP-IDF app entry (CMake component)
├── docs/refactoring-roadmap.md    # Decision log + future refactoring plan (nix, pure ESP-IDF)
├── docs/nixify-plan.md            # Nixification work packages (WP1–WP7, tests, QEMU)
├── flake.nix                      # Nix build (firmware variants, flash/monitor apps, devShell)
├── nix/                           # Toolchain pin + firmware/variant derivations
├── CMakeLists.txt                 # ESP-IDF project root
├── sdkconfig.defaults             # ESP-IDF configuration
├── partitions.csv                 # Flash partition table
└── README.md
```

## todo

- [ ] automated tests
- [x] web dashboard
- [ ] more modular readings
- [ ] MVP
  - [x] hardware shopping list (see **MVP shopping list** under Hardware)
  - [ ] daylight
  - [ ] rain
  - [ ] air pollution
  - [ ] temperature / humidity
  - [ ] pressure
- [ ] geohash tag for the weather station event per geohash level (https://github.com/nostr-protocol/nips/pull/2163#discussion_r2653978414)
- [ ] faulty sensor/data. omit the sensor reading in the 4223 reading, and mark the sensor as 418 in 16158
- [ ] document that its a string 9https://github.com/nostr-protocol/nips/pull/2163#discussion_r2653971649)
- [ ] nip31 alt tag (for each event?) https://github.com/nostr-protocol/nips/pull/2163#issuecomment-3693704646
- [ ] add `"observed_at": <unix timestamp in seconds>` for each 16158.
- [ ] expand past weather to general environmental data (eg soil acidity, soil moisture, etc). think of a more inclusive name. maybe terrametry (telemetry, but earth scoped)

## Air quality station

A future variant focused on indoor air quality. Goal: factory-calibrated, plug-and-trust — no per-unit calibration or burn-in. All sensors below are I²C (shared GPIO21 SDA / GPIO22 SCL), so they wire in parallel on one ESP32. MQ sensors are deliberately excluded — they need per-unit calibration and only give relative values.

- [ ] PMS5003 — PM1/2.5/10 in µg/m³ (already owned)
- [ ] SCD41 — real CO₂ (NDIR) in ppm, plus temp + humidity
- [ ] SGP41 — VOC + NOx index (relative; good for spotting change/spikes)
- [ ] BME680 (optional) — IAQ gas resistance + temp/humidity/pressure

### Possible future sensors

- [ ] Winsen ZE07-CO / ZE15-CO — carbon monoxide (electrochemical, calibrated; real safety metric). UART, not I²C.
- [ ] Sensirion SFA30 — formaldehyde (HCHO); off-gasses from furniture/flooring. I²C/UART.
- [ ] Radon detector — serious long-term indoor risk, but needs a dedicated (pricier) unit; not an I²C add-on.
- [ ] Sound level (mic) — environmental noise; beyond air quality but useful for an indoor environment station.