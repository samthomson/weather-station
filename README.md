# Nostr Weather Station

ESP8266 weather station that posts sensor data to Nostr via WebSocket with BIP-340 Schnorr signatures.

**Viewer app:** [github.com/samthomson/weather](https://github.com/samthomson/weather) — sample app for displaying weather data from Nostr events, deployed at [weather.shakespeare.wtf](https://weather.shakespeare.wtf/)

## Hardware

- NodeMCU v2 (ESP8266)
- DHT11 temp/humidity (D7) — also works: DHT22
- PMS5003 particulate matter (D5/D6) — reads PM1.0, PM2.5, PM10 | also works: PMS7003
- MQ-135/MQ-7 air quality (A0) — uncalibrated gas detection 0-1023 | also works: MQ-2, other MQ series
- SSD1306 OLED display (I2C) - optional

**Note:** ESP8266 has one analog pin. For more analog sensors, add ADS1115 or use ESP32.

## Setup

1. Install [PlatformIO](https://platformio.org/)
2. Copy `include/secrets.h.example` to `include/secrets.h`
3. Edit `include/secrets.h` with:
   - WiFi credentials
   - Nostr private key (create a new one for this device)
   - Station name
   - Geohash location (use [geohash.jorren.nl](https://geohash.jorren.nl/) to find yours) (optional)
   - Elevation in meters (optional)
   - Power source (mains, solar, battery, etc.)
   - Connectivity type (wifi, cellular, etc.)
4. Build and upload (connect board via USB first):
   ```bash
   pio run --target upload
   pio device monitor
   ```
   PlatformIO will auto-detect the connected board. If multiple devices are connected, specify the port with `-e nodemcuv2 --upload-port /dev/ttyUSB0` (adjust port as needed).

## Configuration

Edit `src/main.cpp` to change:
- `POST_INTERVAL` — posting frequency (default 30s)
- `sensors[]` array — add/remove sensor types

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
├── src/main.cpp           # Main code
├── include/secrets.h      # Your credentials (gitignored)
├── include/secrets.h.example
├── platformio.ini
└── README.md
```

## todo

- [ ] how to handle diff sensor types for the same reading (eg diff brand pm25 sensors)
- [ ] automated tests
- [ ] web dashboard
- [ ] power status for weather station
