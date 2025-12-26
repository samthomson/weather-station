# Nostr Weather Station

ESP8266 weather station that posts sensor data to Nostr via WebSocket with BIP-340 Schnorr signatures.

**Viewer app:** [github.com/samthomson/weather](https://github.com/samthomson/weather) — sample app for displaying weather data from Nostr events, deployed at [weather.shakespeare.wtf](https://weather.shakespeare.wtf/)

## Hardware

**Supported Boards:**
- NodeMCU v2 (ESP8266)
- ESP32 Dev Board

**Sensors:**
- DHT11 temp/humidity — also works: DHT22
- PMS5003 particulate matter — reads PM1.0, PM2.5, PM10 | also works: PMS7003
- MQ-135/MQ-7 air quality — uncalibrated gas detection 0-1023 | also works: MQ-2, other MQ series
- SSD1306 OLED display (I2C) - optional

### Pin Connections

**ESP8266 (NodeMCU v2):**
- DHT11: D7
- PMS5003: TX→D5, RX→D6
- MQ sensor: A0
- OLED: I2C (D1=SCL, D2=SDA)

**ESP32:**
- DHT11: GPIO4
- PMS5003: TX→GPIO16 (RX2), RX→GPIO17 (TX2)
- MQ sensor: GPIO36 (ADC1_CH0)
- OLED: I2C (GPIO21=SDA, GPIO22=SCL)

**Note:** ESP8266 has one analog pin. ESP32 has many - great for multiple analog sensors!

## Setup

1. Install [PlatformIO](https://platformio.org/)
2. Create station-specific secrets files:
   - Copy `include/secrets.h.example` to `include/secrets_station1.h`
   - Copy `include/secrets.h.example` to `include/secrets_station2.h` (if you have multiple stations)
3. Edit each secrets file with:
   - WiFi credentials
   - Nostr private key (create a **unique key for each station**)
   - Station name (unique per station)
   - Geohash location (use [geohash.jorren.nl](https://geohash.jorren.nl/) to find yours) (optional)
   - Elevation in meters (optional)
   - Power source (mains, solar, battery, etc.)
   - Connectivity type (wifi, cellular, etc.)

**Note:** Each station should have its own Nostr keypair so they have separate identities on the network.
4. Build and upload (connect board via USB first):
   
   **Station 1 - ESP8266 (NodeMCU v2):**
   ```bash
   pio run -e nodemcuv2_station1 --target upload
   pio device monitor -e nodemcuv2_station1
   ```
   
   **Station 2 - ESP32:**
   ```bash
   pio run -e esp32dev_station2 --target upload
   pio device monitor -e esp32dev_station2
   ```
   
   If multiple devices are connected, specify the port with `--upload-port /dev/ttyUSB0` (adjust as needed).
   
   To add more stations, create new environments in `platformio.ini` with unique secrets files.

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
