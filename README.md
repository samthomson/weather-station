# Nostr Weather Station

ESP8266 weather station that posts sensor data to Nostr via WebSocket with BIP-340 Schnorr signatures.

**Viewer app:** [github.com/samthomson/weather](https://github.com/samthomson/weather) — sample app for displaying weather data from Nostr events, deployed at [weather.shakespeare.wtf](https://weather.shakespeare.wtf/)

## Hardware

- NodeMCU v2 (ESP8266)
- DHT11 temperature/humidity sensor (D7)
- PMS5003 particulate matter sensor (D5/D6) — reads PM1.0, PM2.5, PM10
- SSD1306 OLED display (I2C) - optional if you are only interested in posting to nostr

## Setup

1. Install [PlatformIO](https://platformio.org/)
2. Copy `include/secrets.h.example` to `include/secrets.h`
3. Edit `include/secrets.h` with:
   - WiFi credentials
   - Nostr private key (create a new one for this device)
   - Station name
   - Geohash location (use [geohash.jorren.nl](https://geohash.jorren.nl/) to find yours) (optional)
   - Elevation in meters (optional)
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

The station publishes two types of Nostr events:

- **kind:16158** (replaceable) — Weather station metadata with name, location (geohash), and sensor capabilities
- **kind:4223** (regular) — Sensor readings with temperature, humidity, PM1.0, PM2.5, PM10 in tags

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
