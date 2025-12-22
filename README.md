# Nostr Weather Station

ESP8266 weather station that posts sensor data to Nostr via WebSocket with BIP-340 Schnorr signatures.

**Viewer app:** [github.com/samthomson/weather](https://github.com/samthomson/weather) — sample app for displaying weather data from Nostr events, deployed at [weather.shakespeare.wtf](https://weather.shakespeare.wtf/)

## Hardware

- NodeMCU v2 (ESP8266)
- DHT11 temperature/humidity sensor (D7)
- PMS5003 PM2.5 sensor (D5/D6) — *currently not working*
- SSD1306 OLED display (I2C) - optional if you are only interested in posting to nostr

## Setup

1. Install [PlatformIO](https://platformio.org/)
2. Copy `include/secrets.h.example` to `include/secrets.h`
3. Edit `include/secrets.h` with your WiFi credentials a Nostr private key (create a new one for this device)
4. Build and upload (connect board via USB first):
   ```bash
   pio run --target upload
   pio device monitor
   ```
   PlatformIO will auto-detect the connected board. If multiple devices are connected, specify the port with `-e nodemcuv2 --upload-port /dev/ttyUSB0` (adjust port as needed).

## Configuration

Edit `src/main.cpp` to change:
- `POST_INTERVAL` — posting frequency (default 30s)
- Event `kind` — currently 4223

## Project Structure

```
├── src/main.cpp           # Main code
├── include/secrets.h      # Your credentials (gitignored)
├── include/secrets.h.example
├── platformio.ini
└── README.md
```
