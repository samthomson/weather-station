# MVP weather stations — local registry

Track which physical ESP32 is which station. Fill in a row after each flash.

**Flash command** (same firmware for every station; env only sets first-boot factory defaults):

```bash
pio run -e esp32dev_mvp1 --target upload   # station 1
pio run -e esp32dev_mvp2 --target upload   # station 2
# … mvp3–mvp6 (weather), mvp7 (weather, Sam's), aq8 (minimal air station), aq9 (maximal air station), plant10 (plant/soil station)
```

**After flash**, read from serial boot log or join the setup WiFi:

- `device_id` — last 6 hex of MAC (also the `device_id` Nostr tag)
- Setup AP — `WeatherStation-XXXXXX` (same 6 hex suffix)
- Full MAC — printed by esptool during upload

**Reassigning a board** (e.g. old prototype #3 → production #1): flashing alone does **not** wipe dashboard config. Either use **Factory reset** on the dashboard, or update name/relay/WiFi manually.

**Default relay** (first boot only): `wss://relay.relaying.earth`

---

| # | PlatformIO env   | device_id | Setup AP (SSID)        | Full MAC           | Flashed    | Notes                                      |
|---|------------------|-----------|------------------------|--------------------|------------|--------------------------------------------|
| 1 | esp32dev_mvp1    | B865E4    | WeatherStation-B865E4  | e4:65:b8:1b:43:08  | 2026-05-24 | OLD firmware (wrong MAC suffix); will become 1B4308 after reflash |
| 2 | esp32dev_mvp2    | B3A3A0    | WeatherStation-B3A3A0  | a0:a3:b3:2f:95:d8  | 2026-05-24 | OLD firmware (wrong MAC suffix); will become 2F95D8 after reflash |
| 3 | esp32dev_mvp3    | 19D3B8    | WeatherStation-19D3B8  | e4:65:b8:19:d3:b8  | 2026-05-24 | Fixed firmware flashed                     |
| 4 | esp32dev_mvp4    | 3141C4    | WeatherStation-3141C4  |                    | 2026-05-24 |                                            |
| 5 | esp32dev_mvp5    | 0BF6BC    | WeatherStation-0BF6BC  |                    | 2026-05-24 |                                            |
| 6 | esp32dev_mvp6    | 65371C    | WeatherStation-65371C  | d4:e9:f4:65:37:1c  | 2026-11-13 | BMP280 (no humidity); needs real BME280     |
| 7 | esp32dev_mvp7    |           |                        |                    |            | Standard MVP model (BME280, BH1750, MH-RD, PMS5003) — for Sam |
| 8 | esp32dev_aq8     | 7030D0    | WeatherStation-7030D0  |                    | 2026-11-14 | Minimal air station: PMS7003 + MQ-135 + MQ-7 (raw, uncalibrated)      |
| 9 | esp32dev_aq9     | 6FBC98    | WeatherStation-6FBC98  |                    | 2026-11-14 | Maximal air station: PMS7003 + SCD41 + SGP41 + BME680        |
| 10 | esp32dev_plant10 | 7D3BEC    | WeatherStation-7D3BEC  |                    | 2026-11-19 | Plant station: 3x soil probes (HW-080 w/ D25 switched excitation, CAPACITIVE-V1.2, SOIL-METAL-1M) · guide: docs/plant-assembly-guide.html · reflash 2026-11-19 wiped NVS → new Nostr key (old b9977e61… orphaned) |

---

## Optional (keep private)

| # | Nostr pubkey | Home WiFi SSID | Location / geohash |
|---|--------------|----------------|--------------------|
| 1 |              |                |                    |
| 2 |              |                |                    |
| 3 |              |                |                    |
| 4 |              |                |                    |
| 5 |              |                |                    |
| 6 |              |                |                    |
| 7 |              |                |                    |
| 8 |              |                |                    |
| 9 |              |                |                    |
| 10 |              |                |                    |
