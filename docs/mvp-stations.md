# MVP weather stations — local registry

Track which physical ESP32 is which station. Fill in a row after each flash.

**Flash command** (same firmware for every station; identity is set from the dashboard):

```bash
pio run -e esp32dev --target upload
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

---

## Optional (keep private)

| # | Nostr pubkey | Home WiFi SSID | Location / geohash |
|---|--------------|----------------|--------------------|
| 1 |              |                |                    |
| 2 |              |                |                    |
| 3 |              |                |                    |
| 4 |              |                |                    |
| 5 |              |                |                    |
