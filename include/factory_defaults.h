#ifndef FACTORY_DEFAULTS_H
#define FACTORY_DEFAULTS_H

// -----------------------------------------------------------------------------
// FACTORY DEFAULTS (committed, contains no secrets)
// -----------------------------------------------------------------------------
// Everything below is used ONLY on first boot (or after a factory reset) to
// seed the on-device NVS configuration. All station identity (WiFi, name,
// geohash, Nostr key, relay) is configured at runtime from the dashboard:
// join the "WeatherStation-XXXXXX" WiFi the device broadcasts and the page
// opens automatically. NOSTR_PRIVKEY is generated on-device on first boot.
// -----------------------------------------------------------------------------

#define WIFI_SSID ""
#define WIFI_PASS ""

// Blank: auto-generate a fresh secp256k1 key on first boot.
#define NOSTR_PRIVKEY ""

#define NOSTR_RELAY_HOST "wss://relay.relaying.earth"

#define STATION_NAME "Weather Station"
#define STATION_DESCRIPTION ""
#define STATION_GEOHASH ""
#define STATION_ELEVATION ""
#define STATION_POWER "mains"          // mains / solar / battery / solar_battery / usb
#define STATION_CONNECTIVITY "wifi"    // wifi / cellular / ethernet / lora / satellite

// Which sensor drivers are LINKED INTO this firmware build (the MVP sensor
// set). The user toggles per-sensor at runtime from the dashboard; runtime
// toggles only matter for sensors that are also compiled in here.
#define ENABLE_DHT    false
#define ENABLE_BME280 true
#define ENABLE_BH1750 true
#define ENABLE_RAIN   true
#define ENABLE_PMS    true
#define ENABLE_MQ     false
#define ENABLE_OLED   false
#define ENABLE_SPS30  false
#define ENABLE_SDS011 false
#define ENABLE_BMP280 false

#define PMS_MODEL "PMS5003"  // PMS5003 or PMS7003

#endif
