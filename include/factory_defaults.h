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

#endif
