#pragma once

#include <Arduino.h>

// Runtime, NVS-persisted configuration for the weather station.
// Anything the user can change from the dashboard lives here.
struct WxConfig {
  // Home WiFi (STA)
  String wifi_ssid;
  String wifi_pass;

  // Nostr identity + relay
  String nostr_relay;     // "wss://host[:port][/path]" or "host"
  String nostr_privkey;   // 64-hex; auto-generated on first boot if blank

  // Station identity (mirrors kind:16158 tags)
  String station_name;
  String station_description;
  String station_geohash;
  String station_elevation;
  String station_power;          // mains / solar / battery / solar_battery / usb
  String station_connectivity;   // wifi / cellular / ethernet / lora / satellite

  // Behaviour
  uint32_t post_interval_ms = 60000;

  // Per-sensor runtime enables (compile-time #if ENABLE_* still gates whether
  // the driver is linked in; these say whether the user has it wired up).
  bool en_bme280 = false;
  bool en_bh1750 = false;
  bool en_rain   = false;
  bool en_pms    = false;
  String pms_model = "PMS5003";  // "PMS5003" or "PMS7003"
};

namespace config_store {
  // Load config from NVS; if the namespace is empty (first boot / factory
  // reset), seed it with `defaults` (and auto-generate a Nostr key if the
  // factory default is blank), then persist.
  void begin(const WxConfig& defaults);

  // Currently active config (mutable; call save() after changing).
  WxConfig& current();

  // Persist current() to NVS.
  bool save();

  // Wipe NVS so the next boot re-seeds from compile-time defaults.
  void factoryReset();

  // Generate a fresh 64-hex secp256k1-suitable private key (non-zero).
  String generateNostrPrivKeyHex();

  // True if `hex` is exactly 64 lowercase or uppercase hex chars.
  bool isValidPrivKeyHex(const String& hex);
}
