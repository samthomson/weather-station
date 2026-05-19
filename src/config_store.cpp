#include "config_store.h"

#include <Preferences.h>
#ifdef ESP32
  #include <esp_system.h>
#endif

namespace {
  Preferences prefs;
  WxConfig g_cfg;
  const char* NS = "wxcfg";
  const char* SEEDED_KEY = "seeded";

  String loadStr(const char* key, const String& fallback) {
    return prefs.isKey(key) ? prefs.getString(key) : fallback;
  }
  uint32_t loadU32(const char* key, uint32_t fallback) {
    return prefs.isKey(key) ? prefs.getUInt(key) : fallback;
  }
  bool loadBool(const char* key, bool fallback) {
    return prefs.isKey(key) ? prefs.getBool(key) : fallback;
  }
}

namespace config_store {

bool isValidPrivKeyHex(const String& hex) {
  if (hex.length() != 64) return false;
  for (size_t i = 0; i < 64; i++) {
    char c = hex.charAt(i);
    bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
    if (!ok) return false;
  }
  // Not all-zero
  for (size_t i = 0; i < 64; i++) if (hex.charAt(i) != '0') return true;
  return false;
}

String generateNostrPrivKeyHex() {
  static const char hexchars[] = "0123456789abcdef";
  String out;
  out.reserve(64);
  while (true) {
    uint8_t k[32];
    #ifdef ESP32
      for (int i = 0; i < 32; i += 4) {
        uint32_t r = esp_random();
        k[i]     = (uint8_t)(r);
        k[i + 1] = (uint8_t)(r >> 8);
        k[i + 2] = (uint8_t)(r >> 16);
        k[i + 3] = (uint8_t)(r >> 24);
      }
    #else
      for (int i = 0; i < 32; i++) k[i] = (uint8_t)random(256);
    #endif
    // Trivially below secp256k1 order: clamp top byte away from 0xFF.
    if (k[0] == 0xFF) k[0] = 0xFE;
    // Reject all-zero.
    bool isZero = true;
    for (int i = 0; i < 32; i++) if (k[i]) { isZero = false; break; }
    if (isZero) continue;
    out = "";
    for (int i = 0; i < 32; i++) {
      out += hexchars[k[i] >> 4];
      out += hexchars[k[i] & 0x0F];
    }
    return out;
  }
}

WxConfig& current() { return g_cfg; }

void begin(const WxConfig& defaults) {
  prefs.begin(NS, /*readOnly=*/false);
  bool seeded = prefs.isKey(SEEDED_KEY) && prefs.getBool(SEEDED_KEY, false);

  if (!seeded) {
    // First boot (or after factory reset). Persist defaults so the user can
    // start changing things from the dashboard.
    g_cfg = defaults;
    if (!isValidPrivKeyHex(g_cfg.nostr_privkey)) {
      g_cfg.nostr_privkey = generateNostrPrivKeyHex();
    }
    save();
    prefs.putBool(SEEDED_KEY, true);
    return;
  }

  // Subsequent boots: load each field, falling back to compile-time defaults
  // for anything missing (e.g., a field added in a firmware upgrade).
  g_cfg.wifi_ssid             = loadStr("wifi_ssid",   defaults.wifi_ssid);
  g_cfg.wifi_pass             = loadStr("wifi_pass",   defaults.wifi_pass);
  g_cfg.nostr_relay           = loadStr("nostr_relay", defaults.nostr_relay);
  g_cfg.nostr_privkey         = loadStr("nostr_pk",    defaults.nostr_privkey);
  g_cfg.station_name          = loadStr("name",        defaults.station_name);
  g_cfg.station_description   = loadStr("desc",        defaults.station_description);
  g_cfg.station_geohash       = loadStr("geohash",     defaults.station_geohash);
  g_cfg.station_elevation     = loadStr("elev",        defaults.station_elevation);
  g_cfg.station_power         = loadStr("power",       defaults.station_power);
  g_cfg.station_connectivity  = loadStr("conn",        defaults.station_connectivity);
  g_cfg.post_interval_ms      = loadU32("post_int",    defaults.post_interval_ms);
  g_cfg.en_bme280             = loadBool("en_bme",     defaults.en_bme280);
  g_cfg.en_bh1750             = loadBool("en_bh",      defaults.en_bh1750);
  g_cfg.en_rain               = loadBool("en_rain",    defaults.en_rain);
  g_cfg.en_pms                = loadBool("en_pms",     defaults.en_pms);
  g_cfg.pms_model             = loadStr("pms_model",   defaults.pms_model);

  // Guarantee a usable Nostr key after every boot.
  if (!isValidPrivKeyHex(g_cfg.nostr_privkey)) {
    g_cfg.nostr_privkey = generateNostrPrivKeyHex();
    prefs.putString("nostr_pk", g_cfg.nostr_privkey);
  }
}

bool save() {
  prefs.putString("wifi_ssid",   g_cfg.wifi_ssid);
  prefs.putString("wifi_pass",   g_cfg.wifi_pass);
  prefs.putString("nostr_relay", g_cfg.nostr_relay);
  prefs.putString("nostr_pk",    g_cfg.nostr_privkey);
  prefs.putString("name",        g_cfg.station_name);
  prefs.putString("desc",        g_cfg.station_description);
  prefs.putString("geohash",     g_cfg.station_geohash);
  prefs.putString("elev",        g_cfg.station_elevation);
  prefs.putString("power",       g_cfg.station_power);
  prefs.putString("conn",        g_cfg.station_connectivity);
  prefs.putUInt("post_int",      g_cfg.post_interval_ms);
  prefs.putBool("en_bme",        g_cfg.en_bme280);
  prefs.putBool("en_bh",         g_cfg.en_bh1750);
  prefs.putBool("en_rain",       g_cfg.en_rain);
  prefs.putBool("en_pms",        g_cfg.en_pms);
  prefs.putString("pms_model",   g_cfg.pms_model);
  return true;
}

void factoryReset() {
  prefs.clear();
}

}  // namespace config_store
