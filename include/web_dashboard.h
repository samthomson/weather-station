#pragma once

#include <Arduino.h>

// Live snapshot of sensor readings + connectivity, fed into the dashboard's
// /api/status endpoint. main.cpp owns the source of truth and just calls
// updateStatus() periodically.
struct DashboardStatus {
  // Connectivity
  bool sta_connected = false;
  String sta_ip;
  String sta_ssid;
  int    sta_rssi = 0;
  bool   ws_connected = false;
  uint32_t last_post_ms = 0;     // millis() of last successful publish
  String last_publish_at;        // human-readable, optional

  // Latest readings (NAN/0 if unavailable)
  float temp_c = NAN;
  float humidity = NAN;
  float pressure_hpa = NAN;
  float lux = NAN;
  unsigned int rain_raw = 0;
  unsigned int pm1 = 0;
  unsigned int pm25 = 0;
  unsigned int pm10 = 0;
  unsigned int co2_ppm = 0;      // SCD41
  int    voc_index = 0;          // SGP41 (1-500, unitless; 100 = average)
  int    nox_index = 0;          // SGP41 (1-500, unitless; 1 = average)
  uint32_t gas_res_ohm = 0;      // BME680 heater resistance
  unsigned int mq135_raw = 0;    // MQ-135 raw ADC (0-4095)
  unsigned int mq7_raw = 0;      // MQ-7 raw ADC (0-4095)
  unsigned int soil_a_raw = 0;  // Soil A raw ADC (0-4095)
  unsigned int soil_b_raw = 0;  // Soil B raw ADC (0-4095)
  unsigned int soil_c_raw = 0;  // Soil C raw ADC (0-4095)
  bool   has_temp = false;
  bool   has_humidity = false;
  bool   has_pressure = false;
  bool   has_lux = false;
  bool   has_pm = false;
  bool   has_co2 = false;
  bool   has_voc = false;
  bool   has_gas = false;

  // Nostr identity (display)
  String npub;       // bech32 "npub1..." computed in main.cpp
  String pubkey_hex;

  // Stable hardware identity (eFuse MAC suffix, matches AP SSID suffix)
  String device_id;
};

// Side effects triggered by the dashboard when the user changes config.
struct DashboardCallbacks {
  // Called after config has been persisted; main.cpp should re-apply any
  // subsystems that depend on the changed fields. Bitmask via the booleans.
  void (*onApply)(bool wifiChanged,
                  bool relayChanged,
                  bool keyChanged,
                  bool sensorsChanged,
                  bool intervalChanged) = nullptr;
  void (*onRestart)() = nullptr;
  void (*onFactoryReset)() = nullptr;
  void (*onRegenerateKey)() = nullptr;
};

namespace web_dashboard {
  // Start AP+STA, DNS captive portal, HTTP server, and mDNS. Safe to call
  // exactly once from setup().
  void begin(const DashboardCallbacks& cb);

  // Service HTTP + DNS requests. Call from loop() on every tick.
  void handle();

  // Push the latest sensor + connectivity snapshot. Cheap; copy by value.
  void updateStatus(const DashboardStatus& snapshot);

  // AP IP (typically 192.168.4.1). Useful for log messages.
  String apIp();
  String apSsid();
}
