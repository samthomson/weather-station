#include <Arduino.h>
#include <math.h>

// Factory defaults FIRST (defines the compile-time sensor feature flags)
#include "factory_defaults.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include <time.h>
#include <ArduinoJson.h>
#if ENABLE_DHT
  #include "DHT.h"
#endif
#include <Wire.h>
#if ENABLE_BME280
  #include <Adafruit_BME280.h>
  #include <Adafruit_BMP280.h>
#endif
#if ENABLE_BH1750
  #include <BH1750.h>
#endif
#if ENABLE_SPS30
  #include <SensirionI2cSps30.h>
#endif
#if ENABLE_SDS011
  #include <SdsDustSensor.h>
#endif
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WebSocketsClient.h>

#include "config_store.h"
#include "web_dashboard.h"

#include <mbedtls/sha256.h>
#include "uECC.h"

// secp256k1 curve order n
static const uint8_t SECP256K1_ORDER[32] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE,
  0xBA, 0xAE, 0xDC, 0xE6, 0xAF, 0x48, 0xA0, 0x3B,
  0xBF, 0xD2, 0x5E, 0x8C, 0xD0, 0x36, 0x41, 0x41
};

// Pre-computed tag hashes for BIP-340
static const uint8_t TAG_CHALLENGE[32] = {
  0x7b, 0xb5, 0x2d, 0x7a, 0x9f, 0xef, 0x58, 0x32,
  0x3e, 0xb1, 0xbf, 0x7a, 0x40, 0x7d, 0xb3, 0x82,
  0xd2, 0xf3, 0xf2, 0xd8, 0x1b, 0xb1, 0x22, 0x4f,
  0x49, 0xfe, 0x51, 0x8f, 0x6d, 0x48, 0xd3, 0x7c
};
static const uint8_t TAG_NONCE[32] = {
  0x07, 0x49, 0x77, 0x34, 0xa7, 0x9b, 0xcb, 0x35,
  0x5b, 0x9b, 0x8c, 0x7d, 0x03, 0x4f, 0x12, 0x1c,
  0xf4, 0x34, 0xd7, 0x3a, 0xf6, 0x39, 0x7b, 0x96,
  0xb8, 0x0a, 0xab, 0x80, 0x51, 0x85, 0x45, 0x69
};
static const uint8_t TAG_AUX[32] = {
  0xf1, 0xef, 0x4e, 0x5e, 0xc0, 0x63, 0xca, 0xda,
  0xf7, 0xef, 0xf4, 0xb9, 0x8e, 0x3b, 0x67, 0xc8,
  0xa7, 0x90, 0xae, 0xfe, 0x0f, 0x05, 0xfc, 0x7a,
  0x5f, 0x21, 0xa9, 0x93, 0x01, 0x6c, 0x3a, 0xba
};

// All user-settable values come from config_store::current(); the compile-time
// #defines in SECRETS_FILE are only consulted on first boot to seed NVS.
// Default values for things that aren't user-settable from the dashboard yet:
const uint16_t NOSTR_DEFAULT_TLS_PORT = 443;
const uint16_t NOSTR_DEFAULT_WS_PORT  = 80;

// Parsed relay URL kept in sync with config_store::current().nostr_relay.
struct RelayURL { String host; uint16_t port; String path; bool ssl; };
static RelayURL currentRelay;

// Nostr tag names (standardized)
const char* TAG_TEMP = "temp";
const char* TAG_HUMIDITY = "humidity";
const char* TAG_PM1 = "pm1";
const char* TAG_PM25 = "pm25";
const char* TAG_PM10 = "pm10";
const char* TAG_AIR_QUALITY = "air_quality";
const char* TAG_HASHTAG = "t";
const char* HASHTAG_WEATHER = "weather";

// Sensor models
const char* MODEL_DHT11 = "DHT11";
const char* MODEL_BME280 = "BME280";
const char* MODEL_BMP280 = "BMP280";
const char* MODEL_PMS5003 = "PMS5003";
const char* MODEL_PMS7003 = "PMS7003";
const char* MODEL_SPS30 = "SPS30";
const char* MODEL_SDS011 = "SDS011";
const char* MODEL_MQ135 = "MQ-135";
const char* MODEL_BH1750 = "BH1750";
const char* MODEL_MHRD = "MH-RD";

// Nostr tag names for additional sensors
const char* TAG_PRESSURE = "pressure";
const char* TAG_LIGHT = "light";
const char* TAG_RAIN = "rain";

// Sensor types available on this station (for metadata)
struct SensorInfo {
  const char* type;
  const char* model;
};

// Build sensor list modularly - each enabled sensor adds its readings
// Validate that required models are defined if sensors are enabled
#if ENABLE_PMS && !defined(PMS_MODEL)
  #error "PMS_MODEL must be defined in secrets file when ENABLE_PMS is true"
#endif

const SensorInfo sensors[] = {
  #if ENABLE_DHT
    {TAG_TEMP, MODEL_DHT11},
    {TAG_HUMIDITY, MODEL_DHT11},
  #endif
  #if ENABLE_BME280
    {TAG_TEMP, MODEL_BME280},
    {TAG_HUMIDITY, MODEL_BME280},  // Only for BME280, not BMP280
    {TAG_PRESSURE, MODEL_BME280},
  #endif
  #if ENABLE_BMP280
    {TAG_TEMP, MODEL_BMP280},
    {TAG_PRESSURE, MODEL_BMP280},
  #endif
  #if ENABLE_PMS
    {TAG_PM1, PMS_MODEL},
    {TAG_PM25, PMS_MODEL},
    {TAG_PM10, PMS_MODEL},
  #endif
  #if ENABLE_SPS30
    {TAG_PM1, MODEL_SPS30},
    {TAG_PM25, MODEL_SPS30},
    {TAG_PM10, MODEL_SPS30},
  #endif
  #if ENABLE_SDS011
    {TAG_PM25, MODEL_SDS011},
    {TAG_PM10, MODEL_SDS011},
  #endif
  #if ENABLE_MQ
    {TAG_AIR_QUALITY, MODEL_MQ135},
  #endif
};

const int sensorCount = sizeof(sensors) / sizeof(sensors[0]);

// PM sensor serial port (PMS5003/PMS7003 and SDS011) - ESP32 UART2
HardwareSerial pmsSerial(2);
#define PMS_SERIAL pmsSerial

unsigned int pm1 = 0, pm2_5 = 0, pm10 = 0;
bool pmDataReceived = false;  // set on first valid PM frame; omit PM reading tags until then (NIP)

#if ENABLE_SPS30
  SensirionI2cSps30 sps30;
  bool sps30DataReceived = false;
#endif
#if ENABLE_SDS011
  SdsDustSensor sds011(PMS_SERIAL);  // Uses same UART2 on ESP32 (RX=16, TX=17)
  bool sds011DataReceived = false;
#endif

// DHT sensor pin - board-specific
#if ENABLE_DHT
  #define DHTTYPE DHT11
  #define DHT_PIN 4  // GPIO4
  DHT dht(DHT_PIN, DHTTYPE);
#endif

// BME280/BMP280 sensor (I2C)
#if ENABLE_BME280
  Adafruit_BME280 bme;
  Adafruit_BMP280 bmp;
  bool bmeAvailable = false;
  bool bmpAvailable = false;  // BMP280 = no humidity, just temp+pressure
  bool bmeHasReading = false;
  bool bmpHasReading = false;
#endif

// BH1750 light sensor (I2C)
#if ENABLE_BH1750
  BH1750 lightMeter;
  bool bh1750Available = false;
#endif

// Rain sensor pin
#define RAIN_PIN 34  // GPIO34 (ADC1_CH6)

float t = 0, h = 0, p = 0;  // temperature, humidity, pressure
float lux = NAN;            // BH1750 lux; NAN until first valid read
unsigned int rainValue = 0; // rain sensor (0-4095)

// MQ sensor analog pin
#define MQ_PIN 36  // GPIO36 (ADC1_CH0)
unsigned int airQuality = 0;
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
WebSocketsClient webSocket;
bool wsConnected = false;
bool displayAvailable = false;
String nostrPubkey = "";
uint8_t privKeyBytes[32];
uint8_t pubKeyBytes[32];
// millis() of last successful publish; surfaced on the dashboard.
unsigned long lastPublishMs = 0;

// Forward declarations
void connectWiFi();
void setupNostrRelay();
static RelayURL parseRelayUrl(const String& url);
static void applyConfigChanges(bool wifiChanged, bool relayChanged, bool keyChanged, bool sensorsChanged, bool intervalChanged);
static void publishDashboardStatus();
static void initRuntimeSensors();
static WxConfig buildFactoryDefaults();
// 6-char uppercase hex suffix of the eFuse MAC. Stable per physical board,
// burned at factory, identical to the AP SSID suffix.
static String getDeviceId();
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);
void pmReadData();
#if ENABLE_SPS30
void sps30ReadData();
#endif
#if ENABLE_SDS011
void sds011ReadData();
#endif
void dhtData();
#if ENABLE_BME280
void bmeData();
#endif
#if ENABLE_BH1750
void lightData();
#endif
#if ENABLE_RAIN
void rainData();
// GPIO34 has no internal pull, so a "not connected" pin reads random noise
// across the full ADC range. We treat the sensor as present only after we've
// seen several consecutive stable, high (dry) readings -- a signature a
// floating pin never holds. Once latched, every subsequent reading is treated
// as real weather data regardless of value.
static unsigned rainSpread = 999;
static unsigned rainSampleCount = 0;
static unsigned rainDryStreak = 0;
static bool rainEverConnected = false;

static bool rainValid() { return rainEverConnected; }

static void rainUpdateConnectedLatch(unsigned avg, unsigned spread) {
  if (rainEverConnected) return;
  if (spread <= 80 && avg >= 1800) rainDryStreak++;
  else rainDryStreak = 0;
  if (rainDryStreak >= 5) rainEverConnected = true;
}
#endif
void mqReadData();
void displayOled();
void derivePubkey();
String bytesToHex(uint8_t* bytes, int len);
void hexToBytes(const char* hex, uint8_t* bytes, int len);
void sha256Raw(const uint8_t* data, size_t len, uint8_t* out);
void taggedHash(const uint8_t* tag, const uint8_t* data, size_t len, uint8_t* out);
bool schnorrSign(const uint8_t* privkey, const uint8_t* msg, uint8_t* sig);
String createAndSignNostrEvent(float temp, float humidity, unsigned int pm1_val, unsigned int pm25_val, unsigned int pm10_val, unsigned int aq_val);
String createMetadataEvent();
void sendMetadataEvent();

// --- Tag builders (Nostr reading + metadata tags) ---
static void appendReadingTag(String& dest, const char* type, const char* model, float value) {
  dest += ",[\""; dest += type; dest += "\",\""; dest += String(value, 1); dest += "\",\""; dest += model; dest += "\"]";
}
static void appendReadingTag(String& dest, const char* type, const char* model, unsigned int value) {
  dest += ",[\""; dest += type; dest += "\",\""; dest += String(value); dest += "\",\""; dest += model; dest += "\"]";
}
static void appendSensorTag(String& dest, const char* type, const char* model) {
  dest += ",[\"sensor\",\""; dest += type; dest += "\",\""; dest += model; dest += "\"]";
}
static void appendSensorStatusTag(String& dest, const char* type, const char* model, bool ok) {
  dest += ",[\"sensor_status\",\""; dest += type; dest += "\",\""; dest += model; dest += "\",\""; dest += (ok ? "ok" : "418"); dest += "\"]";
}
// One (type, model) → always add sensor tag + sensor_status(ok). Use for DHT, BH1750, Rain, PMS, MQ.
static void appendSensorAndStatus(String& dest, const char* type, const char* model, bool ok) {
  appendSensorTag(dest, type, model);
  appendSensorStatusTag(dest, type, model, ok);
}

// --- Per-sensor validation (NIP: omit reading / status 418 when invalid) ---
// Each sensor type has one validator. Rain/MQ have no fault detection, so we always pass true for them.
#if ENABLE_DHT
static bool dhtValid() { return !isnan(t) && !isnan(h); }
#endif
#if ENABLE_BME280
static bool enviroReadingOkForNostr() { return (bmeAvailable && bmeHasReading) || (bmpAvailable && bmpHasReading); }

// BME280 sensor range (datasheet); values outside are almost always corrupt I2C frames.
static bool bmeEnviroPlausible(float tempC, float humPct, float pressHpa) {
  if (isnan((double)tempC) || isnan((double)humPct) || isnan((double)pressHpa)) return false;
  if (tempC < -40.0f || tempC > 85.0f) return false;
  if (humPct < 0.0f || humPct > 100.0f) return false;
  if (pressHpa < 260.0f || pressHpa > 1100.0f) return false;
  return true;
}
#endif
#if ENABLE_PMS
static bool pmsValid() { return pmDataReceived; }  // set on first valid frame; omit PM tags until then (NIP)
#endif
#if ENABLE_SPS30
static bool sps30Valid() { return sps30DataReceived; }
#endif
#if ENABLE_SDS011
static bool sds011Valid() { return sds011DataReceived; }
#endif
#if ENABLE_BH1750
// claws/BH1750 returns negative values on I2C read failure — do not relay as lux.
static bool bh1750Valid() {
  return bh1750Available && !isnan((double)lux) && lux >= 0.0f;
}
#endif

// BME280 (temp+humidity+pressure) vs BMP280 (temp+pressure only) — one helper, two model names
#if ENABLE_BME280
static void appendPressureSensorReadingTags(String& dest, const char* model, bool includeHumidity) {
  appendReadingTag(dest, TAG_TEMP, model, t);
  if (includeHumidity) appendReadingTag(dest, TAG_HUMIDITY, model, h);
  appendReadingTag(dest, TAG_PRESSURE, model, p);
}
static void appendPressureSensorMetadataTags(String& dest, const char* model, bool includeHumidity, bool ok) {
  if (ok) {
    appendSensorTag(dest, TAG_TEMP, model);
    if (includeHumidity) appendSensorTag(dest, TAG_HUMIDITY, model);
    appendSensorTag(dest, TAG_PRESSURE, model);
  }
  appendSensorStatusTag(dest, TAG_TEMP, model, ok);
  if (includeHumidity) appendSensorStatusTag(dest, TAG_HUMIDITY, model, ok);
  appendSensorStatusTag(dest, TAG_PRESSURE, model, ok);
}
#endif
#if ENABLE_DHT
static void appendDhtReadingTags(String& dest) {
  if (!dhtValid()) return;
  appendReadingTag(dest, TAG_TEMP, MODEL_DHT11, t);
  appendReadingTag(dest, TAG_HUMIDITY, MODEL_DHT11, h);
}
#endif
// Shared helper: append pm1, pm25, pm10 with given model (used by PMS and SPS30)
static void appendPmReadingTagsWithModel(String& dest, const char* model, unsigned int v1, unsigned int v25, unsigned int v10) {
  appendReadingTag(dest, TAG_PM1, model, v1);
  appendReadingTag(dest, TAG_PM25, model, v25);
  appendReadingTag(dest, TAG_PM10, model, v10);
}
#if ENABLE_PMS
static void appendPmReadingTags(String& dest) {
  if (!pmsValid()) return;
  appendPmReadingTagsWithModel(dest, PMS_MODEL, pm1, pm2_5, pm10);
}
#endif
#if ENABLE_SPS30
static void appendSps30ReadingTags(String& dest) {
  if (!sps30Valid()) return;
  appendPmReadingTagsWithModel(dest, MODEL_SPS30, pm1, pm2_5, pm10);
}
#endif
#if ENABLE_SDS011
static void appendSds011ReadingTags(String& dest) {
  if (!sds011Valid()) return;
  appendReadingTag(dest, TAG_PM25, MODEL_SDS011, pm2_5);
  appendReadingTag(dest, TAG_PM10, MODEL_SDS011, pm10);
}
#endif

// 256-bit big number operations (big-endian)
int bn_compare(const uint8_t* a, const uint8_t* b) {
  for (int i = 0; i < 32; i++) {
    if (a[i] > b[i]) return 1;
    if (a[i] < b[i]) return -1;
  }
  return 0;
}

void bn_sub(const uint8_t* a, const uint8_t* b, uint8_t* result) {
  int16_t borrow = 0;
  for (int i = 31; i >= 0; i--) {
    int16_t diff = (int16_t)a[i] - (int16_t)b[i] - borrow;
    if (diff < 0) {
      diff += 256;
      borrow = 1;
    } else {
      borrow = 0;
    }
    result[i] = (uint8_t)diff;
  }
}

void bn_add(const uint8_t* a, const uint8_t* b, uint8_t* result) {
  uint16_t carry = 0;
  for (int i = 31; i >= 0; i--) {
    uint16_t sum = (uint16_t)a[i] + (uint16_t)b[i] + carry;
    result[i] = (uint8_t)(sum & 0xFF);
    carry = sum >> 8;
  }
}

void bn_mod(uint8_t* a, const uint8_t* n) {
  while (bn_compare(a, n) >= 0) {
    bn_sub(a, n, a);
  }
}

// Modular addition: result = (a + b) mod n
void bn_add_mod(const uint8_t* a, const uint8_t* b, const uint8_t* n, uint8_t* result) {
  uint8_t temp[33] = {0}; // Extra byte for overflow
  uint16_t carry = 0;
  for (int i = 31; i >= 0; i--) {
    uint16_t sum = (uint16_t)a[i] + (uint16_t)b[i] + carry;
    temp[i + 1] = (uint8_t)(sum & 0xFF);
    carry = sum >> 8;
  }
  temp[0] = carry;
  
  // Reduce mod n
  uint8_t* ptr = temp + 1;
  if (temp[0] || bn_compare(ptr, n) >= 0) {
    bn_sub(ptr, n, ptr);
  }
  memcpy(result, ptr, 32);
}

// Modular multiplication using double-and-add (slower but correct and memory-efficient)
void bn_mul_mod(const uint8_t* a, const uint8_t* b, const uint8_t* n, uint8_t* result) {
  uint8_t acc[32] = {0};  // Accumulator
  uint8_t temp[32];
  memcpy(temp, a, 32);    // temp = a
  
  // Process b bit by bit, from LSB to MSB
  for (int i = 31; i >= 0; i--) {
    for (int bit = 0; bit < 8; bit++) {
      if ((b[i] >> bit) & 1) {
        // acc = (acc + temp) mod n
        bn_add_mod(acc, temp, n, acc);
      }
      // temp = (temp + temp) mod n = (2 * temp) mod n
      bn_add_mod(temp, temp, n, temp);
    }
  }
  
  memcpy(result, acc, 32);
}

static int RNG(uint8_t *dest, unsigned size) {
  while (size--) *dest++ = (uint8_t)random(256);
  return 1;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  randomSeed(analogRead(0) ^ micros());
  uECC_set_rng(&RNG);

  Serial.println("\n=== Weather Station (Schnorr) ===");

  // Load (or first-boot seed) the user-settable configuration.
  config_store::begin(buildFactoryDefaults());
  WxConfig& cfg = config_store::current();
  currentRelay = parseRelayUrl(cfg.nostr_relay);

  // Bring up the always-on AP + captive portal + HTTP dashboard. STA gets
  // started in connectWiFi() below.
  DashboardCallbacks dcb;
  dcb.onApply = &applyConfigChanges;
  dcb.onRestart = []() { ESP.restart(); };
  dcb.onFactoryReset = []() { config_store::factoryReset(); ESP.restart(); };
  dcb.onRegenerateKey = []() { applyConfigChanges(false, false, true, false, false); };
  web_dashboard::begin(dcb);

  // Initialize PM sensor serial port (if compiled in and user enabled it)
  #if ENABLE_PMS
    if (cfg.en_pms) {
      PMS_SERIAL.begin(9600, SERIAL_8N1, 16, 17);  // UART2, RX=GPIO16, TX=GPIO17
      Serial.println("PMS sensor enabled");
    } else {
      Serial.println("PMS sensor disabled (runtime)");
    }
  #else
    Serial.println("PMS sensor disabled (compile-time)");
  #endif

  // Initialize DHT sensor (if compiled in)
  #if ENABLE_DHT
    dht.begin();
    Serial.println("DHT sensor enabled");
  #else
    Serial.println("DHT sensor disabled");
  #endif
  
  // Initialize I2C bus for any I2C sensors or OLED
  #if ENABLE_BME280 || ENABLE_BH1750 || ENABLE_SPS30 || ENABLE_OLED
    Wire.begin(21, 22);  // SDA=21, SCL=22
    delay(100);  // Give I2C bus time to stabilize
  #endif

  // Initialize SPS30 (I2C) - SEL pin must be tied to GND for I2C mode
  #if ENABLE_SPS30
    sps30.begin(Wire, SPS30_I2C_ADDR_69);
    if (sps30.wakeUpSequence() == 0 &&
        sps30.startMeasurement(SPS30_OUTPUT_FORMAT_OUTPUT_FORMAT_UINT16) == 0) {
      Serial.println("SPS30 enabled (I2C)");
    } else {
      Serial.println("SPS30 not found on I2C - check wiring (SEL to GND)");
    }
  #endif

  // Initialize SDS011 (UART) - same pins as PMS on ESP32
  #if ENABLE_SDS011
    PMS_SERIAL.begin(9600, SERIAL_8N1, 16, 17);  // RX=GPIO16, TX=GPIO17
    sds011.begin();
    sds011.setActiveReportingMode();
    Serial.println("SDS011 enabled (UART)");
  #endif

  // Initialize BME280/BMP280 sensor (compiled in + user-enabled)
  #if ENABLE_BME280
    if (cfg.en_bme280) {
      if (bme.begin(0x76)) {
        bmeAvailable = true;
        Serial.println("BME280 enabled (addr 0x76)");
      } else if (bme.begin(0x77)) {
        bmeAvailable = true;
        Serial.println("BME280 enabled (addr 0x77)");
      } else if (bmp.begin(0x76)) {
        bmpAvailable = true;
        Serial.println("BMP280 enabled (addr 0x76) - no humidity");
      } else if (bmp.begin(0x77)) {
        bmpAvailable = true;
        Serial.println("BMP280 enabled (addr 0x77) - no humidity");
      } else {
        Serial.println("BME280/BMP280 not found - continuing without it");
      }
    } else {
      Serial.println("BME280 sensor disabled (runtime)");
    }
  #else
    Serial.println("BME280 sensor disabled (compile-time)");
  #endif

  // Initialize BH1750 light sensor (compiled in + user-enabled)
  #if ENABLE_BH1750
    if (cfg.en_bh1750) {
      if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
        bh1750Available = true;
        Serial.println("BH1750 light sensor enabled");
      } else {
        Serial.println("BH1750 not found - continuing without it");
      }
    } else {
      Serial.println("BH1750 sensor disabled (runtime)");
    }
  #else
    Serial.println("BH1750 sensor disabled (compile-time)");
  #endif

  // Initialize rain sensor (compiled in + user-enabled)
  #if ENABLE_RAIN
    if (cfg.en_rain) {
      pinMode(RAIN_PIN, INPUT);
      Serial.println("Rain sensor enabled on GPIO34");
    } else {
      Serial.println("Rain sensor disabled (runtime)");
    }
  #else
    Serial.println("Rain sensor disabled (compile-time)");
  #endif
  
  // Initialize OLED display (if enabled)
  #if ENABLE_OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      Serial.println(F("OLED failed - continuing without display"));
      displayAvailable = false;
    } else {
      displayAvailable = true;
      display.clearDisplay();
      display.setTextColor(WHITE);
      Serial.println("OLED enabled");
    }
  #else
    displayAvailable = false;
    Serial.println("OLED disabled");
  #endif
  
  Serial.println("About to connect WiFi...");
  Serial.flush();

  // Disable brownout detector (overly sensitive on some ESP32 boards)
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.println("Brownout detector disabled");

  delay(500);
  connectWiFi();

  if (WiFi.status() == WL_CONNECTED) {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    Serial.print("Time sync");
    time_t now = time(nullptr);
    unsigned long syncStart = millis();
    while (now < 1700000000 && millis() - syncStart < 15000) {
      delay(500);
      Serial.print(".");
      now = time(nullptr);
    }
    Serial.println(now < 1700000000 ? " timeout" : " OK");
  } else {
    Serial.println("STA not connected; skipping NTP / Nostr connect for now.");
  }

  derivePubkey();
  if (WiFi.status() == WL_CONNECTED) {
    setupNostrRelay();
    delay(2000);
    sendMetadataEvent();
  }

  Serial.println("Ready! Connect to AP '" + web_dashboard::apSsid() + "' (open) and visit http://" + web_dashboard::apIp() + "/");
}

void loop() {
  // Always service the dashboard + DNS captive portal first so the UI stays
  // responsive even when STA / WebSocket are flapping.
  web_dashboard::handle();
  webSocket.loop();

  WxConfig& cfg = config_store::current();

  // Best-effort background STA reconnect if user has WiFi creds but we are
  // currently disconnected (e.g. credentials were just changed).
  static unsigned long lastStaRetry = 0;
  if (cfg.wifi_ssid.length() > 0 && WiFi.status() != WL_CONNECTED && millis() - lastStaRetry > 15000) {
    Serial.println("[wifi] reconnect attempt");
    WiFi.begin(cfg.wifi_ssid.c_str(), cfg.wifi_pass.c_str());
    lastStaRetry = millis();
  }

  static unsigned long lastSensor = 0, lastPost = 0, lastStatus = 0, lastWifiLog = 0;
  unsigned long now = millis();

  if (now - lastWifiLog > 10000) {
    lastWifiLog = now;
    Serial.printf("[wifi] AP SSID: %s  AP IP: %s  STA: %s\n",
      web_dashboard::apSsid().c_str(),
      web_dashboard::apIp().c_str(),
      WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "disconnected");
  }

  if (now - lastSensor > 2000) {
    #if ENABLE_PMS
      if (cfg.en_pms) pmReadData();
    #endif
    #if ENABLE_SPS30
      sps30ReadData();
    #endif
    #if ENABLE_SDS011
      sds011ReadData();
    #endif
    #if ENABLE_DHT
      dhtData();
    #endif
    #if ENABLE_BME280
      if (cfg.en_bme280) bmeData();
    #endif
    #if ENABLE_BH1750
      if (cfg.en_bh1750) lightData();
    #endif
    #if ENABLE_RAIN
      if (cfg.en_rain) rainData();
    #endif
    #if ENABLE_MQ
      mqReadData();
    #endif
    #if ENABLE_OLED
      displayOled();
    #endif
    lastSensor = now;
  }

  if (now - lastStatus > 1000) {
    publishDashboardStatus();
    lastStatus = now;
  }

  uint32_t postIntervalMs = cfg.post_interval_ms > 10000 ? cfg.post_interval_ms : 60000;
  if (now - lastPost > postIntervalMs) {
    // Do we have any valid reading to publish? Each validator already returns
    // false when the sensor is runtime-disabled (because nothing wrote to the
    // backing globals), so we don't re-check cfg.en_* here.
    bool hasData = false;
    #if ENABLE_DHT
      hasData = hasData || (t > 0);
    #endif
    #if ENABLE_BME280
      hasData = hasData || enviroReadingOkForNostr();
    #endif
    #if ENABLE_BH1750
      hasData = hasData || bh1750Valid();
    #endif
    #if ENABLE_RAIN
      hasData = hasData || rainValid();
    #endif
    #if ENABLE_PMS
      hasData = hasData || (pm2_5 > 0);
    #endif
    #if ENABLE_SPS30
      hasData = hasData || sps30DataReceived;
    #endif
    #if ENABLE_SDS011
      hasData = hasData || sds011DataReceived;
    #endif

    if (wsConnected && (hasData || !ENABLE_DHT)) {
      String event = createAndSignNostrEvent(t, h, pm1, pm2_5, pm10, airQuality);
      if (event.length() > 0) {
        Serial.println("Sending reading event...");
        String msg = "[\"EVENT\"," + event + "]";
        webSocket.sendTXT(msg);
        lastPublishMs = millis();
      }
      sendMetadataEvent();
    }
    lastPost = now;
  }
  delay(2);
}

void connectWiFi() {
  WxConfig& cfg = config_store::current();
  if (cfg.wifi_ssid.length() == 0) {
    Serial.println("[wifi] No SSID set; staying in AP-only mode for now");
    return;
  }
  // AP mode is already running (see web_dashboard::begin); WIFI_AP_STA was set
  // there. Just start the STA connection.
  WiFi.begin(cfg.wifi_ssid.c_str(), cfg.wifi_pass.c_str());
  Serial.print("WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(" OK ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(" timeout (continuing; STA will retry in background)");
  }
}

static RelayURL parseRelayUrl(const String& url) {
  RelayURL r{ "", NOSTR_DEFAULT_TLS_PORT, "/", true };
  if (url.length() == 0) return r;
  String u = url;
  if (u.startsWith("wss://"))      { r.ssl = true;  r.port = NOSTR_DEFAULT_TLS_PORT; u = u.substring(6); }
  else if (u.startsWith("ws://"))  { r.ssl = false; r.port = NOSTR_DEFAULT_WS_PORT;  u = u.substring(5); }
  // bare "host[:port][/path]" defaults to wss://
  int slash = u.indexOf('/');
  String hostport = slash >= 0 ? u.substring(0, slash) : u;
  r.path = slash >= 0 ? u.substring(slash) : "/";
  int colon = hostport.indexOf(':');
  if (colon >= 0) {
    r.host = hostport.substring(0, colon);
    r.port = (uint16_t)hostport.substring(colon + 1).toInt();
  } else {
    r.host = hostport;
  }
  return r;
}

void setupNostrRelay() {
  WxConfig& cfg = config_store::current();
  currentRelay = parseRelayUrl(cfg.nostr_relay);
  if (currentRelay.host.length() == 0) {
    Serial.println("[nostr] No relay configured; skipping connect");
    return;
  }
  Serial.print("[nostr] Connecting to ");
  Serial.print(currentRelay.ssl ? "wss://" : "ws://");
  Serial.print(currentRelay.host);
  Serial.print(":"); Serial.print(currentRelay.port);
  Serial.println(currentRelay.path);
  if (currentRelay.ssl) webSocket.beginSSL(currentRelay.host.c_str(), currentRelay.port, currentRelay.path.c_str());
  else                  webSocket.begin(currentRelay.host.c_str(), currentRelay.port, currentRelay.path.c_str());
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  webSocket.enableHeartbeat(15000, 3000, 2);
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED: wsConnected = false; Serial.println("[WS] Disconnected"); break;
    case WStype_CONNECTED: wsConnected = true; Serial.println("[WS] Connected"); break;
    case WStype_TEXT: Serial.print("[WS] "); Serial.println((char*)payload); break;
    default: break;
  }
}

void pmReadData() {
  int index = 0;
  char value, prev = 0;
  while (PMS_SERIAL.available()) {
    value = PMS_SERIAL.read();
    if ((index == 0 && value != 0x42) || (index == 1 && value != 0x4d)) break;
    if (index == 4 || index == 6 || index == 8) prev = value;
    else if (index == 5) pm1 = 256 * prev + value;
    else if (index == 7) pm2_5 = 256 * prev + value;
    else if (index == 9) {
      pm10 = 256 * prev + value;
      pmDataReceived = true;  // valid frame received
    }
    else if (index > 15) break;
    index++;
  }
  while(PMS_SERIAL.available()) PMS_SERIAL.read();
}

#if ENABLE_SPS30
void sps30ReadData() {
  uint16_t mc1p0, mc2p5, mc4p0, mc10p0;
  uint16_t nc0p5, nc1p0, nc2p5, nc4p0, nc10p0;
  uint16_t typical;
  int16_t err = sps30.readMeasurementValuesUint16(mc1p0, mc2p5, mc4p0, mc10p0, nc0p5, nc1p0, nc2p5, nc4p0, nc10p0, typical);
  if (err == 0) {
    pm1 = (unsigned int)mc1p0;
    pm2_5 = (unsigned int)mc2p5;
    pm10 = (unsigned int)mc10p0;
    sps30DataReceived = true;
    Serial.print("SPS30 PM1:"); Serial.print(pm1); Serial.print(" PM2.5:"); Serial.print(pm2_5); Serial.print(" PM10:"); Serial.println(pm10);
  }
}
#endif

#if ENABLE_SDS011
void sds011ReadData() {
  PmResult res = sds011.readPm();
  if (res.isOk()) {
    pm1 = 0;  // SDS011 does not report PM1
    pm2_5 = (unsigned int)(res.pm25 + 0.5f);
    pm10 = (unsigned int)(res.pm10 + 0.5f);
    sds011DataReceived = true;
    Serial.print("SDS011 PM2.5:"); Serial.print(pm2_5); Serial.print(" PM10:"); Serial.println(pm10);
  }
}
#endif

#if ENABLE_DHT
void dhtData() {
  h = dht.readHumidity();
  t = dht.readTemperature();
  if (!isnan(t)) {
    Serial.print("T:"); Serial.print(t,1); Serial.print(" H:"); Serial.println(h,1);
  }
}
#endif

#if ENABLE_BME280
void bmeData() {
  if (bmeAvailable) {
    float nt = bme.readTemperature();
    float nh = bme.readHumidity();
    float np = bme.readPressure() / 100.0F;
    if (bmeEnviroPlausible(nt, nh, np)) {
      t = nt;
      h = nh;
      p = np;
      bmeHasReading = true;
      Serial.print("BME280 - T:");
      Serial.print(t, 1);
      Serial.print(" H:");
      Serial.print(h, 1);
      Serial.print(" P:");
      Serial.print(p, 1);
      Serial.println("hPa");
    }
  } else if (bmpAvailable) {
    float nt = bmp.readTemperature();
    float np = bmp.readPressure() / 100.0F;
    if (!isnan((double)nt) && !isnan((double)np) && nt >= -40.0f && nt <= 85.0f && np > 260.0f && np < 1100.0f) {
      t = nt;
      h = 0;
      p = np;
      bmpHasReading = true;
      Serial.print("BMP280 - T:");
      Serial.print(t, 1);
      Serial.print(" P:");
      Serial.print(p, 1);
      Serial.println("hPa");
    }
  }
}
#endif

#if ENABLE_BH1750
void lightData() {
  if (!bh1750Available) return;

  float v = lightMeter.readLightLevel();
  // Negative = driver I2C error; do not update lux with invalid reads.
  if (bh1750Available && !isnan((double)v) && v >= 0.0f) {
    lux = v;
    Serial.print("Light: ");
    Serial.print(lux);
    Serial.println(" lux");
  }
}
#endif

#if ENABLE_RAIN
void rainData() {
  unsigned vals[5];
  for (int i = 0; i < 5; i++) {
    vals[i] = analogRead(RAIN_PIN);
    delayMicroseconds(800);
  }
  unsigned vmin = vals[0], vmax = vals[0], sum = 0;
  for (int i = 0; i < 5; i++) {
    if (vals[i] < vmin) vmin = vals[i];
    if (vals[i] > vmax) vmax = vals[i];
    sum += vals[i];
  }
  rainValue = sum / 5;
  rainSpread = vmax - vmin;
  rainSampleCount++;
  rainUpdateConnectedLatch(rainValue, rainSpread);
  // Note: Higher value = drier, Lower value = more water
  Serial.print("Rain sensor: avg=");
  Serial.print(rainValue);
  Serial.print(" spread=");
  Serial.print(rainSpread);
  Serial.print(" connected=");
  Serial.println(rainEverConnected ? "yes" : "no");
}
#endif

void mqReadData() {
  airQuality = analogRead(MQ_PIN);
  Serial.print("Air Quality: "); Serial.println(airQuality);
}

void displayOled() {
  if (!displayAvailable) return;
  
  display.clearDisplay();
  display.setCursor(0, 0); display.setTextSize(1); display.print("PM2.5: "); display.println(pm2_5);
  display.setCursor(0, 20); display.print("Temp: "); display.print(t,1); display.println("C");
  display.setCursor(0, 40); display.print("Hum: "); display.print(h,1); display.println("%");
  display.display();
}

String bytesToHex(uint8_t* bytes, int len) {
  String hex = "";
  for (int i = 0; i < len; i++) {
    if (bytes[i] < 16) hex += "0";
    hex += String(bytes[i], HEX);
  }
  return hex;
}

void hexToBytes(const char* hex, uint8_t* bytes, int len) {
  for (int i = 0; i < len; i++) {
    char buf[3] = {hex[i*2], hex[i*2+1], 0};
    bytes[i] = strtol(buf, NULL, 16);
  }
}

void sha256Raw(const uint8_t* data, size_t len, uint8_t* out) {
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);  // 0 = SHA256 (not SHA224)
  mbedtls_sha256_update(&ctx, data, len);
  mbedtls_sha256_finish(&ctx, out);
  mbedtls_sha256_free(&ctx);
}

void taggedHash(const uint8_t* tag, const uint8_t* data, size_t len, uint8_t* out) {
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  mbedtls_sha256_update(&ctx, tag, 32);
  mbedtls_sha256_update(&ctx, tag, 32);
  mbedtls_sha256_update(&ctx, data, len);
  mbedtls_sha256_finish(&ctx, out);
  mbedtls_sha256_free(&ctx);
}

void derivePubkey() {
  hexToBytes(config_store::current().nostr_privkey.c_str(), privKeyBytes, 32);

  uint8_t pubFull[64];
  uECC_Curve curve = uECC_secp256k1();
  uECC_compute_public_key(privKeyBytes, pubFull, curve);

  // x-coordinate only (first 32 bytes)
  memcpy(pubKeyBytes, pubFull, 32);
  nostrPubkey = bytesToHex(pubKeyBytes, 32);
  Serial.print("Pubkey: ");
  Serial.println(nostrPubkey);
}

// BIP-340 Schnorr signature
bool schnorrSign(const uint8_t* privkey, const uint8_t* msg32, uint8_t* sig64) {
  uECC_Curve curve = uECC_secp256k1();
  
  // 1. Compute public key P = d*G
  uint8_t pubFull[64];
  if (!uECC_compute_public_key(privkey, pubFull, curve)) return false;
  uint8_t px[32];
  memcpy(px, pubFull, 32);
  
  // 2. Check if P.y is even (BIP-340 requires even y, negate privkey if odd)
  // y-coordinate is in pubFull[32..63], check last bit
  uint8_t d[32];
  memcpy(d, privkey, 32);
  if (pubFull[63] & 1) {
    // Negate: d = n - d
    bn_sub(SECP256K1_ORDER, d, d);
  }
  
  // 3. Generate deterministic nonce k
  // aux = random 32 bytes
  uint8_t aux[32];
  for (int i = 0; i < 32; i++) aux[i] = random(256);
  
  // t = d XOR tagged_hash("BIP0340/aux", aux)
  uint8_t t[32], auxHash[32];
  taggedHash(TAG_AUX, aux, 32, auxHash);
  for (int i = 0; i < 32; i++) t[i] = d[i] ^ auxHash[i];
  
  // k' = tagged_hash("BIP0340/nonce", t || px || msg)
  uint8_t nonceData[96];
  memcpy(nonceData, t, 32);
  memcpy(nonceData + 32, px, 32);
  memcpy(nonceData + 64, msg32, 32);
  uint8_t kPrime[32];
  taggedHash(TAG_NONCE, nonceData, 96, kPrime);
  
  // Ensure k is non-zero and < n
  bn_mod(kPrime, SECP256K1_ORDER);
  bool isZero = true;
  for (int i = 0; i < 32; i++) if (kPrime[i]) isZero = false;
  if (isZero) return false;
  
  // 4. R = k' * G
  uint8_t R[64];
  if (!uECC_compute_public_key(kPrime, R, curve)) return false;
  uint8_t rx[32];
  memcpy(rx, R, 32);
  
  // 5. Check if R.y is even, negate k if odd
  uint8_t k[32];
  memcpy(k, kPrime, 32);
  if (R[63] & 1) {
    bn_sub(SECP256K1_ORDER, k, k);
  }
  
  // 6. e = tagged_hash("BIP0340/challenge", rx || px || msg) mod n
  uint8_t challengeData[96];
  memcpy(challengeData, rx, 32);
  memcpy(challengeData + 32, px, 32);
  memcpy(challengeData + 64, msg32, 32);
  uint8_t e[32];
  taggedHash(TAG_CHALLENGE, challengeData, 96, e);
  bn_mod(e, SECP256K1_ORDER);
  
  // 7. s = (k + e * d) mod n
  uint8_t ed[32];
  bn_mul_mod(e, d, SECP256K1_ORDER, ed);
  uint8_t s[32];
  bn_add_mod(k, ed, SECP256K1_ORDER, s);
  
  // 8. Signature = rx || s
  memcpy(sig64, rx, 32);
  memcpy(sig64 + 32, s, 32);
  
  return true;
}

String createAndSignNostrEvent(float temp, float humidity, unsigned int pm1_val, unsigned int pm25_val, unsigned int pm10_val, unsigned int aq_val) {
  unsigned long createdAt = (unsigned long)time(nullptr);
  
  // Build tags with station reference and sensor data (only for enabled sensors)
  String readingTags = "[";
  readingTags += "[\"" + String(TAG_HASHTAG) + "\",\"" + String(HASHTAG_WEATHER) + "\"],";
  readingTags += "[\"a\",\"16158:" + nostrPubkey + ":\"]";
  
  // Add sensor readings only when valid (NIP: omit tag for missing/broken sensor)
  WxConfig& cfg = config_store::current();
  #if ENABLE_DHT
    appendDhtReadingTags(readingTags);
  #endif
  #if ENABLE_BME280
    if (cfg.en_bme280 && enviroReadingOkForNostr()) {
      if (bmeAvailable) appendPressureSensorReadingTags(readingTags, MODEL_BME280, true);
      else /* bmp */    appendPressureSensorReadingTags(readingTags, MODEL_BMP280, false);
    }
  #endif
  #if ENABLE_BH1750
    if (cfg.en_bh1750 && bh1750Valid()) appendReadingTag(readingTags, TAG_LIGHT, MODEL_BH1750, lux);
  #endif
  #if ENABLE_RAIN
    if (cfg.en_rain && rainValid()) appendReadingTag(readingTags, TAG_RAIN, MODEL_MHRD, (unsigned int)rainValue);
  #endif
  #if ENABLE_PMS
    if (cfg.en_pms && pmsValid()) {
      const char* pmsModel = cfg.pms_model.c_str();
      appendReadingTag(readingTags, TAG_PM1,  pmsModel, pm1);
      appendReadingTag(readingTags, TAG_PM25, pmsModel, pm2_5);
      appendReadingTag(readingTags, TAG_PM10, pmsModel, pm10);
    }
  #endif
  #if ENABLE_SPS30
    appendSps30ReadingTags(readingTags);
  #endif
  #if ENABLE_SDS011
    appendSds011ReadingTags(readingTags);
  #endif
  #if ENABLE_MQ
    appendReadingTag(readingTags, TAG_AIR_QUALITY, MODEL_MQ135, aq_val);
  #endif
  
  readingTags += "]";
  
  String canonical = "[0,\"" + nostrPubkey + "\"," + String(createdAt) + ",4223," + readingTags + ",\"\"]";
  
  uint8_t eventIdBytes[32];
  sha256Raw((const uint8_t*)canonical.c_str(), canonical.length(), eventIdBytes);
  String eventId = bytesToHex(eventIdBytes, 32);
  
  uint8_t sig[64];
  if (!schnorrSign(privKeyBytes, eventIdBytes, sig)) {
    Serial.println("Sign failed!");
    return "";
  }
  String signature = bytesToHex(sig, 64);
  
  String event = "{\"id\":\"" + eventId + "\",";
  event += "\"pubkey\":\"" + nostrPubkey + "\",";
  event += "\"created_at\":" + String(createdAt) + ",";
  event += "\"kind\":4223,";
  event += "\"tags\":" + readingTags + ",";
  event += "\"content\":\"\",";
  event += "\"sig\":\"" + signature + "\"}";
  
  return event;
}

String createMetadataEvent() {
  unsigned long createdAt = (unsigned long)time(nullptr);
  WxConfig& cfg = config_store::current();

  // Build tags array for metadata
  String metadataTags = "[";
  metadataTags += "[\"name\",\"" + cfg.station_name + "\"]";

  if (cfg.station_description.length() > 0) {
    metadataTags += ",[\"description\",\"" + cfg.station_description + "\"]";
  }
  if (cfg.station_geohash.length() > 0) {
    metadataTags += ",[\"g\",\"" + cfg.station_geohash + "\"]";
  }
  if (cfg.station_elevation.length() > 0) {
    metadataTags += ",[\"elevation\",\"" + cfg.station_elevation + "\"]";
  }
  metadataTags += ",[\"power\",\"" + cfg.station_power + "\"]";
  metadataTags += ",[\"connectivity\",\"" + cfg.station_connectivity + "\"]";
  // Stable hardware identity. Same value across firmware upgrades, factory
  // resets, and Nostr key rotations -- useful for matching prior events from
  // the same physical board.
  metadataTags += ",[\"device_id\",\"" + getDeviceId() + "\"]";

  // Sensor capability + status (NIP: sensor tag + sensor_status ok/418).
  // Only declared sensors that are runtime-enabled are advertised; toggling a
  // sensor off in the dashboard makes it disappear from the metadata.
  #if ENABLE_DHT
    appendSensorAndStatus(metadataTags, TAG_TEMP, MODEL_DHT11, dhtValid());
    appendSensorAndStatus(metadataTags, TAG_HUMIDITY, MODEL_DHT11, dhtValid());
  #endif
  #if ENABLE_BME280
    if (cfg.en_bme280) {
      if (bmeAvailable) appendPressureSensorMetadataTags(metadataTags, MODEL_BME280, true, bmeHasReading);
      else if (bmpAvailable) appendPressureSensorMetadataTags(metadataTags, MODEL_BMP280, false, bmpHasReading);
      else appendPressureSensorMetadataTags(metadataTags, MODEL_BME280, true, false);
    }
  #endif
  #if ENABLE_BH1750
    if (cfg.en_bh1750) appendSensorAndStatus(metadataTags, TAG_LIGHT, MODEL_BH1750, bh1750Valid());
  #endif
  #if ENABLE_RAIN
    if (cfg.en_rain) appendSensorAndStatus(metadataTags, TAG_RAIN, MODEL_MHRD, rainValid());
  #endif
  #if ENABLE_PMS
    if (cfg.en_pms) {
      const char* pmsModel = cfg.pms_model.c_str();
      appendSensorAndStatus(metadataTags, TAG_PM1,  pmsModel, pmsValid());
      appendSensorAndStatus(metadataTags, TAG_PM25, pmsModel, pmsValid());
      appendSensorAndStatus(metadataTags, TAG_PM10, pmsModel, pmsValid());
    }
  #endif
  #if ENABLE_SPS30
    appendSensorAndStatus(metadataTags, TAG_PM1, MODEL_SPS30, sps30Valid());
    appendSensorAndStatus(metadataTags, TAG_PM25, MODEL_SPS30, sps30Valid());
    appendSensorAndStatus(metadataTags, TAG_PM10, MODEL_SPS30, sps30Valid());
  #endif
  #if ENABLE_SDS011
    appendSensorAndStatus(metadataTags, TAG_PM25, MODEL_SDS011, sds011Valid());
    appendSensorAndStatus(metadataTags, TAG_PM10, MODEL_SDS011, sds011Valid());
  #endif
  #if ENABLE_MQ
    appendSensorAndStatus(metadataTags, TAG_AIR_QUALITY, MODEL_MQ135, true);
  #endif
  
  metadataTags += "]";
  
  String canonical = "[0,\"" + nostrPubkey + "\"," + String(createdAt) + ",16158," + metadataTags + ",\"\"]";
  
  uint8_t eventIdBytes[32];
  sha256Raw((const uint8_t*)canonical.c_str(), canonical.length(), eventIdBytes);
  String eventId = bytesToHex(eventIdBytes, 32);
  
  uint8_t sig[64];
  if (!schnorrSign(privKeyBytes, eventIdBytes, sig)) {
    Serial.println("Metadata sign failed!");
    return "";
  }
  String signature = bytesToHex(sig, 64);
  
  String event = "{\"id\":\"" + eventId + "\",";
  event += "\"pubkey\":\"" + nostrPubkey + "\",";
  event += "\"created_at\":" + String(createdAt) + ",";
  event += "\"kind\":16158,";
  event += "\"tags\":" + metadataTags + ",";
  event += "\"content\":\"\",";
  event += "\"sig\":\"" + signature + "\"}";
  
  return event;
}

void sendMetadataEvent() {
  if (!wsConnected) return;

  String metadataEvent = createMetadataEvent();
  if (metadataEvent.length() > 0) {
    Serial.println("Sending metadata event...");
    String msg = "[\"EVENT\"," + metadataEvent + "]";
    webSocket.sendTXT(msg);
  }
}

// ---------------------------------------------------------------------------
// Dashboard glue: factory defaults, hot-apply on config change, status push.
// ---------------------------------------------------------------------------

static WxConfig buildFactoryDefaults() {
  WxConfig d;
  #ifdef WIFI_SSID
    d.wifi_ssid = WIFI_SSID;
  #endif
  #ifdef WIFI_PASS
    d.wifi_pass = WIFI_PASS;
  #endif
  #ifdef NOSTR_RELAY_HOST
    {
      String r = NOSTR_RELAY_HOST;
      // Compile-time default was a bare host; assume wss:// for new-style URL.
      if (r.length() > 0 && !r.startsWith("ws://") && !r.startsWith("wss://")) r = "wss://" + r;
      d.nostr_relay = r;
    }
  #endif
  #ifdef NOSTR_PRIVKEY
    d.nostr_privkey = NOSTR_PRIVKEY;
  #endif
  #ifdef STATION_NAME
    d.station_name = STATION_NAME;
  #endif
  #ifdef STATION_DESCRIPTION
    d.station_description = STATION_DESCRIPTION;
  #endif
  #ifdef STATION_GEOHASH
    d.station_geohash = STATION_GEOHASH;
  #endif
  #ifdef STATION_ELEVATION
    d.station_elevation = STATION_ELEVATION;
  #endif
  #ifdef STATION_POWER
    d.station_power = STATION_POWER;
  #else
    d.station_power = "mains";
  #endif
  #ifdef STATION_CONNECTIVITY
    d.station_connectivity = STATION_CONNECTIVITY;
  #else
    d.station_connectivity = "wifi";
  #endif
  d.post_interval_ms = 60000;
  #if ENABLE_BME280
    d.en_bme280 = (bool)(ENABLE_BME280);
  #endif
  #if ENABLE_BH1750
    d.en_bh1750 = (bool)(ENABLE_BH1750);
  #endif
  #if ENABLE_RAIN
    d.en_rain = (bool)(ENABLE_RAIN);
  #endif
  #if ENABLE_PMS
    d.en_pms = (bool)(ENABLE_PMS);
  #endif
  #ifdef PMS_MODEL
    d.pms_model = PMS_MODEL;
  #endif
  return d;
}

static void initRuntimeSensors() {
  WxConfig& cfg = config_store::current();
  #if ENABLE_PMS
    static bool pmsStarted = false;
    if (cfg.en_pms && !pmsStarted) {
      PMS_SERIAL.begin(9600, SERIAL_8N1, 16, 17);
      pmsStarted = true;
      Serial.println("[sensors] PMS started");
    }
  #endif
  #if ENABLE_BME280
    if (cfg.en_bme280 && !bmeAvailable && !bmpAvailable) {
      if (bme.begin(0x76) || bme.begin(0x77)) {
        bmeAvailable = true;
        Serial.println("[sensors] BME280 init OK");
      } else if (bmp.begin(0x76) || bmp.begin(0x77)) {
        bmpAvailable = true;
        Serial.println("[sensors] BMP280 init OK");
      } else {
        Serial.println("[sensors] BME280/BMP280 still not found");
      }
    }
  #endif
  #if ENABLE_BH1750
    if (cfg.en_bh1750 && !bh1750Available) {
      if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
        bh1750Available = true;
        Serial.println("[sensors] BH1750 init OK");
      }
    }
  #endif
  #if ENABLE_RAIN
    if (cfg.en_rain) {
      pinMode(RAIN_PIN, INPUT);
    } else {
      rainEverConnected = false;
      rainDryStreak = 0;
      rainSampleCount = 0;
      rainSpread = 999;
      rainValue = 0;
    }
  #endif
}

static void applyConfigChanges(bool wifiChanged, bool relayChanged, bool keyChanged, bool sensorsChanged, bool intervalChanged) {
  WxConfig& cfg = config_store::current();
  if (wifiChanged) {
    Serial.println("[apply] WiFi creds changed; reconnecting STA");
    WiFi.disconnect(false, true);
    if (cfg.wifi_ssid.length() > 0) {
      WiFi.begin(cfg.wifi_ssid.c_str(), cfg.wifi_pass.c_str());
    }
  }
  if (keyChanged) {
    Serial.println("[apply] Nostr key changed; recomputing pubkey");
    derivePubkey();
    // Drop the relay so next post re-publishes metadata with the new pubkey.
    webSocket.disconnect();
    wsConnected = false;
  }
  if (relayChanged) {
    Serial.println("[apply] Nostr relay changed; reconnecting");
    webSocket.disconnect();
    wsConnected = false;
    setupNostrRelay();
  }
  if (sensorsChanged) {
    Serial.println("[apply] Sensor toggles changed; (re)initialising drivers");
    initRuntimeSensors();
  }
  if (intervalChanged) {
    Serial.print("[apply] Post interval is now ");
    Serial.print(cfg.post_interval_ms / 1000);
    Serial.println("s");
  }
}

static String getDeviceId() {
  // Same 3-byte suffix as the AP SSID (WeatherStation-XXXXXX).
  // getEfuseMac() packs mac[0..5] little-endian; mac[3..5] (unique device
  // bytes) are at bit offsets 24, 32, 40. mac[0..2] is the shared OUI.
  uint64_t chipId = ESP.getEfuseMac();
  char buf[7];
  snprintf(buf, sizeof(buf), "%02X%02X%02X",
           (unsigned)((chipId >> 24) & 0xFF),
           (unsigned)((chipId >> 32) & 0xFF),
           (unsigned)((chipId >> 40) & 0xFF));
  return String(buf);
}

static void publishDashboardStatus() {
  WxConfig& cfg = config_store::current();
  DashboardStatus s;
  s.sta_connected = (WiFi.status() == WL_CONNECTED);
  if (s.sta_connected) {
    s.sta_ssid = WiFi.SSID();
    s.sta_ip   = WiFi.localIP().toString();
    s.sta_rssi = WiFi.RSSI();
  }
  s.ws_connected = wsConnected;
  s.pubkey_hex = nostrPubkey;
  s.device_id  = getDeviceId();
  s.last_post_ms = lastPublishMs;

  #if ENABLE_BME280
    if (cfg.en_bme280 && (bmeHasReading || bmpHasReading)) {
      s.has_temp = true; s.temp_c = t;
      s.has_pressure = true; s.pressure_hpa = p;
      if (bmeHasReading) { s.has_humidity = true; s.humidity = h; }
    }
  #endif
  #if ENABLE_BH1750
    if (cfg.en_bh1750 && bh1750Valid()) { s.has_lux = true; s.lux = lux; }
  #endif
  #if ENABLE_RAIN
    if (cfg.en_rain && rainValid()) s.rain_raw = rainValue;
  #endif
  #if ENABLE_PMS
    if (cfg.en_pms && pmDataReceived) {
      s.has_pm = true;
      s.pm1 = pm1; s.pm25 = pm2_5; s.pm10 = pm10;
    }
  #endif

  web_dashboard::updateStatus(s);
}
