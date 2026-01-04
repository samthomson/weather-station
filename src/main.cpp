#include <Arduino.h>

// Include station-specific secrets FIRST (defines sensor flags)
#ifndef SECRETS_FILE
  #error "SECRETS_FILE must be defined! Use -DSECRETS_FILE=\"secrets_stationX.h\" in platformio.ini"
#endif
#include SECRETS_FILE

// Board-specific includes
#ifdef ESP32
  #include <WiFi.h>
  #include <WiFiClientSecure.h>
  #include "soc/soc.h"
  #include "soc/rtc_cntl_reg.h"
#else
  #include <ESP8266WiFi.h>
  #include <WiFiClientSecure.h>
  #include <SoftwareSerial.h>
#endif

#include <time.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include <Wire.h>
#if ENABLE_BME280
  #include <Adafruit_BME280.h>
  #include <Adafruit_BMP280.h>
#endif
#if ENABLE_BH1750
  #include <BH1750.h>
#endif
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WebSocketsClient.h>

#ifdef ESP32
  #include <mbedtls/sha256.h>
  #include "uECC.h"
#else
  #include <bearssl/bearssl_hash.h>
  #include "uECC.h"
#endif

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

char ssid[] = WIFI_SSID;
char pass[] = WIFI_PASS;
const char* nostrRelayHost = NOSTR_RELAY_HOST;
const uint16_t nostrRelayPort = 443;
const char* nostrRelayPath = "/";
const char* nostrPrivateKeyHex = NOSTR_PRIVKEY;
const char* stationName = STATION_NAME;
const char* stationGeohash = STATION_GEOHASH;
const char* stationElevation = STATION_ELEVATION;
const char* stationPower = STATION_POWER;
const char* stationConnectivity = STATION_CONNECTIVITY;
const unsigned long POST_INTERVAL = 30000;

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
  #if ENABLE_MQ
    {TAG_AIR_QUALITY, MODEL_MQ135},
  #endif
};

const int sensorCount = sizeof(sensors) / sizeof(sensors[0]);

// PM sensor serial port - board-specific
#ifdef ESP32
  HardwareSerial pmsSerial(2);  // ESP32 UART2
  #define PMS_SERIAL pmsSerial
#else
  SoftwareSerial mySerial(D5, D6);  // ESP8266
  #define PMS_SERIAL mySerial
#endif

unsigned int pm1 = 0, pm2_5 = 0, pm10 = 0;

// DHT sensor pin - board-specific
#define DHTTYPE DHT11
#ifdef ESP32
  #define DHT_PIN 4  // GPIO4 on ESP32
#else
  #define DHT_PIN D7  // D7 on ESP8266
#endif
DHT dht(DHT_PIN, DHTTYPE);

// BME280/BMP280 sensor (I2C)
#if ENABLE_BME280
  Adafruit_BME280 bme;
  Adafruit_BMP280 bmp;
  bool bmeAvailable = false;
  bool bmpAvailable = false;  // BMP280 = no humidity, just temp+pressure
#endif

// BH1750 light sensor (I2C)
#if ENABLE_BH1750
  BH1750 lightMeter;
  bool bh1750Available = false;
#endif

// Rain sensor pin - board-specific
#ifdef ESP32
  #define RAIN_PIN 34  // GPIO34 (ADC1_CH6) on ESP32
#else
  #define RAIN_PIN A0  // A0 on ESP8266 (shared with MQ if both enabled)
#endif

float t = 0, h = 0, p = 0;  // temperature, humidity, pressure
float lux = 0;              // light level
unsigned int rainValue = 0; // rain sensor (0-4095 on ESP32, 0-1023 on ESP8266)

// MQ sensor analog pin - board-specific
#ifdef ESP32
  #define MQ_PIN 36  // GPIO36 (ADC1_CH0) on ESP32
#else
  #define MQ_PIN A0  // A0 on ESP8266
#endif
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

// Forward declarations
void connectWiFi();
void setupNostrRelay();
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);
void pmReadData();
void dhtData();
#if ENABLE_BME280
void bmeData();
#endif
#if ENABLE_BH1750
void lightData();
#endif
#if ENABLE_RAIN
void rainData();
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
  
  // Initialize PM sensor serial port (if enabled)
  #if ENABLE_PMS
    #ifdef ESP32
      PMS_SERIAL.begin(9600, SERIAL_8N1, 16, 17);  // RX=GPIO16, TX=GPIO17
      Serial.println("ESP32: Using Hardware Serial (UART2)");
    #else
      PMS_SERIAL.begin(9600);  // D5/D6 on ESP8266
      Serial.println("ESP8266: Using Software Serial");
    #endif
    Serial.println("PMS sensor enabled");
  #else
    Serial.println("PMS sensor disabled");
  #endif
  
  // Initialize DHT sensor (if enabled)
  #if ENABLE_DHT
    dht.begin();
    Serial.println("DHT sensor enabled");
  #else
    Serial.println("DHT sensor disabled");
  #endif
  
  // Initialize I2C bus for any I2C sensors
  #if ENABLE_BME280 || ENABLE_BH1750
    Wire.begin();
    delay(100);  // Give I2C bus time to stabilize
  #endif

  // Initialize BME280/BMP280 sensor (if enabled)
  #if ENABLE_BME280
    // Try BME280 first (has humidity)
    if (bme.begin(0x76)) {
      bmeAvailable = true;
      Serial.println("BME280 enabled (addr 0x76)");
    } else if (bme.begin(0x77)) {
      bmeAvailable = true;
      Serial.println("BME280 enabled (addr 0x77)");
    } 
    // If BME280 not found, try BMP280 (no humidity)
    else if (bmp.begin(0x76)) {
      bmpAvailable = true;
      Serial.println("BMP280 enabled (addr 0x76) - no humidity");
    } else if (bmp.begin(0x77)) {
      bmpAvailable = true;
      Serial.println("BMP280 enabled (addr 0x77) - no humidity");
    } else {
      Serial.println("BME280/BMP280 not found - continuing without it");
    }
  #else
    Serial.println("BME280 sensor disabled");
  #endif
  
  // Initialize BH1750 light sensor (if enabled)
  #if ENABLE_BH1750
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
      bh1750Available = true;
      Serial.println("BH1750 light sensor enabled");
    } else {
      Serial.println("BH1750 not found - continuing without it");
    }
  #else
    Serial.println("BH1750 sensor disabled");
  #endif
  
  // Initialize rain sensor (if enabled)
  #if ENABLE_RAIN
    pinMode(RAIN_PIN, INPUT);
    Serial.println("Rain sensor enabled on GPIO34");
  #else
    Serial.println("Rain sensor disabled");
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
  
  #ifdef ESP32
    // Disable brownout detector (overly sensitive on some ESP32 boards)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    Serial.println("Brownout detector disabled");
  #endif
  
  delay(500);
  connectWiFi();
  
  Serial.println("WiFi connected successfully!");
  Serial.flush();
  
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Time sync");
  time_t now = time(nullptr);
  while (now < 1700000000) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println(" OK");
  
  derivePubkey();
  setupNostrRelay();
  
  // Wait a bit for WebSocket to connect, then send metadata
  delay(2000);
  sendMetadataEvent();
  
  Serial.println("Ready!");
}

void loop() {
  webSocket.loop();
  
  static unsigned long lastSensor = 0, lastPost = 0;
  unsigned long now = millis();
  
  if (now - lastSensor > 2000) {
    #if ENABLE_PMS
      pmReadData();
    #endif
    #if ENABLE_DHT
      dhtData();
    #endif
    #if ENABLE_BME280
      bmeData();
    #endif
    #if ENABLE_BH1750
      lightData();
    #endif
    #if ENABLE_RAIN
      rainData();
    #endif
    #if ENABLE_MQ
      mqReadData();
    #endif
    #if ENABLE_OLED
      displayOled();
    #endif
    lastSensor = now;
  }
  
  if (now - lastPost > POST_INTERVAL) {
    // Check if we have any valid sensor data
    bool hasData = false;
    #if ENABLE_DHT
      hasData = hasData || (t > 0);
    #endif
    #if ENABLE_BME280
      hasData = hasData || ((bmeAvailable || bmpAvailable) && t > 0);
    #endif
    #if ENABLE_BH1750
      hasData = hasData || (bh1750Available && lux >= 0);
    #endif
    #if ENABLE_RAIN
      hasData = hasData || (rainValue > 0);
    #endif
    #if ENABLE_PMS
      hasData = hasData || (pm2_5 > 0);
    #endif
    
    if (wsConnected && (hasData || !ENABLE_DHT)) {  // Send if we have data OR if DHT is disabled
      // Send reading event with all sensor data in tags
      String event = createAndSignNostrEvent(t, h, pm1, pm2_5, pm10, airQuality);
      if (event.length() > 0) {
        Serial.println("Sending reading event...");
        String msg = "[\"EVENT\"," + event + "]";
        webSocket.sendTXT(msg);
      }
      
      // Send metadata event right after
      sendMetadataEvent();
    }
    lastPost = now;
  }
  delay(10);
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println(" OK");
  Serial.println(WiFi.localIP());
}

void setupNostrRelay() {
  webSocket.beginSSL(nostrRelayHost, nostrRelayPort, nostrRelayPath);
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
    else if (index == 9) pm10 = 256 * prev + value;
    else if (index > 15) break;
    index++;
  }
  while(PMS_SERIAL.available()) PMS_SERIAL.read();
}

void dhtData() {
  h = dht.readHumidity();
  t = dht.readTemperature();
  if (!isnan(t)) {
    Serial.print("T:"); Serial.print(t,1); Serial.print(" H:"); Serial.println(h,1);
  }
}

#if ENABLE_BME280
void bmeData() {
  if (bmeAvailable) {
    t = bme.readTemperature();
    h = bme.readHumidity();
    p = bme.readPressure() / 100.0F;  // Convert Pa to hPa
    
    Serial.print("BME280 - T:"); Serial.print(t,1); 
    Serial.print(" H:"); Serial.print(h,1);
    Serial.print(" P:"); Serial.print(p,1); Serial.println("hPa");
  } else if (bmpAvailable) {
    t = bmp.readTemperature();
    h = 0;  // BMP280 has no humidity sensor
    p = bmp.readPressure() / 100.0F;  // Convert Pa to hPa
    
    Serial.print("BMP280 - T:"); Serial.print(t,1); 
    Serial.print(" P:"); Serial.print(p,1); Serial.println("hPa");
  }
}
#endif

#if ENABLE_BH1750
void lightData() {
  if (!bh1750Available) return;
  
  lux = lightMeter.readLightLevel();
  Serial.print("Light: "); Serial.print(lux); Serial.println(" lux");
}
#endif

#if ENABLE_RAIN
void rainData() {
  rainValue = analogRead(RAIN_PIN);
  // Note: Higher value = drier, Lower value = more water
  Serial.print("Rain sensor: "); Serial.println(rainValue);
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
  #ifdef ESP32
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);  // 0 = SHA256 (not SHA224)
    mbedtls_sha256_update(&ctx, data, len);
    mbedtls_sha256_finish(&ctx, out);
    mbedtls_sha256_free(&ctx);
  #else
    br_sha256_context ctx;
    br_sha256_init(&ctx);
    br_sha256_update(&ctx, data, len);
    br_sha256_out(&ctx, out);
  #endif
}

void taggedHash(const uint8_t* tag, const uint8_t* data, size_t len, uint8_t* out) {
  #ifdef ESP32
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);
    mbedtls_sha256_update(&ctx, tag, 32);
    mbedtls_sha256_update(&ctx, tag, 32);
    mbedtls_sha256_update(&ctx, data, len);
    mbedtls_sha256_finish(&ctx, out);
    mbedtls_sha256_free(&ctx);
  #else
    br_sha256_context ctx;
    br_sha256_init(&ctx);
    br_sha256_update(&ctx, tag, 32);
    br_sha256_update(&ctx, tag, 32);
    br_sha256_update(&ctx, data, len);
    br_sha256_out(&ctx, out);
  #endif
}

void derivePubkey() {
  hexToBytes(nostrPrivateKeyHex, privKeyBytes, 32);
  
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
  
  // Add sensor readings only if enabled
  #if ENABLE_DHT
    readingTags += ",[\"" + String(TAG_TEMP) + "\",\"" + String(temp, 1) + "\",\"" + String(MODEL_DHT11) + "\"]";
    readingTags += ",[\"" + String(TAG_HUMIDITY) + "\",\"" + String(humidity, 1) + "\",\"" + String(MODEL_DHT11) + "\"]";
  #endif
  
  #if ENABLE_BME280
    if (bmeAvailable) {
      // BME280 has temp, humidity, and pressure
      readingTags += ",[\"" + String(TAG_TEMP) + "\",\"" + String(temp, 1) + "\",\"" + String(MODEL_BME280) + "\"]";
      readingTags += ",[\"" + String(TAG_HUMIDITY) + "\",\"" + String(humidity, 1) + "\",\"" + String(MODEL_BME280) + "\"]";
      readingTags += ",[\"" + String(TAG_PRESSURE) + "\",\"" + String(p, 1) + "\",\"" + String(MODEL_BME280) + "\"]";
    } else if (bmpAvailable) {
      // BMP280 has temp and pressure only (no humidity)
      readingTags += ",[\"" + String(TAG_TEMP) + "\",\"" + String(temp, 1) + "\",\"" + String(MODEL_BMP280) + "\"]";
      readingTags += ",[\"" + String(TAG_PRESSURE) + "\",\"" + String(p, 1) + "\",\"" + String(MODEL_BMP280) + "\"]";
    }
  #endif
  
  #if ENABLE_BH1750
    if (bh1750Available) {
      readingTags += ",[\"" + String(TAG_LIGHT) + "\",\"" + String(lux, 1) + "\",\"" + String(MODEL_BH1750) + "\"]";
    }
  #endif
  
  #if ENABLE_RAIN
    readingTags += ",[\"" + String(TAG_RAIN) + "\",\"" + String(rainValue) + "\",\"" + String(MODEL_MHRD) + "\"]";
  #endif
  
  #if ENABLE_PMS
    readingTags += ",[\"" + String(TAG_PM1) + "\",\"" + String(pm1_val) + "\",\"" + String(PMS_MODEL) + "\"]";
    readingTags += ",[\"" + String(TAG_PM25) + "\",\"" + String(pm25_val) + "\",\"" + String(PMS_MODEL) + "\"]";
    readingTags += ",[\"" + String(TAG_PM10) + "\",\"" + String(pm10_val) + "\",\"" + String(PMS_MODEL) + "\"]";
  #endif
  
  #if ENABLE_MQ
    readingTags += ",[\"" + String(TAG_AIR_QUALITY) + "\",\"" + String(aq_val) + "\",\"" + String(MODEL_MQ135) + "\"]";
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
  
  // Build tags array for metadata
  String metadataTags = "[";
  metadataTags += "[\"name\",\"" + String(stationName) + "\"]";
  
  // Add geohash location (NIP-52)
  if (strlen(stationGeohash) > 0) {
    metadataTags += ",[\"g\",\"" + String(stationGeohash) + "\"]";
  }
  
  // Add elevation if provided
  if (strlen(stationElevation) > 0) {
    metadataTags += ",[\"elevation\",\"" + String(stationElevation) + "\"]";
  }
  
  // Add power and connectivity info
  metadataTags += ",[\"power\",\"" + String(stationPower) + "\"]";
  metadataTags += ",[\"connectivity\",\"" + String(stationConnectivity) + "\"]";
  
  // Add sensor capabilities with models (dynamically based on what's detected)
  #if ENABLE_DHT
    metadataTags += ",[\"sensor\",\"" + String(TAG_TEMP) + "\",\"" + String(MODEL_DHT11) + "\"]";
    metadataTags += ",[\"sensor\",\"" + String(TAG_HUMIDITY) + "\",\"" + String(MODEL_DHT11) + "\"]";
  #endif
  
  #if ENABLE_BME280
    if (bmeAvailable) {
      metadataTags += ",[\"sensor\",\"" + String(TAG_TEMP) + "\",\"" + String(MODEL_BME280) + "\"]";
      metadataTags += ",[\"sensor\",\"" + String(TAG_HUMIDITY) + "\",\"" + String(MODEL_BME280) + "\"]";
      metadataTags += ",[\"sensor\",\"" + String(TAG_PRESSURE) + "\",\"" + String(MODEL_BME280) + "\"]";
    } else if (bmpAvailable) {
      metadataTags += ",[\"sensor\",\"" + String(TAG_TEMP) + "\",\"" + String(MODEL_BMP280) + "\"]";
      metadataTags += ",[\"sensor\",\"" + String(TAG_PRESSURE) + "\",\"" + String(MODEL_BMP280) + "\"]";
    }
  #endif
  
  #if ENABLE_BH1750
    if (bh1750Available) {
      metadataTags += ",[\"sensor\",\"" + String(TAG_LIGHT) + "\",\"" + String(MODEL_BH1750) + "\"]";
    }
  #endif
  
  #if ENABLE_RAIN
    metadataTags += ",[\"sensor\",\"" + String(TAG_RAIN) + "\",\"" + String(MODEL_MHRD) + "\"]";
  #endif
  
  #if ENABLE_PMS
    metadataTags += ",[\"sensor\",\"" + String(TAG_PM1) + "\",\"" + String(PMS_MODEL) + "\"]";
    metadataTags += ",[\"sensor\",\"" + String(TAG_PM25) + "\",\"" + String(PMS_MODEL) + "\"]";
    metadataTags += ",[\"sensor\",\"" + String(TAG_PM10) + "\",\"" + String(PMS_MODEL) + "\"]";
  #endif
  
  #if ENABLE_MQ
    metadataTags += ",[\"sensor\",\"" + String(TAG_AIR_QUALITY) + "\",\"" + String(MODEL_MQ135) + "\"]";
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
