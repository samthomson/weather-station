#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <time.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WebSocketsClient.h>
#include <bearssl/bearssl_hash.h>
#include "uECC.h"
#include "secrets.h"

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
const unsigned long POST_INTERVAL = 30000;

// Sensor types available on this station
const char* sensors[] = {"temp", "humidity", "pm1", "pm25", "pm10"};
const int sensorCount = sizeof(sensors) / sizeof(sensors[0]);

SoftwareSerial mySerial(D5, D6);
unsigned int pm1 = 0, pm2_5 = 0, pm10 = 0;
#define DHTTYPE DHT11
DHT dht(D7, DHTTYPE);
float t = 0, h = 0;
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
const int ledRed = D0, ledGreen = D8;
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
void displayOled();
void ledStatus();
void derivePubkey();
String bytesToHex(uint8_t* bytes, int len);
void hexToBytes(const char* hex, uint8_t* bytes, int len);
void sha256Raw(const uint8_t* data, size_t len, uint8_t* out);
void taggedHash(const uint8_t* tag, const uint8_t* data, size_t len, uint8_t* out);
bool schnorrSign(const uint8_t* privkey, const uint8_t* msg, uint8_t* sig);
String createAndSignNostrEvent(float temp, float humidity, unsigned int pm1_val, unsigned int pm25_val, unsigned int pm10_val);
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
  
  mySerial.begin(9600);  // PMS5003/PMS7003 default baud rate
  dht.begin();
  pinMode(D0, OUTPUT);
  pinMode(D8, OUTPUT);
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED failed - continuing without display"));
    displayAvailable = false;
  } else {
    displayAvailable = true;
    display.clearDisplay();
    display.setTextColor(WHITE);
  }
  
  connectWiFi();
  
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
    pmReadData();
    dhtData();
    displayOled();
    ledStatus();
    lastSensor = now;
  }
  
  if (now - lastPost > POST_INTERVAL) {
    if (wsConnected && t > 0) {
      // Send reading event with all sensor data in tags
      String event = createAndSignNostrEvent(t, h, pm1, pm2_5, pm10);
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
  while (mySerial.available()) {
    value = mySerial.read();
    if ((index == 0 && value != 0x42) || (index == 1 && value != 0x4d)) break;
    if (index == 4 || index == 6 || index == 8) prev = value;
    else if (index == 5) pm1 = 256 * prev + value;
    else if (index == 7) pm2_5 = 256 * prev + value;
    else if (index == 9) pm10 = 256 * prev + value;
    else if (index > 15) break;
    index++;
  }
  while(mySerial.available()) mySerial.read();
}

void dhtData() {
  h = dht.readHumidity();
  t = dht.readTemperature();
  if (!isnan(t)) {
    Serial.print("T:"); Serial.print(t,1); Serial.print(" H:"); Serial.println(h,1);
  }
}

void ledStatus() {
  digitalWrite(ledGreen, pm2_5 <= 100 ? HIGH : LOW);
  digitalWrite(ledRed, pm2_5 > 50 ? HIGH : LOW);
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
  br_sha256_context ctx;
  br_sha256_init(&ctx);
  br_sha256_update(&ctx, data, len);
  br_sha256_out(&ctx, out);
}

void taggedHash(const uint8_t* tag, const uint8_t* data, size_t len, uint8_t* out) {
  br_sha256_context ctx;
  br_sha256_init(&ctx);
  br_sha256_update(&ctx, tag, 32);
  br_sha256_update(&ctx, tag, 32);
  br_sha256_update(&ctx, data, len);
  br_sha256_out(&ctx, out);
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

String createAndSignNostrEvent(float temp, float humidity, unsigned int pm1_val, unsigned int pm25_val, unsigned int pm10_val) {
  unsigned long createdAt = (unsigned long)time(nullptr);
  
  // Build tags with station reference and sensor data
  String readingTags = "[";
  readingTags += "[\"a\",\"16158:" + nostrPubkey + ":\"],";
  readingTags += "[\"temp\",\"" + String(temp, 1) + "\"],";
  readingTags += "[\"humidity\",\"" + String(humidity, 1) + "\"],";
  readingTags += "[\"pm1\",\"" + String(pm1_val) + "\"],";
  readingTags += "[\"pm25\",\"" + String(pm25_val) + "\"],";
  readingTags += "[\"pm10\",\"" + String(pm10_val) + "\"]";
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
  
  // Add sensor capabilities
  for (int i = 0; i < sensorCount; i++) {
    metadataTags += ",[\"sensor\",\"" + String(sensors[i]) + "\"]";
  }
  
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
