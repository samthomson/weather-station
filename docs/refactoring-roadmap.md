# Refactoring roadmap

Decisions made 2026-07 while planning the nixification. Records *why* the
current shape exists and what the intended end state is.

## Done

- **ESP8266 dropped.** ESP32-only. Removed `nodemcuv2_station1`, all
  `#ifdef ESP32` board branches, SoftwareSerial, bearssl.
- **Per-station builds dropped.** One firmware. Station identity (WiFi,
  name, geohash, Nostr key, relay) is runtime state in NVS, configured via
  the captive-portal dashboard. First-boot defaults come from the committed,
  secret-free `include/factory_defaults.h`. Rationale: per-station secrets
  headers would leak into the world-readable `/nix/store` once builds are
  pure, and the dashboard already covers configuration.

## Step 1 — Nixify the build (in progress)

Replace PlatformIO with **ESP-IDF + arduino-esp32 as an IDF component**,
pinned via nix flake inputs (`nixpkgs-esp-dev` for toolchain + IDF, each
Arduino library as a pinned `fetchFromGitHub` wrapped as an IDF component).

- Build stays CMake-native → unlocks host-side unit tests and QEMU
  (Espressif fork) integration tests.
- Compile-time sensor feature set (`ENABLE_*` in `factory_defaults.h`)
  becomes a nix-level feature-set abstraction generating `-D` defines; one
  derivation per hardware variant (MVP, air-quality, …).
- Constraint inherited: arduino-esp32 version pins the IDF version
  (arduino-esp32 3.x ↔ IDF 5.x).

## Step 2 — Testability refactor

`src/main.cpp` (~1.4 kLoC) mixes pure logic with hardware I/O. Extract the
pure parts into components testable on the host with a normal gcc:

- **Nostr event building + BIP-340 signing** (`sha256Raw`, `taggedHash`,
  `schnorrSign`, event JSON serialization, relay URL parsing) — no hardware
  dependency; highest-value unit-test target (protocol correctness,
  known-answer test vectors from BIP-340).
- **Sensor plausibility / latch logic** (e.g. the rain-sensor
  floating-pin latch) — pure functions over readings.
- Keep `main.cpp` as thin glue: read sensors → build event → publish.

## Step 3 — Retire the Arduino layer (future, incremental)

The IDF component structure from step 1 allows swapping Arduino libraries
for native IDF drivers one at a time, no big-bang rewrite:

| Arduino lib | Native replacement |
|---|---|
| `WebSocketsClient` (links2004) | `esp_websocket_client` (IDF built-in) |
| `WiFi.h` / `WiFiClientSecure` | `esp_wifi` + `esp_tls` |
| Adafruit BME280/BMP280 | IDF I2C driver (community BME280 components exist) |
| BH1750 (claws) | trivial I2C register reads |
| Adafruit SSD1306/GFX | `esp_lcd` or community ssd1306 component |
| DHT sensor library | RMT-based DHT component |
| micro-ecc | keep (plain C, no Arduino dependency) |
| ArduinoJson | keep, or `cJSON` (IDF built-in) |

Each swap shrinks flash, removes the arduino-esp32 ↔ IDF version coupling,
and moves config to native Kconfig. Only start once step 2's tests exist —
they are the safety net proving protocol behavior unchanged.

## Test layers (target state)

1. **Host unit tests** — pure components (step 2), plain gcc + CTest via
   nix check. Fast, run on every build.
2. **QEMU integration tests** — full firmware image on Espressif QEMU
   (`qemu-system-xtensa` fork, packaged in `nixpkgs-esp-dev`); boot, seed
   NVS, assert serial output / published events against a mock relay.
3. **Hardware-in-the-loop** — real board flash + serial assertions,
   manual/on-demand, outside nix checks.
