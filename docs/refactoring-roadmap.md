# Refactoring roadmap

Decisions made 2026-07 while planning the nixification. Records *why* the
current shape exists and what the intended end state is.

## Decision log (2026-07-10)

| # | Question | Decision |
|---|---|---|
| Q1 | ESP8266 support | Dropped. ESP32-only; all board `#ifdef`s purged. |
| Q2 | Arduino layer | Keep: ESP-IDF + arduino-esp32 as IDF component. Pure-IDF migration is step 3, incremental. |
| Q3 | Per-station builds | Dropped. Identity = runtime NVS via dashboard; committed secret-free `factory_defaults.h`. No secrets may ever enter the nix store. |
| Q4 | Feature-flag abstraction | Nix attrset: `variants = { mvp = { bme280 = true; … }; }` → `-DENABLE_*` defines + per-variant component set. No Kconfig migration now. |
| Q5 | Day-one variants | `mvp`, `airquality-sps30`, `airquality-sds011`. |
| Q6 | Toolchain versions | Pin arduino-esp32 **2.0.17** + ESP-IDF **4.4.x** (identical to the last proven PlatformIO build). Upgrade to 3.x / IDF 5.x later as its own commit, after tests exist. |
| Q7 | Dependency pinning | Every Arduino library = flake input with `flake = false`, pinned via flake.lock. |
| Q8 | PlatformIO removal | Same commit as the flake — clean cutover. Flash/monitor via `nix run` wrappers (esptool). |
| Q9 | Host test framework | Catch2 v3 + CTest, wired into `nix flake check`. |
| Q10 | CI | GitHub Actions running `nix flake check` from day one. |
| Q11 | QEMU emulation | Boot-smoke check in phase 2 (serial banner + AP SSID line). QEMU has **no WiFi emulation** — dashboard/relay flows are never emulated. |

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
