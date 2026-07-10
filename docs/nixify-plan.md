# Nixify implementation plan

Executes the decisions in `docs/refactoring-roadmap.md` (see Decision log).
Goal: PlatformIO fully replaced by a nix flake; all firmware variants,
dev tooling, tests, and CI flow through `nix build` / `nix flake check`.

Reference build to reproduce (last PlatformIO build, 2026-07-10):
arduino-esp32 core **2.0.17** (framework-arduinoespressif32 3.20017,
sha dcc1105b), ESP-IDF **4.4.x**, xtensa-esp32 gcc 8.4.0 (2021r2-patch5),
partition table `min_spiffs.csv`. Result was RAM 15.5%, Flash 55.2%
(1,085,725 B of 1,966,080 B app partition) — use as sanity band (±5%)
for the nix-built image.

## Phase 1 — flake + clean cutover

### WP1: Toolchain research + pin (blocking; do first)

Determine the exact flake inputs providing ESP-IDF 4.4 + xtensa-esp32
toolchain. Primary candidate: `mirrexagon/nixpkgs-esp-dev` (pin a revision
that still carries IDF 4.4; current tip targets 5.x). Fallbacks: espressif
toolchain binaries via nixpkgs `stdenv` wrapper, or building gcc-xtensa
from source (avoid). Deliverable: flake input pins + a devshell where
`idf.py --version` reports 4.4.x and `xtensa-esp32-elf-gcc --version`
reports 8.4.0.

### WP2: IDF project scaffolding (after WP1)

- Top-level `CMakeLists.txt` (IDF project), `main/` component whose
  `CMakeLists.txt` compiles the existing `src/*.cpp` (do NOT move source
  files; point `SRCS` at `../src`).
- `partitions.csv` committed (copy of PlatformIO's `min_spiffs` layout).
- `sdkconfig.defaults`: partition table path, flash size 4 MB, brownout
  disable, FreeRTOS/lwip settings matching arduino-esp32 2.0.17 component
  requirements (documented in arduino-esp32 docs "Using as IDF component").
- arduino-esp32 2.0.17 as flake input, exposed to the build as
  `components/arduino` (nix-side symlink/copy into the build dir, not a
  git submodule).

### WP3: Arduino libraries as IDF components (parallel with WP2)

One flake input (`flake = false`) per library, each wrapped as an IDF
component via a generated `CMakeLists.txt` (`idf_component_register` with
the lib's `src`/root include dirs, `REQUIRES arduino`):

| Input | Repo | Pin |
|---|---|---|
| ArduinoJson | bblanchon/ArduinoJson | v6.21.x tag used by last build |
| WebSockets | Links2004/arduinoWebSockets | 2.4.x |
| Adafruit BusIO | adafruit/Adafruit_BusIO | (transitive dep of all Adafruit sensor libs — PlatformIO pulled it implicitly) |
| Adafruit Unified Sensor | adafruit/Adafruit_Sensor | (transitive) |
| Adafruit GFX | adafruit/Adafruit-GFX-Library | 1.11.x |
| Adafruit SSD1306 | adafruit/Adafruit_SSD1306 | 2.5.x |
| Adafruit BME280 | adafruit/Adafruit_BME280_Library | 2.2.x |
| Adafruit BMP280 | adafruit/Adafruit_BMP280_Library | 2.6.x |
| DHT sensor library | adafruit/DHT-sensor-library | 1.4.x |
| BH1750 | claws/BH1750 | 1.3.x |
| micro-ecc | kmackay/micro-ecc | master (pin rev) |
| SPS30 | Sensirion/arduino-i2c-sps30 + Sensirion/arduino-core | pin revs |
| SDS011 | lewapek/sds-dust-sensors-arduino-library | pin rev |

Check each lib's `library.properties` for further transitive deps. Exact
pins: read `.pio/libdeps/esp32dev/*/library.properties` from the last
PlatformIO build if still present, else use latest matching the semver
ranges above.

### WP4: Variant abstraction (after WP2+WP3 compile)

`nix/variants.nix`:

```nix
{
  mvp              = { bme280 = true; bh1750 = true; rain = true;
                       pms = "PMS5003"; };
  airquality-sps30 = { sps30 = true; oled = true; };
  airquality-sds011= { sds011 = true; oled = true; };
}
```

A builder function maps attrs → `-DENABLE_*=1/0` (+ `-DPMS_MODEL="…"`)
compile definitions on the `main` component and selects which library
components are linked. `include/factory_defaults.h` loses the `ENABLE_*`
block (flags come from the build) and keeps only runtime identity
defaults. Outputs: `packages.<system>.firmware-<variant>` (dir with
`firmware.bin`, `firmware.elf`, `bootloader.bin`, `partition-table.bin`,
flash-offset manifest).

### WP5: Flash/monitor tooling (parallel with WP4)

- `apps.flash-<variant>`: wraps nixpkgs `esptool` with correct offsets
  (`0x1000 bootloader, 0x8000 partition table, 0x10000 app` for IDF 4.4);
  erase variant `flash-erase-<variant>` (`esptool erase_flash` first).
- `apps.monitor`: serial monitor at 115200 (picocom or idf monitor).
- devshell: esptool, cmake/ninja, xtensa toolchain, clangd config.

### WP6: Cutover (same commit as flake lands)

- Delete `platformio.ini`.
- README + AGENTS.md + docs/mvp-stations.md: replace every `pio …` command
  with `nix build .#firmware-mvp` / `nix run .#flash-mvp` /
  `nix run .#monitor`. AGENTS.md "never esptool.py directly" rule becomes
  "always via nix run wrappers".

### WP7: CI (parallel with WP5/WP6)

`.github/workflows/ci.yml`: checkout, install-nix-action (or
DeterminateSystems), `nix flake check`. Cache via magic-nix-cache or
cachix (optional day one).

### Phase 1 acceptance

- `nix build .#firmware-mvp .#firmware-airquality-sps30
  .#firmware-airquality-sds011` all succeed on x86_64-linux.
- mvp `firmware.bin` size within ±5% of 1,085,725 B.
- No PlatformIO reference left in repo (`grep -ri platformio` clean,
  `pio` commands gone from docs).
- `nix flake check` green; CI workflow runs it.
- **Hardware parity (manual, human):** flash one physical board via
  `nix run .#flash-mvp`, confirm serial banner, `WeatherStation-XXXXXX` AP,
  dashboard loads, a kind:4223 event reaches the relay. This gates
  declaring phase 1 done, not the commit itself (Q8: clean cutover).

## Phase 2 — QEMU boot-smoke

- Flake input: Espressif QEMU (`qemu-system-xtensa` with esp32 machine;
  packaged in nixpkgs-esp-dev).
- Check `checks.<system>.qemu-boot-mvp`: merge bin images into a flash
  image, boot `-machine esp32 -nographic`, assert within 30 s the serial
  output contains the boot banner and `AP SSID: WeatherStation-` line
  (WiFi init may fail in QEMU — asserting the log line printed *before*
  radio init; adjust the assertion to whatever the firmware prints
  pre-WiFi if needed).
- Known limit: no WiFi emulation. Dashboard/relay flows stay on hardware.

## Phase 3 — testability refactor + Catch2 (TDD)

Extract pure logic from `src/main.cpp` into host-compilable components
(no Arduino headers; interfaces take bytes/strings, return bytes/strings):

1. `wx/crypto` — sha256Raw, taggedHash, schnorrSign, derivePubkey glue.
   Tests: BIP-340 official test vectors (known-answer).
2. `wx/nostr` — event serialization (NIP-01 canonical JSON, id hash),
   kind:16158 + kind:4223 tag building. Tests: golden events, id
   round-trip, escaping edge cases.
3. `wx/relay_url` — parseRelayUrl. Tests: ws/wss/bare-host/port/path.
4. `wx/rain_latch` — floating-pin latch logic. Tests: noise vs dry-streak
   sequences.

Rules: strict TDD (one behavior per red-green cycle, see skill://tdd),
tests via public interfaces only. Firmware `main/` consumes the same
component sources; host build is a plain-gcc CMake target registered as
`checks.<system>.host-tests` (Catch2 v3 from nixpkgs).

## Risks / gotchas

- **IDF 4.4 in nixpkgs-esp-dev**: tip may have dropped 4.4 — pinning an
  old revision of that flake is expected, not a failure. If 4.4 is
  unobtainable at reasonable cost, STOP and escalate (decision Q6 would
  need revisiting) rather than silently building against 5.x.
- **arduino-esp32 2.0.17 as component** requires specific sdkconfig
  options (e.g. `CONFIG_FREERTOS_HZ=1000`, mbedtls settings); build
  errors here mean sdkconfig.defaults is incomplete, not a broken lib.
- **micro-ecc**: plain C, needs `uECC_OPTIMIZATION_LEVEL`/platform defines
  consistent with what PlatformIO used (defaults are fine on xtensa; do
  not enable asm optimizations).
- **Adafruit transitive deps** (BusIO, Unified Sensor) are easy to miss —
  PlatformIO resolved them silently.
- **`min_spiffs` partition layout** must be copied exactly; app partition
  size 0x1E0000 is what makes the 1.09 MB image fit.
- **Secrets**: nothing user-specific may enter the nix store. Identity is
  runtime NVS only (decision Q3).
