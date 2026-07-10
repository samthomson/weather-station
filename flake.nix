{
  description = "ESP32 weather station firmware — ESP-IDF 4.4.7 + arduino-esp32 2.0.17 as IDF component";

  inputs = {
    # Only consumed for its locked nixpkgs (2021-11): the IDF 4.4 era needs
    # that snapshot (pkgconfig, ncurses5, python 3.9, cmake 3.21; modern
    # cmake >= 4 breaks IDF 4.4's cmake_minimum_required(3.5)). The esp-idf
    # and xtensa-toolchain derivations themselves are vendored in nix/.
    esp-dev.url = "github:mirrexagon/nixpkgs-esp-dev/48413ee362b4d0709e1a0dff6aba7fd99060335e";
    nixpkgs.follows = "esp-dev/nixpkgs";

    # Current nixpkgs for host-side tooling only (esptool 5.x, picocom):
    # the flash/monitor wrappers have zero coupling to the IDF-era
    # snapshot. Locked like everything else; moves on `nix flake update`.
    nixpkgs-tools.url = "github:NixOS/nixpkgs/nixpkgs-unstable";

    # Arduino core, consumed as the IDF `arduino` component.
    arduino-esp32 = { url = "github:espressif/arduino-esp32/2.0.17"; flake = false; };

    # Arduino libraries (exact pins of the last proven reference build,
    # see docs/nixify-plan.md);
    # packaged as IDF components by nix/arduino-libs.nix.
    arduinojson = { url = "github:bblanchon/ArduinoJson/v6.21.6"; flake = false; };
    websockets = { url = "github:Links2004/arduinoWebSockets/e3edca4907e8524856fbc38a6bb00eb6073c982e"; flake = false; };
    adafruit-busio = { url = "github:adafruit/Adafruit_BusIO/1.17.4"; flake = false; };
    adafruit-sensor = { url = "github:adafruit/Adafruit_Sensor/1.1.15"; flake = false; };
    adafruit-gfx = { url = "github:adafruit/Adafruit-GFX-Library/1.12.6"; flake = false; };
    adafruit-ssd1306 = { url = "github:adafruit/Adafruit_SSD1306/2.5.17"; flake = false; };
    adafruit-bme280 = { url = "github:adafruit/Adafruit_BME280_Library/2.3.0"; flake = false; };
    adafruit-bmp280 = { url = "github:adafruit/Adafruit_BMP280_Library/2.6.8"; flake = false; };
    dht-sensor = { url = "github:adafruit/DHT-sensor-library/1.4.7"; flake = false; };
    bh1750 = { url = "github:claws/BH1750/1.3.0"; flake = false; };
    micro-ecc = { url = "github:kmackay/micro-ecc/541b3a78026420a3e369c4c9281c396b5e531113"; flake = false; };
    sps30 = { url = "github:Sensirion/arduino-i2c-sps30/1.0.1"; flake = false; };
    sensirion-core = { url = "github:Sensirion/arduino-core/0.7.3"; flake = false; };
    sds011 = { url = "github:lewapek/sds-dust-sensors-arduino-library/1.5.1"; flake = false; };
  };

  outputs = inputs@{ self, nixpkgs, arduino-esp32, ... }:
    let
      inherit (nixpkgs) lib;

      # x86_64-linux is the reference (CI) platform; aarch64-darwin builds
      # the same firmware from Espressif's macos-arm64 toolchain binaries.
      systems = [ "x86_64-linux" "aarch64-darwin" ];

      variants = import ./nix/variants.nix;

      perSystem = system:
        let
          pkgs = import nixpkgs { inherit system; };
          toolsPkgs = import inputs.nixpkgs-tools { inherit system; };

          # arduino-esp32 2.0.17 officially pairs with IDF v4.4.7; IDF v4.4.7
          # requires crosstool-NG esp-2021r2-patch5 (still gcc 8.4.0).
          esp-idf-447 = pkgs.callPackage ./nix/esp-idf.nix { };
          xtensa-toolchain-patch5 = pkgs.callPackage ./nix/xtensa-toolchain.nix { };

          arduinoLibs = import ./nix/arduino-libs.nix { inherit pkgs; srcs = inputs; };

          firmwarePackages = lib.mapAttrs'
            (name: flags: lib.nameValuePair "firmware-${name}"
              (import ./nix/firmware.nix {
                inherit pkgs flags arduinoLibs;
                variantName = name;
                espIdf = esp-idf-447;
                xtensaToolchain = xtensa-toolchain-patch5;
                arduinoEsp32Src = arduino-esp32;
                projectRoot = self;
              }))
            variants;

          # Flash/monitor helpers (nix run .#flash-<variant>, .#monitor).
          # esptool/picocom stay out of the firmware drvs — apps only.
          espApps = import ./nix/apps.nix { inherit pkgs lib toolsPkgs firmwarePackages; };
        in
        {
          packages = firmwarePackages // espApps.packages;

          # Building all variants + flash/monitor scripts IS the check.
          checks = firmwarePackages // espApps.packages;

          apps = espApps.apps;

          devShell = pkgs.mkShell {
            name = "weather-station-idf";
            # NB: no esptool package directly — it propagates packaging ->
            # newer pyparsing, which can shadow the IDF env's 2.3.1 and break
            # ldgen. The devTools exec wrapper avoids that.
            buildInputs = (with pkgs; [
              git
              cmake
              ninja
              flex
              bison
              gperf
              pkgconfig
              ncurses5
            ]) ++ [ esp-idf-447 xtensa-toolchain-patch5 ]
              # esptool 5.x (exec wrapper, keeps pyparsing off PYTHONPATH) + picocom
              # for driving flash/monitor manually; see nix/apps.nix.
              ++ espApps.devTools;
            shellHook = ''
              export IDF_COMPONENT_MANAGER=0
            '';
          };
        };

      bySystem = lib.genAttrs systems perSystem;
    in
    {
      packages = lib.mapAttrs (_: s: s.packages) bySystem;
      checks = lib.mapAttrs (_: s: s.checks) bySystem;
      apps = lib.mapAttrs (_: s: s.apps) bySystem;
      devShells = lib.mapAttrs (_: s: { default = s.devShell; }) bySystem;
    };
}
