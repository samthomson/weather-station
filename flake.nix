{
  description = "ESP32 weather station firmware — ESP-IDF 4.4.7 + arduino-esp32 2.0.17 as IDF component";

  inputs = {
    # Default nixpkgs (current): host-side tooling (esptool 5.x, picocom)
    # and everything else not welded to the IDF 4.4 era. Locked; moves on
    # `nix flake update`.
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";

    # 2021-11 nixpkgs snapshot for the firmware toolchain ONLY: IDF 4.4
    # needs that era (pkgconfig, ncurses5, python 3.9, cmake 3.21; modern
    # cmake >= 4 breaks IDF 4.4's cmake_minimum_required(3.5)). esp-dev is
    # consumed just for this lock; the esp-idf and xtensa-toolchain
    # derivations themselves are vendored in nix/.
    esp-dev.url = "github:mirrexagon/nixpkgs-esp-dev/48413ee362b4d0709e1a0dff6aba7fd99060335e";
    nixpkgs-idf.follows = "esp-dev/nixpkgs";

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

  outputs = inputs@{ self, nixpkgs, nixpkgs-idf, arduino-esp32, ... }:
    let
      inherit (nixpkgs) lib;

      # x86_64-linux is the reference (CI) platform; aarch64-darwin builds
      # the same firmware from Espressif's macos-arm64 toolchain binaries.
      systems = [ "x86_64-linux" "aarch64-darwin" ];

      variants = import ./nix/variants.nix;

      perSystem = system:
        let
          # Default package set (current nixpkgs): wrappers, host tools.
          pkgs = import nixpkgs { inherit system; };
          # IDF-era snapshot: ONLY the firmware toolchain below may use it.
          idfPkgs = import nixpkgs-idf { inherit system; };

          # arduino-esp32 2.0.17 officially pairs with IDF v4.4.7; IDF v4.4.7
          # requires crosstool-NG esp-2021r2-patch5 (still gcc 8.4.0).
          espIdf = idfPkgs.callPackage ./nix/esp-idf.nix { };
          xtensaToolchain = idfPkgs.callPackage ./nix/xtensa-toolchain.nix { };

          # IDF-era host build tools, single source: consumed verbatim by
          # both the firmware drv (hostTools) and the devShell — they MUST
          # match (modern cmake >= 4 rejects IDF 4.4's cmake files).
          idfTools = with idfPkgs; [
            git
            cmake
            ninja
            flex
            bison
            gperf
            pkgconfig
            ncurses5
          ];

          arduinoLibs = import ./nix/arduino-libs.nix { pkgs = idfPkgs; srcs = inputs; };

          firmwarePackages = lib.mapAttrs'
            (name: flags: lib.nameValuePair "firmware-${name}"
              (import ./nix/firmware.nix {
                inherit flags arduinoLibs espIdf xtensaToolchain;
                pkgs = idfPkgs;
                hostTools = idfTools;
                variantName = name;
                arduinoEsp32Src = arduino-esp32;
                projectRoot = self;
              }))
            variants;

          # Flash/monitor helpers (nix run .#flash-<variant>, .#monitor).
          # esptool/picocom stay out of the firmware drvs — apps only.
          espApps = import ./nix/apps.nix { inherit pkgs lib firmwarePackages variants; };

          # Building all variants + flash/monitor scripts IS the check, so
          # checks IS packages by construction.
          allPackages = firmwarePackages // espApps.packages;
        in
        {
          packages = allPackages;

          checks = allPackages;

          apps = espApps.apps;

          # IDF-era shell: shares idfTools with the firmware drv above.
          devShell = idfPkgs.mkShell {
            name = "weather-station-idf";
            # NB: no esptool package directly — it propagates packaging ->
            # newer pyparsing, which can shadow the IDF env's 2.3.1 and break
            # ldgen. The devTools exec wrapper avoids that.
            buildInputs = idfTools ++ [ espIdf xtensaToolchain ]
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
