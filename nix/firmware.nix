# One firmware variant, built with ESP-IDF 4.4.7 + arduino-esp32 2.0.17 as an
# IDF component, entirely inside the pure nix build sandbox (no network; the
# toolchain, IDF python env and every library come from the store).
#
# Build pattern follows the verified nixpkgs-esp-dev recipe: HOME=$TMPDIR,
# IDF_COMPONENT_MANAGER=0, `idf.py set-target esp32 && idf.py build`.
# NB: pkgs.esptool must NOT appear in the inputs — it propagates
# packaging -> pyparsing 2.4.7 onto PYTHONPATH, which breaks IDF 4.4's ldgen
# (needs pyparsing < 2.4). IDF uses its bundled esptool.
{ pkgs
, espIdf            # esp-idf 4.4.7 derivation (nix/esp-idf.nix)
, xtensaToolchain   # gcc-xtensa-esp32-elf-bin, crosstool esp-2021r2-patch5
, arduinoEsp32Src   # arduino-esp32 2.0.17 flake input (becomes components/arduino)
, arduinoLibs       # import ./arduino-libs.nix { ... }
, projectRoot       # flake self (filtered down to the IDF project files)
, variantName
, flags             # full flag set from nix/variants.nix
}:

let
  inherit (pkgs) lib;

  onOff = v: if v then "1" else "0";

  # Every ENABLE_* flag the sources test with `#if` must be defined 0 or 1;
  # PMS_MODEL (a quoted string) only exists when a PMS model is selected.
  variantDefines = [
    "ENABLE_DHT=${onOff flags.dht}"
    "ENABLE_BME280=${onOff flags.bme280}"
    "ENABLE_BH1750=${onOff flags.bh1750}"
    "ENABLE_RAIN=${onOff flags.rain}"
    "ENABLE_PMS=${onOff (flags.pms != null)}"
    "ENABLE_MQ=${onOff flags.mq}"
    "ENABLE_OLED=${onOff flags.oled}"
    "ENABLE_SPS30=${onOff flags.sps30}"
    "ENABLE_SDS011=${onOff flags.sds011}"
    "ENABLE_BMP280=${onOff flags.bmp280}"
  ] ++ lib.optional (flags.pms != null) ''PMS_MODEL="${flags.pms}"'';

  # Variant-only library components (e.g. sps30 pulls sensirion_core).
  extraComponents = lib.concatLists
    (lib.mapAttrsToList
      (flag: comps: lib.optionals (flags.${flag} == true) comps)
      arduinoLibs.conditional);

  selectedComponents = arduinoLibs.baseComponents ++ extraComponents;

  # One dir with an entry per Arduino library component; handed to the IDF
  # build via EXTRA_COMPONENT_DIRS (dir name == IDF component name).
  componentsFarm = pkgs.linkFarm "wx-arduino-components-${variantName}"
    (map (name: { inherit name; path = arduinoLibs.components.${name}; })
      selectedComponents);

  definesArg = lib.concatStringsSep ";" variantDefines;
  extraComponentsArg = lib.concatStringsSep ";" extraComponents;

  flashManifest = pkgs.writeText "flash-manifest-${variantName}.json"
    (builtins.toJSON {
      variant = variantName;
      chip = "esp32";
      flash_mode = "dio";
      flash_freq = "40m";
      flash_size = "4MB";
      parts = [
        { offset = "0x1000"; file = "bootloader.bin"; }
        { offset = "0x8000"; file = "partition-table.bin"; }
        { offset = "0xe000"; file = "ota_data_initial.bin"; }
        { offset = "0x10000"; file = "firmware.bin"; }
      ];
    });

  # Only the files the IDF build consumes; docs/STLs/nix changes don't
  # invalidate the firmware derivation.
  projectSrc = lib.cleanSourceWith {
    name = "weather-station-project";
    src = projectRoot;
    filter = path: type:
      let rel = lib.removePrefix (toString projectRoot + "/") (toString path);
      in lib.any (keep: rel == keep || lib.hasPrefix (keep + "/") rel) [
        "CMakeLists.txt"
        "sdkconfig.defaults"
        "partitions.csv"
        "main"
        "src"
        "include"
      ];
  };
in
pkgs.stdenv.mkDerivation {
  pname = "weather-station-firmware-${variantName}";
  version = "2.0.17-idf4.4.7";

  src = projectSrc;

  nativeBuildInputs = (with pkgs; [
    git
    cmake
    ninja
    flex
    bison
    gperf
    pkgconfig
    ncurses5
  ]) ++ [ espIdf xtensaToolchain ];

  buildPhase = ''
    # idf.py wants a writable HOME; no network in the sandbox, so the
    # component manager must stay off. IDF's cmake runs `git describe` on
    # IDF_PATH; it fails gracefully and reads the version from source.
    export HOME=$TMPDIR
    export IDF_COMPONENT_MANAGER=0
    # Read by main/CMakeLists.txt. Env var, not a -D cache entry: IDF 4.4
    # collects REQUIRES in a separate script-mode cmake process that sees
    # the environment but not the cache.
    export WX_EXTRA_COMPONENTS='${extraComponentsArg}'

    # arduino-esp32 registers itself as the `arduino` component when placed
    # at components/arduino. Copy (not symlink): the store is read-only and
    # 0444, and the component tree must be writable for the build.
    mkdir -p components
    cp -rL ${arduinoEsp32Src} components/arduino
    chmod -R u+w components/arduino

    idfArgs=(
      -DWX_VARIANT_DEFINES='${definesArg}'
      -DEXTRA_COMPONENT_DIRS='${componentsFarm}'
    )
    idf.py "''${idfArgs[@]}" set-target esp32
    idf.py "''${idfArgs[@]}" build
  '';

  installPhase = ''
    mkdir -p $out
    cp build/weather_station.bin $out/firmware.bin
    cp build/weather_station.elf $out/firmware.elf
    cp build/bootloader/bootloader.bin $out/bootloader.bin
    cp build/partition_table/partition-table.bin $out/partition-table.bin
    # otadata seed: selects app0. min_spiffs has ota_0/ota_1 slots; flashing
    # this forces the new image at app0 to boot even if a previous otadata
    # pointed at app1 (same role as the reference build's boot_app0.bin).
    cp build/ota_data_initial.bin $out/ota_data_initial.bin
    cp ${flashManifest} $out/flash-manifest.json
  '';

  dontConfigure = true;
  dontFixup = true;
}
