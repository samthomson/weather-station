# Flash/monitor CLI: `nix run .#flash-<variant>`, `.#flash-erase-<variant>`,
# `.#monitor` — see docs for usage.
#
# esptool (3.2) and picocom (3.2a) come from the pinned nixpkgs and are
# referenced ONLY inside these wrapper scripts. They must never enter the
# firmware derivations' inputs: nixpkgs esptool propagates packaging ->
# pyparsing 2.4.7, which shadows the IDF env's 2.3.1 and breaks ldgen
# (WP1 gotcha). `devTools` below wraps esptool for the devshell the same
# way, so nothing python-propagated lands on the shell's PYTHONPATH.
#
# nixpkgs snapshot is 2021-11: no writeShellApplication yet, hence
# writeShellScriptBin + explicit `set -euo pipefail`.
{ pkgs, lib, firmwarePackages }:

let
  esptool = "${pkgs.esptool}/bin/esptool.py";
  picocom = "${pkgs.picocom}/bin/picocom";

  variantNames = map (lib.removePrefix "firmware-") (builtins.attrNames firmwarePackages);

  # Offsets fixed by the IDF 4.4 ESP32 image layout; identical to
  # flash-manifest.json inside every firmware package.
  offsetLine = "0x1000 bootloader.bin   0x8000 partition-table.bin   0xe000 ota_data_initial.bin   0x10000 firmware.bin";

  requirePort = ''
    PORT="''${1:-/dev/ttyUSB0}"
    if [ ! -e "$PORT" ]; then
      echo "error: serial port '$PORT' does not exist." >&2
      echo "hint: pass the port as the first argument, e.g. -- /dev/ttyUSB1" >&2
      exit 1
    fi
  '';

  mkFlash = { variant, erase ? false }:
    let
      name = "flash-${lib.optionalString erase "erase-"}${variant}";
      fw = firmwarePackages."firmware-${variant}";
    in
    pkgs.writeShellScriptBin name ''
      set -euo pipefail
      PORT="''${1:-/dev/ttyUSB0}"
      echo "variant:  ${variant}"
      echo "firmware: ${fw}"
      echo "port:     $PORT @ 115200 baud"
      echo "offsets:  ${offsetLine}"
      ${requirePort}
      ${lib.optionalString erase ''
        echo "erasing entire flash (this wipes NVS, i.e. the station identity) ..."
        ${esptool} --chip esp32 --port "$PORT" --baud 115200 erase_flash
      ''}
      exec ${esptool} --chip esp32 --port "$PORT" --baud 115200 write_flash \
        0x1000 ${fw}/bootloader.bin \
        0x8000 ${fw}/partition-table.bin \
        0xe000 ${fw}/ota_data_initial.bin \
        0x10000 ${fw}/firmware.bin
    '';

  monitor = pkgs.writeShellScriptBin "monitor" ''
    set -euo pipefail
    PORT="''${1:-/dev/ttyUSB0}"
    echo "port: $PORT @ 115200 baud"
    echo "exit: Ctrl-A Ctrl-X"
    ${requirePort}
    exec ${picocom} --baud 115200 "$PORT"
  '';

  mkApp = drv: name: description: {
    type = "app";
    program = "${drv}/bin/${name}";
    meta = { inherit description; };
  };

  flashApps = lib.foldl'
    (acc: variant: acc // {
      "flash-${variant}" =
        mkApp (mkFlash { inherit variant; }) "flash-${variant}"
          "Flash the ${variant} firmware over serial (esptool, 115200 baud)";
      "flash-erase-${variant}" =
        mkApp (mkFlash { inherit variant; erase = true; }) "flash-erase-${variant}"
          "Erase entire flash (wipes NVS), then flash the ${variant} firmware";
    })
    { }
    variantNames;
in
{
  apps = flashApps // {
    monitor = mkApp monitor "monitor" "Serial monitor at 115200 baud (picocom; exit: Ctrl-A Ctrl-X)";
  };

  # For the devshell: picocom directly (plain C program), esptool through a
  # thin exec wrapper so its python propagation stays out of PYTHONPATH.
  devTools = [
    pkgs.picocom
    (pkgs.writeShellScriptBin "esptool.py" ''exec ${esptool} "$@"'')
  ];
}
