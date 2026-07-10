# Flash/monitor CLI: `nix run .#flash-<variant>`, `.#flash-erase-<variant>`,
# `.#monitor` — see docs for usage.
#
# `pkgs` is the flake's default (current) nixpkgs: esptool (5.x) and
# picocom track modern releases — flashing only talks the serial protocol
# and has zero coupling to the pinned IDF-era snapshot. They must never
# enter the firmware derivations' inputs, and in the devshell esptool is
# exposed through a thin exec wrapper so its python propagation cannot
# land on PYTHONPATH and shadow the IDF env's pyparsing 2.3.1 (breaks
# ldgen — WP1 gotcha).
{ pkgs, lib, firmwarePackages }:

let
  esptool = "${pkgs.esptool}/bin/esptool";
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
        ${esptool} --chip esp32 --port "$PORT" --baud 115200 erase-flash
      ''}
      exec ${esptool} --chip esp32 --port "$PORT" --baud 115200 write-flash \
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

  # One writeShellScriptBin per command, exposed twice: as `apps` for
  # `nix run`, and as `packages` so they are buildable (`nix build
  # .#flash-mvp`) and CI/checks catch eval or platform regressions.
  scripts = lib.foldl'
    (acc: variant: acc // {
      "flash-${variant}" = {
        drv = mkFlash { inherit variant; };
        description = "Flash the ${variant} firmware over serial (esptool, 115200 baud)";
      };
      "flash-erase-${variant}" = {
        drv = mkFlash { inherit variant; erase = true; };
        description = "Erase entire flash (wipes NVS), then flash the ${variant} firmware";
      };
    })
    {
      monitor = {
        drv = monitor;
        description = "Serial monitor at 115200 baud (picocom; exit: Ctrl-A Ctrl-X)";
      };
    }
    variantNames;
in
{
  apps = lib.mapAttrs
    (name: s: {
      type = "app";
      program = "${s.drv}/bin/${name}";
      meta = { inherit (s) description; };
    })
    scripts;

  packages = lib.mapAttrs (_: s: s.drv) scripts;

  # For the devshell: picocom directly (plain C program), esptool through a
  # thin exec wrapper so its python propagation stays out of PYTHONPATH.
  devTools = [
    pkgs.picocom
    (pkgs.writeShellScriptBin "esptool" ''exec ${esptool} "$@"'')
  ];
}
