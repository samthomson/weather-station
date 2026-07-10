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
{ pkgs, lib, firmwarePackages, variants }:

let
  esptool = "${pkgs.esptool}/bin/esptool";
  picocom = "${pkgs.picocom}/bin/picocom";

  variantNames = builtins.attrNames variants;

  defaultPort = "/dev/ttyUSB0";
  baud = "115200";

  # Shared with flash-manifest.json inside every firmware package
  # (nix/firmware.nix imports the same list).
  flashParts = import ./flash-layout.nix;
  offsetLine = lib.concatMapStringsSep "   " (p: "${p.offset} ${p.file}") flashParts;

  # The only place PORT is assigned: default, existence check, and a strict
  # arg count so a stray flag (e.g. `--baud 921600`) fails loudly instead of
  # being silently ignored.
  resolvePort = ''
    if [ "$#" -gt 1 ]; then
      echo "error: expected at most one argument (the serial port), got $#: $*" >&2
      exit 2
    fi
    PORT="''${1:-${defaultPort}}"
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
      writeFlashArgs =
        lib.concatMapStringsSep " " (p: "${p.offset} ${fw}/${p.file}") flashParts;
    in
    pkgs.writeShellApplication {
      inherit name;
      text = ''
        ${resolvePort}
        echo "variant:  ${variant}"
        echo "firmware: ${fw}"
        echo "port:     $PORT @ ${baud} baud"
        echo "offsets:  ${offsetLine}"
        ${lib.optionalString erase ''
          echo "erasing entire flash (this wipes NVS, i.e. the station identity) ..."
          ${esptool} --chip esp32 --port "$PORT" --baud ${baud} erase-flash
        ''}
        exec ${esptool} --chip esp32 --port "$PORT" --baud ${baud} write-flash ${writeFlashArgs}
      '';
    };

  monitor = pkgs.writeShellApplication {
    name = "monitor";
    text = ''
      ${resolvePort}
      echo "port: $PORT @ ${baud} baud"
      echo "exit: Ctrl-A Ctrl-X"
      exec ${picocom} --baud ${baud} "$PORT"
    '';
  };

  # One writeShellApplication per command (strict mode + shellcheck enforced
  # at build time), exposed twice: as `apps` for `nix run`, and as `packages`
  # so they are buildable (`nix build .#flash-mvp`) and CI/checks catch eval
  # or platform regressions.
  scripts = lib.foldl'
    (acc: variant: acc // {
      "flash-${variant}" = {
        drv = mkFlash { inherit variant; };
        description = "Flash the ${variant} firmware over serial (esptool, ${baud} baud)";
      };
      "flash-erase-${variant}" = {
        drv = mkFlash { inherit variant; erase = true; };
        description = "Erase entire flash (wipes NVS), then flash the ${variant} firmware";
      };
    })
    {
      monitor = {
        drv = monitor;
        description = "Serial monitor at ${baud} baud (picocom; exit: Ctrl-A Ctrl-X)";
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
  # Stays writeShellScriptBin: a single exec line has nothing for strict
  # mode or shellcheck to catch.
  devTools = [
    pkgs.picocom
    (pkgs.writeShellScriptBin "esptool" ''exec ${esptool} "$@"'')
  ];
}
