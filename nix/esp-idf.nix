# ESP-IDF v4.4.7 for the weather-station firmware build.
#
# Vendored from nixpkgs-esp-dev 48413ee362b4d0709e1a0dff6aba7fd99060335e
# pkgs/esp-idf/default.nix (which pins v4.4.1), adapted for v4.4.7 — the IDF
# release arduino-esp32 2.0.17 was built against. setup-hook.sh is copied
# verbatim from the same upstream revision.
#
# Differences vs upstream:
# - rev/sha256 bumped to v4.4.7.
# - idf-component-manager~=1.2 commented out: not present in the pinned
#   pypi-deps-db, and we never want build-time component downloads anyway.
# - Environment markers stripped/handled: mach-nix skips lines with
#   `; python_version ...` markers, but idf.py 4.4.7's
#   check_python_dependencies evaluates them, so the env must actually
#   contain gdbgui/python-socketio/jinja2/itsdangerous/construct.
# - `packaging` pinned to 20.9 and forced to the wheel provider: 4.4.7 adds a
#   bare `packaging` requirement, and mach-nix's nixpkgs provider would
#   propagate its own pyparsing 2.4.7, breaking ldgen (needs pyparsing < 2.4).
# - $out/requirements.txt is replaced with the patched text so that idf.py's
#   runtime dependency check validates against what the env provides.
{ rev ? "v4.4.7"
, sha256 ? "sha256-WamgTXUw1djuIBFucMwVJcYbjTapuIy3pV9gIrZqXR0="
, stdenv
, fetchFromGitHub
, writeText
, mach-nix
}:

let
  src = fetchFromGitHub {
    owner = "espressif";
    repo = "esp-idf";
    rev = rev;
    sha256 = sha256;
    fetchSubmodules = true;
  };

  requirementsOriginalText = builtins.readFile "${src}/requirements.txt";
  requirementsText = builtins.replaceStrings
    [
      "file://"
      "--only-binary"
      "idf-component-manager~=1.2"
      "importlib_metadata; python_version < \"3.8\""
      "construct==2.10.69; python_version > \"3.11\""
      "pygdbmi<=0.9.0.2; python_version > \"3.10\""
      "; python_version < \"3.11\""
      "; python_version < \"3.12\""
      "packaging"
    ]
    [
      "#file://"
      "#--only-binary"
      "#idf-component-manager~=1.2"
      "#importlib_metadata"
      "#construct==2.10.69"
      "#pygdbmi<=0.9.0.2 (duplicate)"
      ""
      ""
      "packaging==20.9"
    ]
    requirementsOriginalText;

  requirementsFile = writeText "esp-idf-requirements.txt" requirementsText;

  # ldgen (linker script generator) requires pyparsing < 2.4; the nixpkgs
  # provider would propagate its own newer pyparsing regardless of the pin,
  # so force the wheel provider for packaging/pyparsing.
  pythonEnv = mach-nix.mkPython {
    requirements = requirementsText;
    providers = {
      packaging = "wheel";
      pyparsing = "wheel";
    };
  };
in
stdenv.mkDerivation rec {
  pname = "esp-idf";
  version = rev;

  inherit src;

  setupHook = ./setup-hook.sh;

  propagatedBuildInputs = [ pythonEnv.python ];

  installPhase = ''
    mkdir -p $out
    cp -r $src/* $out/

    # idf.py checks installed packages against $IDF_PATH/requirements.txt;
    # make it check against the same (patched) list the env was built from.
    rm -f $out/requirements.txt
    cp ${requirementsFile} $out/requirements.txt

    # Link the Python environment in so that in shell derivations, the Python
    # setup hook will add the site-packages directory to PYTHONPATH.
    ln -s ${pythonEnv}/lib $out/
  '';
}
