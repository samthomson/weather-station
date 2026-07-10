# ESP-IDF v4.4.7 for the weather-station firmware build.
#
# Vendored from nixpkgs-esp-dev 48413ee362b4d0709e1a0dff6aba7fd99060335e
# pkgs/esp-idf/default.nix (which pins v4.4.1), adapted for v4.4.7 — the IDF
# release arduino-esp32 2.0.17 was built against. setup-hook.sh is copied
# verbatim from the same upstream revision.
#
# Differences vs upstream:
# - rev/sha256 bumped to v4.4.7.
# - The Python env is a plain python39.withPackages from the pinned nixpkgs
#   (upstream used mach-nix; unmaintained, and its pypi snapshot has no
#   aarch64-darwin wheels). Targeted leaf overrides — never a scope-wide
#   packageOverrides, which rebuilds the pytest world and trips
#   pythonCatchConflictsPhase:
#     * pyparsing 2.3.1 — ldgen requires pyparsing < 2.4 (nixpkgs has 2.4.7);
#       packaging + cryptography rebuilt against it so stock 2.4.7 stays out.
#     * kconfiglib 13.7.1 — IDF pins ==13.7.1 (nixpkgs has 14.1.0); same
#       version the proven reference env shipped.
# - $out/requirements.txt is replaced: idf.py's check_python_dependencies
#   validates every non-comment line against the installed set, so the file
#   lists exactly what the env provides. Dropped relative to upstream:
#     * idf-component-manager (never wanted: build-time downloads).
#     * gdbgui/pygdbmi/python-socketio/jinja2/itsdangerous — only the
#       `idf.py gdbgui` debug frontend needs them; builds never do.
#     * construct relaxed from ==2.10.54 (espcoredump-only, not build).
{ rev ? "v4.4.7"
, sha256 ? "sha256-WamgTXUw1djuIBFucMwVJcYbjTapuIy3pV9gIrZqXR0="
, stdenv
, fetchFromGitHub
, writeText
, python39
}:

let
  src = fetchFromGitHub {
    owner = "espressif";
    repo = "esp-idf";
    inherit rev sha256;
    fetchSubmodules = true;
  };

  # Targeted overrides only — a python39.override packageOverrides scope
  # rebuilds the pytest/packaging world and trips pythonCatchConflictsPhase
  # on pyparsing duplicates. These leaves keep the stock package set intact.
  pyparsing231 = python39.pkgs.buildPythonPackage rec {
    pname = "pyparsing";
    version = "2.3.1";
    src = python39.pkgs.fetchPypi {
      inherit pname version;
      hash = "sha256-ZskmiGJkGrysSpa6dFBuWUyITj9XaQppbSGtghDtZno=";
    };
    doCheck = false;
  };

  # check_python_dependencies.py hard-imports packaging; rebuild it (and
  # cryptography, which propagates it) against pyparsing 2.3.1 so the stock
  # 2.4.7 never enters the env closure.
  packaging209 = python39.pkgs.packaging.override { pyparsing = pyparsing231; };
  # Tests + catchConflicts off: the check env (pytest -> stock packaging ->
  # pyparsing 2.4.7) trips the duplicate scan; the compiled artifact is the
  # same code hydra already tested, only the packaging dep pin differs.
  cryptography348 = (python39.pkgs.cryptography.override { packaging = packaging209; })
    .overridePythonAttrs (old: { doCheck = false; catchConflicts = false; });

  kconfiglib1371 = python39.pkgs.kconfiglib.overridePythonAttrs (_: rec {
    version = "13.7.1";
    src = python39.pkgs.fetchPypi {
      pname = "kconfiglib";
      inherit version;
      hash = "sha256-ou6PsGECRCxFllsFlpRPAsKhUX8JL6IIyjB/P9EqCiI=";
    };
  });

  pythonEnv = python39.withPackages (p: with p; [
    setuptools
    packaging209  # 20.9, same as the reference env
    click
    pyserial
    future
    cryptography348
    pyparsing231  # ldgen requires pyparsing < 2.4
    pyelftools
    urllib3
    kconfiglib1371  # IDF pins ==13.7.1
    # esptool requirements (components/esptool_py/esptool runs from source)
    reedsolo
    bitstring
    ecdsa
    # espcoredump
    construct
  ]);

  # What idf.py's check_python_dependencies must be able to satisfy; every
  # non-comment line is checked with pkg_resources against the env above.
  requirementsFile = writeText "esp-idf-requirements.txt" ''
    setuptools
    packaging
    click>=7.0
    pyserial>=3.3
    future>=0.15.2
    cryptography>=2.1.4
    pyparsing>=2.0.3,<2.4.0
    pyelftools>=0.22
    urllib3<2
    kconfiglib==13.7.1
    reedsolo>=1.5.3,<=1.5.4
    bitstring>=3.1.6,<4
    ecdsa>=0.16.0
    construct>=2.10.54
  '';
in
stdenv.mkDerivation {
  pname = "esp-idf";
  version = rev;

  inherit src;

  setupHook = ./setup-hook.sh;

  # pythonEnv also in nativeBuildInputs so patchShebangs below can resolve
  # `python`: the darwin sandbox has no /usr/bin/env, so `#!/usr/bin/env
  # python` scripts (idf.py itself) must get absolute store-path shebangs.
  # Linux only worked by accident (its sandbox provides /usr/bin/env).
  nativeBuildInputs = [ pythonEnv ];
  propagatedBuildInputs = [ pythonEnv ];

  installPhase = ''
    mkdir -p $out
    cp -r $src/* $out/
    # store modes (0555/0444) come along; patchShebangs needs writable dirs
    chmod -R u+w $out

    # idf.py checks installed packages against $IDF_PATH/requirements.txt;
    # make it check against the same (patched) list the env was built from.
    rm -f $out/requirements.txt
    cp ${requirementsFile} $out/requirements.txt

    # Link the Python environment in so that in shell derivations, the Python
    # setup hook will add the site-packages directory to PYTHONPATH.
    ln -s ${pythonEnv}/lib $out/

    patchShebangs $out/tools
  '';
}
