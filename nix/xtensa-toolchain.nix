# Espressif prebuilt xtensa-esp32-elf toolchain, gcc 8.4.0, crosstool-NG
# esp-2021r2-patch5 — the pairing IDF v4.4.7 requires.
#
# Vendored from nixpkgs-esp-dev 48413ee362b4d0709e1a0dff6aba7fd99060335e
# pkgs/esp32-toolchain-bin.nix (x86_64-linux only there), extended with an
# aarch64-darwin variant. The linux binaries expect an FHS layout (/lib64
# interpreter), hence the buildFHSUserEnv wrapper; the macos binaries link
# only system libraries and are used as-is.
{ version ? "2021r2-patch5"
, stdenv
, lib
, fetchurl
, makeWrapper
, buildFHSUserEnv ? null
}:

let
  binDist = {
    x86_64-linux = {
      suffix = "linux-amd64";
      hash = "sha256-jvFOBAnCARtB5QSjD3DT41KHMTp5XR8kYq0s0OIFLTc=";
    };
    aarch64-darwin = {
      suffix = "macos-arm64";
      hash = "sha256-sUGJdy1wqWgTiV//dzHQ8v7AyCXPwC4ALW2RoMxLax0=";
    };
  }.${stdenv.system} or (throw "xtensa-esp32-elf: no prebuilt toolchain pinned for ${stdenv.system}");

  fhsEnv = buildFHSUserEnv {
    name = "esp32-toolchain-env";
    targetPkgs = pkgs: with pkgs; [ zlib ];
    runScript = "";
  };
in
stdenv.mkDerivation rec {
  pname = "esp32-toolchain";
  inherit version;

  src = fetchurl {
    url = "https://github.com/espressif/crosstool-NG/releases/download/esp-${version}/xtensa-esp32-elf-gcc8_4_0-esp-${version}-${binDist.suffix}.tar.gz";
    inherit (binDist) hash;
  };

  buildInputs = lib.optionals stdenv.isLinux [ makeWrapper ];

  phases = [ "unpackPhase" "installPhase" ];

  installPhase = ''
    cp -r . $out
  '' + lib.optionalString stdenv.isLinux ''
    for FILE in $(ls $out/bin); do
      FILE_PATH="$out/bin/$FILE"
      if [[ -x $FILE_PATH ]]; then
        mv $FILE_PATH $FILE_PATH-unwrapped
        makeWrapper ${fhsEnv}/bin/esp32-toolchain-env $FILE_PATH --add-flags "$FILE_PATH-unwrapped"
      fi
    done
  '';

  meta = with lib; {
    description = "ESP32 compiler toolchain (Espressif prebuilt, gcc 8.4.0)";
    homepage = "https://github.com/espressif/crosstool-NG";
    license = licenses.gpl3;
  };
}
