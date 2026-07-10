# IDF `main` component; the actual sources stay in ../src and ../include.
#
# Variant inputs from the nix build (nix/firmware.nix):
#   WX_VARIANT_DEFINES   cache var (idf.py -D...): ';'-separated NAME=VALUE
#                        compile definitions — every ENABLE_* feature flag
#                        (always defined 0 or 1) plus PMS_MODEL="<model>"
#                        when a PMS sensor is selected.
#   WX_EXTRA_COMPONENTS  environment var: ';'-separated variant-only library
#                        components (e.g. sensirion_core;sps30 or sds011).
#                        Must be an env var, not a cache var: IDF 4.4 collects
#                        REQUIRES in a separate script-mode cmake process
#                        (tools/cmake/scripts/component_get_requirements.cmake)
#                        that inherits the environment but not the cache.
set(WX_VARIANT_DEFINES "" CACHE STRING "Variant feature-flag compile definitions")
set(WX_EXTRA_COMPONENTS $ENV{WX_EXTRA_COMPONENTS})

idf_component_register(
  SRCS
    "../src/main.cpp"
    "../src/config_store.cpp"
    "../src/web_dashboard.cpp"
  INCLUDE_DIRS "../include"
  REQUIRES
    arduino
    mbedtls
    ArduinoJson
    WebSockets
    Adafruit_BusIO
    Adafruit_Sensor
    Adafruit_GFX
    Adafruit_SSD1306
    Adafruit_BME280
    Adafruit_BMP280
    BH1750
    DHT
    micro_ecc
    ${WX_EXTRA_COMPONENTS}
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
  # Parity with the reference build's build_flags.
  CONFIG_ESP32_DISABLE_BROWNOUT_DETECTOR=1
  ${WX_VARIANT_DEFINES}
)
