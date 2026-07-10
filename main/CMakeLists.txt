# IDF `main` component; the actual sources stay in ../src and ../include.
#
# Variant inputs from the nix build (nix/firmware.nix):
#   WX_VARIANT_DEFINES   cache var (idf.py -D...): ';'-separated NAME=VALUE
#                        compile definitions — every ENABLE_* feature flag
#                        (always defined 0 or 1) plus PMS_MODEL="<model>"
#                        when a PMS sensor is selected.
#   WX_COMPONENTS        environment var: ';'-separated Arduino library
#                        components, always-linked base set plus variant-only
#                        extras — owned by nix/arduino-libs.nix (single
#                        source; nothing is hand-listed here).
#                        Must be an env var, not a cache var: IDF 4.4 collects
#                        REQUIRES in a separate script-mode cmake process
#                        (tools/cmake/scripts/component_get_requirements.cmake)
#                        that inherits the environment but not the cache.
set(WX_VARIANT_DEFINES "" CACHE STRING "Variant feature-flag compile definitions")
set(WX_COMPONENTS $ENV{WX_COMPONENTS})

idf_component_register(
  SRCS
    "../src/main.cpp"
    "../src/config_store.cpp"
    "../src/web_dashboard.cpp"
  INCLUDE_DIRS "../include"
  REQUIRES
    arduino
    mbedtls
    ${WX_COMPONENTS}
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
  ${WX_VARIANT_DEFINES}
)
