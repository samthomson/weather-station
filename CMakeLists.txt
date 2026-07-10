# ESP-IDF 4.4 project. Sources live in src/ and include/; the IDF `main`
# component (main/CMakeLists.txt) points at them. Arduino core + libraries
# arrive as IDF components (components/arduino + EXTRA_COMPONENT_DIRS from
# the nix build; see nix/firmware.nix).
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(weather_station)
