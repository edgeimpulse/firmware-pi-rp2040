# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/dmitrymaslov/github/rp2040/pico-sdk/tools/pioasm"
  "/Users/dmitrymaslov/github/edgeimpulse-rp2040/firmware-pi-rp2040/build/pioasm"
  "/Users/dmitrymaslov/github/edgeimpulse-rp2040/firmware-pi-rp2040/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm"
  "/Users/dmitrymaslov/github/edgeimpulse-rp2040/firmware-pi-rp2040/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm/tmp"
  "/Users/dmitrymaslov/github/edgeimpulse-rp2040/firmware-pi-rp2040/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm/src/PioasmBuild-stamp"
  "/Users/dmitrymaslov/github/edgeimpulse-rp2040/firmware-pi-rp2040/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm/src"
  "/Users/dmitrymaslov/github/edgeimpulse-rp2040/firmware-pi-rp2040/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm/src/PioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/dmitrymaslov/github/edgeimpulse-rp2040/firmware-pi-rp2040/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm/src/PioasmBuild-stamp/${subDir}")
endforeach()
