# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/dmitrymaslov/github/rp2040/pico-sdk/tools/elf2uf2"
  "/Users/dmitrymaslov/github/edgeimpulse-rp2040/firmware-pi-rp2040/build/elf2uf2"
  "/Users/dmitrymaslov/github/edgeimpulse-rp2040/firmware-pi-rp2040/build/elf2uf2"
  "/Users/dmitrymaslov/github/edgeimpulse-rp2040/firmware-pi-rp2040/build/elf2uf2/tmp"
  "/Users/dmitrymaslov/github/edgeimpulse-rp2040/firmware-pi-rp2040/build/elf2uf2/src/ELF2UF2Build-stamp"
  "/Users/dmitrymaslov/github/edgeimpulse-rp2040/firmware-pi-rp2040/build/elf2uf2/src"
  "/Users/dmitrymaslov/github/edgeimpulse-rp2040/firmware-pi-rp2040/build/elf2uf2/src/ELF2UF2Build-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/dmitrymaslov/github/edgeimpulse-rp2040/firmware-pi-rp2040/build/elf2uf2/src/ELF2UF2Build-stamp/${subDir}")
endforeach()
