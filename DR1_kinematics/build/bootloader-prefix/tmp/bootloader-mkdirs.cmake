# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/seth/esp/v6.0/esp-idf/components/bootloader/subproject"
  "/home/seth/DR1_kinetics/build/bootloader"
  "/home/seth/DR1_kinetics/build/bootloader-prefix"
  "/home/seth/DR1_kinetics/build/bootloader-prefix/tmp"
  "/home/seth/DR1_kinetics/build/bootloader-prefix/src/bootloader-stamp"
  "/home/seth/DR1_kinetics/build/bootloader-prefix/src"
  "/home/seth/DR1_kinetics/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/seth/DR1_kinetics/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/seth/DR1_kinetics/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
