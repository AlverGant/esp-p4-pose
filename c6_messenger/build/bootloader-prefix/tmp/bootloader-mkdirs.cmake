# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/alvaro/Downloads/esp-idf/components/bootloader/subproject"
  "/home/alvaro/Downloads/esp-p4-pose/c6_messenger/build/bootloader"
  "/home/alvaro/Downloads/esp-p4-pose/c6_messenger/build/bootloader-prefix"
  "/home/alvaro/Downloads/esp-p4-pose/c6_messenger/build/bootloader-prefix/tmp"
  "/home/alvaro/Downloads/esp-p4-pose/c6_messenger/build/bootloader-prefix/src/bootloader-stamp"
  "/home/alvaro/Downloads/esp-p4-pose/c6_messenger/build/bootloader-prefix/src"
  "/home/alvaro/Downloads/esp-p4-pose/c6_messenger/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/alvaro/Downloads/esp-p4-pose/c6_messenger/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/alvaro/Downloads/esp-p4-pose/c6_messenger/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
