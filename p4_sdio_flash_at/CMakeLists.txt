cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET esp32p4)

# Tell esp-serial-flasher to build the SDIO interface
set(SERIAL_FLASHER_INTERFACE_SDIO 1)

# Use local esp-serial-flasher as an extra component
set(EXTRA_COMPONENT_DIRS "/home/alvaro/Downloads/esp-serial-flasher")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(p4_sdio_flash)
