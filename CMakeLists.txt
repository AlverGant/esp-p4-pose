cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(esp_p4_eye_preview)

# Disable maybe-uninitialized warning with -O2 (false positive in esp_lvgl_port)
# Must come AFTER project() to override ESP-IDF defaults
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=maybe-uninitialized" APPEND)

