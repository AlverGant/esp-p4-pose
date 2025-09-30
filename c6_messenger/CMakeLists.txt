cmake_minimum_required(VERSION 3.16)

# Permite um arquivo extra opcional para segredos (não versionado)
set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.secrets")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(c6_messenger)
