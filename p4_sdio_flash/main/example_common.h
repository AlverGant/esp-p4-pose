#pragma once

#include <stdint.h>
#include "esp_loader.h"

typedef struct {
    struct { const uint8_t *data; uint32_t size; const uint8_t *md5; uint32_t addr; } boot;
    struct { const uint8_t *data; uint32_t size; const uint8_t *md5; uint32_t addr; } part;
    struct { const uint8_t *data; uint32_t size; const uint8_t *md5; uint32_t addr; } app;
} example_binaries_t;

typedef struct { struct { const uint8_t *data; uint32_t size; } ram_app; } example_ram_app_binary_t;

void get_example_binaries(target_chip_t target, example_binaries_t *bins);
esp_loader_error_t connect_to_target(uint32_t higher_transmission_rate);
esp_loader_error_t flash_binary(const uint8_t *bin, size_t size, size_t address);

