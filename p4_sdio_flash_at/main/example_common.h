#pragma once

#include <stdint.h>
#include "esp_loader.h"

typedef struct {
    const uint8_t *data;
    uint32_t size;
    const uint8_t *md5;
    uint32_t addr;
} example_binary_t;

// ESP-AT requires 6 binaries
typedef struct {
    example_binary_t bootloader;    // 0x0
    example_binary_t partition;     // 0x8000
    example_binary_t ota_data;      // 0xd000
    example_binary_t at_customize;  // 0x1e000
    example_binary_t mfg_nvs;       // 0x1f000
    example_binary_t app;           // 0x60000
} esp_at_binaries_t;

void get_esp_at_binaries(esp_at_binaries_t *bins);
esp_loader_error_t connect_to_target(uint32_t higher_transmission_rate);
esp_loader_error_t flash_binary(const uint8_t *bin, size_t size, size_t address);
