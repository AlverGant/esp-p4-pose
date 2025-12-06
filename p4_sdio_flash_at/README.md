# P4 SDIO Flash - ESP-AT Firmware

Flash ESP-AT firmware to ESP32-C6 via SDIO from ESP32-P4 on the P4-Eye board.

## Quick Start

### 1. Download ESP-AT firmware

```bash
./download_esp_at.sh
```

Or manually:
1. Download from: https://dl.espressif.com/esp-at/firmwares/esp32c6/
2. Extract and copy `factory/factory_xxx.bin` to `target_binaries/ESP32_C6/factory.bin`

### 2. Build

```bash
idf.py build
```

### 3. Flash to P4

Connect P4-Eye via USB and flash:

```bash
idf.py -p /dev/ttyUSBx flash monitor
```

The P4 will then flash the ESP-AT firmware to the C6 via SDIO.

## ESP-AT UART Configuration

The default ESP-AT firmware uses:
- **TX**: GPIO 7
- **RX**: GPIO 6
- **Baud**: 115200

On the P4-Eye, the P4 connects to C6 via:
- P4 GPIO35 (TX) -> C6 RX
- P4 GPIO36 (RX) <- C6 TX

**Note**: The default ESP-AT pins (GPIO 6/7) may differ from the P4-Eye hardware connections (GPIO 16/17). You may need to compile a custom ESP-AT firmware with modified UART pins if communication doesn't work.

## Troubleshooting

If ESP-AT doesn't respond:
1. Check if C6 is booting (look for output in monitor)
2. Verify UART connections
3. Consider building custom ESP-AT with correct pin configuration

## Building Custom ESP-AT

To build ESP-AT with custom UART pins for P4-Eye:

```bash
cd /path/to/esp-at
./build.py install  # Select ESP32C6-4MB
./build.py menuconfig
# Component config -> AT -> AT UART Settings
# Change TX to 16, RX to 17
./build.py build
```

Then copy `build/factory/factory_xxx.bin` to `target_binaries/ESP32_C6/factory.bin`.
