#!/bin/bash
# Script para build e flash do C6 messenger

cd "$(dirname "$0")/c6_messenger"

echo "====================================="
echo "Building C6 Messenger Firmware..."
echo "====================================="
idf.py build

if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

echo ""
echo "====================================="
echo "Flashing to C6 via P4 SDIO..."
echo "====================================="
echo "Make sure P4 is running p4_sdio_flash!"
echo ""

# Copiar binários para o diretório onde p4_sdio_flash espera
cp build/bootloader/bootloader.bin ../p4_sdio_flash/target_binaries/ESP32_C6/bootloader.bin
cp build/partition_table/partition-table.bin ../p4_sdio_flash/target_binaries/ESP32_C6/partition_table.bin
cp build/c6_messenger.bin ../p4_sdio_flash/target_binaries/ESP32_C6/c6_messenger.bin

echo "Binaries copied. Run p4_sdio_flash on P4 to flash C6."
echo "Or use direct USB flash: idf.py -p /dev/ttyUSB0 flash monitor"
