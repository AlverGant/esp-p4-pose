#!/bin/bash
# Script para rebuild e reflash do C6 com correções UART

set -e

echo "========================================="
echo "Building C6 Messenger with UART fixes..."
echo "========================================="
cd "$(dirname "$0")/c6_messenger"

# Clean and rebuild
rm -rf build/
idf.py fullclean
idf.py build

if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

echo ""
echo "========================================="
echo "Copying binaries to p4_sdio_flash..."
echo "========================================="

# Copy binaries
cp build/bootloader/bootloader.bin ../p4_sdio_flash/target_binaries/ESP32_C6/bootloader.bin
cp build/partition_table/partition-table.bin ../p4_sdio_flash/target_binaries/ESP32_C6/partition_table.bin
cp build/c6_messenger.bin ../p4_sdio_flash/target_binaries/ESP32_C6/c6_messenger.bin

echo ""
echo "========================================="
echo "Binaries ready!"
echo "========================================="
echo ""
echo "Now run on P4:"
echo "  cd ~/Downloads/esp-p4-pose/p4_sdio_flash"
echo "  idf.py flash monitor"
echo ""
echo "This will flash the updated C6 firmware with:"
echo "  - Console disabled on UART0"
echo "  - Explicit pin configuration (GPIO16=TX, GPIO17=RX)"
echo "  - UART driver cleanup before reinit"
echo "  - ACK on every received message"
echo ""
