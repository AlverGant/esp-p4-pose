#!/bin/bash
set -e  # Exit on error

# Paths
SRC_DIR="/home/alvaro/Downloads/esp-p4-pose/c6_messenger/build"
DST_DIR="/home/alvaro/Downloads/esp-p4-pose/p4_sdio_flash/target_binaries/ESP32_C6"

# Check if build directory exists
if [ ! -d "$SRC_DIR" ]; then
    echo "Error: Build directory not found: $SRC_DIR"
    echo "Run 'idf.py build' first"
    exit 1
fi

# Copy binaries
echo "Copying C6 binaries to P4 flash bridge..."
cp -v "$SRC_DIR/bootloader/bootloader.bin" "$DST_DIR/"
cp -v "$SRC_DIR/c6_messenger.bin" "$DST_DIR/"
cp -v "$SRC_DIR/partition_table/partition-table.bin" "$DST_DIR/"

echo "✓ C6 binaries copied successfully!"
echo "  - bootloader.bin"
echo "  - c6_messenger.bin"
echo "  - partition-table.bin"
echo ""
echo "Next: Flash to P4 with 'cd ../p4_sdio_flash && idf.py flash'"
