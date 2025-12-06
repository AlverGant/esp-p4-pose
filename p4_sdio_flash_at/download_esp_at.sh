#!/bin/bash
# Download ESP-AT firmware for ESP32-C6 and extract factory.bin

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_DIR="$SCRIPT_DIR/target_binaries/ESP32_C6"
TEMP_DIR="/tmp/esp_at_download"

# ESP-AT version (latest stable for ESP32-C6)
AT_VERSION="4.1.1.0"
AT_URL="https://dl.espressif.com/esp-at/firmwares/esp32c6/ESP32-C6-4MB-AT-V${AT_VERSION}.zip"
ZIP_FILE="ESP32-C6-4MB-AT-V${AT_VERSION}.zip"

echo "==================================="
echo "  ESP-AT Firmware Downloader"
echo "==================================="
echo ""
echo "Downloading ESP-AT v${AT_VERSION} for ESP32-C6..."

# Create temp directory
mkdir -p "$TEMP_DIR"
cd "$TEMP_DIR"

# Download
if command -v wget &> /dev/null; then
    wget -q --show-progress "$AT_URL" -O "$ZIP_FILE"
elif command -v curl &> /dev/null; then
    curl -L -o "$ZIP_FILE" "$AT_URL"
else
    echo "Error: wget or curl required"
    exit 1
fi

echo ""
echo "Extracting..."
unzip -q -o "$ZIP_FILE"

# Find factory binary
FACTORY_BIN=$(find . -name "factory*.bin" -type f | head -1)

if [ -z "$FACTORY_BIN" ]; then
    echo "Error: factory.bin not found in archive"
    exit 1
fi

echo "Found: $FACTORY_BIN"

# Copy to target directory
mkdir -p "$TARGET_DIR"
cp "$FACTORY_BIN" "$TARGET_DIR/factory.bin"

echo ""
echo "Copied to: $TARGET_DIR/factory.bin"
echo ""

# Cleanup
rm -rf "$TEMP_DIR"

echo "Done! Now you can build and flash:"
echo ""
echo "  cd $SCRIPT_DIR"
echo "  idf.py build"
echo "  idf.py -p /dev/ttyUSBx flash monitor"
echo ""
