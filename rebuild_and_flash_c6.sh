#!/bin/bash
set -e  # Exit on error

echo "======================================"
echo "Rebuild & Flash C6 via P4 SDIO"
echo "======================================"
echo ""

# Step 1: Rebuild C6 messenger
echo "[1/4] Rebuilding C6 messenger..."
cd /home/alvaro/Downloads/esp-p4-pose/c6_messenger
idf.py fullclean
idf.py build
echo "✓ C6 build complete"
echo ""

# Step 2: Copy binaries to P4 flash bridge
echo "[2/4] Copying C6 binaries to P4 flash bridge..."
./copy_to_c6.sh
echo "✓ Binaries copied"
echo ""

# Step 3: Rebuild P4 SDIO flasher
echo "[3/4] Rebuilding P4 SDIO flasher..."
cd /home/alvaro/Downloads/esp-p4-pose/p4_sdio_flash
idf.py fullclean
idf.py build
echo "✓ P4 SDIO flasher build complete"
echo ""

# Step 4: Flash C6 via P4
echo "[4/4] Flashing C6 via P4 SDIO..."
echo "Connect P4 to USB and press ENTER"
read -r

idf.py flash monitor

echo ""
echo "======================================"
echo "Done! C6 should now be running."
echo "======================================"
