#!/bin/bash
set -e  # Exit on error

echo "======================================"
echo "Rebuild & Flash P4 Main Application"
echo "======================================"
echo ""

echo "[1/2] Rebuilding P4 application..."
cd /home/alvaro/Downloads/esp-p4-pose
idf.py fullclean
idf.py build
echo "✓ P4 build complete"
echo ""

echo "[2/2] Flashing P4..."
echo "Connect P4 to USB and press ENTER"
read -r

idf.py flash monitor

echo ""
echo "======================================"
echo "Done! P4 should now be running."
echo "Check for 'C6[u2]: C6_ACK' in logs"
echo "======================================"
