# ESP32-P4 Pose Detection System

Real-time human pose detection system using ESP32-P4-EYE with Telegram integration for fall notifications.

## Overview

This project implements an embedded computer vision system that:
- Detects human poses in real time using YOLO11n-Pose
- Identifies potential falls through keypoint analysis
- Sends alerts and photos automatically via Telegram
- Uses a dual-chip architecture (ESP32-P4 + ESP32-C6) for distributed processing

## Hardware

- **Main Chip**: ESP32-P4-EYE (video processing and inference)
- **Co-processor**: ESP32-C6 (WiFi and Telegram communication)
- **Camera**: OV2710 (custom configuration)
- **Display**: Built-in LCD for real-time visualization
- **Communication**: UART between P4 and C6

## Features

### Pose Detection
- Model: YOLO11n-Pose V2 (QAT - Quantization-Aware Training)
- Inference resolution: 640x640 pixels (~3s on ESP32-P4)
- Accuracy: mAP50-95 = 0.449 (+4.2% vs V1)
- Detection of 17 COCO skeleton keypoints
- Visual overlay with detected pose on LCD

### Fall Detection
- Automatic body orientation analysis
- Alert LED (GPIO 23)
- Configurable cooldown system to prevent spam

### Telegram Integration
- Alert message delivery
- JPEG photo delivery with detected pose
- Configuration via `menuconfig`
- Local HTTP server for receiving remote commands

### Dual-Buffer Architecture
- Continuous capture while inference runs
- Buffer protection during photo transmission
- Synchronization via mutex for thread-safety

## Project Structure

```
.
├── main/                    # Main application (ESP32-P4)
│   ├── app_main.c          # Entry point and main loop
│   ├── app_video.c         # Camera and display control
│   ├── coco_pose.cpp       # YOLO11 model wrapper
│   ├── pose_overlay.cpp    # Keypoint drawing on LCD
│   ├── fall_notifier.c     # Fall detection logic
│   ├── net_telegram.c      # Telegram API
│   ├── telegram_photo.c    # JPEG encoding for Telegram
│   ├── coproc_uart.c       # UART communication with C6
│   └── c6_flash_bridge.c   # Remote C6 flashing via P4
│
├── c6_messenger/           # Co-processor firmware (ESP32-C6)
│   └── main/
│       ├── main.c          # WiFi, HTTP client, UART
│       └── hosted_alert_server.c  # HTTP server for commands
│
├── components/             # Custom components
├── p4_sdio_flash/         # SDIO flash utility
├── resources/             # Required binaries for C6 flashing
└── docs/                  # Technical documentation

```

## Recent Improvements

### v1.1 - Performance Optimizations (2025-01)
- ✅ **Upgrade to YOLO11n-Pose V2**: +4.2% accuracy (mAP 0.449 vs 0.431)
- ⚡ **Optimized resolution**: 640x640 (2x faster than 960x960)
- 🎯 **Adjusted thresholds**: Optimized for V2 model with QAT
- 📊 **Inference latency**: Reduced from ~6s to ~3s

## How to Build

### Prerequisites
- ESP-IDF v5.x or higher
- Toolchain for ESP32-P4 and ESP32-C6

### Main Project Build (P4)
```bash
idf.py set-target esp32p4
idf.py menuconfig  # Configure WiFi, Telegram, etc.
idf.py build
idf.py flash monitor
```

Or use the convenience script:
```bash
./rebuild_and_flash_p4.sh
```

### C6 Messenger Build
```bash
cd c6_messenger
idf.py set-target esp32c6
idf.py menuconfig
idf.py build
```

Or use the script:
```bash
./rebuild_and_flash_c6.sh
```

## Configuration

### Telegram Bot
1. Create a bot via [@BotFather](https://t.me/botfather)
2. Get the bot token
3. Get your chat_id via [@userinfobot](https://t.me/userinfobot)
4. Configure via `idf.py menuconfig`:
   - `Component config → Telegram → Enable Telegram`
   - Enter `Bot Token` and `Chat ID`
   - Adjust message cooldown (default: 60s)

### WiFi
Configure credentials via `menuconfig`:
- `Component config → Wi-Fi Configuration`

## P4 and C6 Communication

The system uses a custom UART protocol:
- **P4 → C6**: JSON commands (message/photo delivery)
- **C6 → P4**: Responses and notifications
- Baud rate: Configurable (default: 115200)

### Message Example
```json
{"cmd":"telegram","msg":"Fall detected!","photo":"<base64_jpeg>"}
```

## Software Architecture

### Video Pipeline (P4)
1. Frame capture from camera (OV2710)
2. Resize via PPA to 960x960
3. YOLO11 inference on alternating buffer
4. Keypoint extraction and pose analysis
5. Overlay drawing on LCD
6. Fall detection → Notification trigger

### Messenger Task (C6)
1. Connects to WiFi
2. Syncs time via SNTP
3. Listens for UART commands from P4
4. Sends HTTP requests to Telegram API
5. Serves local HTTP for remote commands

## Flash Bridge
The P4 can flash the C6 remotely via UART using `c6_flash_bridge`:
```bash
# On P4, enable bridge mode and use esptool via UART
```

## Troubleshooting

### Corrupted Photos on Telegram
- Check cache sync in `telegram_photo.c:125`
- Increase `CONFIG_SPIRAM_FETCH_INSTRUCTIONS`

### UART Communication Issues
- Check GPIO pins (TX/RX)
- Confirm matching baud rate on P4 and C6
- See logs: `main/coproc_uart.c`

### Inference Performance
- Current resolution: 640x640 (~3s on ESP32-P4)
- V2 model with QAT: mAP50-95 = 0.449
- For higher quality: use 960x960 (~6s, same accuracy)
- For real-time: try 320x320 (~600ms, -25% accuracy)

## Technical Documents

- [SDIO Troubleshooting](docs/sdio_troubleshooting.md)
- [SDIO Packet Mode Analysis](docs/sdio_packet_mode_analysis.md)
- [P4 Eye SDIO Nets](docs/p4_eye_sdio_nets.txt)

## Contributing

Pull requests are welcome! For major changes, please open an issue first to discuss what you would like to change.

## License

[Specify license here]

## Credits

- YOLO11n-Pose model: Ultralytics
- ESP-DL: Espressif Deep Learning Library
- ESP32-P4-EYE: Espressif Systems
