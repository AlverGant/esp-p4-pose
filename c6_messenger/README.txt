ESP32-C6 Messenger (UART -> Wi‑Fi -> Telegram)

Target: ESP32-C6 (module on ESP32‑P4‑EYE)

Usage
- Select target: idf.py set-target esp32c6
- Configure: idf.py menuconfig
  - Wi‑Fi/Telegram credentials
  - UART baud
- Flash/monitor: idf.py -p <PORT> flash monitor

Wiring
- P4 TX (GPIO35) -> C6 RX GPIO16 (TP46)
- Optional: P4 RX (GPIO36) <- C6 TX GPIO17 (TP47)
- GND common (TP48)

Notes
- Firmware fixed to use UART0 TX=GPIO17, RX=GPIO16.

Protocol
- One line per event, terminated with '\n'.
- Lines starting with "FALL" trigger a Telegram message.
