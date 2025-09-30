ESP32-C6 Messenger (UART -> Wi‑Fi -> Telegram)

Target: ESP32-C6 (module on ESP32‑P4‑EYE)

Usage
- Select target: idf.py set-target esp32c6
- Configure: idf.py menuconfig
  - Wi‑Fi/Telegram credentials
  - UART baud
- (Opcional) configure o servidor HTTP de alertas (porta e tamanho máximo)
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
- Alertas também podem ser enviados via HTTP (ESP-Hosted):
  - Endpoint: POST http://<ip-do-c6>:<porta>/alert
  - Corpo: texto simples com a linha (ex: "FALL persons=1 age_ms=200 seq=42")
  - Respostas:
      * 200/JSON `{ "status": "sent" }` quando enviado ao Telegram
      * 429/JSON `{ "status": "cooldown" }` se ainda estiver no intervalo de cooldown
      * 204 quando o corpo não contém um alerta reconhecido
