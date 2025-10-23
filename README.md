# ESP32-P4 Pose Detection System

Sistema de detecção de pose humana em tempo real usando ESP32-P4-EYE com integração Telegram para notificações de quedas.

## Visão Geral

Este projeto implementa um sistema de visão computacional embarcado que:
- Detecta poses humanas em tempo real usando YOLO11n-Pose
- Identifica potenciais quedas através de análise de keypoints
- Envia alertas e fotos automaticamente via Telegram
- Utiliza arquitetura dual-chip (ESP32-P4 + ESP32-C6) para processamento distribuído

## Hardware

- **Chip Principal**: ESP32-P4-EYE (processamento de vídeo e inferência)
- **Co-processador**: ESP32-C6 (comunicação WiFi e Telegram)
- **Câmera**: OV2710 (configuração customizada)
- **Display**: LCD integrado para visualização em tempo real
- **Comunicação**: UART entre P4 e C6

## Funcionalidades

### Detecção de Pose
- Modelo: YOLO11n-Pose V2 (QAT - Quantization-Aware Training)
- Resolução de inferência: 640x640 pixels (~3s no ESP32-P4)
- Precisão: mAP50-95 = 0.449 (+4.2% vs V1)
- Detecção de 17 keypoints do esqueleto COCO
- Overlay visual com pose detectada no LCD

### Detecção de Quedas
- Análise automática de orientação corporal
- LED de alerta (GPIO 23)
- Sistema de cooldown configurável para evitar spam

### Integração Telegram
- Envio de mensagens de alerta
- Envio de fotos JPEG com pose detectada
- Configuração via `menuconfig`
- Servidor HTTP local para receber comandos remotos

### Dual-Buffer Architecture
- Captura contínua enquanto inferência executa
- Proteção de buffer durante envio de fotos
- Sincronização via mutex para thread-safety

## Estrutura do Projeto

```
.
├── main/                    # Aplicação principal (ESP32-P4)
│   ├── app_main.c          # Entry point e loop principal
│   ├── app_video.c         # Controle de câmera e display
│   ├── coco_pose.cpp       # Wrapper do modelo YOLO11
│   ├── pose_overlay.cpp    # Desenho de keypoints no LCD
│   ├── fall_notifier.c     # Lógica de detecção de quedas
│   ├── net_telegram.c      # API Telegram
│   ├── telegram_photo.c    # Codificação JPEG para Telegram
│   ├── coproc_uart.c       # Comunicação UART com C6
│   └── c6_flash_bridge.c   # Flash remoto do C6 via P4
│
├── c6_messenger/           # Firmware do co-processador (ESP32-C6)
│   └── main/
│       ├── main.c          # WiFi, HTTP client, UART
│       └── hosted_alert_server.c  # Servidor HTTP para comandos
│
├── components/             # Componentes customizados
├── p4_sdio_flash/         # Utilitário de flash SDIO
├── resources/             # Binários necessários para flash do C6
└── docs/                  # Documentação técnica

```

## Melhorias Recentes

### v1.1 - Otimizações de Performance (2025-01)
- ✅ **Upgrade para YOLO11n-Pose V2**: +4.2% precisão (mAP 0.449 vs 0.431)
- ⚡ **Resolução otimizada**: 640x640 (2x mais rápido que 960x960)
- 🎯 **Thresholds ajustados**: Otimizados para modelo V2 com QAT
- 📊 **Latência de inferência**: Reduzida de ~6s para ~3s

## Como Compilar

### Pré-requisitos
- ESP-IDF v5.x ou superior
- Toolchain para ESP32-P4 e ESP32-C6

### Build do Projeto Principal (P4)
```bash
idf.py set-target esp32p4
idf.py menuconfig  # Configure WiFi, Telegram, etc.
idf.py build
idf.py flash monitor
```

Ou use o script de conveniência:
```bash
./rebuild_and_flash_p4.sh
```

### Build do C6 Messenger
```bash
cd c6_messenger
idf.py set-target esp32c6
idf.py menuconfig
idf.py build
```

Ou use o script:
```bash
./rebuild_and_flash_c6.sh
```

## Configuração

### Telegram Bot
1. Crie um bot via [@BotFather](https://t.me/botfather)
2. Obtenha o token do bot
3. Obtenha seu chat_id via [@userinfobot](https://t.me/userinfobot)
4. Configure via `idf.py menuconfig`:
   - `Component config → Telegram → Enable Telegram`
   - Insira `Bot Token` e `Chat ID`
   - Ajuste cooldown de mensagens (padrão: 60s)

### WiFi
Configure as credenciais via `menuconfig`:
- `Component config → Wi-Fi Configuration`

## Comunicação entre P4 e C6

O sistema usa protocolo UART customizado:
- **P4 → C6**: Comandos JSON (envio de mensagens/fotos)
- **C6 → P4**: Respostas e notificações
- Baudrate: Configurável (padrão: 115200)

### Exemplo de Mensagem
```json
{"cmd":"telegram","msg":"Queda detectada!","photo":"<base64_jpeg>"}
```

## Arquitetura de Software

### Pipeline de Vídeo (P4)
1. Captura de frame da câmera (OV2710)
2. Redimensionamento via PPA para 960x960
3. Inferência YOLO11 em buffer alternado
4. Extração de keypoints e análise de pose
5. Desenho de overlay no LCD
6. Detecção de queda → Trigger de notificação

### Messenger Task (C6)
1. Conecta WiFi
2. Sincroniza tempo via SNTP
3. Escuta comandos UART do P4
4. Envia requisições HTTP para API Telegram
5. Serve HTTP local para comandos remotos

## Flash Bridge
O P4 pode flashar o C6 remotamente via UART usando `c6_flash_bridge`:
```bash
# No P4, ative o modo bridge e use esptool via UART
```

## Troubleshooting

### Fotos Corrompidas no Telegram
- Verifique cache sync em `telegram_photo.c:125`
- Aumente `CONFIG_SPIRAM_FETCH_INSTRUCTIONS`

### UART Communication Issues
- Verifique GPIO pins (TX/RX)
- Confirme baudrate igual em P4 e C6
- Veja logs: `main/coproc_uart.c`

### Performance de Inferência
- Resolução atual: 640x640 (~3s no ESP32-P4)
- Modelo V2 com QAT: mAP50-95 = 0.449
- Para maior qualidade: use 960x960 (~6s, mesma precisão)
- Para tempo real: teste 320x320 (~600ms, -25% precisão)

## Documentos Técnicos

- [SDIO Troubleshooting](docs/sdio_troubleshooting.md)
- [SDIO Packet Mode Analysis](docs/sdio_packet_mode_analysis.md)
- [P4 Eye SDIO Nets](docs/p4_eye_sdio_nets.txt)

## Contribuindo

Pull requests são bem-vindos! Para mudanças maiores, abra uma issue primeiro para discutir o que você gostaria de mudar.

## Licença

[Especifique a licença aqui]

## Créditos

- Modelo YOLO11n-Pose: Ultralytics
- ESP-DL: Espressif Deep Learning Library
- ESP32-P4-EYE: Espressif Systems
