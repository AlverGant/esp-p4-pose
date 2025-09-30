# Diagnóstico da comunicação SDIO entre ESP32-P4 e ESP32-C6

Este documento resume o fluxo atual implementado no projeto e lista pontos de verificação
para investigar falhas na comunicação SDIO usando o kit ESP32-P4-EYE e o firmware
`c6_messenger` fornecido no repositório.

## 1. Fluxo de inicialização no ESP32-P4 (host)

1. `app_main()` chama `coproc_sdio_init()` assim que o sistema termina de inicializar a
   NVS. Se a inicialização SDIO falhar, há um *fallback* para UART.
2. `coproc_sdio_init()` delega para `sdio_host_init()` que:
   - Reseta o ESP32-C6 mantendo `C6_BOOT` (GPIO33) em nível alto para boot normal e
     pulsando `C6_EN` (GPIO9).【F:main/coproc_sdio.c†L38-L56】
   - Aguarda ~8 segundos para o C6 inicializar o *slave* antes de tentar enumerar a
     interface.【F:main/coproc_sdio.c†L48-L52】
   - Configura o host SDMMC em modo 4 bits no *slot* 1 com as GPIOs
     28/27/29/30/31/32.【F:main/coproc_sdio.c†L58-L82】
   - Faz `sdmmc_card_init()` em laço com até 20 tentativas (500 ms de intervalo) antes
     de inicializar o driver ESSL.【F:main/coproc_sdio.c†L106-L141】
   - Cria o *handle* ESSL (`essl_sdio_init_dev` + `essl_init`) e aguarda `ready` do
     *slave* caso a fase anterior tenha sucesso.【F:main/coproc_sdio.c†L116-L139】
3. Caso `sdmmc_card_init()` falhe em todas as tentativas o código agora retorna erro e
   desmonta o host, evitando falsas indicações de sucesso para as camadas superiores.【F:main/coproc_sdio.c†L142-L149】
4. `coproc_sdio_start_rx_log()` pode ser chamado em seguida para criar uma *task* que
   usa `essl_get_packet()` e imprime tudo que chegar do C6.【F:main/coproc_sdio.c†L175-L232】【F:main/coproc_sdio.c†L278-L307】

## 2. Fluxo de inicialização no ESP32-C6 (slave)

1. `coproc_sdio_slave_init()` usa os pinos fixos da controladora SDIO do C6 (CLK=19,
   CMD=18, D0..D3=20..23) e inicializa o driver `sdio_slave` em modo de pacotes com
   *queue* de 8 buffers de 256 bytes cada.【F:c6_messenger/main/coproc_sdio_slave.c†L50-L115】
2. Os buffers são registrados e carregados na fila de recepção antes de iniciar o
   periférico, em linha com o exemplo oficial `sdio/slave`.【F:c6_messenger/main/coproc_sdio_slave.c†L89-L115】
3. O código limpa os registradores compartilhados e habilita todas as interrupções de
   host (`sdio_slave_set_host_intena`) antes de chamar `sdio_slave_start()`.【F:c6_messenger/main/coproc_sdio_slave.c†L117-L151】
4. Há duas *tasks*: uma para transmitir (`sdio_slave_transmit`) consumindo mensagens de
   uma fila, e outra para receber (`sdio_slave_recv`) que valida o *checksum* e chama a
   *callback* da aplicação, reenfilando o buffer após cada uso.【F:c6_messenger/main/coproc_sdio_slave.c†L23-L48】【F:c6_messenger/main/coproc_sdio_slave.c†L58-L83】【F:c6_messenger/main/coproc_sdio_slave.c†L153-L198】

## 3. Pontos críticos e verificações

1. **Alimentação e pull-ups** – O barramento SDIO 4-bit precisa de resistores de *pull-up*
   estáveis em CMD e D0..D3. O ESP32-P4-EYE já possui resistores discretos, então não
   é necessário habilitar *pull-ups* internos. Porém, qualquer cabo extenso ou montagem
   fora da *motherboard* pode degradar o sinal.
2. **Estado de boot do C6** – `sdio_host_init()` assume que o C6 deve iniciar em modo
   de *flash* (BOOT=1). Se o firmware do C6 precisar entrar em modo *download*, o host
   nunca verá a função SDIO e `sdmmc_card_init()` falhará após as 20 tentativas.
3. **Tempo de *boot*** – O atraso de 8 s foi escolhido empiricamente. Logs indicam que
   o C6 termina a inicialização do `sdio_slave` em ~5 s. Se o firmware customizado do C6
   demorar mais, aumente o timeout antes do laço ou o número de tentativas.
4. **Propagação de erro** – Antes desta alteração a função retornava sucesso mesmo se
   `sdmmc_card_init()` falhasse, o que fazia `coproc_sdio_send_line()` receber um
   *handle* nulo e responder com `ESP_ERR_INVALID_ARG`. Verifique os logs após esta
   correção: a mensagem “SDIO card init failed…” agora indica claramente que o host
   não detectou o *slave* e evita tentativas subsequentes sobre um *handle* inválido.
5. **Interrupções do *slave*** – O ESSL depende que o C6 gere interrupções através de
   `sdio_slave_send_queue`/`sdio_slave_transmit`. Confirme que o firmware do C6 chama
   `coproc_sdio_slave_send_line()` em algum momento ou, ao depurar, force o envio de uma
   mensagem após o *boot* para liberar o host do `essl_wait_for_ready()`.
6. **Fallback UART** – `app_main()` ainda mantém a UART como canal de contingência.
   Utilize `coproc_uart_send_line()` para validar se o firmware do C6 está ativo mesmo
   quando o canal SDIO falha.【F:main/app_main.c†L276-L304】【F:main/coproc_uart.c†L1-L120】

## 4. Sequência sugerida de diagnóstico

1. Capture os logs simultâneos de P4 (via USB) e C6 (via UART2 com *bridge*) para
   confirmar que o *slave* subiu antes que o host termine o laço de `sdmmc_card_init()`.
2. Caso o host reporte “SDIO card init failed…”, verifique se o C6 manteve `BOOT=1` e
   se não há outro firmware ocupando o periférico SDIO (por exemplo, modo `esp-serial-flasher`).
3. No C6, habilite logs adicionais dentro de `sdio_slave_rx_task` para confirmar se
   `sdio_slave_recv()` está retornando `ESP_OK` – ausência total de interrupções indica
   problemas de cabeamento ou clock.
4. Use o projeto auxiliar `p4_sdio_flash` para validar rapidamente a camada física.
   Se o `esp_loader_connect()` falhar também, o problema é elétrico/clock. Se funcionar,
   foque na troca de mensagens ESSL.

Seguindo este roteiro é possível isolar se a falha está na configuração do host, no
firmware *slave* ou no enlace físico do SDIO.
