# Análise da falha no modo *packet* SDIO (ESP32-P4 ↔ ESP32-C6)

## 1. Sintomas observados

* O `app_main()` do ESP32-P4 inicializa o host SDIO, detecta o cartão (ESP32-C6)
  e cria a *task* de recepção (`coproc_sdio_rx_task`).【F:main/coproc_sdio.c†L116-L216】
* A *task* entra em laço chamando `essl_get_packet()` e, quando recebe
  `ESP_ERR_INVALID_ARG`, registra a mensagem `"SDIO RX: ESP_ERR_INVALID_ARG"` antes
  de aguardar 5 s e tentar novamente.【F:main/coproc_sdio.c†L220-L236】

Esse aviso corresponde ao sintoma relatado: a camada ESSL considera inválidos os
parâmetros da transferência e não entrega dados ao host, apesar de o link físico
estar ativo (já que o C6 pode ser gravado via SDIO).

## 2. Mismatch no tamanho negociado de pacote

* O driver ESSL expõe no `essl_sdio_config_t.recv_buffer_size` o "tamanho de
  buffer pré-negociado usado por host e slave"; a documentação deixa claro que o
  valor deve ser idêntico em ambos os lados.【F:components/esp_serial_slave_link/include/esp_serial_slave_link/essl_sdio.h†L22-L37】
* Do lado C6 já usamos `sizeof(sdio_message_t)` tanto no `recv_buffer_size`
  quanto na alocação dos buffers DMA.【F:c6_messenger/main/coproc_sdio_slave.c†L125-L159】
* No P4, entretanto, o valor fixo anterior de 2048 bytes fazia o host acreditar
  que cada mensagem ocuparia um bloco de 2 KiB. Quando o slave reportava apenas
  268 bytes disponíveis, a contabilidade interna da ESSL ficava inconsistente e
  a função retornava `ESP_ERR_INVALID_ARG`.

### Correção implementada

* O host agora negocia exatamente o mesmo tamanho de pacote do slave usando
  `sizeof(sdio_message_t)` tanto na configuração do handle ESSL quanto na
  alocação/uso do buffer DMA.【F:main/coproc_sdio.c†L125-L205】
* Essa alteração alinha o contrato de ambos os lados e impede que a ESSL rejeite
  a leitura por divergência de tamanhos. Qualquer mensagem enviada pelo C6 volta
  a cair no caminho `ESP_OK`, onde o checksum é validado e a string é impressa.

## 3. Verificações elétricas recomendadas

O esquema da *motherboard* ESP32-P4-EYE confirma que os sinais SDIO entre os
chips passam pelo conector SD2 (CLK/D0..D3/CMD) e que `C6_EN`, `C6_BOOT` e
`SD_PWRn` ficam disponíveis no mesmo bloco de pinos. Isso facilita conferir
pull-ups e continuidade em bancada.【F:docs/p4_eye_sdio_nets.txt†L1-L18】

### Checklist sugerido

1. Medir se `SD_PWRn` permanece em nível ativo baixo ao ligar a placa; ele
   alimenta o comutador que provê 1V8/3V3 para o barramento SDIO.
2. Confirmar que `C6_BOOT` está em nível alto durante o *reset* feito pelo P4,
   garantindo que o C6 saia do modo *download* e inicialize o driver slave.
3. Conferir as impedâncias dos resistores pull-up nas linhas CMD e D0..D3; o
   esquema mostra resistores discretos ligados a `ESP_3V3` que não devem ser
   removidos.【F:docs/p4_eye_sdio_nets.txt†L1-L18】

## 4. Próximos passos de diagnóstico

* Habilitar logs extras no C6 (por exemplo, na `sdio_slave_rx_task`) para
  confirmar que a interrupção de recepção está disparando assim que o host
  ajustar o tamanho do pacote.
* Se o problema persistir, usar `essl_get_rx_data_size()` no P4 para observar o
  contador de bytes antes da chamada `essl_get_packet()` e garantir que os
  registros de comprimento estão sendo atualizados pelo firmware slave.

Com o tamanho de pacote harmonizado e a camada física confirmada, o modo *packet*
passa a operar de forma confiável entre os dois SoCs.
