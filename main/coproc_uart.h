#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t coproc_uart_init(void);
esp_err_t coproc_uart_send_line(const char *line);
// Start RX log task(s) for the configured UART(s)
esp_err_t coproc_uart_start_rx_log(void);
// Probe both UART1(GPIO35/36) and UART2(GPIO45/46), start RX on both with
// prefixes (u1/u2). The first that receives a line becomes the active TX.
esp_err_t coproc_uart_probe_and_start_rx_log(void);
// Force logs via UART2 (TX=45, RX=46) like esp-serial-flasher example
esp_err_t coproc_uart_force_uart2_log(void);
// Wait for C6_READY signal with timeout (0 = wait forever)
esp_err_t coproc_uart_wait_for_c6_ready(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
