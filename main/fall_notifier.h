#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>


void start_fall_notifier_task(void);

/**
 * @brief Send a fall-related event through every enabled backend.
 *
 * @param source_human  Texto utilizado para o Telegram (ex.: "Queda detectada!").
 * @param persons       Número de pessoas detectadas na última inferência.
 * @param age_ms        Idade da inferência em milissegundos.
 * @param seq           Número de sequência da inferência.
 * @param urgent        Quando true ignora o cooldown interno (usado pelo botão P1).
 *
 * @return ESP_OK se pelo menos um backend confirmou envio, erro caso contrário.
 */
esp_err_t fall_notifier_send_event(const char *source_human,
                                   int persons,
                                   int age_ms,
                                   int seq,
                                   bool urgent);

#ifdef __cplusplus
}
#endif

