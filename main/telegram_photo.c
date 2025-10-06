/**
 * Telegram Photo Support
 * Captures frame and sends to C6 for Telegram upload
 */

#include "telegram_photo.h"
#include "app_video_utils.h"  // for swap_rgb565_bytes()
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "coproc_uart.h"
#include "driver/jpeg_encode.h"
#include "driver/uart.h"
#include "bsp/esp-bsp.h"
#include <string.h>

static const char *TAG = "tg_photo";

// Protocol: "PHOTO:<size_bytes>\n" followed by binary JPEG data
#define PHOTO_CMD_PREFIX "PHOTO:"
#define PHOTO_QUALITY 70  // JPEG quality (0-100) - reduced to fit C6 RAM for 960x960

esp_err_t telegram_send_photo_from_frame(const uint16_t *rgb565_frame, int width, int height, const char *caption)
{
    if (!rgb565_frame || width <= 0 || height <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Encoding %dx%d RGB565 frame to JPEG (quality=%d)", width, height, PHOTO_QUALITY);

    esp_err_t ret = ESP_OK;
    jpeg_encoder_handle_t encoder = NULL;
    uint8_t *jpeg_buf = NULL;
    uint16_t *temp_frame = NULL;

    // CRITICAL: The LCD frame buffer has byte-swapped RGB565 (for LCD endianness)
    // but JPEG encoder expects normal RGB565. Create a temporary copy and reverse the swap.
    size_t frame_size = width * height * sizeof(uint16_t);
    temp_frame = heap_caps_malloc(frame_size, MALLOC_CAP_SPIRAM);
    if (!temp_frame) {
        ESP_LOGE(TAG, "Failed to allocate temporary frame buffer");
        return ESP_ERR_NO_MEM;
    }

    // Copy and reverse byte-swap
    memcpy(temp_frame, rgb565_frame, frame_size);

    // Validate frame data before encoding (check for obvious corruption)
    uint16_t *p = temp_frame;
    int suspicious_pixels = 0;
    for (int i = 0; i < 100; i++) {  // Sample first 100 pixels
        if (p[i] == 0xFFFF || p[i] == 0x0000) suspicious_pixels++;
    }
    if (suspicious_pixels > 50) {
        ESP_LOGW(TAG, "⚠️  Frame appears corrupted (%d/100 pixels all-white/black)", suspicious_pixels);
    }

    swap_rgb565_bytes(temp_frame, width * height);  // Reverse the LCD byte-swap
    ESP_LOGI(TAG, "Created un-swapped RGB565 copy for JPEG encoding");

    // Create JPEG encoder
    jpeg_encode_engine_cfg_t encode_eng_cfg = {
        .timeout_ms = 5000,
        .intr_priority = 0,
    };

    ret = jpeg_new_encoder_engine(&encode_eng_cfg, &encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create JPEG encoder: %s", esp_err_to_name(ret));
        free(temp_frame);
        return ret;
    }

    // Allocate output buffer for JPEG (increased for higher quality)
    // For 960x960: 1.5x = ~1.3MB, reduced to 1MB for quality 90 with YUV420
    size_t jpeg_buf_size = width * height;  // 1.0x raw size is sufficient with YUV420 + quality 90
    jpeg_encode_memory_alloc_cfg_t mem_cfg = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };
    size_t allocated_size = 0;
    jpeg_buf = jpeg_alloc_encoder_mem(jpeg_buf_size, &mem_cfg, &allocated_size);

    if (!jpeg_buf) {
        ESP_LOGE(TAG, "Failed to allocate JPEG buffer");
        jpeg_del_encoder_engine(encoder);
        free(temp_frame);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Allocated JPEG buffer: %u bytes", (unsigned)allocated_size);

    // Configure JPEG encoding (matching factory demo settings)
    jpeg_encode_cfg_t enc_cfg = {
        .width = width,
        .height = height,
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV420,  // Match factory demo (better compression)
        .image_quality = PHOTO_QUALITY,
    };

    // Encode using the un-swapped temporary frame
    uint32_t jpeg_len = 0;
    ret = jpeg_encoder_process(
        encoder,
        &enc_cfg,
        (const uint8_t *)temp_frame,  // Use un-swapped frame!
        width * height * 2,  // RGB565 = 2 bytes per pixel
        jpeg_buf,
        allocated_size,
        &jpeg_len
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JPEG encode failed: %s", esp_err_to_name(ret));
        free(jpeg_buf);
        jpeg_del_encoder_engine(encoder);
        free(temp_frame);
        return ret;
    }

    // Validate JPEG output
    bool valid_start = (jpeg_len >= 2 && jpeg_buf[0] == 0xFF && jpeg_buf[1] == 0xD8);
    bool valid_end = (jpeg_len >= 2 && jpeg_buf[jpeg_len-2] == 0xFF && jpeg_buf[jpeg_len-1] == 0xD9);

    ESP_LOGI(TAG, "JPEG encoded: %u bytes (compression: %.1f%%), valid: start=%d end=%d",
             (unsigned)jpeg_len, 100.0f * jpeg_len / (width * height * 2),
             valid_start, valid_end);

    if (!valid_start || !valid_end) {
        ESP_LOGE(TAG, "⚠️ JPEG encoding produced invalid output!");
        free(jpeg_buf);
        jpeg_del_encoder_engine(encoder);
        free(temp_frame);
        return ESP_FAIL;
    }

    // Send photo command to C6 via UART
    // NOTE: coproc_uart_send_line() adds '\n' automatically, so don't add it here!
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "%s%u", PHOTO_CMD_PREFIX, (unsigned)jpeg_len);

    ESP_LOGI(TAG, "Sending photo to C6 (%u bytes)...", (unsigned)jpeg_len);
    ret = coproc_uart_send_line(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send photo command");
        free(jpeg_buf);
        jpeg_del_encoder_engine(encoder);
        free(temp_frame);
        return ret;
    }

    // Wait a bit for C6 to prepare
    vTaskDelay(pdMS_TO_TICKS(100));

    // Send binary JPEG data in chunks - increased for faster 960x960 transfer
    const size_t chunk_size = 512;  // 512 bytes for good balance of speed/reliability
    size_t sent = 0;
    ESP_LOGI(TAG, "Starting UART transfer: %u bytes total", (unsigned)jpeg_len);

    while (sent < jpeg_len) {
        size_t remaining_total = jpeg_len - sent;
        size_t to_send = remaining_total > chunk_size ? chunk_size : remaining_total;
        size_t chunk_sent = 0;

        while (chunk_sent < to_send) {
            int written = uart_write_bytes(UART_NUM_2,
                                           (const char *)(jpeg_buf + sent + chunk_sent),
                                           to_send - chunk_sent);

            if (written < 0) {
                ESP_LOGE(TAG, "UART write failed at offset %u/%u", (unsigned)(sent + chunk_sent),
                         (unsigned)jpeg_len);
                free(jpeg_buf);
                jpeg_del_encoder_engine(encoder);
                free(temp_frame);
                return ESP_FAIL;
            }

            if (written == 0) {
                ESP_LOGW(TAG, "UART write returned 0 bytes at offset=%u, retrying",
                         (unsigned)(sent + chunk_sent));
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            if (written != (int)(to_send - chunk_sent)) {
                ESP_LOGW(TAG,
                         "Partial write: requested=%u, written=%d (chunk progress %u/%u, total offset=%u)",
                         (unsigned)(to_send - chunk_sent),
                         written,
                         (unsigned)(chunk_sent + written),
                         (unsigned)to_send,
                         (unsigned)(sent + chunk_sent));
            }

            chunk_sent += written;
        }

        sent += chunk_sent;

        // Log progress every 10KB
        if (sent % 10240 == 0 || sent == jpeg_len) {
            ESP_LOGI(TAG, "UART TX progress: %u/%u bytes (%.1f%%)",
                     (unsigned)sent, (unsigned)jpeg_len, 100.0f * sent / jpeg_len);
        }

        if (sent < jpeg_len) {
            vTaskDelay(pdMS_TO_TICKS(5));  // Reduced to 5ms with 460800 baud
        }
    }

    // Wait for transmission to complete
    uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "Photo sent successfully (%u bytes)", (unsigned)sent);

    // Cleanup
    free(jpeg_buf);
    jpeg_del_encoder_engine(encoder);
    free(temp_frame);  // Free temporary un-swapped frame

    // Optionally send caption
    if (caption && strlen(caption) > 0) {
        char caption_cmd[256];
        snprintf(caption_cmd, sizeof(caption_cmd), "CAPTION:%s", caption);
        coproc_uart_send_line(caption_cmd);
    }

    return ESP_OK;
}
