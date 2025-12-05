/**
 * Telegram Photo Support
 * Captures frame and sends to C6 for Telegram upload via ESP-AT
 */

#include "telegram_photo.h"
#include "app_video_utils.h"  // for swap_rgb565_bytes()
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"  // for cache sync before JPEG DMA
#include "coproc_uart.h"
#include "driver/jpeg_encode.h"
#include "driver/uart.h"
#include "bsp/esp-bsp.h"
#include "sdkconfig.h"
#include <string.h>

#if CONFIG_AT_TELEGRAM_VIA_C6
#include "at_http.h"
#endif

static const char *TAG = "tg_photo";

#define PHOTO_QUALITY 85  // JPEG quality (0-100) - higher quality reduces artifacts in dark areas
#define CACHE_ALIGN 128

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
    // Buffer MUST be cache-line aligned (128 bytes for ESP32-P4) for esp_cache_msync!
    size_t frame_size = width * height * sizeof(uint16_t);
    size_t aligned_size = (frame_size + 127) & ~127;  // Round up to 128-byte boundary
    temp_frame = heap_caps_aligned_alloc(128, aligned_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
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

    // CRITICAL: Sync CPU cache to PSRAM before JPEG encoder DMA reads!
    // The JPEG encoder uses DMA which reads from PSRAM directly, bypassing CPU cache.
    // Without this sync, DMA may read stale data causing horizontal banding artifacts.
    // Must use aligned_size to sync the entire cache-aligned region!
    esp_err_t cache_ret = esp_cache_msync(temp_frame, aligned_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    if (cache_ret != ESP_OK) {
        ESP_LOGW(TAG, "Cache sync failed: %s", esp_err_to_name(cache_ret));
    }

    ESP_LOGI(TAG, "Created un-swapped RGB565 copy for JPEG encoding (cache synced)");

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
    // Allocate JPEG output buffer in internal DMA-capable RAM to avoid PSRAM DMA issues
    size_t allocated_size = jpeg_buf_size;
    jpeg_buf = heap_caps_aligned_alloc(CACHE_ALIGN, allocated_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

    if (!jpeg_buf) {
        ESP_LOGE(TAG, "Failed to allocate JPEG buffer");
        jpeg_del_encoder_engine(encoder);
        free(temp_frame);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Allocated JPEG buffer: %u bytes (internal DMA)", (unsigned)allocated_size);

    // Invalidate output buffer cache before DMA writes into it
    size_t pre_sync_len = (allocated_size + (CACHE_ALIGN - 1)) & ~(CACHE_ALIGN - 1);
    esp_cache_msync(jpeg_buf, pre_sync_len, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

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
    for (int attempt = 0; attempt < 2; ++attempt) {
        jpeg_len = 0;
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
            ESP_LOGE(TAG, "JPEG encode failed (try %d): %s", attempt + 1, esp_err_to_name(ret));
            if (attempt == 0) {
                // Retry once after clearing buffer and resyncing cache
                memset(jpeg_buf, 0, allocated_size);
                esp_cache_msync(jpeg_buf, allocated_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
                continue;
            }
            free(jpeg_buf);
            jpeg_del_encoder_engine(encoder);
            free(temp_frame);
            return ret;
        }

        // JPEG encoder writes via DMA; invalidate cache so CPU sees fresh data
        size_t jpeg_sync_len = (jpeg_len + (CACHE_ALIGN - 1)) & ~(CACHE_ALIGN - 1);
        if (jpeg_sync_len > allocated_size) {
            jpeg_sync_len = allocated_size;
        }
        esp_err_t jpeg_cache_ret = esp_cache_msync(jpeg_buf, jpeg_sync_len, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        if (jpeg_cache_ret != ESP_OK) {
            ESP_LOGW(TAG, "Cache sync (JPEG output) failed: %s", esp_err_to_name(jpeg_cache_ret));
        }

        // Validate JPEG output
        bool valid_start = (jpeg_len >= 2 && jpeg_buf[0] == 0xFF && jpeg_buf[1] == 0xD8);
        bool valid_end = (jpeg_len >= 2 && jpeg_buf[jpeg_len-2] == 0xFF && jpeg_buf[jpeg_len-1] == 0xD9);

        ESP_LOGI(TAG, "JPEG encoded: %u bytes (compression: %.1f%%), valid: start=%d end=%d (try %d) first8=%02X %02X %02X %02X %02X %02X %02X %02X",
                 (unsigned)jpeg_len, 100.0f * jpeg_len / (width * height * 2),
                 valid_start, valid_end, attempt + 1,
                 jpeg_buf[0], jpeg_buf[1], jpeg_buf[2], jpeg_buf[3],
                 jpeg_buf[4], jpeg_buf[5], jpeg_buf[6], jpeg_buf[7]);

        if (valid_start && valid_end) {
            break;  // success
        }

        if (attempt == 0) {
            ESP_LOGW(TAG, "Invalid JPEG markers, retrying encode...");
            memset(jpeg_buf, 0, allocated_size);
            esp_cache_msync(jpeg_buf, allocated_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
            continue;
        } else {
            ESP_LOGE(TAG, "⚠️ JPEG encoding produced invalid output! len=%u first8=%02X %02X %02X %02X %02X %02X %02X %02X",
                     (unsigned)jpeg_len,
                     jpeg_buf[0], jpeg_buf[1], jpeg_buf[2], jpeg_buf[3],
                     jpeg_buf[4], jpeg_buf[5], jpeg_buf[6], jpeg_buf[7]);
            free(jpeg_buf);
            jpeg_del_encoder_engine(encoder);
            free(temp_frame);
            return ESP_FAIL;
        }
    }

    // Send photo to Telegram via ESP-AT HTTP client on C6
    ESP_LOGI(TAG, "Sending photo to Telegram via ESP-AT (%u bytes)...", (unsigned)jpeg_len);

#if CONFIG_AT_TELEGRAM_VIA_C6
    at_response_t at_ret = at_telegram_send_photo(
        CONFIG_TELEGRAM_BOT_TOKEN,
        CONFIG_TELEGRAM_CHAT_ID,
        jpeg_buf,
        jpeg_len,
        caption);

    // Cleanup
    free(jpeg_buf);
    jpeg_del_encoder_engine(encoder);
    free(temp_frame);

    if (at_ret == AT_OK) {
        ESP_LOGI(TAG, "Photo sent successfully via ESP-AT");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Photo send failed: %s", at_response_to_str(at_ret));
        return ESP_FAIL;
    }
#else
    // Fallback: log error if AT client not enabled
    ESP_LOGE(TAG, "AT_TELEGRAM_VIA_C6 not enabled - cannot send photo");
    free(jpeg_buf);
    jpeg_del_encoder_engine(encoder);
    free(temp_frame);
    return ESP_ERR_NOT_SUPPORTED;
#endif
}
