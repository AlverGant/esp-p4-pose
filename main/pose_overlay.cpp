#include "pose_overlay.h"
#include "coco_pose.hpp"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include <math.h>
#include <algorithm>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG_POSE = "pose";

static COCOPose *s_pose = nullptr;
static std::list<dl::detect::result_t> s_last;
static SemaphoreHandle_t s_last_mutex = nullptr;  // Protects s_last from concurrent access
static int s_width = 0, s_height = 0;
static uint16_t *s_input = nullptr; // big-endian RGB565 snapshot
static uint16_t *s_pending = nullptr; // staging buffer when inference is busy
static int s_pending_w = 0, s_pending_h = 0;
static SemaphoreHandle_t s_sem = nullptr;
static TaskHandle_t s_task = nullptr;
static volatile bool s_ready = false;
static volatile bool s_fall_detected = false;
// Simple temporal smoothing for fall events
static int s_single_fall_streak = 0;
static int s_multi_fall_streak = 0;
static int64_t s_fall_latch_until_us = 0; // keep detection latched for a while
static volatile bool s_inference_busy = false;
static volatile bool s_has_pending = false;
static volatile int s_last_persons = 0;
static volatile int64_t s_last_infer_time_us = 0;
static volatile int s_infer_seq = 0;
static volatile int s_input_rotation_deg = 0; // 0/90/180/270
// Last ROI (normalized): center (cx,cy), radius ~ max dimension fraction

// COCO 17-keypoint skeleton pairs (indices per esp-dl example order)
static const int kpt_pairs[][2] = {
    {5, 7},  {7, 9},  {6, 8},  {8, 10}, {5, 6},
    {5, 11}, {6, 12}, {11, 12},{11, 13},{13, 15},
    {12, 14},{14, 16}, {1, 2},  {0, 1},  {0, 2},
    {1, 3},  {2, 4}
};
static const int kpt_pairs_num = sizeof(kpt_pairs) / sizeof(kpt_pairs[0]);

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static inline uint16_t be16(uint16_t v)
{
    return (uint16_t)((v >> 8) | (v << 8));
}

static void put_px_be(uint16_t *buf, int w, int h, int x, int y, uint16_t color_be)
{
    if ((unsigned)x >= (unsigned)w || (unsigned)y >= (unsigned)h) return;
    buf[y * w + x] = color_be;
}

static void draw_line_be(uint16_t *buf, int w, int h, int x0, int y0, int x1, int y1, uint16_t color_be)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        put_px_be(buf, w, h, x0, y0, color_be);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void pose_task(void *arg)
{
    (void)arg;
    dl::image::img_t img;
    img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565;
    for (;;) {
        // Periodic wait so we can feed the watchdog even when no new frames arrive
        if (xSemaphoreTake(s_sem, pdMS_TO_TICKS(500)) == pdTRUE) {
            if (!s_pose || !s_input || s_width <= 0 || s_height <= 0) continue;

            // Reset watchdog before heavy computation
            esp_task_wdt_reset();

            img.data = s_input;
            img.width = (uint16_t)s_width;
            img.height = (uint16_t)s_height;

            // Run inference with periodic watchdog resets
            int64_t t0 = esp_timer_get_time();
            s_inference_busy = true;
            auto &res = s_pose->run(img);

            // Update s_last with mutex protection to prevent race conditions with draw code
            if (xSemaphoreTake(s_last_mutex, portMAX_DELAY) == pdTRUE) {
                s_last = res;
                xSemaphoreGive(s_last_mutex);
            }

            int64_t t1 = esp_timer_get_time();
            // Log only on person count change or every 10th inference
            if (s_last_persons != (int)res.size() || (s_infer_seq % 10) == 0) {
                ESP_LOGI("POSE", "inference %.1fs, persons=%zu", (t1 - t0) / 1000000.0, res.size());
            }
            s_inference_busy = false;
            s_last_persons = (int)res.size();
            s_last_infer_time_us = t1;
            s_infer_seq = s_infer_seq + 1;


            // If a pending frame was staged during inference, promote it and trigger next run
            if (s_has_pending) {
                s_has_pending = false;
                // If resolution changed, reallocate s_input to pending size
                if (s_width != s_pending_w || s_height != s_pending_h) {
                    if (s_input) heap_caps_free(s_input);
                    s_input = (uint16_t *)heap_caps_calloc(1, s_pending_w * s_pending_h * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
                    if (!s_input) {
                        // Allocation failed; drop pending and continue
                        ESP_LOGE("POSE", "Failed to alloc input for pending frame %dx%d", s_pending_w, s_pending_h);
                    } else {
                        s_width = s_pending_w;
                        s_height = s_pending_h;
                    }
                }
                if (s_input && s_pending && s_width == s_pending_w && s_height == s_pending_h) {
                    memcpy(s_input, s_pending, s_width * s_height * sizeof(uint16_t));
                    if (s_sem) xSemaphoreGive(s_sem);
                }
            }

            // Reset watchdog after inference
            esp_task_wdt_reset();

            // Yield longer to allow other tasks / idle to run and prevent watchdog
            vTaskDelay(pdMS_TO_TICKS(20));
        } else {
            // Timeout waiting for a frame: still reset WDT to avoid spurious trigger if camera stalls
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

esp_err_t pose_overlay_init(void)
{
    if (!s_pose) {
        // Using V2 model with QAT (Quantization-Aware Training)
        // V2 provides +4.2% better mAP50-95 (0.449 vs 0.431) with same performance
        // lazy_load = false forces immediate model load (detects errors early)
        s_pose = new COCOPose(COCOPose::YOLO11N_POSE_S8_V2, false);
        if (!s_pose) {
            ESP_LOGE(TAG_POSE, "Failed to create COCOPose");
            return ESP_ERR_NO_MEM;
        }
        // Increase confidence threshold from default 0.25 to 0.45 to reduce false positives
        // (detections where no person exists). Higher = fewer false positives but may miss some poses.
        s_pose->set_score_thr(0.45);
        ESP_LOGI(TAG_POSE, "Pose initialized (YOLO11n-Pose V2 with QAT, score_thr=0.45)");
    }
    if (!s_sem) {
        s_sem = xSemaphoreCreateBinary();
    }
    if (!s_last_mutex) {
        s_last_mutex = xSemaphoreCreateMutex();
        if (!s_last_mutex) {
            ESP_LOGE(TAG_POSE, "Failed to create s_last mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    if (!s_task) {
        // Pin to core 1 (video stream on core 0), lower priority, bigger stack
        xTaskCreatePinnedToCore(pose_task, "pose_task", 12288, NULL, 2, &s_task, 1);
        // Add task to watchdog
        esp_task_wdt_add(s_task);
    }
    return ESP_OK;
}

esp_err_t pose_overlay_submit(const uint16_t *rgb565_be_buf, int width, int height)
{
    if (!rgb565_be_buf || width <= 0 || height <= 0) return ESP_ERR_INVALID_ARG;
    // If inference is idle, copy into s_input and queue immediately.
    if (!s_inference_busy) {
        if (s_width != width || s_height != height || !s_input) {
            if (s_input) heap_caps_free(s_input);
            s_input = (uint16_t *)heap_caps_calloc(1, width * height * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
            if (!s_input) return ESP_ERR_NO_MEM;
            s_width = width;
            s_height = height;
        }
        memcpy(s_input, rgb565_be_buf, width * height * sizeof(uint16_t));
        if (s_sem) xSemaphoreGive(s_sem);
        s_ready = true;
        return ESP_OK; // enqueued now
    }

    // If inference is busy, stage into pending buffer; do not queue yet
    if (s_pending_w != width || s_pending_h != height || !s_pending) {
        if (s_pending) heap_caps_free(s_pending);
        s_pending = (uint16_t *)heap_caps_calloc(1, width * height * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
        if (!s_pending) return ESP_ERR_NO_MEM;
        s_pending_w = width;
        s_pending_h = height;
    }
    memcpy(s_pending, rgb565_be_buf, width * height * sizeof(uint16_t));
    s_has_pending = true;
    s_ready = true;
    return ESP_ERR_INVALID_STATE; // staged, not enqueued yet
}

esp_err_t pose_overlay_draw(uint16_t *rgb565_be_buf, int width, int height)
{
    if (!rgb565_be_buf || width <= 0 || height <= 0) return ESP_ERR_INVALID_ARG;
    if (!s_ready) return ESP_OK;

    // Make a local copy of s_last with mutex protection to avoid race conditions
    std::list<dl::detect::result_t> local_results;
    if (xSemaphoreTake(s_last_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        local_results = s_last;  // Copy the list
        xSemaphoreGive(s_last_mutex);
    } else {
        // Could not acquire mutex in time, skip drawing this frame
        ESP_LOGD(TAG_POSE, "Could not acquire mutex for drawing, skipping frame");
        return ESP_OK;
    }

    // Overlay skeleton on buffer (buffer is big-endian; write big-endian values)
    const uint16_t col_kpt = be16(rgb565(255, 0, 0));   // red for keypoints

    // Scale coordinates from input image size to display size
    float scale_x = (float)width / (float)s_width;
    float scale_y = (float)height / (float)s_height;

    // Draw all detected persons (up to 3 to maintain performance)
    int person_count = 0;
    const uint16_t colors[] = {
        be16(rgb565(0, 200, 255)), // cyan for person 1
        be16(rgb565(255, 200, 0)), // orange for person 2
        be16(rgb565(255, 0, 255))  // magenta for person 3
    };

    for (const auto &r : local_results) {
        if (person_count >= 3) break; // Limit to 3 persons for performance

        // SAFETY: Check if keypoint vector has enough elements (17 keypoints * 2 coords = 34)
        if (r.keypoint.size() < 34) {
            ESP_LOGW(TAG_POSE, "Person %d: Skipping draw - invalid keypoint data (size=%zu)",
                     person_count + 1, r.keypoint.size());
            person_count++;
            continue;
        }

        uint16_t person_color = colors[person_count % 3];

        // Helper to map keypoints from pose-input rotation back to display
        auto unrotate = [&](int x, int y) {
            int X = x, Y = y;
            int Rw = s_width, Rh = s_height; // pose input dimensions (square)
            switch (s_input_rotation_deg) {
                case 90:  X = Rh - 1 - y; Y = x; break;
                case 180: X = Rw - 1 - x; Y = Rh - 1 - y; break;
                case 270: X = y; Y = Rw - 1 - x; break;
                default: break; // 0 deg
            }
            int sx = (int)(X * scale_x);
            int sy = (int)(Y * scale_y);
            return std::pair<int,int>(sx, sy);
        };

        // Skeleton lines and keypoints for this person
        for (int p = 0; p < kpt_pairs_num; ++p) {
            int a = kpt_pairs[p][0];
            int b = kpt_pairs[p][1];

            int xa = r.keypoint[2 * a];
            int ya = r.keypoint[2 * a + 1];
            int xb = r.keypoint[2 * b];
            int yb = r.keypoint[2 * b + 1];
            auto pa = unrotate(xa, ya);
            auto pb = unrotate(xb, yb);
            int x0 = pa.first, y0 = pa.second;
            int x1 = pb.first, y1 = pb.second;

            // Draw keypoints and connections if both points are detected
            bool point_a_valid = (r.keypoint[2 * a] > 0 && r.keypoint[2 * a + 1] > 0);
            bool point_b_valid = (r.keypoint[2 * b] > 0 && r.keypoint[2 * b + 1] > 0);

            if (point_a_valid && point_b_valid) {
                // Draw connection line
                draw_line_be(rgb565_be_buf, width, height, x0, y0, x1, y1, person_color);
            }

            // Draw individual keypoints as small circles
            if (point_a_valid) {
                // Draw small 3x3 square as keypoint
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        put_px_be(rgb565_be_buf, width, height, x0 + dx, y0 + dy, col_kpt);
                    }
                }
            }
            if (point_b_valid) {
                // Draw small 3x3 square as keypoint
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        put_px_be(rgb565_be_buf, width, height, x1 + dx, y1 + dy, col_kpt);
                    }
                }
            }
        }
        person_count++;
    }

    // Fall detection algorithm
    bool detected_now = false;
    int fall_person_count = 0;
    int valid_detections = 0;
    int potential_falls = 0;

    for (const auto &r : local_results) {
        fall_person_count++;

        // SAFETY: Check if keypoint vector has enough elements (17 keypoints * 2 coords = 34)
        const size_t kp_size = r.keypoint.size();
        if (kp_size < 34) {
            ESP_LOGW(TAG_POSE, "Person %d: SKIPPING - Invalid keypoint data (size=%zu, expected=34)",
                     fall_person_count, kp_size);
            continue;  // Skip this person entirely
        }

        // Additional safety: verify the vector has valid data pointer
        if (r.keypoint.empty() || r.keypoint.data() == nullptr) {
            ESP_LOGW(TAG_POSE, "Person %d: SKIPPING - Null keypoint data pointer", fall_person_count);
            continue;
        }

        // COCO keypoints: nose=0, left_hip=11, right_hip=12, left_shoulder=5, right_shoulder=6
        // Each keypoint has [x,y] so multiply index by 2
        auto unrotate_raw = [&](int x, int y) {
            int X = x, Y = y;
            int Rw = s_width, Rh = s_height; // pose input dims (square)
            switch (s_input_rotation_deg) {
                case 90:  X = Rh - 1 - y; Y = x; break;
                case 180: X = Rw - 1 - x; Y = Rh - 1 - y; break;
                case 270: X = y; Y = Rw - 1 - x; break;
                default: break; // 0 deg
            }
            return std::pair<int,int>(X, Y);
        };

        // Read and unrotate only if valid
        // IMPORTANT: (0,0) is INVALID - it means no detection!
        auto rd_kpt = [&](int idx, int &ox, int &oy, bool &valid) -> void {
            // Always initialize outputs to safe defaults
            valid = false;
            ox = 0;
            oy = 0;

            // Strict bounds checking - calculate required indices first
            const size_t idx_x = static_cast<size_t>(idx * 2);
            const size_t idx_y = static_cast<size_t>(idx * 2 + 1);
            const size_t vec_size = r.keypoint.size();

            // Return early if vector is too small (must have both x and y)
            if (vec_size <= idx_y) {
                ESP_LOGD(TAG_POSE, "rd_kpt: vector too small (size=%zu, need=%zu)", vec_size, idx_y + 1);
                return;
            }

            // Verify data pointer is valid before accessing
            if (r.keypoint.data() == nullptr) {
                ESP_LOGW(TAG_POSE, "rd_kpt: null data pointer!");
                return;
            }

            // Safe array access - indices are now verified to be in bounds
            const int rx = r.keypoint[idx_x];
            const int ry = r.keypoint[idx_y];

            // Keypoint is valid if BOTH coordinates are > 0 (not at origin)
            // AND within bounds
            const int Rw = s_width;
            const int Rh = s_height;

            if (rx > 0 && ry > 0 && rx < Rw && ry < Rh) {
                auto p = unrotate_raw(rx, ry);
                ox = p.first;
                oy = p.second;
                valid = true;
            }
        };

        int nose_x = 0, nose_y = 0; bool nose_valid = false;
        int left_hip_x = 0, left_hip_y = 0; bool lhip_valid = false;
        int right_hip_x = 0, right_hip_y = 0; bool rhip_valid = false;
        int lshoulder_x = 0, lshoulder_y = 0; bool lshoulder_valid = false;
        int rshoulder_x = 0, rshoulder_y = 0; bool rshoulder_valid = false;

        rd_kpt(0, nose_x, nose_y, nose_valid);
        rd_kpt(11, left_hip_x, left_hip_y, lhip_valid);
        rd_kpt(12, right_hip_x, right_hip_y, rhip_valid);
        rd_kpt(5, lshoulder_x, lshoulder_y, lshoulder_valid);
        rd_kpt(6, rshoulder_x, rshoulder_y, rshoulder_valid);

        // Quick scan: count valid keypoints to filter out empty detections (all zero)
        int valid_kpts = 0;
        for (int k = 0; k < 17; ++k) {
            int rx = r.keypoint[2 * k];
            int ry = r.keypoint[2 * k + 1];
            if (rx > 0 && ry > 0 && rx < s_width && ry < s_height) {
                valid_kpts++;
            }
        }
        if (valid_kpts < 4) {
            ESP_LOGI("FALL", "Person %d: skipping - only %d/17 keypoints valid", fall_person_count, valid_kpts);
            continue;
        }

        // Debug log for keypoint detection (only log first person)
        if (fall_person_count == 1) {
            ESP_LOGI("FALL", "Person %d keypoints: nose=%d,%d(%d) lhip=%d,%d(%d) rhip=%d,%d(%d) lsh=%d,%d(%d) rsh=%d,%d(%d)",
                     fall_person_count,
                     nose_x, nose_y, nose_valid,
                     left_hip_x, left_hip_y, lhip_valid,
                     right_hip_x, right_hip_y, rhip_valid,
                     lshoulder_x, lshoulder_y, lshoulder_valid,
                     rshoulder_x, rshoulder_y, rshoulder_valid);
        }

        // Skip fall detection if key points are not properly detected
        // Need at least hips or shoulders to be detected reliably
        bool hips_detected = (lhip_valid && rhip_valid);
        bool shoulders_detected = false;

        // Check shoulders
        int left_shoulder_x = 0, left_shoulder_y = 0; bool lsho_valid = false;
        int right_shoulder_x = 0, right_shoulder_y = 0; bool rsho_valid = false;
        rd_kpt(5, left_shoulder_x, left_shoulder_y, lsho_valid);
        rd_kpt(6, right_shoulder_x, right_shoulder_y, rsho_valid);
        shoulders_detected = (lsho_valid && rsho_valid);

        // NEW: Also accept single-side detection (hip + shoulder on same side)
        bool left_side_detected = (lhip_valid && lsho_valid);
        bool right_side_detected = (rhip_valid && rsho_valid);
        bool single_side_detected = (left_side_detected || right_side_detected);

        // Only log and process if we have reliable keypoints
        if (hips_detected || shoulders_detected || single_side_detected) {
            valid_detections++;
            ESP_LOGI("FALL", "Person %d: nose(%d,%d) lhip(%d,%d) rhip(%d,%d) lsh(%d,%d) rsh(%d,%d) [hips=%d sh=%d Lside=%d Rside=%d]",
                     fall_person_count, nose_x, nose_y,
                     left_hip_x, left_hip_y, right_hip_x, right_hip_y,
                     left_shoulder_x, left_shoulder_y, right_shoulder_x, right_shoulder_y,
                     hips_detected, shoulders_detected, left_side_detected, right_side_detected);
        }

        // SAFETY: Check if keypoint vector has enough elements before bbox calculation
        if (r.keypoint.size() < 34) {
            ESP_LOGW(TAG_POSE, "Person %d: Skipping bbox calculation - invalid keypoint data (size=%zu)",
                     fall_person_count, r.keypoint.size());
            continue;
        }

        // Compute bbox over available keypoints (for drawing/heuristics)
        int bb_min_x = s_width, bb_min_y = s_height, bb_max_x = 0, bb_max_y = 0;
        valid_kpts = 0;
        for (int k = 0; k < 17; ++k) {
            int rx = r.keypoint[2 * k];
            int ry = r.keypoint[2 * k + 1];
            if (rx > 0 && ry > 0) {
                auto p = unrotate_raw(rx, ry);
                int x = p.first, y = p.second;
                if (x < bb_min_x) bb_min_x = x;
                if (y < bb_min_y) bb_min_y = y;
                if (x > bb_max_x) bb_max_x = x;
                if (y > bb_max_y) bb_max_y = y;
                valid_kpts++;
            }
        }

        bool fell_this_person = false;

        // Calculate average hip position if both hips are detected
        if (hips_detected) {
            // If shoulders are also detected, use them for more accurate fall detection
            if (shoulders_detected) {
                int avg_shoulder_y = (left_shoulder_y + right_shoulder_y) / 2;
                int avg_shoulder_x = (left_shoulder_x + right_shoulder_x) / 2;
                int avg_hip_y = (left_hip_y + right_hip_y) / 2;
                int avg_hip_x = (left_hip_x + right_hip_x) / 2;

                int dy = (avg_hip_y - avg_shoulder_y);
                int dx = (avg_hip_x - avg_shoulder_x);

                // Use torso inclination angle w.r.t. vertical to decide
                // Standing: |dy| >> |dx| (angle small)
                // Lying:    |dx| >= |dy| (angle large)
                float v = fabsf((float)dy);
                float h = fabsf((float)dx);
                float angle_deg = atan2f(h, v) * 57.2958f; // 0 = vertical, 90 = horizontal

                // Require a minimum torso length to avoid noise with tiny detections
                float torso_len = sqrtf(h * h + v * v);
                float min_dim = (float)std::min(s_width, s_height);
                float min_torso_len = 0.06f * min_dim; // ~6% of min dimension

                if (torso_len >= min_torso_len) {
                    // Consider potential fall only if torso is near-horizontal
                    if (angle_deg >= 42.0f) {
                        fell_this_person = true;
                        ESP_LOGW("FALL", "Person %d: POTENTIAL FALL! angle=%.1f dx=%d dy=%d len=%.1f",
                                 fall_person_count, angle_deg, dx, dy, torso_len);
                    } else {
                        ESP_LOGI("FALL", "Person %d: Upright/normal posture angle=%.1f dx=%d dy=%d len=%.1f",
                                 fall_person_count, angle_deg, dx, dy, torso_len);
                    }
                } else {
                    ESP_LOGI("FALL", "Person %d: Torso too small/uncertain len=%.1f (min %.1f)",
                             fall_person_count, torso_len, min_torso_len);
                }
            } else if (nose_valid) {
                // Fallback to nose vs hips: use angle to vertical as well
                int avg_hip_y = (left_hip_y + right_hip_y) / 2;
                int avg_hip_x = (left_hip_x + right_hip_x) / 2;
                int dy = (avg_hip_y - nose_y);
                int dx = (avg_hip_x - nose_x);
                float v = fabsf((float)dy);
                float h = fabsf((float)dx);
                float angle_deg = atan2f(h, v) * 57.2958f;
                float len = sqrtf(h * h + v * v);
                float min_dim = (float)std::min(s_width, s_height);
                float min_len = 0.08f * min_dim; // slightly stricter for nose-based fallback
                if (len >= min_len && angle_deg >= 50.0f) {
                    fell_this_person = true;
                    ESP_LOGW("FALL", "Person %d: POTENTIAL FALL! (nose) angle=%.1f len=%.1f", fall_person_count, angle_deg, len);
                } else {
                    ESP_LOGI("FALL", "Person %d: Nose fallback upright angle=%.1f len=%.1f (min %.1f)",
                             fall_person_count, angle_deg, len, min_len);
                }
            }

            // Hips-only horizontal rule: if shoulders missing but both hips present and nearly level horizontally,
            // and hip distance is significant, treat as potential fall
            if (!shoulders_detected && hips_detected) {
                float min_dim = (float)std::min(s_width, s_height);
                int hip_dy = abs(left_hip_y - right_hip_y);
                int hip_dx = abs(left_hip_x - right_hip_x);
                if (hip_dx >= (int)(0.28f * min_dim) && hip_dy <= (int)(0.07f * min_dim)) {
                    fell_this_person = true;
                    ESP_LOGW("FALL", "Person %d: POTENTIAL FALL! (hips-only) hip_dx=%d hip_dy=%d", fall_person_count, hip_dx, hip_dy);
                }
            }
        } else if (single_side_detected) {
            // NEW: Single-side fall detection (hip + shoulder on same side only)
            int side_hip_x = 0, side_hip_y = 0;
            int side_shoulder_x = 0, side_shoulder_y = 0;

            if (right_side_detected) {
                side_hip_x = right_hip_x;
                side_hip_y = right_hip_y;
                side_shoulder_x = right_shoulder_x;
                side_shoulder_y = right_shoulder_y;
            } else {
                side_hip_x = left_hip_x;
                side_hip_y = left_hip_y;
                side_shoulder_x = left_shoulder_x;
                side_shoulder_y = left_shoulder_y;
            }

            int dy = (side_hip_y - side_shoulder_y);
            int dx = (side_hip_x - side_shoulder_x);
            float v = fabsf((float)dy);
            float h = fabsf((float)dx);
            float angle_deg = atan2f(h, v) * 57.2958f; // 0 = vertical, 90 = horizontal
            float torso_len = sqrtf(h * h + v * v);
            float min_dim = (float)std::min(s_width, s_height);
            float min_torso_len = 0.06f * min_dim;

            if (torso_len >= min_torso_len) {
                if (angle_deg >= 45.0f) {  // Slightly stricter for single-side
                    fell_this_person = true;
                    ESP_LOGW("FALL", "Person %d: POTENTIAL FALL! (single-side %s) angle=%.1f len=%.1f",
                             fall_person_count, right_side_detected ? "R" : "L", angle_deg, torso_len);
                } else {
                    ESP_LOGI("FALL", "Person %d: Single-side upright angle=%.1f len=%.1f",
                             fall_person_count, angle_deg, torso_len);
                }
            }
        }

        // Additional bbox-based heuristic to catch partial keypoints when fallen
        // Compute bbox over available keypoints; if it's wide and short, flag potential fall
        if (bb_max_x > bb_min_x && bb_max_y > bb_min_y) {
            float w = (float)(bb_max_x - bb_min_x);
            float h = (float)(bb_max_y - bb_min_y);
            float ar = (h > 1.0f) ? (w / h) : 999.0f;
            float min_dim = (float)std::min(s_width, s_height);
            // Relaxed bbox rule for low-keypoint cases: wide and relatively short
            if (valid_kpts >= 6) {
                // Increased threshold from 4 to 6 to reduce false positives
                if (ar >= 1.5f && h <= 0.35f * min_dim) {
                    fell_this_person = true;
                    ESP_LOGW("FALL", "Person %d: POTENTIAL FALL! (bbox-std) w=%.1f h=%.1f ar=%.2f kpts=%d", fall_person_count, w, h, ar, valid_kpts);
                }
            } else if (valid_kpts >= 5) {
                // Increased threshold to reduce spurious detections (was 2, now 5)
                // Low-keypoints fallback: require even wider and short bbox
                if (ar >= 1.8f && h <= 0.30f * min_dim && w >= 0.45f * min_dim) {
                    fell_this_person = true;
                    ESP_LOGW("FALL", "Person %d: POTENTIAL FALL! (bbox-lowkp) w=%.1f h=%.1f ar=%.2f kpts=%d", fall_person_count, w, h, ar, valid_kpts);
                }
            }

            // Upright veto: if bbox is strongly vertical and tall, avoid false positive
            if (fell_this_person) {
                float w = (float)(bb_max_x - bb_min_x);
                float h = (float)(bb_max_y - bb_min_y);
                if (w > 1.0f && h > 1.0f) {
                    float ar_tall = w / h; // < 1.0 means tall
                    float min_dim2 = (float)std::min(s_width, s_height);
                    if (ar_tall <= 0.9f && (h >= 0.45f * min_dim2)) {
                        // Likely upright tall human; veto this potential fall
                        fell_this_person = false;
                        ESP_LOGI("FALL", "Person %d: VETO fall (tall bbox) w=%.1f h=%.1f ar=%.2f", fall_person_count, w, h, ar_tall);
                    }
                }
            }
        }

        // If this person is considered a potential fall, increment once and draw red bbox
        if (fell_this_person) {
            potential_falls++;
            // Expand bbox slightly for visibility
            int pad_x = (int)(0.03f * (float)s_width);
            int pad_y = (int)(0.03f * (float)s_height);
            int x0r = std::max(0, bb_min_x - pad_x);
            int y0r = std::max(0, bb_min_y - pad_y);
            int x1r = std::min(s_width - 1, bb_max_x + pad_x);
            int y1r = std::min(s_height - 1, bb_max_y + pad_y);
            if (x1r > x0r && y1r > y0r) {
                // Un-rotate bbox corners (define local helper in this scope)
                auto unrotate = [&](int x, int y) {
                    int X = x, Y = y;
                    int Rw = s_width, Rh = s_height; // pose input dimensions (square)
                    switch (s_input_rotation_deg) {
                        case 90:  X = Rh - 1 - y; Y = x; break;
                        case 180: X = Rw - 1 - x; Y = Rh - 1 - y; break;
                        case 270: X = y; Y = Rw - 1 - x; break;
                        default: break; // 0 deg
                    }
                    int sx = (int)(X * scale_x);
                    int sy = (int)(Y * scale_y);
                    return std::pair<int,int>(sx, sy);
                };
                auto p0 = unrotate(x0r, y0r);
                auto p1 = unrotate(x1r, y1r);
                int X0 = p0.first, Y0 = p0.second;
                int X1 = p1.first, Y1 = p1.second;
                uint16_t col_bb = be16(rgb565(255, 0, 0));
                // Draw rectangle (single pixel thickness)
                for (int t = 0; t < 2; ++t) {
                    int ox = t, oy = t;
                    draw_line_be(rgb565_be_buf, width, height, X0+ox, Y0+oy, X1-ox, Y0+oy, col_bb);
                    draw_line_be(rgb565_be_buf, width, height, X1-ox, Y0+oy, X1-ox, Y1-oy, col_bb);
                    draw_line_be(rgb565_be_buf, width, height, X1-ox, Y1-oy, X0+ox, Y1-oy, col_bb);
                    draw_line_be(rgb565_be_buf, width, height, X0+ox, Y1-oy, X0+ox, Y0+oy, col_bb);
                }
            }
        }
    }

    // Improved fall detection logic with temporal smoothing
    if (valid_detections == 1) {
        // Single person: require a short streak of potential falls
        if (potential_falls > 0) {
            s_single_fall_streak++;
        } else {
            s_single_fall_streak = 0;
        }
        if (s_single_fall_streak >= 2) {
            detected_now = true;
            ESP_LOGW("FALL", "FALL DETECTED! Single person (streak=%d)", s_single_fall_streak);
        }
    } else if (valid_detections > 1) {
        // Multiple people: require majority AND a short streak
        float fall_ratio = (valid_detections > 0) ? ((float)potential_falls / (float)valid_detections) : 0.0f;
        if (fall_ratio >= 0.7f) {
            s_multi_fall_streak++;
        } else {
            s_multi_fall_streak = 0;
        }
        if (s_multi_fall_streak >= 3) {
            detected_now = true;
            ESP_LOGW("FALL", "FALL DETECTED! Multiple people %.0f%% (streak=%d)", fall_ratio * 100.0f, s_multi_fall_streak);
        }
    } else {
        // No valid detections: reset streaks
        s_single_fall_streak = 0;
        s_multi_fall_streak = 0;
    }

    // Latch detection for a few seconds so LED stays on and UI can react
    int64_t now_us = esp_timer_get_time();
    if (detected_now) {
        s_fall_latch_until_us = now_us + 3000000; // 3 seconds
    }
    s_fall_detected = detected_now || (now_us < s_fall_latch_until_us);

    return ESP_OK;
}

bool pose_overlay_is_fall_detected(void)
{
    return s_fall_detected;
}

void pose_overlay_get_stats(int *persons, int *age_ms, int *seq)
{
    if (persons) *persons = s_last_persons;
    if (seq) *seq = s_infer_seq;
    if (age_ms) {
        int64_t now = esp_timer_get_time();
        *age_ms = (s_last_infer_time_us > 0) ? (int)((now - s_last_infer_time_us) / 1000) : -1;
    }
}

void pose_overlay_set_input_rotation(int rot_deg)
{
    if (rot_deg == 0 || rot_deg == 90 || rot_deg == 180 || rot_deg == 270) {
        s_input_rotation_deg = rot_deg;
    }
}
