/*
 * SPDX-License-Identifier: CC0-1.0
 */

#include "camera_tracker.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/ledc.h"
#include "esp_camera.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "serial_comm.h"
#include "motor_control.h"

#if CONFIG_SPIRAM
#include "esp_psram.h"
#endif

static const char *TAG = "cam_track";

#define CAM_PIN_PWDN   -1
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK   15
#define CAM_PIN_SIOD   4
#define CAM_PIN_SIOC   5
#define CAM_PIN_D7     16
#define CAM_PIN_D6     17
#define CAM_PIN_D5     18
#define CAM_PIN_D4     12
#define CAM_PIN_D3     10
#define CAM_PIN_D2     8
#define CAM_PIN_D1     9
#define CAM_PIN_D0     11
#define CAM_PIN_VSYNC  6
#define CAM_PIN_HREF   7
#define CAM_PIN_PCLK   13

#define CAM_H_RES      240
#define CAM_V_RES      240
#define CAM_XCLK_FREQ  (20000000)

#define PUPIL_IGNORE_BOUNDS            12
#define PUPIL_SEARCH_STEP              6
#define PUPIL_SEARCH_AREA              16
#define PUPIL_INTERNAL_STEP            4
#define PUPIL_THRESHOLD_OFFSET         20
#define PUPIL_MASK_SIZE                80
#define PUPIL_MIN_AREA                 160
#define PUPIL_MAX_AREA                 5000
#define PUPIL_MAX_ASPECT_X100          250
#define PUPIL_DILATE_ITERATIONS        2
#define PUPIL_LOST_HOLD_FRAMES         6
#define PUPIL_LOG_INTERVAL_MS          200
#define PUPIL_ORIGIN_CALIBRATION_DELAY_MS 4000
#define PUPIL_DIRECTION_THRESHOLD      20
#define PUPIL_DIRECTION_HOLD_MS        1000
#define PUPIL_SEND_COORD_TEXT          0

typedef enum {
    GAZE_DIRECTION_CENTER = 0,
    GAZE_DIRECTION_UP,
    GAZE_DIRECTION_DOWN,
    GAZE_DIRECTION_LEFT,
    GAZE_DIRECTION_RIGHT,
} gaze_direction_t;

static uint8_t *s_luma;
static uint8_t *s_binary;
static uint8_t s_black_threshold;
static bool s_has_last_pupil;
static int s_last_pupil_x;
static int s_last_pupil_y;
static int s_lost_pupil_frames;
static int64_t s_last_pupil_log_us;
static int s_origin_x = CAM_H_RES / 2;
static int s_origin_y = CAM_V_RES / 2;
static bool s_origin_calibrated;
static bool s_origin_calibration_pending;
static int64_t s_origin_calibration_due_us;
static gaze_direction_t s_candidate_gaze_direction = GAZE_DIRECTION_CENTER;
static int64_t s_candidate_gaze_since_us;
static gaze_direction_t s_last_sent_gaze_direction = GAZE_DIRECTION_CENTER;
static portMUX_TYPE s_origin_calibration_lock = portMUX_INITIALIZER_UNLOCKED;

static uint8_t rgb565_luma(uint16_t pixel)
{
    const uint16_t swapped = (uint16_t)((pixel << 8) | (pixel >> 8));
    pixel = swapped;
    const uint8_t r = (uint8_t)(((pixel >> 11) & 0x1f) << 3);
    const uint8_t g = (uint8_t)(((pixel >> 5) & 0x3f) << 2);
    const uint8_t b = (uint8_t)((pixel & 0x1f) << 3);

    return (uint8_t)((r * 30 + g * 59 + b * 11) / 100);
}

static bool ensure_pupil_buffers(void)
{
    if (!s_luma) {
        s_luma = heap_caps_malloc(CAM_H_RES * CAM_V_RES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_luma) {
            s_luma = heap_caps_malloc(CAM_H_RES * CAM_V_RES, MALLOC_CAP_8BIT);
        }
    }

    if (!s_binary) {
        s_binary = heap_caps_malloc(CAM_H_RES * CAM_V_RES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_binary) {
            s_binary = heap_caps_malloc(CAM_H_RES * CAM_V_RES, MALLOC_CAP_8BIT);
        }
    }

    return s_luma && s_binary;
}

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }

    return value;
}

static void find_darkest_area(int *dark_x, int *dark_y, uint8_t *dark_value)
{
    uint32_t min_sum = UINT32_MAX;
    *dark_x = CAM_H_RES / 2;
    *dark_y = CAM_V_RES / 2;
    *dark_value = 128;

    for (int y = PUPIL_IGNORE_BOUNDS; y < CAM_V_RES - PUPIL_IGNORE_BOUNDS - PUPIL_SEARCH_AREA;
         y += PUPIL_SEARCH_STEP) {
        for (int x = PUPIL_IGNORE_BOUNDS; x < CAM_H_RES - PUPIL_IGNORE_BOUNDS - PUPIL_SEARCH_AREA;
             x += PUPIL_SEARCH_STEP) {
            uint32_t current_sum = 0;
            int sample_count = 0;

            for (int dy = 0; dy < PUPIL_SEARCH_AREA; dy += PUPIL_INTERNAL_STEP) {
                for (int dx = 0; dx < PUPIL_SEARCH_AREA; dx += PUPIL_INTERNAL_STEP) {
                    current_sum += s_luma[(y + dy) * CAM_H_RES + x + dx];
                    sample_count++;
                }
            }

            if (sample_count > 0 && current_sum < min_sum) {
                min_sum = current_sum;
                *dark_x = x + PUPIL_SEARCH_AREA / 2;
                *dark_y = y + PUPIL_SEARCH_AREA / 2;
                *dark_value = s_luma[*dark_y * CAM_H_RES + *dark_x];
            }
        }
    }
}

static void apply_roi_threshold(int roi_left, int roi_top, int roi_right, int roi_bottom, uint8_t threshold)
{
    memset(s_binary, 0, CAM_H_RES * CAM_V_RES);

    for (int y = roi_top; y < roi_bottom; y++) {
        for (int x = roi_left; x < roi_right; x++) {
            const int idx = y * CAM_H_RES + x;
            s_binary[idx] = (s_luma[idx] < threshold) ? 255 : 0;
        }
    }
}

static void dilate_roi(int roi_left, int roi_top, int roi_right, int roi_bottom, int iterations)
{
    for (int iter = 0; iter < iterations; iter++) {
        for (int y = roi_top + 1; y < roi_bottom - 1; y++) {
            for (int x = roi_left + 1; x < roi_right - 1; x++) {
                const int idx = y * CAM_H_RES + x;
                if (s_binary[idx]) {
                    continue;
                }

                if (s_binary[idx - CAM_H_RES] == 255 || s_binary[idx + CAM_H_RES] == 255 ||
                    s_binary[idx - 1] == 255 || s_binary[idx + 1] == 255) {
                    s_binary[idx] = 128;
                }
            }
        }

        for (int y = roi_top + 1; y < roi_bottom - 1; y++) {
            for (int x = roi_left + 1; x < roi_right - 1; x++) {
                const int idx = y * CAM_H_RES + x;
                if (s_binary[idx] == 128) {
                    s_binary[idx] = 255;
                }
            }
        }
    }
}

static bool find_pupil_in_roi(int roi_left, int roi_top, int roi_right, int roi_bottom,
                              int *out_x, int *out_y)
{
    uint32_t sum_x = 0;
    uint32_t sum_y = 0;
    uint32_t pixel_count = 0;
    int min_x = roi_right;
    int max_x = roi_left;
    int min_y = roi_bottom;
    int max_y = roi_top;

    for (int y = roi_top; y < roi_bottom; y++) {
        for (int x = roi_left; x < roi_right; x++) {
            if (!s_binary[y * CAM_H_RES + x]) {
                continue;
            }

            sum_x += x;
            sum_y += y;
            pixel_count++;
            if (x < min_x) {
                min_x = x;
            }
            if (x > max_x) {
                max_x = x;
            }
            if (y < min_y) {
                min_y = y;
            }
            if (y > max_y) {
                max_y = y;
            }
        }
    }

    if (pixel_count < PUPIL_MIN_AREA || pixel_count > PUPIL_MAX_AREA) {
        return false;
    }

    const int blob_w = max_x - min_x + 1;
    const int blob_h = max_y - min_y + 1;
    const int long_side = blob_w > blob_h ? blob_w : blob_h;
    const int short_side = blob_w > blob_h ? blob_h : blob_w;
    const uint32_t aspect_x100 = (uint32_t)long_side * 100 / (uint32_t)(short_side > 0 ? short_side : 1);
    if (aspect_x100 > PUPIL_MAX_ASPECT_X100) {
        return false;
    }

    *out_x = (int)(sum_x / pixel_count);
    *out_y = (int)(sum_y / pixel_count);
    return true;
}

static bool find_pupil_center(const camera_fb_t *fb, int *out_x, int *out_y)
{
    if (!ensure_pupil_buffers()) {
        ESP_LOGW(TAG, "No memory for pupil detector buffers");
        return false;
    }

    const uint16_t *pixels = (const uint16_t *)fb->buf;

    for (int y = 0; y < CAM_V_RES; y++) {
        for (int x = 0; x < CAM_H_RES; x++) {
            s_luma[y * CAM_H_RES + x] = rgb565_luma(pixels[y * CAM_H_RES + x]);
        }
    }

    int dark_x = 0;
    int dark_y = 0;
    uint8_t dark_value = 0;
    find_darkest_area(&dark_x, &dark_y, &dark_value);

    const uint8_t threshold = (dark_value > 255 - PUPIL_THRESHOLD_OFFSET)
                                  ? 255
                                  : (uint8_t)(dark_value + PUPIL_THRESHOLD_OFFSET);
    s_black_threshold = threshold;

    const int half_roi = PUPIL_MASK_SIZE / 2;
    const int roi_left = clamp_int(dark_x - half_roi, PUPIL_IGNORE_BOUNDS, CAM_H_RES - 1);
    const int roi_top = clamp_int(dark_y - half_roi, PUPIL_IGNORE_BOUNDS, CAM_V_RES - 1);
    const int roi_right = clamp_int(dark_x + half_roi, roi_left + 1, CAM_H_RES - PUPIL_IGNORE_BOUNDS);
    const int roi_bottom = clamp_int(dark_y + half_roi, roi_top + 1, CAM_V_RES - PUPIL_IGNORE_BOUNDS);

    apply_roi_threshold(roi_left, roi_top, roi_right, roi_bottom, threshold);
    dilate_roi(roi_left, roi_top, roi_right, roi_bottom, PUPIL_DILATE_ITERATIONS);

    int detected_x = dark_x;
    int detected_y = dark_y;
    const bool found = find_pupil_in_roi(roi_left, roi_top, roi_right, roi_bottom,
                                         &detected_x, &detected_y);

    if (!found) {
        if (s_has_last_pupil && s_lost_pupil_frames < PUPIL_LOST_HOLD_FRAMES) {
            s_lost_pupil_frames++;
            *out_x = s_last_pupil_x;
            *out_y = s_last_pupil_y;
            return true;
        }

        s_has_last_pupil = false;
        s_lost_pupil_frames = 0;
        return false;
    }

    if (s_has_last_pupil) {
        detected_x = (s_last_pupil_x * 3 + detected_x) / 4;
        detected_y = (s_last_pupil_y * 3 + detected_y) / 4;
    }

    s_last_pupil_x = detected_x;
    s_last_pupil_y = detected_y;
    s_has_last_pupil = true;
    s_lost_pupil_frames = 0;
    *out_x = detected_x;
    *out_y = detected_y;
    return true;
}

static const char *gaze_direction_to_string(gaze_direction_t direction)
{
    switch (direction) {
    case GAZE_DIRECTION_UP:
        return "UP";
    case GAZE_DIRECTION_DOWN:
        return "DOWN";
    case GAZE_DIRECTION_LEFT:
        return "LEFT";
    case GAZE_DIRECTION_RIGHT:
        return "RIGHT";
    case GAZE_DIRECTION_CENTER:
    default:
        return "CENTER";
    }
}

static gaze_direction_t get_gaze_direction(int relative_x, int relative_y)
{
    const int abs_x = relative_x < 0 ? -relative_x : relative_x;
    const int abs_y = relative_y < 0 ? -relative_y : relative_y;

    if (abs_x < PUPIL_DIRECTION_THRESHOLD && abs_y < PUPIL_DIRECTION_THRESHOLD) {
        return GAZE_DIRECTION_CENTER;
    }

    if (abs_x >= abs_y) {
        return relative_x >= 0 ? GAZE_DIRECTION_RIGHT : GAZE_DIRECTION_LEFT;
    }

    return relative_y >= 0 ? GAZE_DIRECTION_UP : GAZE_DIRECTION_DOWN;
}

static gaze_direction_t get_current_gaze_direction(int x, int y, int *out_relative_x, int *out_relative_y)
{
    const int relative_x = x - s_origin_x;
    const int relative_y = s_origin_y - y;

    if (out_relative_x) {
        *out_relative_x = relative_x;
    }
    if (out_relative_y) {
        *out_relative_y = relative_y;
    }

    if (!s_origin_calibrated) {
        return GAZE_DIRECTION_CENTER;
    }

    return get_gaze_direction(relative_x, relative_y);
}

static bool gaze_direction_to_cmd(gaze_direction_t direction, uint8_t *out_cmd)
{
    switch (direction) {
    case GAZE_DIRECTION_RIGHT:
        *out_cmd = 0x01;
        return true;
    case GAZE_DIRECTION_LEFT:
        *out_cmd = 0x02;
        return true;
    case GAZE_DIRECTION_UP:
        *out_cmd = 0x03;
        return true;
    case GAZE_DIRECTION_DOWN:
        *out_cmd = 0x04;
        return true;
    case GAZE_DIRECTION_CENTER:
    default:
        return false;
    }
}

static void send_gaze_direction_when_stable(gaze_direction_t direction)
{
    const int64_t now_us = esp_timer_get_time();

    if (direction == GAZE_DIRECTION_CENTER) {
        s_candidate_gaze_direction = GAZE_DIRECTION_CENTER;
        s_candidate_gaze_since_us = 0;
        return;
    }

    if (direction != s_candidate_gaze_direction) {
        s_candidate_gaze_direction = direction;
        s_candidate_gaze_since_us = now_us;
        return;
    }

    if (now_us - s_candidate_gaze_since_us < PUPIL_DIRECTION_HOLD_MS * 1000) {
        return;
    }

    if (direction == s_last_sent_gaze_direction) {
        return;
    }

    uint8_t cmd = 0;
    if (!gaze_direction_to_cmd(direction, &cmd)) {
        return;
    }

    ESP_LOGI(TAG, "Send gaze command: gaze=%s, frame=5A %02X A5",
             gaze_direction_to_string(direction),
             cmd);
    const esp_err_t ret = serial_comm_send(cmd);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret == ESP_OK) {
        serial_comm_set_pending_gaze_cmd(cmd);
    }
    s_last_sent_gaze_direction = direction;
}

static void log_pupil_coords(int x, int y)
{
    const int64_t now_us = esp_timer_get_time();
    if (now_us - s_last_pupil_log_us < PUPIL_LOG_INTERVAL_MS * 1000) {
        return;
    }

    s_last_pupil_log_us = now_us;

    int relative_x = 0;
    int relative_y = 0;
    const gaze_direction_t gaze_direction = get_current_gaze_direction(x, y, &relative_x, &relative_y);

    ESP_LOGI(TAG,
             "Eye center: coord=(%d,%d), origin=(%d,%d), relative=(%d,%d), gaze=%s, threshold=%u",
             x,
             y,
             s_origin_x,
             s_origin_y,
             relative_x,
             relative_y,
             gaze_direction_to_string(gaze_direction),
             s_black_threshold);
}

static void handle_origin_calibration_if_due(int x, int y)
{
    bool should_calibrate = false;
    portENTER_CRITICAL(&s_origin_calibration_lock);
    if (s_origin_calibration_pending && esp_timer_get_time() >= s_origin_calibration_due_us) {
        s_origin_calibration_pending = false;
        should_calibrate = true;
    }
    portEXIT_CRITICAL(&s_origin_calibration_lock);

    if (!should_calibrate) {
        return;
    }

    s_origin_x = x;
    s_origin_y = y;
    s_origin_calibrated = true;
    s_candidate_gaze_direction = GAZE_DIRECTION_CENTER;
    s_candidate_gaze_since_us = 0;
    s_last_sent_gaze_direction = GAZE_DIRECTION_CENTER;
    ESP_LOGI(TAG, "Origin calibrated to coord=(%d,%d)", s_origin_x, s_origin_y);
    ESP_ERROR_CHECK_WITHOUT_ABORT(serial_comm_send_calibration_success_prompt());
}

static bool camera_psram_available(void)
{
#if CONFIG_SPIRAM
    return esp_psram_is_initialized();
#else
    return false;
#endif
}

static void init_camera(void)
{
    const bool has_psram = camera_psram_available();
    const camera_config_t camera_config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = CAM_XCLK_FREQ,
        .ledc_timer = LEDC_TIMER_1,
        .ledc_channel = LEDC_CHANNEL_1,
        .pixel_format = PIXFORMAT_RGB565,
        .frame_size = FRAMESIZE_240X240,
        .jpeg_quality = 12,
        .fb_count = has_psram ? 2 : 1,
        .fb_location = has_psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM,
        .grab_mode = has_psram ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY,
    };

    ESP_LOGI(TAG, "Camera frame buffer: %s", has_psram ? "PSRAM" : "DRAM");
    ESP_ERROR_CHECK(esp_camera_init(&camera_config));

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
        sensor->set_framesize(sensor, FRAMESIZE_240X240);
        sensor->set_pixformat(sensor, PIXFORMAT_RGB565);
    }
}

static void camera_preview_loop(void)
{
    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Camera frame capture failed");
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }

        if (fb->format == PIXFORMAT_RGB565 && fb->width == CAM_H_RES && fb->height == CAM_V_RES) {
            int pupil_x = 0;
            int pupil_y = 0;
            if (find_pupil_center(fb, &pupil_x, &pupil_y)) {
                handle_origin_calibration_if_due(pupil_x, pupil_y);
                log_pupil_coords(pupil_x, pupil_y);
#if PUPIL_SEND_COORD_TEXT
                ESP_ERROR_CHECK_WITHOUT_ABORT(serial_comm_send_pupil_coords(pupil_x, pupil_y));
#endif
                if (s_origin_calibrated) {
                    const gaze_direction_t gaze_direction =
                        get_current_gaze_direction(pupil_x, pupil_y, NULL, NULL);
                    send_gaze_direction_when_stable(gaze_direction);
                }
            }
        } else {
            ESP_LOGW(TAG, "Unexpected frame: %ux%u format=%d len=%u",
                     fb->width, fb->height, fb->format, fb->len);
        }

        esp_camera_fb_return(fb);
    }
}

void camera_tracker_request_origin_calibration(void)
{
    portENTER_CRITICAL(&s_origin_calibration_lock);
    s_origin_calibration_due_us = esp_timer_get_time() + PUPIL_ORIGIN_CALIBRATION_DELAY_MS * 1000;
    s_origin_calibration_pending = true;
    portEXIT_CRITICAL(&s_origin_calibration_lock);
    ESP_LOGI(TAG, "Origin calibration requested, keep looking forward for %d ms",
             PUPIL_ORIGIN_CALIBRATION_DELAY_MS);
}

void camera_tracker_start(void)
{
    ESP_ERROR_CHECK(motor_control_start());
    ESP_ERROR_CHECK(serial_comm_start());

    ESP_LOGI(TAG, "Initialize OV2640 camera");
    init_camera();

    ESP_LOGI(TAG, "Start coordinate tracking over UART");
    camera_preview_loop();
}
