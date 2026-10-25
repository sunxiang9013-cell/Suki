/*
 * SPDX-License-Identifier: CC0-1.0
 */

#include "serial_comm.h"
#include "camera_tracker.h"
#include "motor_control.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "serial";

#define SERIAL_UART_PORT       UART_NUM_1
#define SERIAL_UART_BAUD_RATE  115200
#define SERIAL_UART_TX_PIN     1
#define SERIAL_UART_RX_PIN     3
#define SERIAL_RX_BUFFER_SIZE  256
#define SERIAL_RX_TASK_STACK   4096
#define SERIAL_RX_TASK_PRIO    10
#define SERIAL_FRAME_LEN       3
#define SERIAL_FRAME_HEAD      0x5a
#define SERIAL_FRAME_TAIL      0xa5
#define SERIAL_MOTOR_SPEED_PERCENT 60
#define SERIAL_MOTOR_TURN_DURATION_MS 700
#define SERIAL_MOTOR_FORWARD_DURATION_MS 800
#define SERIAL_MOTOR_REVERSE_DURATION_MS 800

typedef enum {
    SERIAL_CMD_01 = 0x01,
    SERIAL_CMD_02 = 0x02,
    SERIAL_CMD_03 = 0x03,
    SERIAL_CMD_04 = 0x04,
    SERIAL_CMD_05 = 0x05,
    SERIAL_CMD_10 = 0x10,
    SERIAL_CMD_11 = 0x11,
    SERIAL_CMD_12 = 0x12,
    SERIAL_CMD_13 = 0x13,
    SERIAL_CMD_0A = 0x0a,
    SERIAL_CMD_0B = 0x0b,
} serial_cmd_t;

typedef enum {
    PENDING_GAZE_NONE = 0,
    PENDING_GAZE_RIGHT,
    PENDING_GAZE_LEFT,
    PENDING_GAZE_FORWARD,
    PENDING_GAZE_REVERSE,
} pending_gaze_t;

static pending_gaze_t s_pending_gaze = PENDING_GAZE_NONE;

static const char *pending_gaze_to_string(pending_gaze_t pending_gaze)
{
    switch (pending_gaze) {
    case PENDING_GAZE_RIGHT:
        return "right";
    case PENDING_GAZE_LEFT:
        return "left";
    case PENDING_GAZE_FORWARD:
        return "forward";
    case PENDING_GAZE_REVERSE:
        return "reverse";
    case PENDING_GAZE_NONE:
    default:
        return "none";
    }
}

static pending_gaze_t gaze_cmd_to_pending(uint8_t cmd)
{
    switch ((serial_cmd_t)cmd) {
    case SERIAL_CMD_01:
        return PENDING_GAZE_RIGHT;
    case SERIAL_CMD_02:
        return PENDING_GAZE_LEFT;
    case SERIAL_CMD_03:
        return PENDING_GAZE_FORWARD;
    case SERIAL_CMD_04:
        return PENDING_GAZE_REVERSE;
    default:
        return PENDING_GAZE_NONE;
    }
}

static pending_gaze_t action_cmd_to_pending(serial_cmd_t cmd)
{
    switch (cmd) {
    case SERIAL_CMD_10:
        return PENDING_GAZE_RIGHT;
    case SERIAL_CMD_11:
        return PENDING_GAZE_LEFT;
    case SERIAL_CMD_12:
        return PENDING_GAZE_FORWARD;
    case SERIAL_CMD_13:
        return PENDING_GAZE_REVERSE;
    default:
        return PENDING_GAZE_NONE;
    }
}

static void serial_execute_pending_gaze(pending_gaze_t pending_gaze)
{
    switch (pending_gaze) {
    case PENDING_GAZE_RIGHT:
        ESP_LOGI(TAG, "confirmed gaze action: turn right");
        ESP_ERROR_CHECK_WITHOUT_ABORT(motor_control_turn_right_for(SERIAL_MOTOR_SPEED_PERCENT,
                                                                    SERIAL_MOTOR_TURN_DURATION_MS));
        break;
    case PENDING_GAZE_LEFT:
        ESP_LOGI(TAG, "confirmed gaze action: turn left");
        ESP_ERROR_CHECK_WITHOUT_ABORT(motor_control_turn_left_for(SERIAL_MOTOR_SPEED_PERCENT,
                                                                   SERIAL_MOTOR_TURN_DURATION_MS));
        break;
    case PENDING_GAZE_FORWARD:
        ESP_LOGI(TAG, "confirmed gaze action: forward");
        ESP_ERROR_CHECK_WITHOUT_ABORT(motor_control_forward_for(SERIAL_MOTOR_SPEED_PERCENT,
                                                                SERIAL_MOTOR_FORWARD_DURATION_MS));
        break;
    case PENDING_GAZE_REVERSE:
        ESP_LOGI(TAG, "confirmed gaze action: reverse");
        ESP_ERROR_CHECK_WITHOUT_ABORT(motor_control_reverse_for(SERIAL_MOTOR_SPEED_PERCENT,
                                                                SERIAL_MOTOR_REVERSE_DURATION_MS));
        break;
    case PENDING_GAZE_NONE:
    default:
        ESP_LOGW(TAG, "no pending gaze action to confirm");
        break;
    }
}

static void serial_confirm_pending_gaze(serial_cmd_t cmd)
{
    if (s_pending_gaze == PENDING_GAZE_NONE) {
        ESP_LOGW(TAG, "ignore command 0x%02X: no pending gaze action", cmd);
        return;
    }

    const pending_gaze_t action_pending = action_cmd_to_pending(cmd);
    if (cmd != SERIAL_CMD_0A && action_pending != s_pending_gaze) {
        ESP_LOGW(TAG,
                 "ignore command 0x%02X: pending gaze is %s",
                 cmd,
                 pending_gaze_to_string(s_pending_gaze));
        return;
    }

    const pending_gaze_t confirmed_gaze = s_pending_gaze;
    s_pending_gaze = PENDING_GAZE_NONE;
    serial_execute_pending_gaze(confirmed_gaze);
}

static void serial_handle_frame(const uint8_t frame[SERIAL_FRAME_LEN])
{
    if (frame[0] != SERIAL_FRAME_HEAD || frame[2] != SERIAL_FRAME_TAIL) {
        ESP_LOGW(TAG,
                 "drop invalid frame: %02X %02X %02X",
                 frame[0],
                 frame[1],
                 frame[2]);
        return;
    }

    ESP_LOGI(TAG, "RX frame: 5A %02X A5", frame[1]);

    switch ((serial_cmd_t)frame[1]) {
    case SERIAL_CMD_01:
        if (s_pending_gaze == PENDING_GAZE_RIGHT) {
            ESP_LOGI(TAG, "ignore echoed gaze command 0x01");
            break;
        }
        ESP_LOGI(TAG, "handle command 0x01: request origin calibration");
        camera_tracker_request_origin_calibration();
        break;
    case SERIAL_CMD_02:
    case SERIAL_CMD_03:
    case SERIAL_CMD_04:
        ESP_LOGI(TAG, "ignore echoed gaze command 0x%02X", frame[1]);
        break;
    case SERIAL_CMD_0A:
        ESP_LOGI(TAG, "handle command 0x0A: confirm pending gaze");
        serial_confirm_pending_gaze(SERIAL_CMD_0A);
        break;
    case SERIAL_CMD_0B:
        ESP_LOGI(TAG, "handle command 0x0B: brake");
        s_pending_gaze = PENDING_GAZE_NONE;
        ESP_ERROR_CHECK_WITHOUT_ABORT(motor_control_brake());
        break;
    case SERIAL_CMD_10:
        ESP_LOGI(TAG, "handle command 0x10: confirm turn right");
        serial_confirm_pending_gaze(SERIAL_CMD_10);
        break;
    case SERIAL_CMD_11:
        ESP_LOGI(TAG, "handle command 0x11: confirm turn left");
        serial_confirm_pending_gaze(SERIAL_CMD_11);
        break;
    case SERIAL_CMD_12:
        ESP_LOGI(TAG, "handle command 0x12: confirm forward");
        serial_confirm_pending_gaze(SERIAL_CMD_12);
        break;
    case SERIAL_CMD_13:
        ESP_LOGI(TAG, "handle command 0x13: confirm reverse");
        serial_confirm_pending_gaze(SERIAL_CMD_13);
        break;
    default:
        ESP_LOGW(TAG, "unsupported command: 0x%02X", frame[1]);
        break;
    }
}

static void serial_log_rx_bytes(const uint8_t *data, int len)
{
    char line[3 * 32 + 1];
    int offset = 0;

    for (int i = 0; i < len && offset < (int)sizeof(line) - 1; ++i) {
        const int written = snprintf(line + offset,
                                     sizeof(line) - offset,
                                     "%s%02X",
                                     i == 0 ? "" : " ",
                                     data[i]);
        if (written <= 0) {
            break;
        }
        offset += written;
    }

    line[offset] = '\0';
    ESP_LOGI(TAG, "RX bytes(%d): %s", len, line);
}

static void serial_feed_byte(uint8_t byte)
{
    static uint8_t frame[SERIAL_FRAME_LEN];
    static size_t frame_index;

    if (frame_index == 0 && byte != SERIAL_FRAME_HEAD) {
        return;
    }

    if (frame_index > 0 && byte == SERIAL_FRAME_HEAD) {
        frame[0] = SERIAL_FRAME_HEAD;
        frame_index = 1;
        return;
    }

    frame[frame_index++] = byte;

    if (frame_index < SERIAL_FRAME_LEN) {
        return;
    }

    serial_handle_frame(frame);
    frame_index = 0;
}

static void serial_rx_task(void *arg)
{
    (void)arg;

    uint8_t data[32];

    while (true) {
        size_t buffered_len = 0;
        if (uart_get_buffered_data_len(SERIAL_UART_PORT, &buffered_len) != ESP_OK || buffered_len == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        const int len = uart_read_bytes(SERIAL_UART_PORT,
                                        data,
                                        buffered_len > sizeof(data) ? sizeof(data) : buffered_len,
                                        pdMS_TO_TICKS(20));
        if (len > 0) {
            serial_log_rx_bytes(data, len);
        }
        for (int i = 0; i < len; ++i) {
            serial_feed_byte(data[i]);
        }
    }
}

esp_err_t serial_comm_start(void)
{
    const uart_config_t uart_config = {
        .baud_rate = SERIAL_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(SERIAL_UART_PORT, SERIAL_RX_BUFFER_SIZE, 0, 0, NULL, 0),
                        TAG, "install uart driver failed");
    ESP_RETURN_ON_ERROR(uart_param_config(SERIAL_UART_PORT, &uart_config),
                        TAG, "config uart params failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(SERIAL_UART_PORT,
                                     SERIAL_UART_TX_PIN,
                                     SERIAL_UART_RX_PIN,
                                     UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE),
                        TAG, "config uart pins failed");

    BaseType_t ret = xTaskCreate(serial_rx_task,
                                 "serial_rx",
                                 SERIAL_RX_TASK_STACK,
                                 NULL,
                                 SERIAL_RX_TASK_PRIO,
                                 NULL);
    if (ret != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "uart%d started, baud=%d", SERIAL_UART_PORT, SERIAL_UART_BAUD_RATE);
    return ESP_OK;
}

esp_err_t serial_comm_send(uint8_t cmd)
{
    const uint8_t frame[SERIAL_FRAME_LEN] = {
        SERIAL_FRAME_HEAD,
        cmd,
        SERIAL_FRAME_TAIL,
    };

    const int written = uart_write_bytes(SERIAL_UART_PORT, frame, sizeof(frame));
    if (written != sizeof(frame)) {
        ESP_LOGE(TAG, "send frame failed, written=%d", written);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t serial_comm_send_calibration_success_prompt(void)
{
    ESP_LOGI(TAG, "Send calibration success prompt: frame=5A %02X A5", SERIAL_CMD_05);
    return serial_comm_send(SERIAL_CMD_05);
}

void serial_comm_set_pending_gaze_cmd(uint8_t cmd)
{
    const pending_gaze_t pending_gaze = gaze_cmd_to_pending(cmd);
    if (pending_gaze == PENDING_GAZE_NONE) {
        ESP_LOGW(TAG, "ignore pending gaze cmd: 0x%02X", cmd);
        return;
    }

    s_pending_gaze = pending_gaze;
    ESP_LOGI(TAG,
             "pending gaze action: %s, wait for 0x0A or matching action command",
             pending_gaze_to_string(s_pending_gaze));
}

esp_err_t serial_comm_send_pupil_coords(int x, int y)
{
    char line[32];
    const int len = snprintf(line, sizeof(line), "PUPIL,%d,%d\r\n", x, y);
    if (len <= 0 || len >= (int)sizeof(line)) {
        return ESP_FAIL;
    }

    const int written = uart_write_bytes(SERIAL_UART_PORT, line, len);
    if (written != len) {
        ESP_LOGE(TAG, "send pupil coords failed, written=%d", written);
        return ESP_FAIL;
    }

    return ESP_OK;
}
