/*
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t serial_comm_start(void);
esp_err_t serial_comm_send(uint8_t cmd);
esp_err_t serial_comm_send_calibration_success_prompt(void);
esp_err_t serial_comm_send_pupil_coords(int x, int y);
void serial_comm_set_pending_gaze_cmd(uint8_t cmd);

#ifdef __cplusplus
}
#endif
