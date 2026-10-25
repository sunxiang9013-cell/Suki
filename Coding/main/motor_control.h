/*
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t motor_control_start(void);
esp_err_t motor_control_brake(void);
esp_err_t motor_control_forward(uint8_t speed_percent);
esp_err_t motor_control_reverse(uint8_t speed_percent);
esp_err_t motor_control_turn_left(uint8_t speed_percent);
esp_err_t motor_control_turn_right(uint8_t speed_percent);
esp_err_t motor_control_forward_for(uint8_t speed_percent, uint32_t duration_ms);
esp_err_t motor_control_reverse_for(uint8_t speed_percent, uint32_t duration_ms);
esp_err_t motor_control_turn_left_for(uint8_t speed_percent, uint32_t duration_ms);
esp_err_t motor_control_turn_right_for(uint8_t speed_percent, uint32_t duration_ms);

#ifdef __cplusplus
}
#endif
