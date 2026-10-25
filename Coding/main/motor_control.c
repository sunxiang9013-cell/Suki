/*
 * SPDX-License-Identifier: CC0-1.0
 */

#include "motor_control.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "motor";

#define MOTOR1_PWM_GPIO     GPIO_NUM_2
#define MOTOR1_INA_GPIO     GPIO_NUM_41
#define MOTOR1_INB_GPIO     GPIO_NUM_42
#define MOTOR2_PWM_GPIO     GPIO_NUM_38
#define MOTOR2_INA_GPIO     GPIO_NUM_39
#define MOTOR2_INB_GPIO     GPIO_NUM_40

#define MOTOR_PWM_TIMER     LEDC_TIMER_0
#define MOTOR_PWM_MODE      LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_FREQ_HZ   20000
#define MOTOR_PWM_RES       LEDC_TIMER_10_BIT
#define MOTOR_PWM_MAX_DUTY  ((1U << 10) - 1)
#define MOTOR_SWITCH_DELAY_MS 50
#define MOTOR_TURN_INNER_SPEED_RATIO_PERCENT 45
#define MOTOR_STOP_TASK_PRIO    9

typedef enum {
    MOTOR_DIR_BRAKE = 0,
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_REVERSE,
} motor_direction_t;

typedef struct {
    gpio_num_t pwm_gpio;
    gpio_num_t ina_gpio;
    gpio_num_t inb_gpio;
    ledc_channel_t ledc_channel;
    motor_direction_t direction;
    uint32_t duty;
} motor_t;

static motor_t s_motors[] = {
    {
        .pwm_gpio = MOTOR1_PWM_GPIO,
        .ina_gpio = MOTOR1_INA_GPIO,
        .inb_gpio = MOTOR1_INB_GPIO,
        .ledc_channel = LEDC_CHANNEL_2,
        .direction = MOTOR_DIR_BRAKE,
        .duty = 0,
    },
    {
        .pwm_gpio = MOTOR2_PWM_GPIO,
        .ina_gpio = MOTOR2_INA_GPIO,
        .inb_gpio = MOTOR2_INB_GPIO,
        .ledc_channel = LEDC_CHANNEL_3,
        .direction = MOTOR_DIR_BRAKE,
        .duty = 0,
    },
};

static bool s_started;
static uint32_t s_motion_generation;
static portMUX_TYPE s_motion_generation_lock = portMUX_INITIALIZER_UNLOCKED;

static esp_err_t motor_pair_set_with_speed(motor_direction_t motor1_dir,
                                           uint8_t motor1_speed_percent,
                                           motor_direction_t motor2_dir,
                                           uint8_t motor2_speed_percent);
static esp_err_t motor_schedule_brake_after(uint32_t duration_ms);

static uint32_t speed_percent_to_duty(uint8_t speed_percent)
{
    if (speed_percent > 100) {
        speed_percent = 100;
    }

    return (MOTOR_PWM_MAX_DUTY * speed_percent) / 100;
}

static esp_err_t motor_set_pwm(motor_t *motor, uint32_t duty)
{
    ESP_RETURN_ON_ERROR(ledc_set_duty(MOTOR_PWM_MODE, motor->ledc_channel, duty),
                        TAG, "set pwm duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(MOTOR_PWM_MODE, motor->ledc_channel),
                        TAG, "update pwm duty failed");
    motor->duty = duty;
    return ESP_OK;
}

static esp_err_t motor_apply_direction(motor_t *motor, motor_direction_t direction)
{
    int ina_level = 0;
    int inb_level = 0;

    switch (direction) {
    case MOTOR_DIR_FORWARD:
        ina_level = 0;
        inb_level = 1;
        break;
    case MOTOR_DIR_REVERSE:
        ina_level = 1;
        inb_level = 0;
        break;
    case MOTOR_DIR_BRAKE:
    default:
        ina_level = 0;
        inb_level = 0;
        break;
    }

    ESP_RETURN_ON_ERROR(gpio_set_level(motor->ina_gpio, ina_level),
                        TAG, "set INA failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(motor->inb_gpio, inb_level),
                        TAG, "set INB failed");
    motor->direction = direction;
    return ESP_OK;
}

static esp_err_t motor_set(motor_t *motor, motor_direction_t direction, uint32_t duty)
{
    if (!s_started) {
        return ESP_ERR_INVALID_STATE;
    }

    if (direction == MOTOR_DIR_BRAKE) {
        ESP_RETURN_ON_ERROR(motor_set_pwm(motor, 0), TAG, "stop pwm before brake failed");
        return motor_apply_direction(motor, MOTOR_DIR_BRAKE);
    }

    if (motor->direction != direction && motor->duty > 0) {
        ESP_RETURN_ON_ERROR(motor_set_pwm(motor, 0), TAG, "stop pwm before direction switch failed");
        vTaskDelay(pdMS_TO_TICKS(MOTOR_SWITCH_DELAY_MS));
    }

    ESP_RETURN_ON_ERROR(motor_apply_direction(motor, direction), TAG, "set direction failed");
    return motor_set_pwm(motor, duty);
}

static esp_err_t motor_pair_set(motor_direction_t motor1_dir,
                                motor_direction_t motor2_dir,
                                uint8_t speed_percent)
{
    return motor_pair_set_with_speed(motor1_dir,
                                     speed_percent,
                                     motor2_dir,
                                     speed_percent);
}

static esp_err_t motor_pair_set_with_speed(motor_direction_t motor1_dir,
                                           uint8_t motor1_speed_percent,
                                           motor_direction_t motor2_dir,
                                           uint8_t motor2_speed_percent)
{
    const uint32_t duties[] = {
        speed_percent_to_duty(motor1_speed_percent),
        speed_percent_to_duty(motor2_speed_percent),
    };
    const motor_direction_t target_dirs[] = {
        motor1_dir,
        motor2_dir,
    };
    bool direction_switching = false;

    if (!s_started) {
        return ESP_ERR_INVALID_STATE;
    }

    for (size_t i = 0; i < sizeof(s_motors) / sizeof(s_motors[0]); ++i) {
        if (target_dirs[i] != MOTOR_DIR_BRAKE &&
            s_motors[i].direction != target_dirs[i] &&
            s_motors[i].duty > 0) {
            ESP_RETURN_ON_ERROR(motor_set_pwm(&s_motors[i], 0),
                                TAG, "stop pwm before pair direction switch failed");
            direction_switching = true;
        }
    }

    if (direction_switching) {
        vTaskDelay(pdMS_TO_TICKS(MOTOR_SWITCH_DELAY_MS));
    }

    for (size_t i = 0; i < sizeof(s_motors) / sizeof(s_motors[0]); ++i) {
        ESP_RETURN_ON_ERROR(motor_apply_direction(&s_motors[i], target_dirs[i]),
                            TAG, "set pair direction failed");
    }

    for (size_t i = 0; i < sizeof(s_motors) / sizeof(s_motors[0]); ++i) {
        ESP_RETURN_ON_ERROR(motor_set_pwm(&s_motors[i], duties[i]), TAG, "set pair pwm failed");
    }

    return ESP_OK;
}

static uint8_t turn_inner_speed_percent(uint8_t speed_percent)
{
    return (uint8_t)((speed_percent * MOTOR_TURN_INNER_SPEED_RATIO_PERCENT) / 100);
}

typedef struct {
    uint32_t generation;
    uint32_t duration_ms;
} motor_stop_task_ctx_t;

static void motor_stop_after_delay_task(void *arg)
{
    motor_stop_task_ctx_t *ctx = (motor_stop_task_ctx_t *)arg;
    vTaskDelay(pdMS_TO_TICKS(ctx->duration_ms));

    bool should_brake = false;
    portENTER_CRITICAL(&s_motion_generation_lock);
    if (ctx->generation == s_motion_generation) {
        should_brake = true;
    }
    portEXIT_CRITICAL(&s_motion_generation_lock);

    if (should_brake) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(motor_control_brake());
    }

    vPortFree(ctx);
    vTaskDelete(NULL);
}

static esp_err_t motor_schedule_brake_after(uint32_t duration_ms)
{
    motor_stop_task_ctx_t *ctx = pvPortMalloc(sizeof(*ctx));
    if (!ctx) {
        return ESP_ERR_NO_MEM;
    }

    portENTER_CRITICAL(&s_motion_generation_lock);
    ctx->generation = ++s_motion_generation;
    portEXIT_CRITICAL(&s_motion_generation_lock);
    ctx->duration_ms = duration_ms;

    BaseType_t ret = xTaskCreate(motor_stop_after_delay_task,
                                 "motor_stop",
                                 2048,
                                 ctx,
                                 MOTOR_STOP_TASK_PRIO,
                                 NULL);
    if (ret != pdPASS) {
        vPortFree(ctx);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t motor_control_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    const gpio_config_t dir_gpio_config = {
        .pin_bit_mask = (1ULL << MOTOR1_INA_GPIO) |
                        (1ULL << MOTOR1_INB_GPIO) |
                        (1ULL << MOTOR2_INA_GPIO) |
                        (1ULL << MOTOR2_INB_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&dir_gpio_config), TAG, "config direction gpio failed");

    const ledc_timer_config_t timer_config = {
        .speed_mode = MOTOR_PWM_MODE,
        .duty_resolution = MOTOR_PWM_RES,
        .timer_num = MOTOR_PWM_TIMER,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG, "config pwm timer failed");

    for (size_t i = 0; i < sizeof(s_motors) / sizeof(s_motors[0]); ++i) {
        const ledc_channel_config_t channel_config = {
            .gpio_num = s_motors[i].pwm_gpio,
            .speed_mode = MOTOR_PWM_MODE,
            .channel = s_motors[i].ledc_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = MOTOR_PWM_TIMER,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_config), TAG, "config pwm channel failed");
        ESP_RETURN_ON_ERROR(motor_apply_direction(&s_motors[i], MOTOR_DIR_BRAKE),
                            TAG, "set initial brake failed");
    }

    s_started = true;
    ESP_LOGI(TAG, "motor control started");
    return ESP_OK;
}

esp_err_t motor_control_brake(void)
{
    portENTER_CRITICAL(&s_motion_generation_lock);
    s_motion_generation++;
    portEXIT_CRITICAL(&s_motion_generation_lock);
    ESP_RETURN_ON_ERROR(motor_set(&s_motors[0], MOTOR_DIR_BRAKE, 0), TAG, "brake motor1 failed");
    return motor_set(&s_motors[1], MOTOR_DIR_BRAKE, 0);
}

esp_err_t motor_control_forward(uint8_t speed_percent)
{
    return motor_pair_set(MOTOR_DIR_FORWARD, MOTOR_DIR_FORWARD, speed_percent);
}

esp_err_t motor_control_reverse(uint8_t speed_percent)
{
    return motor_pair_set(MOTOR_DIR_REVERSE, MOTOR_DIR_REVERSE, speed_percent);
}

esp_err_t motor_control_turn_left(uint8_t speed_percent)
{
    return motor_pair_set_with_speed(MOTOR_DIR_FORWARD,
                                     turn_inner_speed_percent(speed_percent),
                                     MOTOR_DIR_FORWARD,
                                     speed_percent);
}

esp_err_t motor_control_turn_right(uint8_t speed_percent)
{
    return motor_pair_set_with_speed(MOTOR_DIR_FORWARD,
                                     speed_percent,
                                     MOTOR_DIR_FORWARD,
                                     turn_inner_speed_percent(speed_percent));
}

esp_err_t motor_control_forward_for(uint8_t speed_percent, uint32_t duration_ms)
{
    ESP_RETURN_ON_ERROR(motor_control_forward(speed_percent), TAG, "start forward failed");
    return motor_schedule_brake_after(duration_ms);
}

esp_err_t motor_control_reverse_for(uint8_t speed_percent, uint32_t duration_ms)
{
    ESP_RETURN_ON_ERROR(motor_control_reverse(speed_percent), TAG, "start reverse failed");
    return motor_schedule_brake_after(duration_ms);
}

esp_err_t motor_control_turn_left_for(uint8_t speed_percent, uint32_t duration_ms)
{
    ESP_RETURN_ON_ERROR(motor_control_turn_left(speed_percent), TAG, "start turn left failed");
    return motor_schedule_brake_after(duration_ms);
}

esp_err_t motor_control_turn_right_for(uint8_t speed_percent, uint32_t duration_ms)
{
    ESP_RETURN_ON_ERROR(motor_control_turn_right(speed_percent), TAG, "start turn right failed");
    return motor_schedule_brake_after(duration_ms);
}
