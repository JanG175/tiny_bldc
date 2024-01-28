/**
 * @file tiny_bldc.h
 * @author JanG175
 * @brief ESP-IDF component for tiny BLDC driver
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

#define PWM_TIMEBASE_RESOLUTION_HZ    10000000 // 10 MHz, 0.1 us per tick
#define PWM_TIMEBASE_PERIOD           200      // 200 ticks, 0.02 ms, 50 kHz PWM frequency

#define PWM_MAX_PULSEWIDTH_US    200
#define PWM_MIN_PULSEWIDTH_US    100

#define BLDC_MAX_SPEED    100
#define BLDC_MIN_SPEED    0

#define CW_DIR     0
#define CCW_DIR    1

typedef struct tiny_bldc_conf_t
{
    gpio_num_t pwm_pin;
    gpio_num_t dir_pin;
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operator;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;
} tiny_bldc_conf_t;


void tiny_bldc_init(tiny_bldc_conf_t* bldc_conf);

void tiny_bldc_deinit(tiny_bldc_conf_t* bldc_conf);

void tiny_bldc_set_dir(tiny_bldc_conf_t* bldc_conf, uint8_t dir);

void tiny_bldc_set_speed(tiny_bldc_conf_t* bldc_conf, uint32_t speed);
