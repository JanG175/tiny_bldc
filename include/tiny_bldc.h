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
#include "driver/ledc.h"
#include "esp_log.h"

#define PWM_TIMEBASE_RESOLUTION_HZ    1000000 // 1 MHz, 1 us per tick
#define PWM_TIMEBASE_PERIOD           20000   // 20000 ticks, 20 ms, 50 Hz PWM frequency

#define PWM_FREQUENCY                 50      // 50 Hz PWM frequency
#define LEDC_TIMER_BIT                LEDC_TIMER_14_BIT

#define PWM_MAX_PULSEWIDTH_US         2500
#define PWM_MIN_PULSEWIDTH_US         1000

#define BLDC_MAX_SPEED    (PWM_MAX_PULSEWIDTH_US - PWM_MIN_PULSEWIDTH_US)
#define BLDC_MIN_SPEED    0

#define LED_ON     0
#define LED_OFF    1

// BLDC config struct
typedef struct tiny_bldc_conf_t
{
    gpio_num_t pwm_pin;
    gpio_num_t led_pin;
#ifndef CONFIG_IDF_TARGET_ESP32C3
    uint32_t group_id; // max 3 BLDC motors per group
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operator;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;
#else
    ledc_timer_t timer;
    ledc_channel_t channel;
#endif
} tiny_bldc_conf_t;


void tiny_bldc_init(tiny_bldc_conf_t* bldc_conf);

void tiny_bldc_deinit(tiny_bldc_conf_t* bldc_conf);

void tiny_bldc_set_led(tiny_bldc_conf_t* bldc_conf, uint32_t led_state);

void tiny_bldc_set_speed(tiny_bldc_conf_t* bldc_conf, uint32_t speed);

uint32_t tiny_bldc_soft_start(tiny_bldc_conf_t* bldc_conf);
