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

#define PWM_TIMEBASE_RESOLUTION_HZ    1000000    // 1 MHz, 1 us per tick
#define PWM_TIMEBASE_PERIOD           20000      // 20000 ticks, 20 ms, 50 Hz PWM frequency

// #define PWM_TIMEBASE_RESOLUTION_HZ    10000000    // 10 MHz, 0.1 us per tick
// #define PWM_TIMEBASE_PERIOD           200         // 200 ticks, 0.02 ms, 50 kHz PWM frequency


#define PWM_MAX_PULSEWIDTH_US    2500
#define PWM_MIN_PULSEWIDTH_US    1000

#define BLDC_MAX_SPEED    (PWM_MAX_PULSEWIDTH_US - PWM_MIN_PULSEWIDTH_US)
#define BLDC_MIN_SPEED    0

#define LED_ON     0
#define LED_OFF    1

typedef struct tiny_bldc_conf_t
{
    gpio_num_t pwm_pin;
    gpio_num_t led_pin;
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operator;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;
} tiny_bldc_conf_t;


void tiny_bldc_init(tiny_bldc_conf_t* bldc_conf);

void tiny_bldc_deinit(tiny_bldc_conf_t* bldc_conf);

void tiny_bldc_set_led(tiny_bldc_conf_t* bldc_conf, uint32_t led_state);

void tiny_bldc_set_speed(tiny_bldc_conf_t* bldc_conf, uint32_t speed);
