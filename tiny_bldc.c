/**
 * @file tiny_bldc.c
 * @author JanG175
 * @brief ESP-IDF component for tiny BLDC driver
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include "tiny_bldc.h"


static const char *TAG = "tiny_bldc";


/**
 * @brief map speed to compare value for PWM
 * 
 * @param speed speed of the motor (BLDC_MIN_SPEED ~ BLDC_MAX_SPEED)
 * 
 * @return uint32_t compare value
*/
static inline uint32_t map_speed_to_compare(uint32_t speed)
{
    return (speed - BLDC_MIN_SPEED) * (PWM_MAX_PULSEWIDTH_US - PWM_MIN_PULSEWIDTH_US) /
                (BLDC_MAX_SPEED - BLDC_MIN_SPEED) + PWM_MIN_PULSEWIDTH_US;
}


/**
 * @brief initialize the tiny_bldc
 * 
 * @param bldc_conf bldc config struct pointer
*/
void tiny_bldc_init(tiny_bldc_conf_t* bldc_conf)
{
    // init LED pin
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << bldc_conf->led_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(bldc_conf->led_pin, LED_ON);

    // init PWM pin
    mcpwm_timer_config_t timer_config = {
        .group_id = bldc_conf->group_id,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = PWM_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &(bldc_conf->timer)));

    mcpwm_operator_config_t operator_config = {
        .group_id = bldc_conf->group_id,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &(bldc_conf->operator)));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(bldc_conf->operator, bldc_conf->timer));

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(bldc_conf->operator, &comparator_config, &(bldc_conf->comparator)));

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = bldc_conf->pwm_pin
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(bldc_conf->operator, &generator_config, &(bldc_conf->generator)));

    // set zero init speed
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(bldc_conf->comparator, map_speed_to_compare(BLDC_MIN_SPEED)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(bldc_conf->generator,
                                                                    MCPWM_GEN_TIMER_EVENT_ACTION(
                                                                        MCPWM_TIMER_DIRECTION_UP,
                                                                        MCPWM_TIMER_EVENT_EMPTY,
                                                                        MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(bldc_conf->generator,
                                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(
                                                                        MCPWM_TIMER_DIRECTION_UP,
                                                                        bldc_conf->comparator,
                                                                        MCPWM_GEN_ACTION_LOW)));

    // enable PWM
    ESP_ERROR_CHECK(mcpwm_timer_enable(bldc_conf->timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(bldc_conf->timer, MCPWM_TIMER_START_NO_STOP));
}


/**
 * @brief deinitialize the tiny_bldc
 * 
 * @param bldc_conf bldc config struct pointer
*/
void tiny_bldc_deinit(tiny_bldc_conf_t* bldc_conf)
{
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(bldc_conf->timer, MCPWM_TIMER_STOP_EMPTY));
    ESP_ERROR_CHECK(mcpwm_timer_disable(bldc_conf->timer));

    ESP_ERROR_CHECK(mcpwm_del_generator(bldc_conf->generator));
    ESP_ERROR_CHECK(mcpwm_del_comparator(bldc_conf->comparator));
    ESP_ERROR_CHECK(mcpwm_del_operator(bldc_conf->operator));
    ESP_ERROR_CHECK(mcpwm_del_timer(bldc_conf->timer));

    bldc_conf->timer = NULL;
    bldc_conf->operator = NULL;
    bldc_conf->comparator = NULL;
    bldc_conf->generator = NULL;

    gpio_reset_pin(bldc_conf->led_pin);
}



/**
 * @brief set the state of the onboard LED
 * 
 * @param bldc_conf bldc config struct pointer
 * @param led_state state of the LED (LED_ON or LED_OFF)
*/
void tiny_bldc_set_led(tiny_bldc_conf_t* bldc_conf, uint32_t led_state)
{
    if (led_state != LED_ON && led_state != LED_OFF)
    {
        ESP_LOGE(TAG, "Invalid LED state");
        return;
    }

    gpio_set_level(bldc_conf->led_pin, led_state);
}


/**
 * @brief set the speed of the motor
 * 
 * @param bldc_conf bldc config struct pointer
 * @param speed speed of the motor (BLDC_MIN_SPEED ~ BLDC_MAX_SPEED)
*/
void tiny_bldc_set_speed(tiny_bldc_conf_t* bldc_conf, uint32_t speed)
{
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(bldc_conf->comparator, map_speed_to_compare(speed)));
}


/**
 * @brief soft start the motor (recommended for the first start)
 * 
 * @param bldc_conf bldc config struct pointer
 * 
 * @return speed after soft start
*/
uint32_t tiny_bldc_soft_start(tiny_bldc_conf_t* bldc_conf)
{
    uint32_t end_speed = BLDC_MIN_SPEED + 100;

    for (uint32_t i = BLDC_MIN_SPEED; i <= end_speed; i++)
    {
        tiny_bldc_set_speed(bldc_conf, i);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    return end_speed;
}