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
    uint32_t compare = (speed - BLDC_MIN_SPEED) * (PWM_MAX_PULSEWIDTH_US - PWM_MIN_PULSEWIDTH_US) /
                            (BLDC_MAX_SPEED - BLDC_MIN_SPEED) + PWM_MIN_PULSEWIDTH_US;

#ifdef CONFIG_IDF_TARGET_ESP32C3
    // set duty [%]: (2^SOC_LEDC_TIMER_BIT_WIDTH) * duty / 100
    compare = (uint32_t)((float)(1 << LEDC_TIMER_BIT) * ((float)compare / (float)PWM_TIMEBASE_PERIOD));
#endif

    return compare;
}


/**
 * @brief initialize the tiny_bldc
 * 
 * @param bldc_conf bldc config struct pointer
*/
void tiny_bldc_init(tiny_bldc_conf_t* bldc_conf)
{
    // init PWM pin
#ifndef CONFIG_IDF_TARGET_ESP32C3
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
#else
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = bldc_conf->timer,
        .duty_resolution = LEDC_TIMER_BIT,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

    ledc_channel_config_t pwm_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = bldc_conf->channel,
        .timer_sel = bldc_conf->timer,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = bldc_conf->pwm_pin,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel));
#endif

    // set zero speed
    tiny_bldc_set_speed(bldc_conf, BLDC_MIN_SPEED);
}


/**
 * @brief deinitialize the tiny_bldc
 * 
 * @param bldc_conf bldc config struct pointer
*/
void tiny_bldc_deinit(tiny_bldc_conf_t* bldc_conf)
{
#ifndef CONFIG_IDF_TARGET_ESP32C3
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
#else
    ledc_stop(LEDC_LOW_SPEED_MODE, bldc_conf->channel, 0);
#endif
}


/**
 * @brief set the speed of the motor
 * 
 * @param bldc_conf bldc config struct pointer
 * @param speed speed of the motor (BLDC_MIN_SPEED ~ BLDC_MAX_SPEED)
*/
void tiny_bldc_set_speed(tiny_bldc_conf_t* bldc_conf, uint32_t speed)
{
    if (speed < BLDC_MIN_SPEED || speed > BLDC_MAX_SPEED)
    {
        ESP_LOGE(TAG, "Invalid speed");
        return;
    }

#ifndef CONFIG_IDF_TARGET_ESP32C3
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(bldc_conf->comparator, map_speed_to_compare(speed)));
#else
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, bldc_conf->channel, map_speed_to_compare(speed)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, bldc_conf->channel));
#endif
}


/**
 * @brief arm the motors
 *
 * @param bldc_conf bldc config struct pointer
 * @param motors_num number of motors
*/
void tiny_bldc_arm(tiny_bldc_conf_t* bldc_conf, uint8_t motors_num)
{
    vTaskDelay(1500 / portTICK_PERIOD_MS);

    for (uint32_t i = 0; i < motors_num; i++)
       tiny_bldc_set_speed(&bldc_conf[i], BLDC_MIN_SPEED);
    vTaskDelay(20 / portTICK_PERIOD_MS);

    for (uint32_t i = 0; i < motors_num; i++)
        tiny_bldc_set_speed(&bldc_conf[i], BLDC_MIN_ROT_SPEED);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    for (uint32_t i = 0; i < motors_num; i++)
        tiny_bldc_set_speed(&bldc_conf[i], BLDC_MIN_SPEED);
    vTaskDelay(20 / portTICK_PERIOD_MS);
}


/**
 * @brief calibrate the motors
 * 
 * @param bldc_conf bldc config struct pointer
 * @param motors_num number of motors
*/
void tiny_bldc_calibrate(tiny_bldc_conf_t* bldc_conf, uint8_t motors_num)
{
    for (uint32_t i = 0; i < motors_num; i++)
        tiny_bldc_set_speed(&bldc_conf[i], BLDC_MIN_SPEED);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    for (uint32_t i = 0; i < motors_num; i++)
        tiny_bldc_set_speed(&bldc_conf[i], BLDC_MAX_SPEED);
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    for (uint32_t i = 0; i < motors_num; i++)
        tiny_bldc_set_speed(&bldc_conf[i], BLDC_MIN_SPEED);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
}
