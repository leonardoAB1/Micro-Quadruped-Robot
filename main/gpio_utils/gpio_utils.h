/*******************************************************************************
 * @file        gpio_utils.h
 * @author      Leonardo Acha Boiano
 * @date        4 Dec 2023
 * 
 * @note        This code is written in C and is used on an ESP32 38 PIN BOARD.
 *
 *******************************************************************************/
#ifndef GPIO_UTILS_H_
#define GPIO_UTILS_H_

#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_attr.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define HIGH    1
#define LOW     0

/**
 * @brief GPIO Initialization.
 */
esp_err_t init_gpio(void);

#define PWM_FREQUENCY 50
#define PWM_RESOLUTION LEDC_TIMER_16_BIT

#define MOTOR_1A_GPIO GPIO_NUM_0
#define MOTOR_2A_GPIO GPIO_NUM_4
#define MOTOR_3A_GPIO GPIO_NUM_16

#define MOTOR_1B_GPIO GPIO_NUM_12
#define MOTOR_2B_GPIO GPIO_NUM_14
#define MOTOR_3B_GPIO GPIO_NUM_27

#define MOTOR_1C_GPIO GPIO_NUM_17
#define MOTOR_2C_GPIO GPIO_NUM_5
#define MOTOR_3C_GPIO GPIO_NUM_18

#define MOTOR_1D_GPIO GPIO_NUM_26
#define MOTOR_2D_GPIO GPIO_NUM_25
#define MOTOR_3D_GPIO GPIO_NUM_33

#define MOTOR_1A_SPEED_MODE LEDC_HIGH_SPEED_MODE
#define MOTOR_2A_SPEED_MODE LEDC_HIGH_SPEED_MODE
#define MOTOR_3A_SPEED_MODE LEDC_HIGH_SPEED_MODE

#define MOTOR_1B_SPEED_MODE LEDC_HIGH_SPEED_MODE
#define MOTOR_2B_SPEED_MODE LEDC_HIGH_SPEED_MODE
#define MOTOR_3B_SPEED_MODE LEDC_HIGH_SPEED_MODE

#define MOTOR_1C_SPEED_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_2C_SPEED_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_3C_SPEED_MODE LEDC_LOW_SPEED_MODE

#define MOTOR_1D_SPEED_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_2D_SPEED_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_3D_SPEED_MODE LEDC_LOW_SPEED_MODE

#define MOTOR_1AB_TIMER LEDC_TIMER_0
#define MOTOR_1CD_TIMER LEDC_TIMER_1
#define MOTOR_2AB_TIMER LEDC_TIMER_2
#define MOTOR_2CD_TIMER LEDC_TIMER_0
#define MOTOR_3AB_TIMER LEDC_TIMER_1
#define MOTOR_3CD_TIMER LEDC_TIMER_2

#define MOTOR_1A_CHANNEL LEDC_CHANNEL_0 
#define MOTOR_2A_CHANNEL LEDC_CHANNEL_1
#define MOTOR_3A_CHANNEL LEDC_CHANNEL_2 

#define MOTOR_1B_CHANNEL LEDC_CHANNEL_3 
#define MOTOR_2B_CHANNEL LEDC_CHANNEL_4 
#define MOTOR_3B_CHANNEL LEDC_CHANNEL_5 

#define MOTOR_1C_CHANNEL LEDC_CHANNEL_1 
#define MOTOR_2C_CHANNEL LEDC_CHANNEL_2 
#define MOTOR_3C_CHANNEL LEDC_CHANNEL_3 

#define MOTOR_1D_CHANNEL LEDC_CHANNEL_4 
#define MOTOR_2D_CHANNEL LEDC_CHANNEL_5 
#define MOTOR_3D_CHANNEL LEDC_CHANNEL_6 

#endif /* GPIO_UTILS_H_ */

/********************************* END OF FILE ********************************/
/******************************************************************************/
