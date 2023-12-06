/*******************************************************************************
 * @file        servo_driver.h
 * @author      Leonardo Acha Boiano
 * @date        5 Dec 2023
 * 
  * @note        This code is written in C and is used on an ESP32 38 PIN development board.
 *
 *******************************************************************************/
#ifndef SERVO_DRIVER_H_
#define SERVO_DRIVER_H_

#include <stdint.h>
#include "math.h"
#include "../gpio_utils/gpio_utils.h"
#include "../logging/logging_utils.h"

#define SERVO_FULL_DUTY ((1 << PWM_RESOLUTION) - 1)  // Maximum duty cycle value

#define SERVO_MIN_MS 0.06 // The minimum value that causes the rotation to be 0 degrees
#define SERVO_MAX_MS 2.5  // The maximum value that causes the rotation to be 180 degrees
#define SERVO_AVG_MS ((SERVO_MAX_MS-SERVO_MIN_MS)/2.0)

#define PWM_PERIOD_MS 20
#define SERVO_MAX_ANGLE 180

#define MOTOR_CHECK(a, str, ret_val) \
    if (!(a)) { \
        ESP_LOGE(MOTOR_TAG, "%s(%d): %s", __FUNCTION__, __LINE__, str); \
        return (ret_val); \
    }

esp_err_t servoDeg(uint16_t degrees, uint8_t speed_mode, uint8_t channel);
void set_M1A(uint16_t degrees);
void set_M2A(uint16_t degrees);
void set_M3A(uint16_t degrees);
void set_M1B(uint16_t degrees);
void set_M2B(uint16_t degrees);
void set_M3B(uint16_t degrees);
void set_M1C(uint16_t degrees);
void set_M2C(uint16_t degrees);
void set_M3C(uint16_t degrees);
void set_M1D(uint16_t degrees);
void set_M2D(uint16_t degrees);
void set_M3D(uint16_t degrees);

#endif /* SERVO_DRIVER_H_ */

/********************************* END OF FILE ********************************/
/******************************************************************************/
