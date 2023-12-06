/*******************************************************************************
 * @file        servo_driver.c
 * @author      Leonardo Acha Boiano
 * @date        5 Dec 2023
 * 
  * @note        This code is written in C and is used on an ESP32 38 PIN development board.
 *
 *******************************************************************************/
#include "../servo_driver/servo_driver.h"

esp_err_t servoDeg(uint16_t degress, uint8_t speed_mode, uint8_t channel) {

    float_t servo_ms = ((SERVO_MAX_MS-SERVO_MIN_MS)/180)*degress+SERVO_MIN_MS;
    uint16_t duty = (uint16_t)(100.0*(servo_ms/PWM_PERIOD_MS)*(pow(2, PWM_RESOLUTION)/100));
    printf("%fms, duty = %f%% -> %d\n",SERVO_MIN_MS, 100.0*(servo_ms/PWM_PERIOD_MS), duty);

    // Set the LEDC duty, the range of duty setting is [0, (2**PWM_RESOLUTION) - 1]
    esp_err_t result = ledc_set_duty(speed_mode, channel, duty); 
    result |= ledc_update_duty(speed_mode, channel);

    MOTOR_CHECK(ESP_OK == result, "write motor pwm failed", ESP_FAIL);

    return ESP_OK;
}

void set_M1A(uint16_t degress){
    servoDeg(degress, MOTOR_1A_SPEED_MODE, MOTOR_1A_CHANNEL);
}

void set_M2A(uint16_t degress){
    servoDeg(degress, MOTOR_2A_SPEED_MODE, MOTOR_2A_CHANNEL);
}

void set_M3A(uint16_t degress){
    servoDeg(degress, MOTOR_3A_SPEED_MODE, MOTOR_3A_CHANNEL);
}

void set_M1B(uint16_t degress){
    servoDeg(degress, MOTOR_1B_SPEED_MODE, MOTOR_1B_CHANNEL);
}

void set_M2B(uint16_t degress){
    servoDeg(degress, MOTOR_2B_SPEED_MODE, MOTOR_2B_CHANNEL);
}

void set_M3B(uint16_t degress){
    servoDeg(degress, MOTOR_3B_SPEED_MODE, MOTOR_3B_CHANNEL);
}

void set_M1C(uint16_t degress){
    servoDeg(degress, MOTOR_1C_SPEED_MODE, MOTOR_1C_CHANNEL);
}

void set_M2C(uint16_t degress){
    servoDeg(degress, MOTOR_2C_SPEED_MODE, MOTOR_2C_CHANNEL);
}

void set_M3C(uint16_t degress){
    servoDeg(degress, MOTOR_3C_SPEED_MODE, MOTOR_3C_CHANNEL);
}

void set_M1D(uint16_t degress){
    servoDeg(degress, MOTOR_1D_SPEED_MODE, MOTOR_1D_CHANNEL);
}

void set_M2D(uint16_t degress){
    servoDeg(degress, MOTOR_2D_SPEED_MODE, MOTOR_2D_CHANNEL);
}

void set_M3D(uint16_t degress){
    servoDeg(degress, MOTOR_3D_SPEED_MODE, MOTOR_3D_CHANNEL);
}

/********************************* END OF FILE ********************************/
/******************************************************************************/
