/*******************************************************************************
 * @file        gpio_utils.c
 * @author      Leonardo Acha Boiano
 * @date        4 Dec 2023
 *
 * @note        This code is written in C and is used on an ESP32 38 PIN BOARD.
 *
 *******************************************************************************/
#include "../gpio_utils/gpio_utils.h"

esp_err_t init_gpio(void) {
    //Initialize Timer
    ledc_timer_config_t timerConfig_1AB = {0};
    timerConfig_1AB.speed_mode          = LEDC_HIGH_SPEED_MODE;
    timerConfig_1AB.duty_resolution     = PWM_RESOLUTION; 
    timerConfig_1AB.timer_num           = MOTOR_1AB_TIMER;
    timerConfig_1AB.freq_hz             = PWM_FREQUENCY; 
    ledc_timer_config(&timerConfig_1AB);

    ledc_timer_config_t timerConfig_1CD = {0};
    timerConfig_1CD.speed_mode          = LEDC_HIGH_SPEED_MODE;
    timerConfig_1CD.duty_resolution     = PWM_RESOLUTION; 
    timerConfig_1CD.timer_num           = MOTOR_1CD_TIMER;
    timerConfig_1CD.freq_hz             = PWM_FREQUENCY; 
    ledc_timer_config(&timerConfig_1CD);

    ledc_timer_config_t timerConfig_2AB = {0};
    timerConfig_2AB.speed_mode          = LEDC_HIGH_SPEED_MODE;
    timerConfig_2AB.duty_resolution     = PWM_RESOLUTION; 
    timerConfig_2AB.timer_num           = MOTOR_2AB_TIMER;
    timerConfig_2AB.freq_hz             = PWM_FREQUENCY; 
    ledc_timer_config(&timerConfig_2AB);

    ledc_timer_config_t timerConfig_2CD = {0};
    timerConfig_2CD.speed_mode          = LEDC_LOW_SPEED_MODE;
    timerConfig_2CD.duty_resolution     = PWM_RESOLUTION; 
    timerConfig_2CD.timer_num           = MOTOR_2CD_TIMER;
    timerConfig_2CD.freq_hz             = PWM_FREQUENCY; 
    ledc_timer_config(&timerConfig_2CD);

    ledc_timer_config_t timerConfig_3AB = {0};
    timerConfig_3AB.speed_mode          = LEDC_LOW_SPEED_MODE;
    timerConfig_3AB.duty_resolution     = PWM_RESOLUTION; 
    timerConfig_3AB.timer_num           = MOTOR_3AB_TIMER;
    timerConfig_3AB.freq_hz             = PWM_FREQUENCY; 
    ledc_timer_config(&timerConfig_3AB);

    ledc_timer_config_t timerConfig_3CD = {0};
    timerConfig_3CD.speed_mode          = LEDC_LOW_SPEED_MODE;
    timerConfig_3CD.duty_resolution     = PWM_RESOLUTION; 
    timerConfig_3CD.timer_num           = MOTOR_3CD_TIMER;
    timerConfig_3CD.freq_hz             = PWM_FREQUENCY; 
    ledc_timer_config(&timerConfig_3CD);


    // Initialize GPIO pins for servos as PWM outputs
    // Motor1A
    ledc_channel_config_t channelConfigMotor1A = {0};
    channelConfigMotor1A.gpio_num     = MOTOR_1A_GPIO;
    channelConfigMotor1A.speed_mode   = MOTOR_1A_SPEED_MODE;
    channelConfigMotor1A.channel      = MOTOR_1A_CHANNEL;
    channelConfigMotor1A.intr_type    = LEDC_INTR_DISABLE;
    channelConfigMotor1A.timer_sel    = MOTOR_1AB_TIMER;
    channelConfigMotor1A.duty         = 32767; //Initial Duty Cycle
    ledc_channel_config(&channelConfigMotor1A);

    // Motor2A
    ledc_channel_config_t channelConfigMotor2A = {0};
    channelConfigMotor2A.gpio_num     = MOTOR_2A_GPIO;
    channelConfigMotor2A.speed_mode   = MOTOR_2A_SPEED_MODE;
    channelConfigMotor2A.channel      = MOTOR_2A_CHANNEL;
    channelConfigMotor2A.intr_type    = LEDC_INTR_DISABLE;
    channelConfigMotor2A.timer_sel    = MOTOR_2AB_TIMER;
    channelConfigMotor2A.duty         = 0; //Initial Duty Cycle
    ledc_channel_config(&channelConfigMotor2A);

    // Motor3A
    ledc_channel_config_t channelConfigMotor3A = {0};
    channelConfigMotor3A.gpio_num     = MOTOR_3A_GPIO;
    channelConfigMotor3A.speed_mode   = MOTOR_3A_SPEED_MODE;
    channelConfigMotor3A.channel      = MOTOR_3A_CHANNEL;
    channelConfigMotor3A.intr_type    = LEDC_INTR_DISABLE;
    channelConfigMotor3A.timer_sel    = MOTOR_3AB_TIMER;
    channelConfigMotor3A.duty         = 0; //Initial Duty Cycle
    ledc_channel_config(&channelConfigMotor3A);

    // Motor1B
    ledc_channel_config_t channelConfigMotor1B = {0};
    channelConfigMotor1B.gpio_num     = MOTOR_1B_GPIO;
    channelConfigMotor1B.speed_mode   = MOTOR_1B_SPEED_MODE;
    channelConfigMotor1B.channel      = MOTOR_1B_CHANNEL;
    channelConfigMotor1B.intr_type    = LEDC_INTR_DISABLE;
    channelConfigMotor1B.timer_sel    = MOTOR_1AB_TIMER;
    channelConfigMotor1B.duty         = 0; //Initial Duty Cycle
    ledc_channel_config(&channelConfigMotor1B);

    // Motor2B
    ledc_channel_config_t channelConfigMotor2B = {0};
    channelConfigMotor2B.gpio_num     = MOTOR_2B_GPIO;
    channelConfigMotor2B.speed_mode   = MOTOR_2B_SPEED_MODE;
    channelConfigMotor2B.channel      = MOTOR_2B_CHANNEL;
    channelConfigMotor2B.intr_type    = LEDC_INTR_DISABLE;
    channelConfigMotor2B.timer_sel    = MOTOR_2AB_TIMER;
    channelConfigMotor2B.duty         = 0; //Initial Duty Cycle
    ledc_channel_config(&channelConfigMotor2B);

    // Motor3B
    ledc_channel_config_t channelConfigMotor3B = {0};
    channelConfigMotor3B.gpio_num     = MOTOR_3B_GPIO;
    channelConfigMotor3B.speed_mode   = MOTOR_3B_SPEED_MODE;
    channelConfigMotor3B.channel      = MOTOR_3B_CHANNEL;
    channelConfigMotor3B.intr_type    = LEDC_INTR_DISABLE;
    channelConfigMotor3B.timer_sel    = MOTOR_3AB_TIMER;
    channelConfigMotor3B.duty         = 0; //Initial Duty Cycle
    ledc_channel_config(&channelConfigMotor3B);

    // Motor1C
    ledc_channel_config_t channelConfigMotor1C = {0};
    channelConfigMotor1C.gpio_num     = MOTOR_1C_GPIO;
    channelConfigMotor1C.speed_mode   = MOTOR_1C_SPEED_MODE;
    channelConfigMotor1C.channel      = MOTOR_1C_CHANNEL;
    channelConfigMotor1C.intr_type    = LEDC_INTR_DISABLE;
    channelConfigMotor1C.timer_sel    = MOTOR_1CD_TIMER;
    channelConfigMotor1C.duty         = 0; //Initial Duty Cycle
    ledc_channel_config(&channelConfigMotor1C);

    // Motor2C
    ledc_channel_config_t channelConfigMotor2C = {0};
    channelConfigMotor2C.gpio_num     = MOTOR_2C_GPIO;
    channelConfigMotor2C.speed_mode   = MOTOR_2C_SPEED_MODE;
    channelConfigMotor2C.channel      = MOTOR_2C_CHANNEL;
    channelConfigMotor2C.intr_type    = LEDC_INTR_DISABLE;
    channelConfigMotor2C.timer_sel    = MOTOR_2CD_TIMER;
    channelConfigMotor2C.duty         = 0; //Initial Duty Cycle
    ledc_channel_config(&channelConfigMotor2C);

    // Motor3C
    ledc_channel_config_t channelConfigMotor3C = {0};
    channelConfigMotor3C.gpio_num     = MOTOR_3C_GPIO;
    channelConfigMotor3C.speed_mode   = MOTOR_3C_SPEED_MODE;
    channelConfigMotor3C.channel      = MOTOR_3C_CHANNEL;
    channelConfigMotor3C.intr_type    = LEDC_INTR_DISABLE;
    channelConfigMotor3C.timer_sel    = MOTOR_3CD_TIMER;
    channelConfigMotor3C.duty         = 0; //Initial Duty Cycle
    ledc_channel_config(&channelConfigMotor3C);


    // Motor1D
    ledc_channel_config_t channelConfigMotor1D = {0};
    channelConfigMotor1D.gpio_num     = MOTOR_1D_GPIO;
    channelConfigMotor1D.speed_mode   = MOTOR_1D_SPEED_MODE;
    channelConfigMotor1D.channel      = MOTOR_1D_CHANNEL;
    channelConfigMotor1D.intr_type    = LEDC_INTR_DISABLE;
    channelConfigMotor1D.timer_sel    = MOTOR_1CD_TIMER;
    channelConfigMotor1D.duty         = 0; //Initial Duty Cycle
    ledc_channel_config(&channelConfigMotor1D);

    // Motor2D
    ledc_channel_config_t channelConfigMotor2D = {0};
    channelConfigMotor2D.gpio_num     = MOTOR_2D_GPIO;
    channelConfigMotor2D.speed_mode   = MOTOR_2D_SPEED_MODE;
    channelConfigMotor2D.channel      = MOTOR_2D_CHANNEL;
    channelConfigMotor2D.intr_type    = LEDC_INTR_DISABLE;
    channelConfigMotor2D.timer_sel    = MOTOR_2CD_TIMER;
    channelConfigMotor2D.duty         = 0; //Initial Duty Cycle
    ledc_channel_config(&channelConfigMotor2D);

    // Motor3D
    ledc_channel_config_t channelConfigMotor3D = {0};
    channelConfigMotor3D.gpio_num     = MOTOR_3D_GPIO;
    channelConfigMotor3D.speed_mode   = MOTOR_3D_SPEED_MODE;
    channelConfigMotor3D.channel      = MOTOR_3D_CHANNEL;
    channelConfigMotor3D.intr_type    = LEDC_INTR_DISABLE;
    channelConfigMotor3D.timer_sel    = MOTOR_3CD_TIMER;
    channelConfigMotor3D.duty         = 0; //Initial Duty Cycle
    ledc_channel_config(&channelConfigMotor3D);

    return ESP_OK;
}

/********************************* END OF FILE ********************************/
/******************************************************************************/
