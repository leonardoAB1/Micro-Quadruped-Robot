/*******************************************************************************
 * @file        main.h
 * @author      Leonardo Acha Boiano
 * @date        4 Dec 2023
 * @brief       Main code, Micro Quadruped Robot.
 * 
 * @note        This code is written in C and is used on the ESP32 38PIN DEVboard.
 *
 *******************************************************************************/

#ifndef MAIN_H
#define MAIN_H

#include "stdio.h"
#include "config.h"

#include <esp_system.h>
#include <nvs_flash.h>

#include "connect_wifi/connect_wifi.h"
#include "gpio_utils/gpio_utils.h"
#include "web_server/web_server.c"
#include "http_handlers/http_handlers.h"
#include "logging/logging_utils.h"
#include "servo_driver/servo_driver.h"

// Function prototypes
esp_err_t nvs_flash_init_custom(esp_err_t ret);

#endif /* MAIN_H */

/********************************* END OF FILE ********************************/
/******************************************************************************/
