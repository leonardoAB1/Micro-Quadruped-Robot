/*******************************************************************************
 * @file        http_handlers.h
 * @brief       Header file containing declarations of HTTP request handlers.
 * @author      Leonardo Acha Boiano
 * @date        4 Dec 2023
 * 
 * @note        This code is written in C and is used on an ESP32 38 PIN development board.
 *
 *******************************************************************************/

#ifndef HTTP_HANDLERS_H
#define HTTP_HANDLERS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <esp_log.h>
#include <cJSON.h>
#include <esp_http_server.h>

#include "../logging/logging_utils.h"
#include "../servo_driver/servo_driver.h"

/**
 * @brief       HTTP request handler for getting the camera status.
 * @param[in]   req The HTTP request object.
 * @return      An esp_err_t indicating the success or failure of the operation.
 */
esp_err_t status_httpd_handler(httpd_req_t *req);
esp_err_t move_M1A_handler(httpd_req_t *req);

#endif  // HTTP_HANDLERS_H

/********************************* END OF FILE ********************************/
/******************************************************************************/
