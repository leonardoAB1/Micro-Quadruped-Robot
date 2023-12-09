/*******************************************************************************
 * @file        web_server.c
 * @brief       Web Server Implementation
 * @details     This file contains the implementation of a web server.
 *              It provides functions to start the web server and register URI handlers.
 * @author      Leonardo Acha Boiano
 * @date        4 Dec 2023
 *
 * @note        This code is written in C and is used on the ESP32 38 PIN BOARD.
 *
 *******************************************************************************/

#include "../web_server/web_server.h"

// Function to start the web server
void start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.core_id = 1;  // Set the core ID to 0 (core 0)
    config.max_uri_handlers = 20;
    config.task_priority    = tskIDLE_PRIORITY+3;
    //config.uri_match_fn     = httpd_uri_match_wildcard;

    // Start the HTTP server
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Set URI handlers
        httpd_uri_t status_uri = {
            .uri = "/status",
            .method = HTTP_GET,
            .handler = status_httpd_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &status_uri);

        httpd_uri_t move_M1A_uri = {
            .uri = "/M1A/move",
            .method = HTTP_POST,
            .handler = move_M1A_handler, //needs motor number as a header
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &move_M1A_uri);
    }
}

/********************************* END OF FILE ********************************/
/******************************************************************************/
