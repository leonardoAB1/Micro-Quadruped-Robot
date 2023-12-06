/*******************************************************************************
 * @file        http_handlers.c
 * @author      Leonardo Acha Boiano
 * @date        4  2023
 * 
 * @note        This code is written in C and is used ESP32 BEVKIT V1 board.
 *
 *******************************************************************************/

#include "../http_handlers/http_handlers.h"

// HTTP request handler for getting the camera status
esp_err_t status_httpd_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "status", "online");

    char *response = cJSON_Print(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));

    cJSON_Delete(root);
    free(response);

    return ESP_OK;
}

/********************************* END OF FILE ********************************/
/******************************************************************************/
