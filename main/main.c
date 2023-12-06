/*******************************************************************************
 * @file        main.c
 * @author      Leonardo Acha Boiano
 * @date        4 Dec 2023
 * @brief       Main code Micro Quadruped Robot
 * 
 * @note        This code is written in C and is used on an ESP32 WROOM 38 pin development board.
 *
 *******************************************************************************/

#include "main.h"

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    // Initialize NVS (Non-Volatile Storage)
    nvs_flash_init_custom(ret);

    // Connect to WiFi
    connect_wifi();

    // Start the web server
    start_webserver();

    // Initialize GPIO pins
    init_gpio();

    //Init leg structures
    set_M1A(0);
    vTaskDelay(2000/portTICK_PERIOD_MS );
    set_M1A(90);
    vTaskDelay(2000/portTICK_PERIOD_MS );
    set_M1A(180);

}

esp_err_t nvs_flash_init_custom(esp_err_t ret){
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    return ESP_OK;
}
