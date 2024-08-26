#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

#define SSID "Tomatinho"
#define PASSWORD "naoseiasenha" 

extern xSemaphoreHandle wifiConnection;
    
void wifi_connect(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t _wifi_config = {
        .sta = {
            .ssid = SSID,
            .password = PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &_wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    if(esp_wifi_connect() == ESP_OK)
    {
        xSemaphoreGive(wifiConnection);
    }
}