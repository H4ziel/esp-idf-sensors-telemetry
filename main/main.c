#include <inttypes.h>
#include <stdio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <math.h>

#include "driver/uart.h"
#include "hal/uart_types.h"
#include "portmacro.h"

#include "mpu6050.h"
#include "bmp180.h"
#include "mqtt/mqtt.h"
#include "wifi/wifi.h"
#include "lora.h"

#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_LOW
#define ADDR MPU6050_I2C_ADDRESS_LOW // quando o pino AD0 está conectado no GND, o endereço do dispositivo mpu6050 vai ser 0x68
#else
#define ADDR MPU6050_I2C_ADDRESS_HIGH // quando o pino AD0 está conectado no VCC, o endereço do dispositivo mpu6050 vai ser 0x69
#endif

#define BUFFER 2024
#define FREQUENCY 915e6
#define BW 125e3

#define TX_PIN 17
#define RX_PIN 16

static const char *TAG = "gy_87";  // TAG é utilizada nas funções ESP_LOG para referenciar tal função ou parte do código
static const char *TAG1 = "GPS_NEO6m";
static const char *TAG2 = "MQTT";

typedef struct{
    uint32_t pressure_bmp;
    float temp;
    float temp_mpu6050;
    float temp_bmp;
    float anglePitchRad;
    float anglePitchDeg;
    float angleRollRad;
    float angleRollDeg;
    float raw_lat;
    float lat;
    char lat_dir[1];
    float raw_lon;
    float lon;
    char lon_dir[1];
    float altitude;
    float speed;
    float course; 
    char buf[BUFFER];
}variable;

variable vars;

esp_err_t setupLoRa(void);
void receiveLoRaData(void*);
void wifi_treat(void*);
void mqtt_treat(void*);

xSemaphoreHandle wifiConnection;
xSemaphoreHandle mqttConnection;

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    // Initialize NETIF
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == 
                                                ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifiConnection = xSemaphoreCreateBinary();
    mqttConnection = xSemaphoreCreateBinary();

    wifi_connect();

    xTaskCreatePinnedToCore(wifi_treat, "Tratamento Wifi", configMINIMAL_STACK_SIZE + 2000, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    xTaskCreatePinnedToCore(mqtt_treat, "Tratamento Mqtt", configMINIMAL_STACK_SIZE + 2000, (void*)&vars, configMAX_PRIORITIES - 2, NULL, 1);

}


void wifi_treat(void *pvParameters)
{
    while(true)
    {
        if(xSemaphoreTake(wifiConnection, portMAX_DELAY))
        {
            mqtt_start();
        }
    }
}

void mqtt_treat(void *pvParameters)
{
    variable *variables = (variable*)pvParameters;
    char msg[500];
    if(xSemaphoreTake(mqttConnection, portMAX_DELAY))
    {
        while(true)
        {
            sprintf(msg, "{\n  \"data\":\n  {\n    \"Test\": %f,\n    \"gpstrack\": \"%f,%f\"\n  }\n}", variables->temp, variables->lat, variables->lon);
            mqtt_publish_msg("wnology//state", msg);
            printf(msg);
            vTaskDelay(pdMS_TO_TICKS(3000));

            UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI(TAG2,"Espaço mínimo livre na stack: %u\n",uxHighWaterMark);
        }
    }
}