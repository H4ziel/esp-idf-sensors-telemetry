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

#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_LOW
#define ADDR MPU6050_I2C_ADDRESS_LOW // quando o pino AD0 está conectado no GND, o endereço do dispositivo mpu6050 vai ser 0x68
#else
#define ADDR MPU6050_I2C_ADDRESS_HIGH // quando o pino AD0 está conectado no VCC, o endereço do dispositivo mpu6050 vai ser 0x69
#endif

#define BUFFER 2024

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
    char raw_lat[10];
    char lat[30];
    char lat_dir[1];
    char raw_lon[11];
    char lon[30];
    char lon_dir[1];
    float altitude;
    float speed;
    float course; 
    char buf[BUFFER];
}variable;

variable vars;
void gy87(void*);
void gps_init(void);
void get_nmea(void*);
void gps_neo6m(void*);
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
    xTaskCreatePinnedToCore(gy87, "gy87", configMINIMAL_STACK_SIZE + 2000, (void*)&vars, configMAX_PRIORITIES - 2, NULL, 0);
    xTaskCreatePinnedToCore(gps_neo6m, "gps_neo6m", configMINIMAL_STACK_SIZE + 2000 , (void*)&vars, configMAX_PRIORITIES - 2, NULL, 0);
    xTaskCreatePinnedToCore(get_nmea, "get_nmea", configMINIMAL_STACK_SIZE + 2000, (void*)&vars, configMAX_PRIORITIES - 2, NULL, 0);
}

void gy87(void *pvParameters)
{
    variable *variables = (variable*)pvParameters;

    mpu6050_dev_t dev = { 0 }; // é definido todos os valores da estrutura do dispositivo mpu6050 como 0 (valores padrões), na linha 194 do arquivo mpu6050.h é possível ver q o valor padrão da escala do accel é 2
    bmp180_dev_t dev2;
    memset(&dev2, 0, sizeof(bmp180_dev_t)); // inicia o dev2  alocando memoria do size da estrutura bmp180_dev_t e preenchendo com zeros, parecido com a linha do mpu6050 dev.

    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, ADDR, 0, CONFIG_EXAMPLE_SDA_GPIO, CONFIG_EXAMPLE_SCL_GPIO)); // verifica se o dispositivo dev (device mpu6050) foi iniciado corretamento, com o objeto dev criado, endereço do sensor, i2c0, e as gpios SDA E SCL (são configuradas pelo menuconfig (idf.py menuconfig))
    ESP_ERROR_CHECK(bmp180_init_desc(&dev2, 0, CONFIG_EXAMPLE_SDA_GPIO, CONFIG_EXAMPLE_SCL_GPIO));
    ESP_ERROR_CHECK(bmp180_init(&dev2));

    while (1) // loop para encontrar o sensor e configurar o protocolo i2c
    {
        esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "Found MPU60x0 device");
            break;
        }
        ESP_LOGE(TAG, "Gy-87 not found");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(mpu6050_init(&dev));

    //ESP_LOGI(TAG, "Accel range: %d", dev.ranges.accel);
    //ESP_LOGI(TAG, "Gyro range:  %d", dev.ranges.gyro);

    while (1)
    {
        mpu6050_acceleration_t accel = { 0 };
        mpu6050_rotation_t rotation = { 0 };
        
        ESP_ERROR_CHECK(bmp180_measure(&dev2, &variables->temp_bmp, &variables->pressure_bmp, BMP180_MODE_STANDARD));

        ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &variables->temp_mpu6050));
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));
        ESP_ERROR_CHECK(mpu6050_get_acceleration(&dev, &accel));

        variables->anglePitchRad = atan((-accel.x)/sqrt(pow(accel.y, 2) + pow(accel.z, 2)));
        variables->angleRollRad = atan((-accel.y)/sqrt(pow(accel.x, 2) + pow(accel.z, 2)));

        variables->anglePitchDeg = variables->anglePitchRad*(180.0/M_PI);
        variables->angleRollDeg = variables->angleRollRad*(180.0/M_PI) - 1.5; //offset

        variables->temp = (variables->temp_bmp+variables->temp_mpu6050)/2.0; // média entre as temperaturas medidas entre os dois sensores.

        ESP_LOGI(TAG, "**********************************************************************");
        ESP_LOGI(TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", accel.x, accel.y, accel.z);
        ESP_LOGI(TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", rotation.x, rotation.y, rotation.z);
        ESP_LOGI(TAG, "Angles: Pitch=%.1f   Roll=%.1f", variables->anglePitchDeg, variables->angleRollDeg);
        ESP_LOGI(TAG, "Temperature:  %.2f  Pressão:  %" PRIu32 " Pa", variables->temp, variables->pressure_bmp); // pra referenciar variavel do tipo uint32_t utiliza-se %" PRIu32 " da lib inttypes.h

        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL); // obtenção de espaço livre na task em words
        ESP_LOGI(TAG,"Espaço mínimo livre na stack: %u\n", uxHighWaterMark);
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void gps_neo6m(void *pvParameters)
{
    variable *variables = (variable*)pvParameters;

    gps_init(); // inicia o gps, evitar usar essa função mais de 1 vez.  

    while(1){
        ESP_LOGI(TAG1, "Latitude: %s %.1s", variables->lat, variables->lat_dir);
        ESP_LOGI(TAG1, "Longitude: %.11s %.1s", variables->lon, variables->lon_dir);
        ESP_LOGI(TAG1, "Altitude: %.2f", variables->altitude);
        ESP_LOGI(TAG1, "Velocidade: %.3f", variables->speed);

        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL); // obtenção de espaço livre na task em words
        ESP_LOGI(TAG1,"Espaço mínimo livre na stack: %u\n", uxHighWaterMark);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void gps_init(void)
{
    const uart_port_t uart_numeration = UART_NUM_2; // indica que a UART 2 será utilizada, referente aos pinos GPIO01(tx) e GPIO03(rx) do esp

    uart_config_t uart_configuration = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(uart_numeration, &uart_configuration));
    ESP_ERROR_CHECK(uart_set_pin(uart_numeration, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_numeration, BUFFER*2, 0, 0, NULL, 0));

    vTaskDelay(pdMS_TO_TICKS(500));
}

void get_nmea(void *pvParameters){
    variable *variables = (variable*)pvParameters;

    char aux_lat[6]; // Para armazenar os caracteres do índice 5 ao 9
    char aux_lon[7];
    float aux_lat_float;
    float aux_lon_float;

    const char *GGA;  // identificador que possui latitude e longitude
    //const char *VTG; // identificador que possui velocidade em Km/h
    memset(variables->buf, 0, BUFFER);

    while(1){
        uart_read_bytes(UART_NUM_2, variables->buf, BUFFER, pdMS_TO_TICKS(1000));
        //ESP_LOGI(TAG1, "%s\n", variables->buf);

        GGA = strstr(variables->buf, "$GPGGA");
        if (GGA != NULL) {
            sscanf(GGA, "$GPGGA,%*f,%10[^,],%1[^,],%11[^,],%1[^,],%*d,%*f,%*f,%f", variables->raw_lat, variables->lat_dir, variables->raw_lon, variables->lon_dir, &variables->altitude);
        }

        sscanf(variables->buf, "$GPVTG,,%*s,,%*s,%*f,%*s, %f,%*s,%*s", &variables->speed);

        // Extrai os caracteres do índice 5 ao 9
        strncpy(aux_lat, variables->raw_lat + 5, 5);
        aux_lat[5] = '\0'; // Adiciona o terminador nulo

        strncpy(aux_lon, variables->raw_lon + 6, 5);
        aux_lon[5] = '\0'; // Adiciona o terminador nulo

        // Converte a substring para float
        aux_lat_float = atof(aux_lat);
        aux_lon_float = atof(aux_lon);

        // novas strings
        snprintf(variables->lat, sizeof(variables->lat), "%c%c\xB0%c%c'%.3f\x22", variables->raw_lat[0], variables->raw_lat[1], variables->raw_lat[2], variables->raw_lat[3], aux_lat_float * 60 / 100000);
        snprintf(variables->lon, sizeof(variables->lon), "%c%c%c\xB0%c%c'%.3f\x22", variables->raw_lon[0], variables->raw_lon[1], variables->raw_lon[2], variables->raw_lon[3], variables->raw_lon[4], aux_lon_float * 60 / 100000);
    }   
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
    char msg[50];
    if(xSemaphoreTake(mqttConnection, portMAX_DELAY))
    {
        while(true)
        {
            sprintf(msg, "{\n  \"data\":\n  {\n    \"Test\": %f\n  }\n}", 
                                                            variables->temp);
            mqtt_publish_msg("wnology/66cb41ac09d56a857e5fdda2/state", msg);
            printf("{\n  \"data\":\n  {\n    \"Test\": %f\n  }\n}", 
                                                            variables->temp);
            vTaskDelay(pdMS_TO_TICKS(3000));

            UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL); // obtenção de espaço livre na task em words
            ESP_LOGI(TAG2,"Espaço mínimo livre na stack: %u\n", uxHighWaterMark);
        }
    }
}