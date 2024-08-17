#include <inttypes.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <mpu6050.h>
#include <math.h>
#include "bmp180.h"
#include <string.h>

//#define configMAX_PRIORITIES 5 // nível máximo de prioridades ta definido como 25 no arquivo freertosconfig.h, então as prioridades são de 0(menor prioridade) a 24(maior prioridade), adicionar mais níveis de prioridade requer RAM

#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_LOW
#define ADDR MPU6050_I2C_ADDRESS_LOW // quando o pino AD0 está conectado no GND, o endereço do dispositivo mpu6050 vai ser 0x68
#else
#define ADDR MPU6050_I2C_ADDRESS_HIGH // quando o pino AD0 está conectado no VCC, o endereço do dispositivo mpu6050 vai ser 0x69
#endif

static const char *TAG = "gy_87 test";  // TAG é utilizada nas funções ESP_LOG para referenciar tal função ou parte do código

typedef struct{
    uint32_t pressure_bmp;
    float temp;
    float temp_bmp;
    float anglePitchRad;
    float anglePitchDeg;
    float angleRollRad;
    float angleRollDeg;
}variable;

variable vars;
void mpu6050_test(void*);

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(mpu6050_test, "mpu6050_test", configMINIMAL_STACK_SIZE * 15, (void*)&vars, configMAX_PRIORITIES-1, NULL, 0);
}

void mpu6050_test(void *pvParameters)
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

    ESP_LOGI(TAG, "Accel range: %d", dev.ranges.accel);
    ESP_LOGI(TAG, "Gyro range:  %d", dev.ranges.gyro);

    while (1)
    {
        mpu6050_acceleration_t accel = { 0 };
        mpu6050_rotation_t rotation = { 0 };
        
        ESP_ERROR_CHECK(bmp180_measure(&dev2, &variables->temp_bmp, &variables->pressure_bmp, BMP180_MODE_STANDARD));

        ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &variables->temp));
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));
        ESP_ERROR_CHECK(mpu6050_get_acceleration(&dev, &accel));

        variables->anglePitchRad = atan((-accel.x)/sqrt(pow(accel.y, 2) + pow(accel.z, 2)));
        variables->angleRollRad = atan((-accel.y)/sqrt(pow(accel.x, 2) + pow(accel.z, 2)));

        variables->anglePitchDeg = variables->anglePitchRad*(180.0/M_PI);
        variables->angleRollDeg = variables->angleRollRad*(180.0/M_PI) - 1.5; //offset

        ESP_LOGI(TAG, "**********************************************************************");
        ESP_LOGI(TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", accel.x, accel.y, accel.z);
        ESP_LOGI(TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", rotation.x, rotation.y, rotation.z);
        ESP_LOGI(TAG, "Angles: Pitch=%.4f   Roll=%.4f", variables->anglePitchDeg, variables->angleRollDeg);
        ESP_LOGI(TAG, "Temperature_MPU6050:  %.2f", variables->temp);
        ESP_LOGI(TAG, "Temperature_BMP180:  %.2f  Pressão:  %" PRIu32 " Pa", variables->temp_bmp, variables->pressure_bmp); // pra referenciar variavel do tipo uint32_t utiliza-se %" PRIu32 " da lib inttypes.h

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
