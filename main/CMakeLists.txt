idf_component_register(SRCS "main_weg.c" "mqtt/mqtt.c"
                    INCLUDE_DIRS "."
                    REQUIRES i2cdev esp_idf_lib_helpers esp_wifi 
                                            nvs_flash esp_event esp_netif mqtt
                    PRIV_REQUIRES freertos lora bmp180 mpu6050)