idf_component_register(SRCS "main.c" "mqtt/mqtt.c"
                    INCLUDE_DIRS "."
                    REQUIRES mpu6050 i2cdev esp_idf_lib_helpers esp_wifi 
                                            nvs_flash esp_event esp_netif mqtt
                    PRIV_REQUIRES freertos bmp180)
