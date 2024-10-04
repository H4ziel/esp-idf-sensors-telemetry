idf_component_register(SRCS "main.c" "./mqtt/mqtt.c"
                    INCLUDE_DIRS "."
                    REQUIRES mpu6050 i2cdev esp_idf_lib_helpers nvs_flash 
                                                                       esp_event 
                    PRIV_REQUIRES freertos bmp180 lora)
