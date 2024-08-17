idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES mpu6050 i2cdev esp_idf_lib_helpers 
                    PRIV_REQUIRES freertos bmp180)
