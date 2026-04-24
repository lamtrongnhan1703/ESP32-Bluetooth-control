idf_component_register(SRCS "gatt_server_main.c"
                      INCLUDE_DIRS "."
                      PRIV_REQUIRES driver mpu6050 bt nvs_flash)