idf_component_register(SRCS "gatt_client_main.c"
                      INCLUDE_DIRS "."
                      PRIV_REQUIRES driver bt nvs_flash)