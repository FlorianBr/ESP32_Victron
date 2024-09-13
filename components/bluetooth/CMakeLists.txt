idf_component_register(SRCS "bluetooth.c"
    REQUIRES nvs_flash bt mbedtls
    INCLUDE_DIRS "include")
