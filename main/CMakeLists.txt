idf_component_register(SRCS "main.c"
    PRIV_REQUIRES eink lvgl bluetooth
    REQUIRES driver app_update nvs_flash
    INCLUDE_DIRS ".")
