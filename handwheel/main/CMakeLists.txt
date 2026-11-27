idf_component_register(
    SRCS "handwheel.c"
    INCLUDE_DIRS "."
    REQUIRES esp_lcd esp_driver_gpio esp_driver_spi esp_driver_rmt esp_wifi nvs_flash esp_event esp_netif
)
