idf_component_register(
    SRCS "xhc_pendant.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio esp_wifi nvs_flash esp_event esp_netif
)
