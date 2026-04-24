idf_component_register(
    SRCS "softap_example_main.c"
    PRIV_REQUIRES esp_wifi esp_event esp_netif nvs_flash esp_http_server esp_timer driver esp_psram esp_driver_spi
    INCLUDE_DIRS "."
)
