idf_component_register(
    SRCS "../esp_buzzer.cpp"
    INCLUDE_DIRS ".."
    REQUIRES
        esp_driver_gpio
        esp_driver_ledc
        esp_event
        esp_http_server
        esp_netif
        esp_wifi
        mdns
        nvs_flash
)
