idf_component_register(
    SRCS "bulldozer_firmware.c"
    INCLUDE_DIRS "."
    REQUIRES 
        esp_wifi
        esp_event
        nvs_flash
        driver
        esp_timer
        esp_netif
        esp_http_server
        micro_ros_espidf_component
)