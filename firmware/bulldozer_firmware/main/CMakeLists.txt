idf_component_register(
    SRCS "bulldozer_firmware.c"
    INCLUDE_DIRS "."
    REQUIRES micro_ros_espidf_component driver esp_wifi nvs_flash esp_timer
)