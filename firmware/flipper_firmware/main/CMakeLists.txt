idf_component_register(SRCS "flipper_firmware.c"
                       INCLUDE_DIRS "."
                       REQUIRES micro_ros_espidf_component esp_wifi esp_event nvs_flash driver esp_timer)