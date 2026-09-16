cmake_minimum_required(VERSION 3.16)

# Include the micro-ROS component directory
list(APPEND EXTRA_COMPONENT_DIRS "components/micro_ros_espidf_component")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bulldozer_firmware)