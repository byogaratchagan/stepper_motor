cmake_minimum_required(VERSION 3.12)

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include(${PICO_SDK_PATH}/tools/CMakeLists.txt)

pico_sdk_init()

add_subdirectory(include)

add_library(motor_main motor.c)

target_link_libraries(motor_main
    pico_stdlib
    hardware_dma
    hardware_pio
    motor_header
)
