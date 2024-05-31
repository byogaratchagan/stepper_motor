cmake_minimum_required(VERSION 3.12)

add_subdirectory(include)

add_library(stepper_motor stepper_motor.c)

target_link_libraries(stepper_motor
    pico_stdlib
    hardware_dma
    hardware_pio
    stepper_motor_h
)

target_include_directories(stepper_motor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
