pico_generate_pio_header(main ${CMAKE_CURRENT_LIST_DIR}/motor_send_data.pio)

add_library(
    motor_header INTERFACE
    motor.h
)