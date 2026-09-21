cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake OPTIONAL)

project(board1_mavlink_device C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(board1_mavlink_device
    main.c
)

target_link_libraries(board1_mavlink_device
    pico_stdlib
    hardware_uart
)

# UART0 (GPIO 0/1) carries MAVLink to the device, so keep stdio off it.
pico_enable_stdio_uart(board1_mavlink_device 0)
pico_enable_stdio_usb(board1_mavlink_device 0)

pico_add_extra_outputs(board1_mavlink_device)
