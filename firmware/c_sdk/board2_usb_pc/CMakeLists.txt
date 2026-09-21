cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake OPTIONAL)

project(board2_usb_pc C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(board2_usb_pc
    main.c
)

# Enable USB stdio, disable UART stdio for Board 2 so stdin/stdout goes over USB CDC
pico_enable_stdio_usb(board2_usb_pc 1)
pico_enable_stdio_uart(board2_usb_pc 0)

target_link_libraries(board2_usb_pc
    pico_stdlib
    hardware_uart
)

pico_add_extra_outputs(board2_usb_pc)
