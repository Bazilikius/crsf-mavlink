cmake_minimum_required(VERSION 3.13)

# Include raspberry pi pico sdk cmake files if exists
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake OPTIONAL)

project(board1_ground C CXX ASM)

if(TARGET pico_stdlib)
    pico_sdk_init()

    add_executable(board1_ground
        main.c
        routing.c
        tracker.c
        vrx_control.c
        oled.c
        usb_descriptors.c
        ../shared/mux_protocol.c
    )

    target_include_directories(board1_ground PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../shared
    )

    # Enable Dual CDC (2 COM ports) via TinyUSB config override
    target_compile_definitions(board1_ground PRIVATE
        CFG_TUD_CDC=2
    )

    target_link_libraries(board1_ground pico_stdlib hardware_pwm hardware_i2c hardware_uart hardware_clocks)

    pico_add_extra_outputs(board1_ground)
else()
    message(STATUS "Pico SDK not found, skipping target board1_ground build configuration.")
endif()
