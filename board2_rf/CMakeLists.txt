cmake_minimum_required(VERSION 3.13)

# Include raspberry pi pico sdk cmake files if exists
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake OPTIONAL)

project(board2_rf C CXX ASM)

if(TARGET pico_stdlib)
    pico_sdk_init()

    add_executable(board2_rf
        main.c
        switcher.c
        pio_uart.c
        ../shared/mux_protocol.c
    )

    target_include_directories(board2_rf PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../shared
    )

    target_link_libraries(board2_rf pico_stdlib hardware_pio hardware_uart hardware_clocks)

    pico_add_extra_outputs(board2_rf)
else()
    message(STATUS "Pico SDK not found, skipping target board2_rf build configuration.")
endif()
