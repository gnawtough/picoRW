cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(main)

pico_sdk_init()

add_executable(main
    main.cpp
)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

target_link_libraries(main pico_stdlib hardware_i2c)

pico_add_extra_outputs(main)
