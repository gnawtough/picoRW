cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(PicoRW)

pico_sdk_init()

add_executable(PicoRW
    main.cpp
)

target_link_libraries(PicoRW pico_stdlib hardware_i2c)

pico_add_extra_outputs(PicoRW)
