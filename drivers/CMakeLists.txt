add_library(drivers STATIC
    max30102/max30102.c
    max30205/max30205.c
    hm10/hm10.c
)

target_include_directories(drivers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}   # exposes drivers/max30102/... drivers/max30205/... etc.
    ${CMAKE_SOURCE_DIR}           # exposes config.h
)

target_link_libraries(drivers
    pico_stdlib
    hardware_gpio
    hardware_i2c
    hardware_uart
    pico_time
)
