add_library(app STATIC
    lifelink/lifelink.c
    lifelink/vitals_math.c
)

target_include_directories(app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}   # exposes app/lifelink/...
    ${CMAKE_SOURCE_DIR}           # exposes config.h
)

target_link_libraries(app
    drivers
    pico_stdlib
    hardware_i2c
    hardware_sync
    hardware_uart
    hardware_watchdog
    pico_time
)
