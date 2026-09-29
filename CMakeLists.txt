cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(lifelink C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# BLE pairing PIN. Override at configure time so a real PIN never lives in
# tracked source:  cmake -DHM10_PAIR_PIN=481920 ..
# Applied here (before the subdirectories) so it reaches every target that
# includes config.h - notably the HM-10 driver in drivers/. If left unset,
# config.h falls back to an obvious "000000" placeholder.
set(HM10_PAIR_PIN "" CACHE STRING "6-digit HM-10 BLE pairing PIN")
if(HM10_PAIR_PIN)
    add_compile_definitions(HM10_PAIR_PIN="${HM10_PAIR_PIN}")
endif()

add_subdirectory(drivers)
add_subdirectory(app)

add_executable(lifelink main.c)

target_include_directories(lifelink PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# USB CDC printf must not stall the sensor loop if the host stops reading.
# Default is 500_000 us (suited to interactive REPL use); 1 ms is correct
# for streaming-sensor use where the next line is regenerated 2 s later.
target_compile_definitions(lifelink PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)

target_link_libraries(lifelink
    pico_stdlib
    hardware_gpio
    hardware_i2c
    hardware_sync
    hardware_uart
    hardware_watchdog
    pico_time
    drivers
    app
)

# USB for debug serial; UART0 is taken by the HM-10
pico_enable_stdio_usb(lifelink 1)
pico_enable_stdio_uart(lifelink 0)

pico_add_extra_outputs(lifelink)
