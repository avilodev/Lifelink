# Standalone test project for vitals_math.c.
#
# Deliberately NOT part of the top-level CMakeLists.txt: that project calls
# pico_sdk_init() and forces the ARM cross toolchain for everything in the
# build graph. vitals_math.c has no Pico SDK dependency, so it's built and
# run here with the host's native compiler instead, independent of
# PICO_SDK_PATH and any ARM toolchain.
#
# Usage:
#   cmake -S tests -B tests/build
#   cmake --build tests/build
#   ctest --test-dir tests/build --output-on-failure

cmake_minimum_required(VERSION 3.13)
project(lifelink_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

add_library(vitals_math STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../app/lifelink/vitals_math.c
)
target_include_directories(vitals_math PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../app     # exposes lifelink/vitals_math.h
    ${CMAKE_CURRENT_SOURCE_DIR}/..         # exposes config.h
)

add_executable(test_vitals_math test_vitals_math.c)
target_link_libraries(test_vitals_math PRIVATE vitals_math m)

add_test(NAME vitals_math COMMAND test_vitals_math)
