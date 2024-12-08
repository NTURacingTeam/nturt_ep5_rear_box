cmake_minimum_required(VERSION 3.20.0)
set(CMAKE_C_STANDARD 11)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nturt_rear_box LANGUAGES C)

zephyr_include_directories(
    include
)

add_subdirectory(src/ctrl)
add_subdirectory(src/msg)
add_subdirectory(src/sensors)
add_subdirectory(src/states)

target_sources(app PRIVATE
    src/err.c
    src/main.c
    src/sys.c
)
