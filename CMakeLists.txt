cmake_minimum_required(VERSION 3.20.0)
set(CMAKE_C_STANDARD 11)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nturt_rear_box LANGUAGES C)

zephyr_include_directories(
    include
)

target_sources(app PRIVATE
    src/OD.c
    src/control.c
    src/display.c
    src/main.c
    src/message.c
    src/sensors.c
    src/states.c
    src/sys.c
    src/util.c
)
