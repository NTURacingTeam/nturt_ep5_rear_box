cmake_minimum_required(VERSION 3.20.0)
set(CMAKE_C_STANDARD 11)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nturt_rear_box LANGUAGES C)

add_subdirectory(objdict)
add_subdirectory(simulink)

zephyr_include_directories(
    include
)

target_sources(app PRIVATE
    src/dash/button.c
    src/dash/led.c
    src/states/rtd_snd.c
    src/states/states.c
    src/states/status_ctrl.c
    src/ctrl.c
    src/err.c
    src/main.c
    src/msg.c
    src/sensors.c
    src/sys.c
)
