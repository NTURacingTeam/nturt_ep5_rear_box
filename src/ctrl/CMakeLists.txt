zephyr_include_directories(
    simulink
)

file(GLOB SIMULINK_FILES
    ${CMAKE_CURRENT_LIST_DIR}/simulink/*.c
)

target_sources(app PRIVATE
    ${SIMULINK_FILES}
    ctrl.c
)
