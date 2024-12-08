zephyr_include_directories(
    objdict
)

file(GLOB OBJDICT_FILES
    ${CMAKE_CURRENT_LIST_DIR}/objdict/*.c
)

target_sources(app PRIVATE
    ${OBJDICT_FILES}
    msg.c
    sdo.c
)
