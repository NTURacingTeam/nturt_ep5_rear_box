zephyr_include_directories(
    .
)

file(GLOB SRC_FILES
    ${CMAKE_CURRENT_LIST_DIR}/*.c
)

target_sources(app PRIVATE
    ${SRC_FILES}
)
