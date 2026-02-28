set(TASKS_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/task_monitor.c
)

set(TASKS_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

add_library(tasks STATIC
    ${TASKS_SOURCE}
)

target_include_directories(tasks PUBLIC
    ${TASKS_INCLUDES}
)

target_link_libraries(tasks PRIVATE
    stm32cubemx
    freertos
    config
)