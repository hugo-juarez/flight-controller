set(TASKS_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/task_monitor.c
)

set(TASKS_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

add_library(Tasks STATIC
    ${TASKS_SOURCE}
)

target_include_directories(Tasks PUBLIC
    ${TASKS_INCLUDES}
)

target_link_libraries(Tasks
    Config
    stm32cubemx
)

# Adding freertos to project
include(${CMAKE_SOURCE_DIR}/cmake/freertos.cmake)
add_freertos_library(Tasks)