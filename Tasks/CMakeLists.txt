set(TASKS_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/task_monitor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/task_imu.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/task_crsf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/task_navigation.c
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
    drivers
    config
)