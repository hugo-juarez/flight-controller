set(DRIVERS_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/BMI088/bmi088.c
)

add_library(drivers STATIC
    ${DRIVERS_SOURCE}
)

target_include_directories(drivers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(drivers PRIVATE config)