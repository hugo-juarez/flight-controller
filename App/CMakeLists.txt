set(APP_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/app_main.h
)

set(APP_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/app_main.c
)

set(APP_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

add_library(app STATIC
    ${APP_HEADERS}
    ${APP_SOURCE}
)

target_include_directories(app PUBLIC
    ${APP_INCLUDES}
)

target_link_libraries(app PRIVATE
    config
    tasks
    freertos
)