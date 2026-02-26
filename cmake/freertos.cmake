# Usage:
# include(${CMAKE_SOURCE_DIR}/../cmake/freertos.cmake)
# add_freertos_library(TARGET_NAME CONFIG_DIR)


# Sets the port, relative path and memory management for this project FreeRTOS
set(FREERTOS_KERNEL_PATH ${CMAKE_SOURCE_DIR}/ThirdParty/FreeRTOS-Kernel)

# Verifying submodule is initialized
if(NOT EXISTS ${FREERTOS_KERNEL_PATH}/tasks.c)
    message(FATAL_ERROR
            "FreeRTOS submodule not initialized!\n"
            "Run: git submodule update --init --recursive")
endif()

# Set the port BEFORE adding the FreeRTOS subdirectory
set(FREERTOS_PORT GCC_ARM_CM7 CACHE STRING "")  # Adjust for your MCU

# Set heap implementation (1-5, or path to custom)
set(FREERTOS_HEAP 4 CACHE STRING "")

# Config library for FreeRTOS to include FreeRTOSConfig.h to project
add_library(freertos_config INTERFACE)

function(add_freertos_library TARGET_NAME CONFIG_DIR)

    # Include FreeRTOSConfig.h
    target_include_directories(freertos_config INTERFACE
        ${CONFIG_DIR}
    )

    # Add FreeRTOS Kernel
    add_subdirectory(${FREERTOS_KERNEL_PATH} FreeRTOS-Kernel)

    target_compile_options(freertos_kernel PRIVATE
        ### Gnu/Clang C Options
        $<$<COMPILE_LANG_AND_ID:C,GNU>:-fdiagnostics-color=always>
        $<$<COMPILE_LANG_AND_ID:C,Clang>:-fcolor-diagnostics>

        $<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wall>
        $<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wextra>
        $<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wpedantic>
        $<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Werror>
        $<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wconversion>
        $<$<COMPILE_LANG_AND_ID:C,Clang>:-Weverything>

        # Suppressions required to build clean with clang.
        $<$<COMPILE_LANG_AND_ID:C,Clang>:-Wno-unused-macros>
        $<$<COMPILE_LANG_AND_ID:C,Clang>:-Wno-padded>
        $<$<COMPILE_LANG_AND_ID:C,Clang>:-Wno-missing-variable-declarations>
        $<$<COMPILE_LANG_AND_ID:C,Clang>:-Wno-covered-switch-default>
        $<$<COMPILE_LANG_AND_ID:C,Clang>:-Wno-cast-align>
    )

    target_link_libraries(${TARGET_NAME}
        freertos_kernel
        freertos_config
    )

    message(STATUS "FreeRTOS library '${TARGET_NAME}' configured:")
    message(STATUS "  - Port: ${FREERTOS_PORT}")
    message(STATUS "  - Heap: ${FREERTOS_HEAP}")
    message(STATUS "  - Config: ${CONFIG_DIR}")

endfunction()