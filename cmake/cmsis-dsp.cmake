set(CMSIS_DSP_PATH ${CMAKE_SOURCE_DIR}/ThirdParty/CMSIS-DSP)

if(NOT EXISTS ${CMSIS_DSP_PATH}/Include)
    message(FATAL_ERROR
            "CMSIS-DSP submodule not initialized!\n"
            "Run: git submodule update --init --recursive")
endif()

add_library(cmsis_dsp INTERFACE)
target_include_directories(cmsis_dsp INTERFACE
        ${CMSIS_DSP_PATH}/Include
)

target_compile_definitions(cmsis_dsp INTERFACE
        ARM_MATH_CM7
)