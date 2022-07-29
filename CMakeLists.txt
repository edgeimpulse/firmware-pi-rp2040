cmake_minimum_required(VERSION 3.13.1)

set(MODEL_FOLDER .)
set(EI_SDK_FOLDER edge-impulse-sdk)
set(EI_PLATFORM_FOLDER edge-impulse)
set(FIRMWARE_SDK_FOLDER firmware-sdk)

include(pico_sdk_import.cmake)

# Pull in FreeRTOS
include(FreeRTOS_Kernel_import.cmake)

project(ei_rp2040_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

pico_sdk_init()

# Use hardware acceleration for DSP and Neural Network code
# You'll need to disable these on non-Arm cores
add_definitions(-DEIDSP_USE_CMSIS_DSP=1
                -DEIDSP_LOAD_CMSIS_DSP_SOURCES=1
                -DEI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN=1
                -DEIDSP_QUANTIZE_FILTERBANK=0
                -DARM_MATH_LOOPUNROLL
                -DEI_SENSOR_AQ_STREAM=FILE
                )

OPTION(DEFINE_DEBUG
    "Build the project using debugging code"
    OFF)
IF(DEFINE_DEBUG)
    MESSAGE("Adding Debug flag...")
    ADD_DEFINITIONS(-DDEBUG)
    SET(CMAKE_BUILD_TYPE Debug)
    MESSAGE("Build type is " ${CMAKE_BUILD_TYPE})
ENDIF(DEFINE_DEBUG)

add_executable(ei_rp2040_firmware
    src/main.cpp
)

OPTION(DEFINE_PIN_UART
    "Only use TX/RX pins and not USB UART"
    OFF)
IF(DEFINE_PIN_UART)
    MESSAGE("Adding PIN_UART flag...")
    ADD_DEFINITIONS(-DPIN_UART)
    MESSAGE("Build type is " ${CMAKE_BUILD_TYPE})
    pico_enable_stdio_usb(ei_rp2040_firmware 0)
    pico_enable_stdio_uart(ei_rp2040_firmware 1)
ELSE()
    pico_enable_stdio_usb(ei_rp2040_firmware 1)
    pico_enable_stdio_uart(ei_rp2040_firmware 1)
ENDIF(DEFINE_PIN_UART)

include(${EI_SDK_FOLDER}/cmake/utils.cmake)

pico_generate_pio_header(ei_rp2040_firmware ${CMAKE_CURRENT_LIST_DIR}/ThirdParty/PDM/src/pdm.pio)

target_link_libraries(ei_rp2040_firmware
                      pico_stdlib
                      pico_unique_id
                      hardware_gpio
                      hardware_adc
                      hardware_i2c
                      hardware_irq
                      hardware_flash
                      hardware_pio
                      hardware_dma
                      FreeRTOS-Kernel
                      FreeRTOS-Kernel-Heap4)

target_include_directories(ei_rp2040_firmware PRIVATE
    ${MODEL_FOLDER}
    ${MODEL_FOLDER}/classifer
    ${MODEL_FOLDER}/tflite-model
    ${MODEL_FOLDER}/model-parameters
)

target_include_directories(ei_rp2040_firmware PRIVATE
    ${EI_SDK_FOLDER}
    ${EI_SDK_FOLDER}/third_party/ruy
    ${EI_SDK_FOLDER}/third_party/gemmlowp
    ${EI_SDK_FOLDER}/third_party/flatbuffers/include
    ${EI_SDK_FOLDER}/third_party
    ${EI_SDK_FOLDER}/tensorflow
    ${EI_SDK_FOLDER}/dsp
    ${EI_SDK_FOLDER}/classifier
    ${EI_SDK_FOLDER}/anomaly
    ${EI_SDK_FOLDER}/CMSIS/NN/Include
    ${EI_SDK_FOLDER}/CMSIS/DSP/PrivateInclude
    ${EI_SDK_FOLDER}/CMSIS/DSP/Include
    ${EI_SDK_FOLDER}/CMSIS/Core/Include
    ${EI_SDK_FOLDER}/porting
    ${EI_PLATFORM_FOLDER}
    ${EI_PLATFORM_FOLDER}/ingestion-sdk-c
    ${EI_PLATFORM_FOLDER}/ingestion-sdk-platform/raspberry-rp2040
    ${EI_PLATFORM_FOLDER}/ingestion-sdk-platform/sensors
    ${EI_PLATFORM_FOLDER}/mbedtls_hmac_sha256_sw
    ${EI_PLATFORM_FOLDER}/inference
    ${EI_PLATFORM_FOLDER}/QCBOR/inc
    ${FIRMWARE_SDK_FOLDER}
    ${FIRMWARE_SDK_FOLDER}/at-server
    ThirdParty/Seeed_Arduino_UltrasonicRanger
    ThirdParty/Wire/src
    ThirdParty/Arduino_LSM6DSOX/src  
    ThirdParty/rp2040_DHT11_lib
    ThirdParty/PDM/src/include
)

include_directories(${INCLUDES})

RECURSIVE_FIND_FILE(SOURCE_FILES "edge-impulse-sdk" "*.cpp")
RECURSIVE_FIND_FILE(MODEL_FILES "tflite-model" "*.cpp")
RECURSIVE_FIND_FILE(CC_FILES "edge-impulse-sdk" "*.cc")
RECURSIVE_FIND_FILE(S_FILES "edge-impulse-sdk" "*.s")
RECURSIVE_FIND_FILE(C_FILES "edge-impulse-sdk" "*.c")

RECURSIVE_FIND_FILE(PLATFORM_FILES "edge-impulse/ingestion-sdk-platform/raspberry-rp2040" "*.cpp")
RECURSIVE_FIND_FILE(SENSORS_FILES "edge-impulse/ingestion-sdk-platform/sensors" "*.cpp")
RECURSIVE_FIND_FILE(REPL_FILES "edge-impulse/repl" "*.cpp")
RECURSIVE_FIND_FILE(INGESTION_FILES "edge-impulse/ingestion-sdk-c/" "*.cpp")
RECURSIVE_FIND_FILE(INFERENCE_FILES "edge-impulse/inference" "*.cpp")
RECURSIVE_FIND_FILE(QCBOR_FILES "edge-impulse/QCBOR/src" "*.c")
RECURSIVE_FIND_FILE(MBEDTLS_FILES "edge-impulse/mbedtls_hmac_sha256_sw/mbedtls/src" "*.c")
RECURSIVE_FIND_FILE(PORTING_FILES "edge-impulse-sdk/porting/raspberry" "*.cpp")
RECURSIVE_FIND_FILE(FIRMWARE_SDK_FILES "firmware-sdk" "*.cpp")
RECURSIVE_FIND_FILE(THIRDPARTY_FILES "ThirdParty" "*.cpp")
RECURSIVE_FIND_FILE(MIC_FILES "ThirdParty/PDM/src" "*.c")

list(APPEND SOURCE_FILES ${S_FILES})
list(APPEND SOURCE_FILES ${C_FILES})
list(APPEND SOURCE_FILES ${CC_FILES})
list(APPEND SOURCE_FILES ${MODEL_FILES})

list(APPEND SOURCE_FILES ${PLATFORM_FILES})
list(APPEND SOURCE_FILES ${SENSORS_FILES})
list(APPEND SOURCE_FILES ${REPL_FILES})
list(APPEND SOURCE_FILES ${INGESTION_FILES})
list(APPEND SOURCE_FILES ${INFERENCE_FILES})
list(APPEND SOURCE_FILES ${QCBOR_FILES})
list(APPEND SOURCE_FILES ${MBEDTLS_FILES})
list(APPEND SOURCE_FILES ${PORTING_FILES})
list(APPEND SOURCE_FILES ${FIRMWARE_SDK_FILES})
list(APPEND SOURCE_FILES ${THIRDPARTY_FILES})
list(APPEND SOURCE_FILES ${MIC_FILES})

# add all sources to the project
target_sources(ei_rp2040_firmware PRIVATE ${SOURCE_FILES})

pico_add_extra_outputs(ei_rp2040_firmware)

OPTION(DEFINE_UART_UPLOAD
    "Replace Bootloader - will only accept firmware over UART"
    OFF)
IF(DEFINE_UART_UPLOAD)
    MESSAGE("Adding UART_UPLOAD flag...")
    pico_set_linker_script(ei_rp2040_firmware ${CMAKE_CURRENT_SOURCE_DIR}/uart_upload.ld)
ENDIF(DEFINE_UART_UPLOAD)