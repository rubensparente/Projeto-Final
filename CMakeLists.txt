# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == Configuração VS Code Extension (não mexa) ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (antes do project!)
include(pico_sdk_import.cmake)

project(projeto_final C CXX ASM)

# Inicializa o SDK
pico_sdk_init()

# FatFs (SD Card)
add_subdirectory(lib/FatFs_SPI)

# FreeRTOS
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS-Kernel)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Executável + fontes
add_executable(projeto_final
    projeto_final.c
    lib/aht10.c
    lib/ssd1306.c
    lib/bmp280.c
    lib/bmp280_low_level.c
    hw_config.c
)

# Saída stdio (USB + UART)
pico_enable_stdio_uart(projeto_final 1)
pico_enable_stdio_usb(projeto_final 1)

# Bibliotecas (ordem importante!)
target_link_libraries(projeto_final
    pico_stdlib
    pico_multicore
    hardware_i2c
    hardware_spi
    hardware_rtc
    hardware_adc
    hardware_pwm 
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    FatFs_SPI
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
    pico_lwip_sntp
   
)


# Includes
target_include_directories(projeto_final PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/lib
)

# Extra outputs: uf2, hex, etc.
pico_add_extra_outputs(projeto_final)