# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
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
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(main C CXX ASM)

# Fix for slow SPI flash on RP2040-Zero
add_compile_definitions(PICO_FLASH_SPI_CLKDIV=4)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(ps1-igr-mfo
    main.c
    ws2812.c
)

#target_compile_definitions(ps1-igr-mfo PRIVATE)

# pull in common dependencies
target_link_libraries(ps1-igr-mfo
    pico_stdlib
    hardware_i2c
    hardware_spi
    hardware_dma
    hardware_pio
    hardware_uart
)

# create map/bin/hex file etc.
pico_add_extra_outputs(ps1-igr-mfo)
