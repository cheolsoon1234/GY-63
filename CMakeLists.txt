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
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(GY63 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# ====================================================================================

# Project layout

# Add executable. Default name is the project name, version 0.1
set(SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src)
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
    ${SRC_DIR}/app/*.c
    ${SRC_DIR}/core/*.c
    ${SRC_DIR}/bsp/*.c
    ${SRC_DIR}/drivers/*.c
    
    ${SRC_DIR}/platform/hal/*.c
    ${SRC_DIR}/platform/net/*.c
    # ${SRC_DIR}/core/*.cpp
    # ${SRC_DIR}/hw/*.cpp
)

add_executable(GY63 ${APP_SOURCES})

pico_set_program_name(GY63 "GY63")
pico_set_program_version(GY63 "0.1")


# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(GY63 0)
pico_enable_stdio_usb(GY63 1)


# Add the standard library to the build
target_link_libraries(GY63
        pico_stdlib
        )

# Add the standard include files to the build
target_include_directories(GY63 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${SRC_DIR}
        ${SRC_DIR}/app
        ${SRC_DIR}/core
        ${SRC_DIR}/bsp
        ${SRC_DIR}/drivers
        ${SRC_DIR}/platform/hal
        ${SRC_DIR}/platform/net
)

# Add any user requested libraries
target_link_libraries(GY63 
        hardware_i2c
        # pico_cyw43_arch_lwip_threadsafe_background
        )

pico_add_extra_outputs(GY63)

