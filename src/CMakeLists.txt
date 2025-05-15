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
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(FinalProject C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(FinalProject) 

# must match with pio filename and executable name from above
pico_generate_pio_header(FinalProject ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(FinalProject ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(FinalProject ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

# Depending on whether to compile for remote and emulator -> replace first file with "emulator.c" or "remote.c"
target_sources(FinalProject PRIVATE emulator_udp.c bowling_physics.c bowling_graphics.c bowling_score.c mpu6050.c vga16_graphics.c imu_container.c
mfe_algorithm.c)
target_include_directories(FinalProject PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(FinalProject 
    pico_cyw43_arch_lwip_threadsafe_background
    pico_stdlib 
    pico_cyw43_arch_none
    pico_cyw43_arch
    hardware_i2c 
    hardware_pwm 
    hardware_dma 
    hardware_irq 
    hardware_adc 
    hardware_pio 
    hardware_sync
    hardware_rtc
    pico_multicore 
    pico_bootsel_via_double_reset 
    hardware_clocks 
    hardware_pll 
    hardware_gpio
    hardware_spi
) 

# create map/bin/hex file etc.
pico_add_extra_outputs(FinalProject)

add_compile_options(-O3)
