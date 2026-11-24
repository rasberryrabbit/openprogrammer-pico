# Open Programmer firmware — Raspberry Pi Pico (RP2040) port
# Ported from the original PIC18 (MCC18) firmware v0.12.0 by Alberto Maccioni.
# Same USB HID wire protocol (VID 0x1209 / PID 0x5432) so the original
# host software (openprog GUI / CLI) works unchanged.
cmake_minimum_required(VERSION 3.13)

# Pull in the Pico SDK. pico_sdk_import.cmake must sit next to this file
# (the canonical copy shipped by pico-sdk at external/pico_sdk_import.cmake).
include(pico_sdk_import.cmake)

project(openprog_pico C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

# Initialise the SDK; pio_stdio via USB is not used (we run a custom HID device).
pico_sdk_init()

# ---- Application sources -------------------------------------------------
set(OPENPROG_SOURCES
    src/main.c
    src/usb_descriptors.c
    src/usb_app.c
    src/hal.c
    src/opcontrol.c
    src/opcontrol_helpers.c
)

add_executable(${CMAKE_PROJECT_NAME} ${OPENPROG_SOURCES})

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# TinyUSB provides the HID class; the pico-sdk pulls it in via the
# pico_tinyusb library. We enable the device stack + HID class.
target_link_libraries(${CMAKE_PROJECT_NAME}
    pico_stdlib
    pico_bootsel_via_double_reset
    hardware_adc
    hardware_pwm
    hardware_dma
    hardware_irq
    hardware_flash
    tinyusb_device
    tinyusb_board
)

# USB identifiers (match the original PIC firmware so the existing host
# software enumerates the device without changes).
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # TinyUSB descriptors are provided by us (usb_descriptors.c).
    # Board-specific USB identifiers are NOT used; we override via tusb_config.
    PICO_XOSC_STARTUP_DELAY_MS=64
)

# Generate standard Pico output artefacts: .uf2 (drag-flash), .hex, .elf, .map.
pico_add_extra_outputs(${CMAKE_PROJECT_NAME})

# Helpful: print size + map file alongside the .uf2.
pico_set_program_name(${CMAKE_PROJECT_NAME} "openprog")
pico_set_program_version(${CMAKE_PROJECT_NAME} "0.12.0-pico")

# Disable default USB stdio (we run a real HID device on the same USB port).
pico_enable_stdio_usb(${CMAKE_PROJECT_NAME} 0)
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME} 0)
