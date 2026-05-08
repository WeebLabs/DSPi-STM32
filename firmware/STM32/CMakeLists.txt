# DSPi STM32H723VGT6 build (M0: clocks + LED blink + UART)
#
# Build:
#   cmake -S firmware/STM32 -B build-stm32h723
#   cmake --build build-stm32h723
#
# Flash via DFU (hold BOOT0, tap NRST, release BOOT0):
#   dfu-util -a 0 -s 0x08000000:leave -D build-stm32h723/DSPi_stm32h723.bin

cmake_minimum_required(VERSION 3.20)

# Point stm32-cmake at the bundled CubeH7 checkout
set(STM32_CUBE_H7_PATH ${CMAKE_CURRENT_LIST_DIR}/../../vendor/STM32CubeH7
    CACHE PATH "Path to STM32CubeH7 root")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../../vendor/stm32-cmake/cmake)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/../../vendor/stm32-cmake/cmake/stm32_gcc.cmake)

project(DSPi_stm32h723 C ASM)

# Find CMSIS device + HAL drivers we need for M0 (audio peripherals come later)
find_package(CMSIS COMPONENTS STM32H723VG_M7 REQUIRED)
find_package(HAL   COMPONENTS STM32H7_M7
                              RCC GPIO CORTEX PWR FLASH UART DMA SAI ADC SPI
                   REQUIRED)

set(EXE DSPi_stm32h723)

# Reuse the TinyUSB submodule that already lives under firmware/pico-sdk/lib
# rather than adding a second copy under vendor/. Single source of truth, same
# version across both ports.
set(TINYUSB_DIR ${CMAKE_CURRENT_LIST_DIR}/../pico-sdk/lib/tinyusb)

add_executable(${EXE}
    Core/Src/main.c
    Core/Src/stm32h7xx_it.c
    Core/Src/stm32h7xx_hal_msp.c
    Core/Src/syscalls.c
    Core/Src/usb_descriptors.c
    Core/Src/usb_glue.c
    Core/Src/usb_audio.c
    Core/Src/vendor_commands.c
    Core/Src/audio_out.c
    # M7g: internal sensors (ADC3 temp + VREFINT)
    Core/Src/temp_sensor.c
    # M11: W25Q64 SPI NOR driver (preset flash backing store)
    Core/Src/w25q.c
    # M7b: DSP-pipeline state + portable DSP source files
    Core/Src/audio_state.c
    Core/Src/audio_input.c
    Core/Src/bulk_params.c
    Core/Src/notify.c
    Core/Src/crossfeed.c
    Core/Src/leveller.c
    Core/Src/loudness.c
    Core/Src/dsp_pipeline.c   # M7c: actual EQ / filter / delay engine
    # TinyUSB sources — kept as an explicit list so the milestone scope is
    # obvious. M3: device stack + dwc2 only — no built-in class drivers
    # (UAC1 is registered through usbd_app_driver_get_cb() in usb_audio.c).
    ${TINYUSB_DIR}/src/tusb.c
    ${TINYUSB_DIR}/src/common/tusb_fifo.c
    ${TINYUSB_DIR}/src/device/usbd.c
    ${TINYUSB_DIR}/src/device/usbd_control.c
    ${TINYUSB_DIR}/src/portable/synopsys/dwc2/dcd_dwc2.c
    ${TINYUSB_DIR}/src/portable/synopsys/dwc2/dwc2_common.c
)

target_include_directories(${EXE} PRIVATE
    Core/Inc
    ${TINYUSB_DIR}/src
)

target_compile_definitions(${EXE} PRIVATE
    USE_HAL_DRIVER
    STM32H723xx
    HSE_VALUE=25000000U     # WeAct MiniSTM32H723 V1.2 — confirmed in schematic
    # Tell config.h to skip the .time_critical section attribute —
    # the auto-generated linker doesn't define that section. M6c will
    # add it (probably ITCM-resident). This define is consumed in our
    # local copy of config.h to make DSP_TIME_CRITICAL a no-op.
    DSPI_STM32_NO_TIME_CRITICAL_SECTION
)

# Optimization: -O2 by default; tighten to -Og for first bring-up if you want
# easier single-stepping.
target_compile_options(${EXE} PRIVATE
    -O2 -ffunction-sections -fdata-sections -Wall -Wextra
    -Wno-unused-parameter
)

target_link_libraries(${EXE} PRIVATE
    HAL::STM32::H7::M7::RCC
    HAL::STM32::H7::M7::RCCEx
    HAL::STM32::H7::M7::GPIO
    HAL::STM32::H7::M7::CORTEX
    HAL::STM32::H7::M7::PWREx
    HAL::STM32::H7::M7::FLASH
    HAL::STM32::H7::M7::FLASHEx
    HAL::STM32::H7::M7::UART
    HAL::STM32::H7::M7::DMA
    HAL::STM32::H7::M7::SAI
    HAL::STM32::H7::M7::ADC      # M7g — internal temp sensor + VREFINT
    HAL::STM32::H7::M7::ADCEx
    HAL::STM32::H7::M7::SPI      # M11 — W25Q64 on SPI3
    HAL::STM32::H7::M7::SPIEx
    CMSIS::STM32::H723VG::M7
    STM32::NoSys
)

# Generate .bin and .hex alongside the .elf, and print size summary.
stm32_print_size_of_target(${EXE})
stm32_generate_binary_file(${EXE})
stm32_generate_hex_file(${EXE})
