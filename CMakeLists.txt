cmake_minimum_required(VERSION 3.13)

# ---------------------------------------------------------------------------
#  PicoTuner-WH (WinterHill) firmware for the WIZnet W6100-EVB-Pico
#
#  Standalone build. Requires:
#    - Raspberry Pi Pico SDK   (PICO_SDK_PATH, default ../pico-sdk)
#    - WIZnet-PICO-C sources   (WIZNET_DIR / PORT_DIR, default ../WIZnet-PICO-C)
# ---------------------------------------------------------------------------

if(NOT DEFINED WIZNET_DIR)
    set(WIZNET_DIR ${CMAKE_CURRENT_LIST_DIR}/../WIZnet-PICO-C/libraries/ioLibrary_Driver)
endif()

if(NOT DEFINED PORT_DIR)
    set(PORT_DIR ${CMAKE_CURRENT_LIST_DIR}/../WIZnet-PICO-C/port)
endif()

# Select the Raspberry Pi Pico board / platform.
#   pico            -> RP2040 (W5100S/W5500/W6100-EVB-Pico)
#   pico2           -> RP2350 (W5100S/W5500/W6100-EVB-Pico2)
set(PICO_BOARD "pico2" CACHE STRING "Pico board: pico (RP2040) or pico2 (RP2350)")

# Select the WIZnet chip and board (name must match board_list.h)
set(WIZNET_CHIP  "W5500"           CACHE STRING "WIZnet chip: W5100S/W5500/W6100")
set(WIZNET_BOARD "W5500_EVB_PICO2" CACHE STRING "WIZnet board name from board_list.h")

add_definitions(-D_WIZCHIP_=${WIZNET_CHIP})
add_definitions(-DDEVICE_BOARD_NAME=${WIZNET_BOARD})

# Pull in the Raspberry Pi Pico SDK
if(NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/../pico-sdk)
endif()
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(picotunewh C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

set(TARGET_NAME picotunewh)

string(TOLOWER ${WIZNET_CHIP} WIZNET_CHIP_LC)

add_executable(${TARGET_NAME}
        ${TARGET_NAME}.c
        config.c
        webui.c
        nim.c
        rpi2c.c
        stv0910.c
        stv0910_utils.c
        stv6120.c
        stv6120_utils.c
        stvvglna.c
        stvvglna_utils.c
        # WIZnet ioLibrary_Driver (chip selected by WIZNET_CHIP)
        ${WIZNET_DIR}/Ethernet/socket.c
        ${WIZNET_DIR}/Ethernet/wizchip_conf.c
        ${WIZNET_DIR}/Ethernet/${WIZNET_CHIP}/${WIZNET_CHIP_LC}.c
        # WIZnet Pico port layer
        ${PORT_DIR}/ioLibrary_Driver/src/wizchip_spi.c
        ${PORT_DIR}/ioLibrary_Driver/src/wizchip_gpio_irq.c
        ${PORT_DIR}/timer/timer.c
        )

# The original WinterHill/LongMynd sources declare file-local globals such as
# "char temps[128]" in several translation units and relied on the old GCC
# -fcommon behaviour. Newer GCC defaults to -fno-common, so restore it.
target_compile_options(${TARGET_NAME} PRIVATE -fcommon)

# NOTE: the LongMynd-derived sources contain UB that modern GCC exploits at
# -O1 and above (verified on both rp2040 and rp2350): it treats an
# ethernet_setup() wait loop as non-terminating and deletes mainloop() and
# core1_main().  Force -O0 so every variant links a complete binary.
target_compile_options(${TARGET_NAME} PRIVATE -O0)

# RP2040 builds use the LongMynd WinterHill memory map (retained MY_RAM_0 in
# SCRATCH_Z). RP2350 (Pico 2) uses the SDK default linker; MY_RAM_0 is defined
# in picotunewh.c in .uninitialized_data instead.
if(PICO_PLATFORM MATCHES "^rp2350")
    message(STATUS "Target: ${PICO_BOARD} / ${WIZNET_CHIP} (RP2350, SDK default linker)")
else()
    message(STATUS "Target: ${PICO_BOARD} / ${WIZNET_CHIP} (RP2040, custom linker)")
    pico_set_linker_script(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/memmap_custom-2MB.ld)
endif()

pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/${TARGET_NAME}.pio)

target_include_directories(${TARGET_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Ethernet/${WIZNET_CHIP}
        ${WIZNET_DIR}/Internet/DHCP
        ${WIZNET_DIR}/Internet/DNS
        ${PORT_DIR}
        ${PORT_DIR}/ioLibrary_Driver/inc
        ${PORT_DIR}/timer
        )

target_link_libraries(${TARGET_NAME} PRIVATE
        pico_stdlib
        hardware_spi
        hardware_dma
        hardware_i2c
        hardware_pio
        hardware_exception
        hardware_flash
        pico_flash
        pico_multicore
        hardware_irq
        hardware_clocks
        )

pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

pico_add_extra_outputs(${TARGET_NAME})


