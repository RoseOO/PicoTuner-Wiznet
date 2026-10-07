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

# Select the WIZnet chip and board
add_definitions(-D_WIZCHIP_=W6100)
add_definitions(-DDEVICE_BOARD_NAME=W6100_EVB_PICO)

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

add_executable(${TARGET_NAME}
        ${TARGET_NAME}.c
        nim.c
        rpi2c.c
        stv0910.c
        stv0910_utils.c
        stv6120.c
        stv6120_utils.c
        stvvglna.c
        stvvglna_utils.c
        # WIZnet ioLibrary_Driver (W6100)
        ${WIZNET_DIR}/Ethernet/socket.c
        ${WIZNET_DIR}/Ethernet/wizchip_conf.c
        ${WIZNET_DIR}/Ethernet/W6100/w6100.c
        # WIZnet Pico port layer
        ${PORT_DIR}/ioLibrary_Driver/src/wizchip_spi.c
        ${PORT_DIR}/ioLibrary_Driver/src/wizchip_gpio_irq.c
        ${PORT_DIR}/timer/timer.c
        )

# The original WinterHill/LongMynd sources declare file-local globals such as
# "char temps[128]" in several translation units and relied on the old GCC
# -fcommon behaviour. Newer GCC defaults to -fno-common, so restore it.
target_compile_options(${TARGET_NAME} PRIVATE -fcommon)

pico_set_linker_script(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/memmap_custom-2MB.ld)

pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/${TARGET_NAME}.pio)

target_include_directories(${TARGET_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Ethernet/W6100
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
        pico_multicore
        hardware_irq
        hardware_clocks
        )

pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

pico_add_extra_outputs(${TARGET_NAME})
