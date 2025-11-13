cmake_minimum_required(VERSION 3.20)
project(SmartMoisture-MSP430 C)

set(FLASHER "C:/Progra~1/TI/UniFlash_9.3.0/dslite.bat")
set(MCU "${MSP430_MCU}")

set(SRC_DIR "${CMAKE_SOURCE_DIR}/src")
file(GLOB SRC_FILES "${SRC_DIR}/*.c")
add_executable(firmware.elf ${SRC_FILES})

target_compile_options(firmware.elf PRIVATE -g)
target_include_directories(firmware.elf PRIVATE "${MSP430_GCC_INCLUDE_DIR}")
target_link_options(firmware.elf PRIVATE -Wl,-Map=${CMAKE_BINARY_DIR}/firmware.map)

add_custom_target(flash
        COMMAND "${FLASHER}" --config=lib/${MCU}.ccxml --flash --verify $<TARGET_FILE:firmware.elf>
        DEPENDS firmware.elf
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Flashing MSP430 (${MCU}) with UniFlash"
)
