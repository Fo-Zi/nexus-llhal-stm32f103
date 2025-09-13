# Include directories
set(INCLUDE_DIRS
    include
    src/internal  # For internal headers
    ../hal-interface/include
)

# STM32F103 sources
set(STM32F103_SOURCES
    src/stm32f103_clock.c
    src/stm32f103_timing.c
    src/stm32f103_gpio.c
    src/stm32f103_uart.c
    src/nhal_pin_stm32f103.c
)

# Common sources
set(COMMON_SOURCES
    src/nhal_common.c
)

# Create static library
add_library(nexus-llhal-stm32f103 STATIC ${STM32F103_SOURCES} ${COMMON_SOURCES})

# Include directories
target_include_directories(nexus-llhal-stm32f103 PUBLIC ${INCLUDE_DIRS})

# Compiler flags for STM32F103
target_compile_definitions(nexus-llhal-stm32f103 PUBLIC
    STM32F103xB
    STM32F103
)

target_compile_options(nexus-llhal-stm32f103 PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -fdata-sections
    -ffunction-sections
)

# ARM-specific flags (uncomment when using ARM cross-compiler)
target_compile_options(nexus-llhal-stm32f103 PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -mfloat-abi=soft
)

# Installation
install(TARGETS nexus-llhal-stm32f103
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)
