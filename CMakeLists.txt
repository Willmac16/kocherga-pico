# Main CMakeLists.txt file for the picodaq project.

cmake_minimum_required(VERSION 3.12)

set(PICO_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/sdk/pico-sdk)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

# Pull in SDK before project
include(sdk/pico-sdk/external/pico_sdk_import.cmake)

# Set up the main project
project(pico_kocherga C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

# Initialize the SDK
pico_sdk_init()

# Upstream compiler options
add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        -Wno-attributes
        )

# Libraries and dependencies
add_subdirectory(lib)

# Select what app payload to ship with hello-kocherga.elf
set(HELLO_KOCHERGA_PAYLOAD "cyphal++-blink")
# set(HELLO_KOCHERGA_PAYLOAD "kocherga-blink")


# Targets
add_subdirectory(hello-kocherga)
# add_subdirectory(kocherga-blink)
# add_subdirectory(auto-loader)
add_subdirectory(cyphal++-blink)


