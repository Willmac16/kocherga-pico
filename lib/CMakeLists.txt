#[[
	This source tree is responsible for out of tree vendored libraries which
	are used in the T-series unikernel.
	
	For libraries which contain their own build systems, this file calls
	into those systems and lets them handle the compilation.

	For libraries which do not contain a build system, this listfile manages
	the compilation for us.
]]

# Libraries with a build system
add_subdirectory(107-Arduino-Cyphal)


# Libraries without a build system
add_library(o1heap STATIC o1heap/o1heap/o1heap.c)


target_include_directories(o1heap PUBLIC o1heap/o1heap)

# can2040
add_library(can2040 can2040/src/can2040.c)
target_include_directories(can2040 PUBLIC
        can2040/src/
        can2040/pio/
        ../sdk/pico-sdk/src/rp2_common/cmsis/stub/CMSIS/Device/RaspberryPi/RP2040/Include/
        ../sdk/pico-sdk/src/rp2_common/cmsis/stub/CMSIS/Core/Include/
        )

target_link_libraries(can2040 PUBLIC
        cmsis_core
        hardware_irq
        pico_stdlib
        hardware_pio
        )

# Header only libraries
add_library(kocherga INTERFACE)
target_include_directories(kocherga INTERFACE kocherga/kocherga)
