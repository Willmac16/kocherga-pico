set(SC_VERSION_MAJOR 1)
set(SC_VERSION_MINOR 0)
set(SC_VERSION_PATCH 0)

add_executable(hello-kocherga
        hello-kocherga.cpp
        launch_kocherga_bin.S
        )

target_include_directories(hello-kocherga PRIVATE ${CMAKE_CURRENT_LIST_DIR}/inc)

target_link_libraries(hello-kocherga
        pico_stdlib
        pico_platform
        pico_rand
        pico_multicore
        hardware_flash
        hardware_sync
        hardware_watchdog
        can2040
        kocherga
        o1heap
        )

pico_enable_stdio_usb(hello-kocherga 0)
pico_enable_stdio_uart(hello-kocherga 1)

set(SC_VERSION ${SC_VERSION_MAJOR}.${SC_VERSION_MINOR}.${SC_VERSION_PATCH})
string(TIMESTAMP BUILD_DATE "%m-%d-%Y, %H:%M" UTC )
string(ASCII 27 Esc)

target_link_directories(hello-kocherga PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

pico_set_binary_type(hello-kocherga copy_to_ram)
pico_set_linker_script(hello-kocherga ${CMAKE_CURRENT_SOURCE_DIR}/memmap_kocherga_loader_in_ram.ld)

pico_add_extra_outputs(hello-kocherga)

add_dependencies(hello-kocherga ${HELLO_KOCHERGA_PAYLOAD})

add_custom_command(TARGET hello-kocherga
        POST_BUILD
        COMMAND ${PROJECT_SOURCE_DIR}/elf_adder.sh hello-kocherga.elf ../${HELLO_KOCHERGA_PAYLOAD}/${HELLO_KOCHERGA_PAYLOAD}.elf
        DEPENDS ${PROJECT_SOURCE_DIR}/elf_adder.sh
        DEPENDS ${HELLO_KOCHERGA_PAYLOAD}.elf
        VERBATIM
        )
