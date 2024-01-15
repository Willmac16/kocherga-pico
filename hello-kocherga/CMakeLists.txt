set(SC_VERSION_MAJOR 1)
set(SC_VERSION_MINOR 0)
set(SC_VERSION_PATCH 0)

add_executable(hello-kocherga
        hello-kocherga.cpp
        launch_kocherga_bin.S
        )

target_include_directories(hello-kocherga PRIVATE ${CMAKE_CURRENT_LIST_DIR}/inc)

target_link_libraries(hello-kocherga
        hardware_flash
        hardware_sync
        hardware_watchdog
        pico_stdlib
        pico_platform
        pico_rand
        pico_multicore
        pico_unique_id
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


if(DEFINED HELLO_KOCHERGA_PAYLOAD)
  add_dependencies(hello-kocherga ${HELLO_KOCHERGA_PAYLOAD})

  add_custom_command(TARGET hello-kocherga
      POST_BUILD
      COMMAND python3 ${PROJECT_SOURCE_DIR}/elf_adder.py hello-kocherga.elf ../${HELLO_KOCHERGA_PAYLOAD}/${HELLO_KOCHERGA_PAYLOAD}.elf
      DEPENDS ${PROJECT_SOURCE_DIR}/elf_adder.py
      DEPENDS ${HELLO_KOCHERGA_PAYLOAD}.elf
      VERBATIM
      )
endif()
