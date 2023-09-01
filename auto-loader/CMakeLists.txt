# Offset copy elf

add_executable(auto-loader-offset
        auto-loader.cpp
        lkb-c2r.s
        )

# pull in common dependencies
target_link_libraries(auto-loader-offset
        pico_stdlib)

pico_add_extra_outputs(auto-loader-offset)

target_include_directories(auto-loader-offset PRIVATE ${CMAKE_CURRENT_LIST_DIR}/inc)

target_link_directories(auto-loader-offset PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

pico_set_linker_script(auto-loader-offset ${CMAKE_CURRENT_SOURCE_DIR}/memmap_offset_autoloader.ld)


# Copy to Ram elf
add_executable(auto-loader-c2r
        auto-loader.cpp
        lkb-off.s # Include the opposite binary launcher to switch over to the offset XIP Version
        )

# pull in common dependencies
target_link_libraries(auto-loader-c2r
        pico_stdlib)

pico_add_extra_outputs(auto-loader-c2r)


target_include_directories(auto-loader-c2r PRIVATE ${CMAKE_CURRENT_LIST_DIR}/inc)

target_link_directories(auto-loader-c2r PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

pico_set_binary_type(auto-loader-c2r copy_to_ram)
pico_set_linker_script(auto-loader-c2r ${CMAKE_CURRENT_SOURCE_DIR}/memmap_c2r_autoloader.ld)

# We need offset done first so we can create our Fraken-Elf
add_dependencies(auto-loader-c2r auto-loader-offset)

add_custom_command(TARGET auto-loader-c2r
        POST_BUILD
        COMMAND ${PROJECT_SOURCE_DIR}/elf_adder.sh auto-loader-c2r.elf auto-loader-offset.elf
        )
