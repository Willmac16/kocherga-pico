set(KB_MAJ_REV 0)
set(KB_MIN_REV 1)

string(TIMESTAMP BUILD_DATE %s UTC)

execute_process(
    COMMAND git log -1 --format=%h
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    OUTPUT_VARIABLE GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )

add_executable(cyphal++-blink
        cyphal++-blink.cpp
        crossCoreCan2040.c
        )

# pull in common dependencies
target_link_libraries(cyphal++-blink
        hardware_watchdog
        hardware_flash
        hardware_irq
        pico_stdlib
        pico_multicore
        pico_unique_id
        can2040
        cyphal++)

pico_add_extra_outputs(cyphal++-blink)

target_include_directories(cyphal++-blink PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/inc)

target_compile_definitions(cyphal++-blink PRIVATE SOFTWARE_VERSION_MAJOR=${KB_MAJ_REV} SOFTWARE_VERSION_MINOR=${KB_MIN_REV} TIMESTAMP_UTC=${BUILD_DATE} GIT_HASH=0x${GIT_HASH})

pico_set_linker_script(cyphal++-blink ${PROJECT_SOURCE_DIR}/memmap_kocherga_app.ld)
find_package(kocherga REQUIRED)
generate_kocherga_image(cyphal++-blink.elf cyphal++-blink)
