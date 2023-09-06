set(KB_MAJ_REV 0)
set(KB_MIN_REV 2)

string(TIMESTAMP BUILD_DATE %s UTC)

execute_process(
    COMMAND git log -1 --format=%h
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    OUTPUT_VARIABLE GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )

add_executable(kocherga-blink
        kocherga-blink.cpp
        )

# pull in common dependencies
target_link_libraries(kocherga-blink
        pico_stdlib
        pico_multicore)

pico_add_extra_outputs(kocherga-blink)

target_link_directories(kocherga-blink PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_definitions(kocherga-blink PRIVATE SOFTWARE_VERSION_MAJOR=${KB_MAJ_REV} SOFTWARE_VERSION_MINOR=${KB_MIN_REV} TIMESTAMP_UTC=${BUILD_DATE} GIT_HASH=0x${GIT_HASH})

pico_set_linker_script(kocherga-blink ${CMAKE_CURRENT_SOURCE_DIR}/memmap_kocherga_app.ld)
find_package(kocherga REQUIRED)
generate_kocherga_image(kocherga-blink.elf kocherga-blink)
