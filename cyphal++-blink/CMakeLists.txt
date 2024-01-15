set(CB_MAJ_REV 0)
set(CB_MIN_REV 1)

string(TIMESTAMP BUILD_DATE %s UTC)

execute_process(
  COMMAND git log -1 --format=%h
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  OUTPUT_VARIABLE GIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

file(GLOB SOURCES
    "*.cpp"
    "*.c"
)

add_executable(cyphal++-blink ${SOURCES})

# pull in common dependencies
target_link_libraries(cyphal++-blink
    hardware_watchdog
    hardware_flash
    hardware_irq
    pico_stdlib
    pico_multicore
    pico_unique_id
    can2040
    cyphal++
)

target_include_directories(cyphal++-blink PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/inc)

target_compile_definitions(cyphal++-blink PRIVATE SOFTWARE_VERSION_MAJOR=${CB_MAJ_REV} SOFTWARE_VERSION_MINOR=${CB_MIN_REV} TIMESTAMP_UTC=${BUILD_DATE} GIT_HASH=0x${GIT_HASH})

pico_set_linker_script(cyphal++-blink ${PROJECT_SOURCE_DIR}/memmap_kocherga_app.ld)
find_package(kocherga REQUIRED)
generate_kocherga_image(cyphal++-blink.elf org.cwrubaja.pico.testboard)
add_custom_target(cyphal++-blink.elf ALL DEPENDS cyphal++-blink)

# Move the output .app.bin file to app_images directory
set(APP_IMAGES_DIR ${PROJECT_BINARY_DIR}/app_images)


add_custom_target(copy_app_bin_files ALL DEPENDS cyphal++-blink)
add_custom_command(TARGET copy_app_bin_files POST_BUILD
  COMMAND mkdir -p ${APP_IMAGES_DIR}
  COMMAND cp *.app.bin ${APP_IMAGES_DIR}
  COMMENT "Copying cyphal++-blink.app.bin to ${APP_IMAGES_DIR}"
)
