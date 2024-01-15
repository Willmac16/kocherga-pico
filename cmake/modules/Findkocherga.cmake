# Given an .elf file produced by the linker, generate an .app.bin with the correct signature
# that is accepted by the Kocherga bootloader.
# The source ELF file will be also side-patched for convenience.
function(
        generate_kocherga_image
        ARG_ELF_FILE_NAME
        ARG_OUTPUT_BASENAME
)
    set(elf ${ARG_ELF_FILE_NAME})
    set(bin ${ARG_OUTPUT_BASENAME}.bin)
    set(app ${ARG_OUTPUT_BASENAME}.app.bin)
    add_custom_target(
            ${app} ALL
            # COMMAND rm -f *.bin  # Remove all previous build outputs
            COMMAND ${CMAKE_OBJCOPY} -O binary ${elf} ${bin}
            COMMAND ${KOCHERGA_IMAGE_TOOL} --lazy --verbose ${bin} --side-patch ${elf}
            COMMAND rm -f ${bin}
            DEPENDS ${elf}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating application image compatible with the Kocherga bootloader: ${app}"
    )
endfunction()

find_program(
        KOCHERGA_IMAGE_TOOL NAMES kocherga_image kocherga_image.py
        PATHS
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/lib
        PATH_SUFFIXES
        tools
        kocherga/tools
        REQUIRED
)
message(STATUS "Found kocherga_image tool: ${KOCHERGA_IMAGE_TOOL}")

# include(FindPackageHandleStandardArgs)

# find_package_handle_standard_args(kocherga REQUIRED_VARS KOCHERGA_VERSION)
