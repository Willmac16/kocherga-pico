
add_library(crossCoreCan2040 STATIC crossCoreCan2040.c)

target_include_directories(crossCoreCan2040 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/)
target_link_libraries(crossCoreCan2040 PUBLIC
        hardware_exception
        hardware_flash
        pico_multicore
        pico_flash
        can2040
        cyphal++
)
