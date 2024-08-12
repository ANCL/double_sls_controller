cmake_minimum_required(VERSION 3.10)

AUX_SOURCE_DIRECTORY(./ TRACCONTROLLER)

add_library(TracController ${TRACCONTROLLER})

target_include_directories (TracController PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
