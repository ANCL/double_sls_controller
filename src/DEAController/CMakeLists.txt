cmake_minimum_required(VERSION 3.10)

AUX_SOURCE_DIRECTORY(./ DEACONTROLLER)

add_library(DEAController ${DEACONTROLLER})

target_include_directories (DEAController PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
