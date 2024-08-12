cmake_minimum_required(VERSION 3.10)

AUX_SOURCE_DIRECTORY(./ STABCONTROLLER)

add_library(StabController ${STABCONTROLLER})

target_include_directories (StabController PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
