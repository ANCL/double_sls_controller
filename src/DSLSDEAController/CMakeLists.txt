cmake_minimum_required(VERSION 3.10)

AUX_SOURCE_DIRECTORY(./ DSLSDEACONTROLLER)

add_library(DSLSDEAController ${DSLSDEACONTROLLER})

target_include_directories (DSLSDEAController PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
