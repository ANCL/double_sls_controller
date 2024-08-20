cmake_minimum_required(VERSION 3.10)

AUX_SOURCE_DIRECTORY(./ QUADSLS_PT_CONTROLLER_QSF)

add_library(QuadSLS_PT_Controller_QSF ${QUADSLS_PT_CONTROLLER_QSF})

target_include_directories (QuadSLS_PT_Controller_QSF PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
