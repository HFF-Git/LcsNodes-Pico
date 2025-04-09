# ------------------------------------------------------------------------------------------------------------
#  LCS - LcsNodes cmake file at the LcsBoardDescriptor directroy
#
#  Copyright (C) 2024 - 2025  Helmut Fieres
# ------------------------------------------------------------------------------------------------------------
#
# 
#
# ------------------------------------------------------------------------------------------------------------
cmake_minimum_required( VERSION 3.19 )

project( LcsBoardDescriptors C CXX ASM )

set( CMAKE_C_STANDARD 11 )
set( CMAKE_CXX_STANDARD 17 ) 

# add the library files
add_library( ${PROJECT_NAME} STATIC 
    
    LcsBoardDescriptors.h
) 

target_include_directories( ${PROJECT_NAME} PRIVATE ${PICO_SDK_PATH}/src/rp2_common/pico_stdio_usb/include)
target_include_directories( ${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} )
target_sources ( ${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} )

# list the library dependencies. This is primarily the PICO standard and dedicated hardware libraries.
target_link_libraries( ${PROJECT_NAME} 
    pico_stdlib 
)