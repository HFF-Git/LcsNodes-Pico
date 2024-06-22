# This is the CMAKE file for the hardware layer library

cmake_minimum_required( VERSION 3.19 )

project( LcsDccPktFmtLib C CXX ASM )

set( CMAKE_C_STANDARD 11 )
set( CMAKE_CXX_STANDARD 17 ) 

# add the hw library files
add_library( ${PROJECT_NAME} STATIC 
    
    LcsDccPktFmtLib.h 
    LcsDccPktFmtLib.cpp
) 

target_include_directories( ${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} )
target_sources ( ${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} )

# list the library dependencies. This is primarily the PICO standard and dedicated hardware libraries.
target_link_libraries( ${PROJECT_NAME} 
    pico_stdlib 
    )

