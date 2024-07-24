# ------------------------------------------------------------------------------------------------------------
#  LCS - LcsNodes cmake file at the LcsOledDisplayLib directory level
#
#  Copyright (C) 2024 - 2024  Helmut Fieres
# ------------------------------------------------------------------------------------------------------------
#
# 
#
# ------------------------------------------------------------------------------------------------------------
cmake_minimum_required( VERSION 3.19 )

project( LcsOledDisplayLib C CXX ASM )

set( CMAKE_C_STANDARD 11 )
set( CMAKE_CXX_STANDARD 17 ) 

# add the hw library files
add_library( ${PROJECT_NAME} STATIC
    
    LcsOledDisplayLib.h 
    LcsOledDisplayLib.cpp 
    SSD1306Ascii.h
    SSD1306Init.h
    SSD1306Ascii.cpp
) 

target_include_directories( ${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} )
target_sources ( ${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} )

# list the library dependencies. This is primarily the PICO standard and dedicated hardware libraries.
target_link_libraries( ${PROJECT_NAME} 

    pico_stdlib 
    LcsCdcLib 
    )
