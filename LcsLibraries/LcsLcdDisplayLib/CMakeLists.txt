# ----------------------------------------------------------------------------------------
#  LCS - LcsNodes cmake file at the LcsLcdDisplayLib directory level
#
#  Copyright (C) 2024 - 2025  Helmut Fieres
# ----------------------------------------------------------------------------------------
project( LcsLcdDisplayLib C CXX ASM )

add_library( ${PROJECT_NAME} STATIC 
    
    LcsLcdDisplayLib.h 
    LcsLcdDisplayLib.cpp 
) 

target_include_directories( ${PROJECT_NAME} INTERFACE 

    ${CMAKE_CURRENT_SOURCE_DIR} 
)

target_sources ( ${PROJECT_NAME} INTERFACE 

    ${CMAKE_CURRENT_SOURCE_DIR} 
)

target_link_libraries( ${PROJECT_NAME} 

    LcsUtilLib
    LcsCdcLib 
    pico_stdlib 
    hardware_i2c 
    hardware_clocks
)