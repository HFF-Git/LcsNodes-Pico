# ----------------------------------------------------------------------------------------
#  LCS - LcsNodes cmake file at the LcsDrvServoLib directory level
#
#  Copyright (C) 2024 - 2024  Helmut Fieres
# ----------------------------------------------------------------------------------------
project( LcsDrvServoLib C CXX ASM )

add_library( ${PROJECT_NAME} STATIC 
    
    LcsDrvServoLib.h 
    LcsDrvServoLib.cpp
) 

target_include_directories( ${PROJECT_NAME} INTERFACE 

    ${CMAKE_CURRENT_SOURCE_DIR} 
)

target_sources ( ${PROJECT_NAME} INTERFACE 

    ${CMAKE_CURRENT_SOURCE_DIR} 
)

target_link_libraries( ${PROJECT_NAME} 

    LcsRuntimeLib 
    LcsCdcLib 
    pico_stdlib 
)

