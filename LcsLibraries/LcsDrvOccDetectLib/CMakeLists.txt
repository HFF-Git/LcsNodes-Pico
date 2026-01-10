# ----------------------------------------------------------------------------------------
#  LCS - LcsNodes cmake file at the LcsDrvOccDetectLib directory level
#
#  Copyright (C) 2020 - 2026  Helmut Fieres
# ----------------------------------------------------------------------------------------
project( LcsDrvOccDetectLib C CXX ASM )

add_library( ${PROJECT_NAME} STATIC 
    
    LcsDrvOccDetectLib.h 
    LcsDrvOccDetectLib.cpp
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
    LcsRuntimeLib 
    pico_stdlib 
)

