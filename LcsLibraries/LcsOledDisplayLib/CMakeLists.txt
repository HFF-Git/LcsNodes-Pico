# ----------------------------------------------------------------------------------------
#  LCS - LcsNodes cmake file at the LcsOledDisplayLib directory level
#
#  Copyright (C) 2020 - 2026  Helmut Fieres
# ----------------------------------------------------------------------------------------
project( LcsOledDisplayLib C CXX ASM )

add_library( ${PROJECT_NAME} STATIC
    
    LcsOledDisplayLib.h 
    LcsOledDisplayLib.cpp 
) 

target_include_directories( ${PROJECT_NAME} INTERFACE 

    ${CMAKE_CURRENT_SOURCE_DIR} 
)

target_sources ( ${PROJECT_NAME} INTERFACE 

    ${CMAKE_CURRENT_SOURCE_DIR} 
)

target_link_libraries( ${PROJECT_NAME} 

    pico_stdlib 
    LcsUtilLib
    LcsCdcLib 
)
