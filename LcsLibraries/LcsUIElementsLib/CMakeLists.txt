# ------------------------------------------------------------------------------
#  LCS - UI Elements cmake file at the LcsUIElementsLib directory level
#
#  Copyright (C) 2024 - 2025  Helmut Fieres
# ------------------------------------------------------------------------------
#
# 
# ------------------------------------------------------------------------------
cmake_minimum_required( VERSION 3.19 )

project( LcsUIElementsLib C CXX ASM )

add_library( ${PROJECT_NAME} STATIC 

    LcsUIElements.h
    LcsUIElements.cpp
    LcsUIButton.cpp
    LcsUIDisplay.cpp
    LcsUIEncoder.cpp
    LcsUILed.cpp
    LcsUIScreen.cpp
) 

target_include_directories( ${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} )
target_sources ( ${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} )

target_link_libraries( ${PROJECT_NAME} 

    pico_stdlib 
    LcsUtilLib
    LcsCdcLib
    LcsLcdDisplayLib
    LcsOledDisplayLib
)

