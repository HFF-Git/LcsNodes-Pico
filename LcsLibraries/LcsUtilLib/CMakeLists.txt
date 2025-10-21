# ----------------------------------------------------------------------------------------
#  CMAKE File
#  Copyright (C) 2025 - 2025  Helmut Fieres
# ----------------------------------------------------------------------------------------
project( LcsUtilLib C CXX ASM )

# add_library( ${PROJECT_NAME} INTERFACE ) 
add_library( ${PROJECT_NAME} STATIC

    LcsUtilLib.h
    LcsUtilLib.c
) 

target_include_directories( ${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} )