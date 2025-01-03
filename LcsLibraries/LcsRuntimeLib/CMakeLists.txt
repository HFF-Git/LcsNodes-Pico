# ------------------------------------------------------------------------------------------------------------
#  LCS - LcsNodes cmake file at the LcsRuntimeLib directory level
#
#  Copyright (C) 2024 - 2024  Helmut Fieres
# ------------------------------------------------------------------------------------------------------------
#
# 
#
# ------------------------------------------------------------------------------------------------------------
cmake_minimum_required( VERSION 3.29 )

project( LcsRuntimeLib C CXX ASM )

set( CMAKE_C_STANDARD 11 )
set( CMAKE_CXX_STANDARD 17 )

# Add the runtime library. We add the source files and also the source files from a third party lib that is 
# available as only source files. Not very elegant yet. But it works.

add_library( ${PROJECT_NAME} STATIC    
    LcsRuntimeLib.h 
    LcsRtLibInt.h 
    LcsRtAttributes.cpp 
    LcsRtCanBus.cpp
    LcsRtCommands.cpp 
    LcsRtCore.cpp 
    LcsRtEvents.cpp 
    LcsRtMsgBus.cpp 
    LcsRtNvmI2C.cpp 
    LcsRtSetup.cpp 
    Can2040Lib/can2040.c 
    Can2040Lib/can2040.h 
    ) 

# list the library dependencies. This is primarily the PICO standard and dedicated hardware libraries.

target_link_libraries( ${PROJECT_NAME} 

    pico_stdlib 
    pico_multicore
    hardware_sync
    hardware_regs
    LcsCdcLib
    LcsDrvOccDetectLib
    LcsDrvServoLib
    )

# this is a mess.... why do we need so many include directory specificatons ?
target_include_directories( ${PROJECT_NAME} PRIVATE ${PICO_SDK_PATH}/src/rp2_common/pico_platform/include)
target_include_directories( ${PROJECT_NAME} PRIVATE ${PICO_SDK_PATH}/src/rp2_common/hardware_regs/include)
target_include_directories( ${PROJECT_NAME} PRIVATE ${PICO_SDK_PATH}/src/rp2040/hardware_regs/include)
target_include_directories( ${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} ) 

target_sources ( ${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} )
