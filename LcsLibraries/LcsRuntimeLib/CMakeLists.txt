# ----------------------------------------------------------------------------------------
#  LCS - LcsNodes cmake file at the LcsRuntimeLib directory level
#
#  Copyright (C) 2020 - 2026  Helmut Fieres
# ----------------------------------------------------------------------------------------
project( LcsRuntimeLib C CXX ASM )

add_library( ${PROJECT_NAME} STATIC    

    LcsRtLibVersion.h
    LcsRuntimeLib.h 
    LcsRtLibInt.h 
    LcsRtItems.cpp 
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

target_link_libraries( ${PROJECT_NAME} 

    pico_stdlib 
    pico_multicore
    hardware_sync
    hardware_regs
    LcsUtilLib
    LcsCdcLib
    LcsDrvOccDetectLib
    LcsDrvServoLib
)

# this is a mess.... why do we need so many include directory specificatons ?
target_include_directories( ${PROJECT_NAME} 
    PRIVATE ${PICO_SDK_PATH}/src/rp2_common/pico_platform/include)

target_include_directories( ${PROJECT_NAME} 
    PRIVATE ${PICO_SDK_PATH}/src/rp2_common/hardware_regs/include)

target_include_directories( ${PROJECT_NAME} 
    PRIVATE ${PICO_SDK_PATH}/src/rp2040/hardware_regs/include)

target_include_directories( ${PROJECT_NAME} 
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} ) 

target_sources ( ${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} )
