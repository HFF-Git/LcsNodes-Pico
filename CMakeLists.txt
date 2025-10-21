# ------------------------------------------------------------------------------
#  CMAKE File
#  Copyright (C) 2024 - 2025  Helmut Fieres
# ------------------------------------------------------------------------------
# This is the CMAKE file at the LcsNodes directory level. It sets up the cmake 
# version, the languages used, defines the project and finally initializes the 
# Raspberry Pi Pico SDK. The actual cmake work is done in the local cmake files
# of programs and libraries. This file will just define the subdirectories.
#
# ------------------------------------------------------------------------------
cmake_minimum_required( VERSION 3.29 )

Message ( "LcsNodes Cmake" )
message ( "SDK-PATH: $ENV{PICO_SDK_PATH}" )

include( $ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake )

project( LcsNodes C CXX ASM )

pico_sdk_init( )

set( CMAKE_C_STANDARD 11 )
set( CMAKE_CXX_STANDARD 17 )

add_subdirectory( LcsDiagnostics/LcsCdcLib-Test )
add_subdirectory( LcsDiagnostics/LcsRuntimeLib-Test )
add_subdirectory( LcsDiagnostics/LcsOledTest )

add_subdirectory( LcsBaseStation )
add_subdirectory( LcsBlockController )
add_subdirectory( LcsBasicThrottle )
add_subdirectory( LcsDieselThrottle )
add_subdirectory( LcsDccMonitor )

add_subdirectory( LcsLibraries/LcsUtilLib )
add_subdirectory( LcsLibraries/LcsCdcLib )
add_subdirectory( LcsLibraries/LcsDccPktFmtLib )
add_subdirectory( LcsLibraries/LcsDrvOccDetectLib )
add_subdirectory( LcsLibraries/LcsDrvServoLib )
add_subdirectory( LcsLibraries/LcsRuntimeLib )
add_subdirectory(LcsLibraries/LcsUIElementsLib)
add_subdirectory(LcsLibraries/LcsLcdDisplayLib)
add_subdirectory(LcsLibraries/LcsOledDisplayLib)
