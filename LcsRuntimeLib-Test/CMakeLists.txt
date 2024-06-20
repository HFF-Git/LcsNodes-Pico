# This is the CMAKE file at the App1 directory level

cmake_minimum_required( VERSION 3.29 )

project( LcsRuntimeLib-Test C CXX ASM )

set( CMAKE_C_STANDARD 11 )
set( CMAKE_CXX_STANDARD 17 )

if ( TARGET tinyusb_device )

    include_directories( LcsRuntimeLib LcsCdcLibLib ) 

    add_executable( ${PROJECT_NAME} main.cpp )
    target_link_libraries( ${PROJECT_NAME} LcsRuntimeLib LcsCdcLib pico_stdlib )

    pico_add_extra_outputs( ${PROJECT_NAME} )
    pico_enable_stdio_usb( ${PROJECT_NAME} 1 )
    pico_enable_stdio_uart( ${PROJECT_NAME} 0 )

elseif ( PICO_ON_DEVICE ) 

    message(WARNING "not build test because tinyUSB submodule is not iniialized in the SDK")

endif( )
    