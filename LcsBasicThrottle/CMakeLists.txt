# ------------------------------------------------------------------------------------------------------------
#  LCS - Basic Throttle cmake file at the LcsBasicThrottle directory level
#
#  Copyright (C) 2024 - 2024  Helmut Fieres
# ------------------------------------------------------------------------------------------------------------
#
# 
#
# ------------------------------------------------------------------------------------------------------------
cmake_minimum_required( VERSION 3.29 )

project( LcsBasicThrottle C CXX ASM )

set( CMAKE_C_STANDARD 11 )
set( CMAKE_CXX_STANDARD 17 )

if ( TARGET tinyusb_device )

    include_directories( LcsUIElementsLib LcsRuntimeLib LcsCdcLib pico_stdlib ) 

    add_executable( ${PROJECT_NAME} 
    
        main.cpp 
        LcsBasicThrottle.h 
        LcsBtCabStack.cpp 
        LcsBtMsgInterface.cpp 
        LcsBtScreens.cpp 
        LcsBtUiElements.cpp 
    )
    
    target_link_libraries( ${PROJECT_NAME} LcsRuntimeLib LcsUIElementsLib LcsCdcLib pico_stdlib )

    pico_add_extra_outputs( ${PROJECT_NAME} )
    pico_enable_stdio_usb( ${PROJECT_NAME} 1 )
    pico_enable_stdio_uart( ${PROJECT_NAME} 0 )

    # Add a custom command to run a shell script. We would like to move the uf2 file to a dedicated directory.
    # The script is executed after the build proccess. The target directory is "bin" a child at the overall
    # project level.

    add_custom_command(
        TARGET ${PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "Move target to bin directory"
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/../bin
        COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.uf2
        ${CMAKE_BINARY_DIR}/../bin/${PROJECT_NAME}.uf2
    )

elseif ( PICO_ON_DEVICE ) 

    message(WARNING "not build test because tinyUSB submodule is not initialized in the SDK")

endif( )
    