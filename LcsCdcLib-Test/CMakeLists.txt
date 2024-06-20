# This is the CMAKE file at the LcsCdcLib-Test directory level

cmake_minimum_required( VERSION 3.29 )

project( LcsCdcLib-Test C CXX ASM )

set( CMAKE_C_STANDARD 11 )
set( CMAKE_CXX_STANDARD 17 )

if ( TARGET tinyusb_device )

    include_directories( LcsCdcLib pico_stdlib ) 

    add_executable( ${PROJECT_NAME} main.cpp )
    target_link_libraries( ${PROJECT_NAME} LcsCdcLib pico_stdlib )

    pico_add_extra_outputs( ${PROJECT_NAME} )
    pico_enable_stdio_usb( ${PROJECT_NAME} 1 )
    pico_enable_stdio_uart( ${PROJECT_NAME} 0 )

    # Add a custom command to run a shell script. We would like to move the uf2 file to a dedicated directory.
    # The script is executed after the build proccess.
    add_custom_command(
     TARGET ${PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "Running shell script..."
    
        # under construction.
    
    )

elseif ( PICO_ON_DEVICE ) 

    message(WARNING "not build test because tinyUSB submodule is not initialized in the SDK")

endif( )
    