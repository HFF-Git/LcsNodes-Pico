# ----------------------------------------------------------------------------------------
#  LCS -  Runtime Lib test
#
#  Copyright (C) 2020 - 2026  Helmut Fieres
# ----------------------------------------------------------------------------------------
project( LcsRuntimeLibTest C CXX ASM )

if ( TARGET tinyusb_device )

    include_directories( LcsDccPktFmtLib LcsRuntimeLib LcsCdcLib pico_stdlib ) 

    add_executable( ${PROJECT_NAME} 
    
        main.cpp
        LcsMainControllerBoardDesc.h
    )
    
    target_link_libraries( ${PROJECT_NAME}  
    
        LcsDccPktFmtLib 
        LcsRuntimeLib 
        LcsCdcLib 
        pico_stdlib 
    )

    pico_add_extra_outputs( ${PROJECT_NAME} )
    pico_enable_stdio_usb( ${PROJECT_NAME} 1 )
    pico_enable_stdio_uart( ${PROJECT_NAME} 0 )

    # Add a custom command to run a shell script. We would like to move the uf2 file to
    # a dedicated directory. The script is executed after the build proccess. The target
    # directory is "bin" a child at the overall project level.

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

    message(
        WARNING "not build test because tinyUSB submodule is not iniialized in the SDK"
    )

endif( )
    