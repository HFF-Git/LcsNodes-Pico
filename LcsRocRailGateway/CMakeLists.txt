# ----------------------------------------------------------------------------------------
#  LCS - RocRail Gateway
#
#  Copyright (C) 2020 - 2026  Helmut Fieres
# ----------------------------------------------------------------------------------------
project( LcsRocRailGateway C CXX ASM )

if ( TARGET tinyusb_device )

    add_executable( ${PROJECT_NAME} 
        
        LcsRocRailGatewayBoardDesc.h
        LcsRocRailGateway.h 
        LcsRocnetMsg.cpp
        LcsCanBusMsg.cpp
        main.cpp
    )

    # Make sure TinyUSB uses your config at compile time
    target_compile_definitions(tinyusb_device INTERFACE CFG_TUD_ENABLED=1)

    # Ensure TinyUSB config in this directory is used
    target_include_directories(${PROJECT_NAME} PRIVATE

        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    target_include_directories(tinyusb_device INTERFACE

        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    # Make sure TinyUSB is enabled
    target_link_libraries(${PROJECT_NAME} PRIVATE
        
        pico_stdlib
        tinyusb_device
        LcsCdcLib
        LcsRuntimeLib
    )

    # DO NOT enable stdio USB here
    pico_enable_stdio_usb(${PROJECT_NAME} 0)
    pico_enable_stdio_uart(${PROJECT_NAME} 1)
    pico_add_extra_outputs( ${PROJECT_NAME} )

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
        WARNING "not build test because tinyUSB submodule is not initialized in the SDK"
    )

endif( )
    