# ----------------------------------------------------------------------------------------
#  CMAKE File
#  Copyright (C) 2024 - 2025  Helmut Fieres
# ----------------------------------------------------------------------------------------
project( LcsCdcLib C CXX ASM )

add_library( ${PROJECT_NAME} STATIC 
    
    LcsCdcLibVersion.h
    LcsCdcLib.h 
    LcsCdcLib.cpp
) 

target_include_directories( ${PROJECT_NAME} PRIVATE 

    ${PICO_SDK_PATH}/src/rp2_common/pico_stdio_usb/include
)

target_include_directories( ${PROJECT_NAME} INTERFACE 

    ${CMAKE_CURRENT_SOURCE_DIR} 
)

target_sources ( ${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} )

target_link_libraries( ${PROJECT_NAME} 

    pico_stdlib 
    hardware_gpio
    hardware_i2c 
    hardware_clocks
    hardware_adc
    hardware_pwm
    hardware_uart
    hardware_spi
    LcsUtilLib
)

