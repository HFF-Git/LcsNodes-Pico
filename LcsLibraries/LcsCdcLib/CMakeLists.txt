# ----------------------------------------------------------------------------------------
#  CMAKE File
#  Copyright (C) 2024 - 2025  Helmut Fieres
# ----------------------------------------------------------------------------------------
project( LcsCdcLib C CXX ASM )

# ----------------------------------------------------------------------------------------  
add_library( LcsCdcLib STATIC

    LcsCdcLibVersion.h
    LcsCdcLib.h 
    LcsCdcLibInt.h
    LcsCdcLib.cpp
    LcsCdcLibTimer.cpp
    LcsCdcLibGPIO.cpp
    LcsCdcLibADC.cpp
    LcsCdcLibI2C.cpp
    LcsCdcLibUART.cpp
    LcsCdcLibPWM.cpp
    LcsCdcLibCAN.cpp
    LcsCdcLibUSB.cpp
) 

target_include_directories( ${PROJECT_NAME} INTERFACE 

    ${CMAKE_CURRENT_SOURCE_DIR} 
)

target_link_libraries( ${PROJECT_NAME} 

    pico_stdlib 
    hardware_gpio
    hardware_i2c 
    hardware_clocks
    hardware_adc
    hardware_pwm
    hardware_uart
    LcsUtilLib
)
