//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
//
//----------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
// Copyright (C) 2022 - 2025 Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under 
// the terms of the GNU General Public License as published by the Free Software 
// Foundation, either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details. You 
// should have received a copy of the GNU General Public License along with this 
// program. If not, see <http://www.gnu.org/licenses/>.
//
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------
#include "LcsCdcLib.h"
#include "LcsCdcLibInt.h"

//----------------------------------------------------------------------------------------
// Local name space. 
//
//----------------------------------------------------------------------------------------
namespace {

using namespace CDC;


}

//----------------------------------------------------------------------------------------
// Global variables for the CDC lib. Declared in "LcsCdcLib.cpp".
//
//----------------------------------------------------------------------------------------
namespace CDC {

    extern uint16_t                debugMask;
    extern uint16_t                options;

    extern CdcResourceDescMap      dMap;
    extern CdcResourceMap          rMap;

    extern CdcResource *lookupResource( uint8_t rNum, uint8_t type );
    extern CdcResource *allocateResourceType( uint8_t rNum, uint8_t type );
}

//----------------------------------------------------------------------------------------
// The CDC name space routines declared in this file.
//
//----------------------------------------------------------------------------------------
namespace CDC {

//----------------------------------------------------------------------------------------
// CAN bus Section. The CAN bus is the message bus used for LCS. For the PICO 
// there is a library "can2040" which implements the CAN protocol using the PIO 
// state machine. This saves us hardware. The resource is the structure where we
// just keep the HW pins, the baud rate, and whether we run on one or two CPUs. 
// In other words, we do not describe a PICO hardware block.
//
//----------------------------------------------------------------------------------------
uint8_t configureCanBus( uint8_t rNum ) {

    CdcResourceDesc *dPtr = lookupResourceDesc( rNum, CDC_RT_CAN_BUS );
    if ( dPtr == nullptr ) return ( RES_NUM_ERR );
   
    return ( configureCanBus( rNum, 
                              dPtr -> can.rxPin, 
                              dPtr -> can.txPin, 
                              dPtr -> can.baudRate, 
                              dPtr -> can.twoCores ));
}

//----------------------------------------------------------------------------------------
// Configure the CAN bus.
//
//----------------------------------------------------------------------------------------
uint8_t configureCanBus( uint8_t  rNum, 
                         uint8_t  rxPin, 
                         uint8_t  txPin, 
                         uint32_t baudRate, 
                         bool     twoCores ) {

    CdcResource *rPtr = allocateResourceType( rNum, CDC_RT_CAN_BUS );
    if ( rPtr == nullptr ) return ( RES_NUM_ERR );

    rPtr -> can.canPinRx = rxPin;
    rPtr -> can.canPinTx = txPin;
    rPtr -> can.baudRate = baudRate;
    rPtr -> can.twoCores = twoCores;

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Helper functions. 
//
//----------------------------------------------------------------------------------------
uint8_t canGetRxPin( uint8_t rNum ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_CAN_BUS );
    if ( rPtr == nullptr ) return ( UNDEFINED_PIN );

    return ( rPtr -> can.canPinRx );
}

uint8_t canGetTxPin( uint8_t rNum ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_CAN_BUS );
    if ( rPtr == nullptr ) return ( UNDEFINED_PIN );

    return ( rPtr -> can.canPinTx );
}

uint32_t canGetBaudrate( uint8_t rNum ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_CAN_BUS );
    if ( rPtr == nullptr ) return ( 0 );

    return ( rPtr -> can.baudRate );
}

bool canGetTwoCores( uint8_t rNum ) {

    CdcResource *rPtr = lookupResource( rNum, CDC_RT_CAN_BUS );
    if ( rPtr == nullptr ) return ( false );

    return ( rPtr -> can.twoCores );
}

} // namespace CDC
