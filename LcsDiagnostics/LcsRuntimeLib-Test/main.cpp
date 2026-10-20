//----------------------------------------------------------------------------------------
//
// LCS - Runtime Library - Test Program
//
//----------------------------------------------------------------------------------------
// This source file contains a simple wrapper for the runtime library. The runtime 
// library features a simple command interpreter, which will be used to test the 
// library functions. So, all we need to do is to register any callbacks, initialize
// the runtime and the just start it.
//
//----------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Raspberry PI Pico Implementation
// Copyright (C) 2020 - 2026 Helmut Fieres
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
#include "LcsMainControllerBoardDesc.h"
#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------
// Global declarations.
//
//----------------------------------------------------------------------------------------
CdcResourceDescMap dMap = LCS_MAIN_CONTROLLER_BOARD_DESC_B_02_10;

//----------------------------------------------------------------------------------------
// When a main controller board is used to drive an extension board, the DIO pins 
// need to be set to the value 1 for both pins. This is equivalent to leaving the 
// extension boards select input pins open.
//
//----------------------------------------------------------------------------------------
uint8_t setupPinsForExtBoardTests( ) {

    uint8_t rStat = NO_ERR;
    if ( rStat == NO_ERR ) writeDio( RNUM_DIO_0, false  );
    if ( rStat == NO_ERR ) writeDio( RNUM_DIO_1, true );

    usbIoPrintf( CDC_USB_CONSOLE, "Setup DIO pins 0 and 1 for Extension Board Test, stat: %d \n", rStat );
    return( rStat );
}

//----------------------------------------------------------------------------------------
// Callbacks for testing runtime lib. All we do is to list their invocation.
//
//----------------------------------------------------------------------------------------
uint8_t lcsMsgCallback( uint8_t *msg, void *uData ) {

    usbIoPrintf( CDC_USB_CONSOLE, "MsgCallback: " );
    for ( int i = 0; i < 8; i++ ) usbIoPrintf( CDC_USB_CONSOLE, "0x%2x ");
    usbIoPrintf( CDC_USB_CONSOLE, "\n" );
    return( NO_ERR );
}

uint8_t lcsCmdCallback( uint8_t chan, char *cmdLine, void *uData  ) {

    if ( chan == CDC_USB_CONSOLE ) 
        usbIoPrintf( CDC_USB_CONSOLE, "Console Command Callback: %s\n", cmdLine );
     else if ( chan == CDC_USB_COMMAND ) 
        usbIoPrintf( CDC_USB_CONSOLE, "Command Callback: %s\n", cmdLine );
    else 
        usbIoPrintf( CDC_USB_CONSOLE, "Cmd Callback, invalid channel: %d\n", chan );

    return( NO_ERR );
}

uint8_t lcsTaskCallback1( void *uData ) {

    // usbIoPrintf( CDC_USB_CONSOLE, "Task Callback1...\n" );
    return( NO_ERR );    
}
uint8_t lcsTaskCallback2( void *uData ) {

    //usbIoPrintf( CDC_USB_CONSOLE, "Task Callback2...\n" );
    return( NO_ERR );    
}

uint8_t lcsInitCallback( uint16_t npId, void *uData  ) {

    usbIoPrintf( CDC_USB_CONSOLE, "Init Callback: 0x%x\n", npId );
    return( NO_ERR );
}

uint8_t lcsPfailCallback( uint16_t npId, void *uData  ) {

    usbIoPrintf( CDC_USB_CONSOLE, "Pfail Callback: 0x%x\n", npId );
    return( NO_ERR );
}

uint8_t lcsReqCallback( uint16_t npId, 
                        uint16_t item, 
                        uint16_t *arg1, 
                        uint16_t *arg2, 
                        void *uData  ) {

    usbIoPrintf( CDC_USB_CONSOLE, "REQ callback: npId: 0x%x, item: %d", npId, item );
    if ( arg1 != nullptr )  usbIoPrintf( CDC_USB_CONSOLE, ", arg1: 0x%x, ", *arg1 ); 
    else                    usbIoPrintf( CDC_USB_CONSOLE, ", arg1: null" );
    if ( arg2 != nullptr )  usbIoPrintf( CDC_USB_CONSOLE, ", arg2: 0x%x, ", *arg2 ); 
    else                    usbIoPrintf( CDC_USB_CONSOLE, ", arg2: null" );
    usbIoPrintf( CDC_USB_CONSOLE, "\n" );
    return( NO_ERR );
}

uint8_t lcsRepCallback( uint16_t npId, 
                        uint16_t item, 
                        uint16_t arg1, 
                        uint16_t arg2, 
                        uint8_t ret,
                        void *uData ) {

    usbIoPrintf( CDC_USB_CONSOLE, "REP callback: npId: 0x%x, item: %d, arg1: %d, arg2: %d, ret: %d",
             npId, item, arg1, arg2, ret );
    return( NO_ERR );
}

uint8_t lcsEventCallback( uint16_t npId, 
                          uint16_t eId, 
                          uint8_t eAction, 
                          uint16_t eData,
                          void *uData ) {

    usbIoPrintf( CDC_USB_CONSOLE, "Event: npId: 0x%x, eId: %d, eAction: %d, eData: %d\n", 
            npId, eId, eAction, eData );
    return( NO_ERR );
}

uint8_t lcsDccMsgCallback( uint8_t *msg, void *uData ) {

    usbIoPrintf( CDC_USB_CONSOLE, "DCC MsgCallback: " );
    for ( int i = 0; i < 8; i++ ) usbIoPrintf( CDC_USB_CONSOLE, "0x%2x ");
    usbIoPrintf( CDC_USB_CONSOLE, "\n" );
    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// The runtime features a rich set of callbacks. We will register all possible 
// callbacks for testing purposes.
//
//----------------------------------------------------------------------------------------
uint8_t registerLcsCallbacks( ) {

    registerLcsMsgCallback( lcsMsgCallback );
    registerDccMsgCallback( lcsDccMsgCallback );
    registerCmdCallback( lcsCmdCallback );
    registerTaskCallback( lcsTaskCallback1, 1000 );
    registerTaskCallback( lcsTaskCallback2, 2000 );
    registerInitCallback( lcsInitCallback );
    registerPfailCallback( lcsPfailCallback );
    registerReqCallback( lcsReqCallback );
    registerEventCallback( lcsEventCallback );
    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Init runtime and welcome.
//
//----------------------------------------------------------------------------------------
uint8_t initLcsRuntime( ) {

    uint16_t debugMask  =LCS_DBG_NIL;
    uint16_t options    = NPO_NIL;

    options     |= NPO_SKIP_NODE_ID_CONFIG;
    options     |= NPO_DISABLE_WATCHDOG;

    debugMask   |= LCS_DBG_ENABLE;
    debugMask   |= CDC_DBG_ENABLE;
    debugMask   |= LCS_DBG_SETUP;
    debugMask   |= LCS_DBG_ITEMS;
    debugMask   |= LCS_DBG_EVENTS;
    debugMask   |= LCS_DBG_CAN_BUS;
    debugMask   |= LCS_DBG_MSG_BUS;
    debugMask   |= LCS_DBG_NVM_ACCESS;
   // debugMask   |= CDC_DBG_I2C;

    uint8_t rStat = initRuntime( &dMap, options, debugMask );

    usbIoPrintf( CDC_USB_CONSOLE, "LCS Runtime Lib Test Program Setup ==============================\n" );
    return( rStat );
    
}

//----------------------------------------------------------------------------------------
// This is the last routine we call when the setup worked fine. We actually never 
// return.
//
//----------------------------------------------------------------------------------------
uint8_t startLcsRuntime( ) {

    usbIoPrintf( CDC_USB_CONSOLE, "LCS Runtime Lib Test Program Start Runtime ======================\n" );
    return( startRuntime( ));
}

//----------------------------------------------------------------------------------------
// Main. Set up the hardware, register the callbacks and just start the show.
//
//----------------------------------------------------------------------------------------
int main( ) {

    uint8_t rStat = initLcsRuntime( );
    if ( rStat == NO_ERR ) rStat = registerLcsCallbacks( );
    if ( rStat == NO_ERR ) rStat = startLcsRuntime( );
    return( NO_ERR );
}