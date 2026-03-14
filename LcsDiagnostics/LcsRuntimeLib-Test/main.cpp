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
#include "LcsDrvOccDetectLib.h"

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------
// Global declarations.
//
//----------------------------------------------------------------------------------------
CdcResourceDescMap dMap = LCS_MAIN_CONTROLLER_BOARD_DESC_B_01_00;

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

    printf( "Setup DIO pins 0 and 1 for Extension Board Test, stat: %d \n", rStat );
    return( rStat );
}

//----------------------------------------------------------------------------------------
// Callbacks. All we do is to list their invocation.
//
//----------------------------------------------------------------------------------------
uint8_t lcsMsgCallback( uint8_t *msg, void *uData ) {

    printf( "MsgCallback: " );
    for ( int i = 0; i < 8; i++ ) printf( "0x%2x ");
    printf( "\n" );
    return( NO_ERR );
}

uint8_t lcsCmdCallback( char *cmdLine, void *uData  ) {

    printf( "Command Line Callback: %s\n", cmdLine );
    return( NO_ERR );
}

uint8_t lcsTaskCallback1( void *uData ) {

    // printf( "Task Callback1...\n" );
    return( NO_ERR );    
}
uint8_t lcsTaskCallback2( void *uData ) {

    //printf( "Task Callback2...\n" );
    return( NO_ERR );    
}

uint8_t lcsInitCallback( uint16_t npId, void *uData  ) {

    printf( "Init Callback: 0x%x\n", npId );
    return( NO_ERR );
}

uint8_t lcsPfailCallback( uint16_t npId, void *uData  ) {

    printf( "Pfail Callback: 0x%x\n", npId );
    return( NO_ERR );
}

uint8_t lcsReqCallback( uint16_t npId, 
                        uint16_t item, 
                        uint16_t *arg1, 
                        uint16_t *arg2, 
                        void *uData  ) {

    printf( "REQ callback: npId: 0x%x, item: %d", npId, item );
    if ( arg1 != nullptr )  printf( ", arg1: 0x%x, ", *arg1 ); 
    else                    printf( ", arg1: null" );
    if ( arg2 != nullptr )  printf( ", arg2: 0x%x, ", *arg2 ); 
    else                    printf( ", arg2: null" );
    printf( "\n" );
    return( NO_ERR );
}

uint8_t lcsRepCallback( uint16_t npId, 
                        uint16_t item, 
                        uint16_t arg1, 
                        uint16_t arg2, 
                        uint8_t ret,
                        void *uData ) {

    printf( "REP callback: npId: 0x%x, item: %d, arg1: %d, arg2: %d, ret: %d",
             npId, item, arg1, arg2, ret );
    return( NO_ERR );
}

uint8_t lcsEventCallback( uint16_t npId, 
                          uint16_t eId, 
                          uint8_t eAction, 
                          uint16_t eData,
                          void *uData ) {

    printf( "Event: npId: 0x%x, eId: %d, eAction: %d, eData: %d\n", 
            npId, eId, eAction, eData );
    return( NO_ERR );
}

uint8_t lcsDccMsgCallback( uint8_t *msg, void *uData ) {

    printf( "DCC MsgCallback: " );
    for ( int i = 0; i < 8; i++ ) printf( "0x%2x ");
    printf( "\n" );
    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// The runtime features a rich set of callbacks. We will register all possible 
// callbacks for testing purposes.
//
//----------------------------------------------------------------------------------------
uint8_t registerLcsCallbacks( ) {

    printf( "Registering Callbacks\n" );
    registerLcsMsgCallback( lcsMsgCallback );
    registerDccMsgCallback( lcsDccMsgCallback );
    registerCmdCallback( lcsCmdCallback );
    registerTaskCallback( lcsTaskCallback1, 1000 );
    registerTaskCallback( lcsTaskCallback2, 2000 );
    registerInitCallback( lcsInitCallback );
    registerPfailCallback( lcsPfailCallback );
    registerReqCallback( lcsReqCallback );
    registerRepCallback( lcsRepCallback );
    registerEventCallback( lcsEventCallback );
    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Setup the drivers for extension boards.
//
//----------------------------------------------------------------------------------------
uint8_t registerLcsDrvFunctions( ) {

    printf( "Register Extension Board Drivers\n" );
    uint8_t ret = registerDrvFunc( lcsDrvOccDetect, CDC_BT_EXT_OCC_DETECT );
    if ( ret != NO_ERR )  printf( "Registration failed: %d\n, ret ");

    return( ret );
}

//----------------------------------------------------------------------------------------
// Init runtime and welcome.
//
//----------------------------------------------------------------------------------------
uint8_t initLcsRuntime( ) {

    uint8_t rStat = initRuntime( &dMap, 
                                 ( NPO_SKIP_NODE_ID_CONFIG | NPO_DISABLE_WATCHDOG ),
                                 ( LCS_DBG_ENABLE | LCS_DBG_SETUP ));

    printf( "LCS Runtime Lib Test Program Setup ==============================\n" );
    return( rStat );
    
}

//----------------------------------------------------------------------------------------
// This is the last routine we call when the setup worked fine. We actually never 
// return.
//
//----------------------------------------------------------------------------------------
void startLcsRuntime( ) {

    printf( "LCS Runtime Lib Test Program Start Runtime ======================\n" );
    startRuntime( );
}

//----------------------------------------------------------------------------------------
// Main. Set up the hardware, register the callbacks and just start the show.
//
//----------------------------------------------------------------------------------------
int main( ) {

    uint8_t rStat = initLcsRuntime( );
    if ( rStat == NO_ERR ) rStat = setupPinsForExtBoardTests( );
    if ( rStat == NO_ERR ) rStat = registerLcsCallbacks( );
    if ( rStat == NO_ERR ) rStat = registerLcsDrvFunctions( );
    if ( rStat == NO_ERR ) startLcsRuntime( );
    return( NO_ERR );
}