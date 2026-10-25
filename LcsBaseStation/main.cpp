//----------------------------------------------------------------------------------------
//
// LCS - Base Station
//
//----------------------------------------------------------------------------------------
// This is the main program for the LCS base station. Every layout would need at 
// least a base station. Its primary task is to manage the DCC loco sessions, 
// generate the DCC signals and manage the dual DCC track power outputs.
//
// Like all other LcsNodes, the base station will provide a rich set of variable 
// that can be set and queried. In addition, the base features a command line
// extension which implements the DCC++/DCC-EX style commands and some more base 
// station specific commands. The idea for the DCC++ command syntax and commands 
// is that these command can also be submitted by a third party software 
// ( e.g. JMRI ). An example would be the JMRI CV programming tool.
//
// ??? we need an idea of system time like DCC. To be broadcasted periodically.
// ??? we also need a broadcast of the layout system capabilities....
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
// This program is distributed in the hope that it will be useful, but WITHOUT 
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with 
// this program. If not, see <http://www.gnu.org/licenses/>.
//
//----------------------------------------------------------------------------------------
#include "LcsBaseStationBoardDesc.h"
#include "LcsBaseStation.h"

using namespace LCS;
using namespace CDC;

//----------------------------------------------------------------------------------------
// Base station global data.
//
//----------------------------------------------------------------------------------------
uint16_t                    debugMask;
CdcResourceDescMap          dMap;
LcsDccTrack                 *mainTrack;
LcsDccTrack                 *progTrack;
LcsLocoSessions             *locoSessions;

//----------------------------------------------------------------------------------------
// Some little helper functions.
//
//----------------------------------------------------------------------------------------
void printLcsMsg( uint8_t *msg ) {

  int msgLen = (( msg[0] >> 5 ) + 1 ) % 8;

  for ( int i = 0; i < msgLen; i++ ) usbIoPrintf( CDC_USB_CONSOLE, "0x%x ", msg[i] );
  usbIoPrintf( CDC_USB_CONSOLE, "\n" );
}

uint8_t printStatus (uint8_t status ) {

  usbIoPrintf( CDC_USB_CONSOLE, "Status: " );
  if ( status == LCS_OK ) usbIoPrintf( CDC_USB_CONSOLE, "OK\n" );
  else printf ( "FAILED: %d\n", status );
  return ( status );
}

//----------------------------------------------------------------------------------------
// The node and port initialization callback.
//
//----------------------------------------------------------------------------------------
uint8_t lcsInitCallback( uint16_t npId, void *uData ) {

    usbIoPrintf( CDC_USB_CONSOLE, "Node Init Callback: 0x%4x\n" );
    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// The node or port power fail callback.
//
//----------------------------------------------------------------------------------------
uint8_t lcsPfailCallback( uint16_t npId, void *uData ) {

    usbIoPrintf( CDC_USB_CONSOLE, "Node PFAIL Callback: 0x%4x\n" );
    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// The base station has also a command line interpreter. The callback is invoked 
// by the core library when there is a command that it does not handle.
//
//----------------------------------------------------------------------------------------
uint8_t lcsCmdCallback( uint8_t chan, char *cmdLine, void *uData ) {

    if ( chan == CDC_USB_CONSOLE ) {

        return ( handleSerialCommand( chan, cmdLine ));
    }
    else if ( chan == CDC_USB_COMMAND ) {

        if ( cmdLine[ 0 ] == '<' ) {

            return ( handleSerialCommand( chan, cmdLine ));
        }
        else if ( cmdLine[ 0 ] == '@' ) {

            return ( ERR_INVALID_COMMAND );
        }
    }
    else return ( ERR_INVALID_CHAN );
}

//----------------------------------------------------------------------------------------
// Other LCS message callbacks. All we do is to list their invocation. ( for now )
//
//----------------------------------------------------------------------------------------
uint8_t lcsMsgCallback( uint8_t *msg, void *uData ) {

    usbIoPrintf( CDC_USB_CONSOLE, "MsgCallback: ", msg  );

    for ( int i = 0; i < 8; i++ ) usbIoPrintf( CDC_USB_CONSOLE, "0x%2x ");
    usbIoPrintf( CDC_USB_CONSOLE, "\n" );
    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// When the base station node receives a request with an item defined in the 
// user item range or the base station itself issues such a request, the 
// defined callback is invoked.
//
//----------------------------------------------------------------------------------------
uint8_t lcsReqCallback( uint16_t npId, 
                        uint16_t item, 
                        uint16_t *arg1, 
                        uint16_t *arg2, 
                        void *uData ) {

    usbIoPrintf( CDC_USB_CONSOLE, "REQ callback: npId: 0x%x, item: %d", npId, item );
    
    if ( arg1 != nullptr ) usbIoPrintf( CDC_USB_CONSOLE, ", arg1: %d, ", *arg1 ); 
    else usbIoPrintf( CDC_USB_CONSOLE, ", arg1: null" );
    
    if ( arg2 != nullptr ) usbIoPrintf( CDC_USB_CONSOLE, ", arg2: %d, ", *arg2 ); 
    else usbIoPrintf( CDC_USB_CONSOLE, ", arg2: null" );
    
    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// When the base station gets a reply message for a request previously sent, 
// this callback is invoked.
//
// ??? this has changed, as each REQ call can specify its callback. For now,
// this is just for debugging...
//----------------------------------------------------------------------------------------
uint8_t lcsRepCallback( uint16_t npId, 
                        uint16_t item,
                        uint16_t arg1, 
                        uint16_t arg2, 
                        uint8_t ret,
                        void *uData ) {

    usbIoPrintf( CDC_USB_CONSOLE, 
                 "REP callback: npId: 0x%x, item: %d, arg1: %d, arg2: %d, ret: %d ",
                 npId, item , arg1, arg2, ret );
    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// For any event on the LCS system that the base station is interested in, this 
// callback is invoked.
//
// ??? generic version fro debugging...
//----------------------------------------------------------------------------------------
uint8_t lcsEventCallback( uint16_t npId, 
                          uint16_t eId, 
                          uint8_t eAction, 
                          uint16_t eData,
                          void *uData ) {

    usbIoPrintf( CDC_USB_CONSOLE, "Event: npId: 0x%x, eId: %d, eAction: %d, eData: %d\n", 
            npId, eId, eAction, eData );
    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Init the Runtime.
//
//----------------------------------------------------------------------------------------
uint8_t initRunTimeLib( ) {

    dMap = LCS_BASE_STATION_BOARD_DESC_B_01_00;

    uint8_t rStat = initRuntime( &dMap );
    usbIoPrintf( CDC_USB_CONSOLE, "LCS Base Station\n" );
    
    printResourceDescMap( &dMap );
    printResourceMap( );

    return( printStatus( rStat ));
}

//----------------------------------------------------------------------------------------
// This routine initializes the LocoSessions Object.
//
//----------------------------------------------------------------------------------------
uint8_t setupLocoSessions( ) {

    usbIoPrintf( CDC_USB_CONSOLE, "Setup Loco Session Lib -> " );

    uint8_t  rStat   = LCS_OK;
    uint16_t options = SM_OPT_DEFAULT_SETTING;
   
    rStat = locoSessions -> setupLocoSessions( options, mainTrack, progTrack );

    return ( printStatus( rStat ));
}

//----------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------
uint8_t setupDccTrackMain( ) {

    uint8_t         rStat;
    LcsDccTrackDesc trackDescA;

    trackDescA.options                  = DT_OPT_NIL;      
    trackDescA.initTrackMode            = DT_M_NIL;

    trackDescA.rNumEnable               = 0;
    trackDescA.rNumControl              = 0;
    trackDescA.rNumSense                = 0;
    trackDescA.rNumUartRx               = 0;

    trackDescA.limitCurrentMilliAmp     = 0;
    trackDescA.maxCurrentMilliAmp       = 0;
    trackDescA.overloadEventThreshold   = 0;

    trackDescA.referenceMilliVolt       = 0;
    trackDescA.shuntMilliOhm            = 0;
    trackDescA.amplifierGain            = 0;
    trackDescA.adcBitResolution         = 0;
    trackDescA.adcZeroOffset            = 0;

    mainTrack = LcsDccTrack::createTrackA( &trackDescA );
    mainTrack -> printDccTrackConfig( );

    return ( printStatus( rStat ));
}

//----------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------
uint8_t setupDccTrackProg( ) {

    usbIoPrintf( CDC_USB_CONSOLE, "Setup PROG track -> " );

    uint8_t         rStat;
    LcsDccTrackDesc trackDesc;

    trackDesc.options                   = DT_OPT_NIL;      
    trackDesc.initTrackMode             = DT_M_NIL;

    trackDesc.rNumEnable                = RNUM_ENABLE_PROG;
    trackDesc.rNumControl               = RNUM_CONTROL_PROG;
    trackDesc.rNumSense                 = RNUM_ADC_PROG;
    trackDesc.rNumUartRx                = RNUM_UART_RX_PROG;

    trackDesc.limitCurrentMilliAmp      = 1500;
    trackDesc.maxCurrentMilliAmp        = 2500;
    trackDesc.overloadEventThreshold    = 10;

    trackDesc.referenceMilliVolt        = 3300;
    trackDesc.shuntMilliOhm             = 0;
    trackDesc.amplifierGain             = 0;
    trackDesc.adcBitResolution          = 12;
    trackDesc.adcZeroOffset             = 0;

    progTrack = LcsDccTrack::createTrackB( &trackDesc );
    progTrack -> printDccTrackConfig( );
    return ( printStatus( rStat ));
}

//----------------------------------------------------------------------------------------
// The base station has also a command interpreter, primarily for the DCC++ commands.
//
//----------------------------------------------------------------------------------------
uint8_t setupSerialCommand( ) {

  usbIoPrintf( CDC_USB_CONSOLE, "Setup Serial Command -> " );

  uint8_t rStat = initSerialCommand(  );
  return ( printStatus( rStat ));
}

//----------------------------------------------------------------------------------------
// The LCS message interface is initialized in the LCS core library. This routine 
// will set up the receiver handler for incoming LCS message that concern the base
// station.
//
//----------------------------------------------------------------------------------------
uint8_t setupMsgInterface( ) {

  usbIoPrintf( CDC_USB_CONSOLE, "Setup LCS Msg Interface -> " );

  uint8_t rStat = initLcsMsgInterface( );
  return ( printStatus( rStat ));
}

//----------------------------------------------------------------------------------------
// After the initial setup of the runtime library, the callback are registered.
//
//----------------------------------------------------------------------------------------
uint8_t registerCallbacks( ) {

    usbIoPrintf( CDC_USB_CONSOLE, "Registering Callbacks\n" );

    registerLcsMsgCallback( lcsMsgCallback );
    registerCmdCallback( lcsCmdCallback );
    registerInitCallback( lcsInitCallback );
    registerPfailCallback( lcsPfailCallback );
    registerReqCallback( lcsReqCallback );
    registerEventCallback( lcsEventCallback );
    
    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Fire up the base station. First all base station modules are initialized. If OK,
// the DCC tac signal generation is enabled, i.e. the interrupt driven DCC packet 
// broadcasting starts. Finally, the track power is turned on and we give control 
// to the LCS runtime for processing events and requests.
//
// ??? should we do this rather in the init callback ?
// ??? also, should we have an indicator for power fail startup ?
//
//----------------------------------------------------------------------------------------
uint8_t startBaseStation( ) {

    uint8_t rStat = NO_ERR; 
    
    if ( rStat == LCS_OK ) rStat = setupSerialCommand( );
    if ( rStat == LCS_OK ) rStat = setupMsgInterface( );
    if ( rStat == LCS_OK ) rStat = setupDccTrackMain( );
    if ( rStat == LCS_OK ) rStat = setupDccTrackProg( );
    if ( rStat == LCS_OK ) rStat = setupLocoSessions( );
   
    if ( rStat == NO_ERR ) {

        rStat = LcsDccTrack::startDccProcessing( RNUM_TIMER_0 );

        if ( usbIoIsConnected( )) {

            mainTrack -> printDccTrackStatus( );
            progTrack -> printDccTrackStatus( );
            usbIoPrintf( CDC_USB_CONSOLE, "Ready...\n" );
        }
    
        startRuntime( );
    }
    else {

        // ??? a fatal error ?
        // ??? if we do not start the runtime, we will not be able to 
        // enter commands ?
    }

    return( NO_ERR );
}

//----------------------------------------------------------------------------------------
// The main program. Setup the runtime, register the callbacks, and get the show 
// on the road.
//
//----------------------------------------------------------------------------------------
int main( ) {

    uint8_t rStat = NO_ERR;

    if ( rStat == NO_ERR ) rStat = initRunTimeLib( );
    if ( rStat == NO_ERR ) rStat = registerCallbacks( );
    if ( rStat == NO_ERR ) return( startBaseStation( ));
}