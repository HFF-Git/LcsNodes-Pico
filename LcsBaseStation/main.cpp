//------------------------------------------------------------------------------------------------------------
//
// LCS - Base Station
//
//------------------------------------------------------------------------------------------------------------
// This is the main program for the LCS base station. Every layout would need at least a base station. Its
// primary task is to manage the DCC loco sessions, generate the DCC signals and manage the dual DCC track
// power outputs.
//
// Like all other LcsNodes, the base station will provide a rich set of variable that can be set and queried.
// In addition, the base features a command line extension which implements the DCC++ style commands and
// some more base station specific commands. The idea for the DCC++ command syntax and commands is that these
// command can also be submitted by a third party software ( e.g. JMRI ). An example would be the JMRI CV
// programming tool.
//
// ??? we need an idea of system time like DCC. To be broadcasted periodically.
// ??? we also need a broadcast of the layout system capabilities....
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Raspberry PI Pico Implementation
// Copyright (C) 2022 - 2024 Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under the terms of the GNU General
// Public License as published by the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along with this program. If not, see
// http://www.gnu.org/licenses
//
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//------------------------------------------------------------------------------------------------------------
#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"
#include "LcsBaseStation.h"

using namespace LCS;

//------------------------------------------------------------------------------------------------------------
// Base station global data.
//
// ??? can the objects for track and session just use these variables instead of keeping them locally as a 
// field ?
//------------------------------------------------------------------------------------------------------------
uint16_t                        debugMask;
CDC::CdcConfigDesc              cdcConfig;
LCS::LcsConfigDesc              lcsConfig;
LcsBaseStationCommand           serialCmd;
LcsBaseStationDccTrack          mainTrack;
LcsBaseStationDccTrack          progTrack;
LcsBaseStationLocoSession       locoSessions;
LcsBaseStationMsgInterface      msgInterface;

//----------------------------------------------------------------------------------------------------------
// Setup the configuration of the HW board. The CDC config contains the HW pin mapping. The dual bridge pins
// for enabling the bridge and controlling its direction. The pins are mapped to the CDC pin names DIO2 to 
// DIO7 as show below. DIO-0 and DIO-1 are routed to the extension connector board.
//
//      cdcConfig.DIO_PIN_0     -> DIO-0
//      cdcConfig.DIO_PIN_1     -> DIO-1
//      cdcConfig.DIO_PIN_2     -> Main dcc1  
//      cdcConfig.DIO_PIN_3     -> Main dcc2     
//      cdcConfig.DIO_PIN_4     -> Prog ddc1    
//      cdcConfig.DIO_PIN_5     -> Prog ddc2     
//      cdcConfig.DIO_PIN_6     -> Main enable 
//      cdcConfig.DIO_PIN_7     -> Prog enable 
//
// Current mapping: Main Controller Board B.01.00 - PICO - newest version.
//
//      cdcConfig.DIO_PIN_0     = 8;
//      cdcConfig.DIO_PIN_1     = 12;
//      cdcConfig.DIO_PIN_2     = 21; 
//      cdcConfig.DIO_PIN_3     = 20;       
//      cdcConfig.DIO_PIN_4     = 19;  
//      cdcConfig.DIO_PIN_5     = 18;      
//      cdcConfig.DIO_PIN_6     = 6;     
//      cdcConfig.DIO_PIN_7     = 7;     
//
// In addition, the HW pins for I2C, analog inputs and so on are set. Check the schematic for the board 
// to see all pin assign,ents.
//
// ??? one day we will have several base station versions. Although they will perhaps differ, their the CDC
// pin names used should not change. But we would need to come up with an idea which configuration to use
// when preparing an image for the base station board.
//----------------------------------------------------------------------------------------------------------
void setupConfigInfo( ) {

    cdcConfig = CDC::getConfigDefault( );
    lcsConfig = LCS::getConfigDefault( );

    cdcConfig.ADC_PIN_0             = 26;
    cdcConfig.ADC_PIN_1             = 27;

    cdcConfig.PFAIL_PIN             = 5;
    cdcConfig.EXT_INT_PIN           = 22;
    cdcConfig.ACTIVE_LED_PIN        = 14;

    cdcConfig.DIO_PIN_0             = 8;  
    cdcConfig.DIO_PIN_1             = 12; 
    cdcConfig.DIO_PIN_2             = 21;   
    cdcConfig.DIO_PIN_3             = 20;    
    cdcConfig.DIO_PIN_4             = 19;   
    cdcConfig.DIO_PIN_5             = 18;        
    cdcConfig.DIO_PIN_6             = 6;     
    cdcConfig.DIO_PIN_7             = 7;    

    cdcConfig.UART_RX_PIN_1         = 13;
    cdcConfig.UART_RX_PIN_2         = 9;

    cdcConfig.NVM_I2C_SCL_PIN       = 3;
    cdcConfig.NVM_I2C_SDA_PIN       = 2;
    cdcConfig.NVM_I2C_ADR_ROOT      = 0x50;

    cdcConfig.EXT_I2C_SCL_PIN       = 17;
    cdcConfig.EXT_I2C_SDA_PIN       = 16;
    cdcConfig.EXT_I2C_ADR_ROOT      = 0x50;

    cdcConfig.CAN_BUS_RX_PIN        = 0;
    cdcConfig.CAN_BUS_TX_PIN        = 1;
    cdcConfig.CAN_BUS_CTRL_MODE     = CAN_BUS_LIB_PICO_PIO_125K_M_CORE;
    cdcConfig.CAN_BUS_DEF_ID        = 100;

    cdcConfig.NODE_NVM_SIZE         = 8192;
    cdcConfig.EXT_NVM_SIZE          = 4096;

    lcsConfig.options               |= NPO_SKIP_NODE_ID_CONFIG;
}

//------------------------------------------------------------------------------------------------------------
// Some little helper functions.
//
//------------------------------------------------------------------------------------------------------------
void printLcsMsg( uint8_t *msg ) {

  int msgLen = (( msg[0] >> 5 ) + 1 ) % 8;

  for ( int i = 0; i < msgLen; i++ ) printf( "0x%x ", msg[i] );
  printf( "\n" );
}

uint8_t printStatus (uint8_t status ) {

  printf( "Status: " );
  if ( status == LCS::ALL_OK ) printf( "OK\n" );
  else printf ( "FAILED: %d\n", status );
  return ( status );
}

//----------------------------------------------------------------------------------------------------------
// The node and port initialization callback.
//
// ??? when we know what ports we actually need / use, disable the rest of the ports.
//----------------------------------------------------------------------------------------------------------
uint8_t lcsInitCallback( uint16_t npId ) {

    switch ( npId & 0xF ) {

        case 0:     printf( "Node Init Callback: 0x%x\n", npId >> 4     ); break;
        default:    printf( "Port Init Callback: 0x%x\n", npId &  0xF   );
    } 

    return( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// The node or port power fail callback.
//
//----------------------------------------------------------------------------------------------------------
uint8_t lcsPfailCallback( uint16_t npId ) {

    switch ( npId & 0xF ) {

        case 0:     printf( "Node Power Fail Callback: 0x%x\n", npId >> 4     ); break;
        default:    printf( "Port Power Fail Callback: 0x%x\n", npId &  0xF   );
    } 

    return( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// The base station has also a command line interpreter. The callback is invoked by the core library when
// there is a command that it does not handle.
//
//----------------------------------------------------------------------------------------------------------
uint8_t lcsCmdCallback( char *cmdLine ) {

    serialCmd.handleSerialCommand( cmdLine );
    return( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// Other LCS message callbacks. All we do is to list their invocation. ( for now )
//
//----------------------------------------------------------------------------------------------------------
uint8_t lcsMsgCallback( uint8_t *msg ) {

    printf( "MsgCallback: ", msg  );

    for ( int i = 0; i < 8; i++ ) printf( "0x%2x ");
    printf( "\n" );
    return( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// The LCS core library ends in a loop that manages its internal workings, invoking the callbacks where
// needed. One set of callbacks are the periodic tasks. The base station needs to periodically run the DCC
// track state machine for power consumption measurement and so on.  Another periodic task is to refresh the 
// active locomotive session entries.
//
//------------------------------------------------------------------------------------------------------------
uint8_t bsMainTrackCallback( ) {

    mainTrack.runDccTrackStateMachine( );
    return( ALL_OK );
}

uint8_t bsProgTrackCallback( ) {

    progTrack.runDccTrackStateMachine( );
    return( ALL_OK );
}

uint8_t bsRefreshActiveSessionCallback( ) {

    locoSessions.refreshActiveSessions( );
    return( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// When the base station node receives a request with an item defined in the user item range or the base
// station itself issues such a request, the defined callback is invoked.
//
//------------------------------------------------------------------------------------------------------------
uint8_t lcsReqCallback( uint16_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    printf( "REQ callback: npId: 0x%x, item: %d", npId, item );
    if ( arg1 != nullptr ) printf( ", arg1: %d, ", *arg1 ); else printf( ", arg1: null" );
    if ( arg2 != nullptr ) printf( ", arg2: %d, ", *arg2 ); else printf( ", arg2: null" );
    return( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// When the base station gets a reply message for a request previously sent, this callback is invoked.
//
//------------------------------------------------------------------------------------------------------------
uint8_t lcsRepCallback( uint16_t npId, uint8_t item, uint16_t arg1, uint16_t arg2, uint8_t ret ) {

    printf( "REP callback: npId: 0x%x, item: %d, arg1: %d, arg2: %d, ret: %d ", npId, item , arg1, arg2, ret );
    return( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// For any event on the LCS system that the base station is interested in, this callback is invoked.
//
//------------------------------------------------------------------------------------------------------------
uint8_t lcsEventCallback( uint16_t npId, uint16_t eId, uint8_t eAction, uint16_t eData ) {

    printf( "Event: npId: 0x%x, eId: %d, eAction: %d, eData: %d\n", npId, eId, eAction, eData );
    return( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// Init the Runtime.
//
//----------------------------------------------------------------------------------------------------------
uint8_t initLcsRuntime( ) {

    setupConfigInfo( );
  
    uint8_t rStat = LCS::initRuntime( &lcsConfig, &cdcConfig );
    printf( "LCS Base Station\n" );
    
    CDC::printConfigInfo( &cdcConfig );
    
    printStatus( rStat );
    return( rStat );
}

//------------------------------------------------------------------------------------------------------------
// This routine initializes the Loco Session Map Object.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupLocoSessions( ) {

  LcsBaseStationSessionMapDesc sessionDesc;

  sessionDesc.options     = SM_OPT_ENABLE_REFRESH;
  sessionDesc.maxSessions = 16;

  printf( "Setup Session Map -> " );
  return ( printStatus( locoSessions.setupSessionMap( &sessionDesc, &mainTrack, &progTrack )));
}

//------------------------------------------------------------------------------------------------------------
// This routine initializes the MAIN track object.
//
// ??? define constants such as: SENSE_0R1_OPAMP_11 to set the milliVolts per Amp.
//------------------------------------------------------------------------------------------------------------
int setupDccTrackMain( ) {

  LcsBaseStationTrackDesc mainTrackDesc;

  mainTrackDesc.options                     = DT_OPT_RAILCOM | DT_OPT_CUTOUT;

  mainTrackDesc.enablePin                   = cdcConfig.DIO_PIN_6;
  mainTrackDesc.dccSigPin1                  = cdcConfig.DIO_PIN_2;
  mainTrackDesc.dccSigPin2                  = cdcConfig.DIO_PIN_3;
  mainTrackDesc.sensePin                    = cdcConfig.ADC_PIN_0;
  mainTrackDesc.uartRxPin                   = cdcConfig.UART_RX_PIN_1;

  mainTrackDesc.initCurrentMilliAmp         = 500;
  mainTrackDesc.limitCurrentMilliAmp        = 1500;
  mainTrackDesc.maxCurrentMilliAmp          = 2000;
  mainTrackDesc.milliVoltPerAmp             = 100 * 11;  // ??? opAmp has Factor eleven ...
  mainTrackDesc.startTimeThresholdMillis    = 1000;
  mainTrackDesc.stopTimeThresholdMillis     = 500;
  mainTrackDesc.overloadTimeThresholdMillis = 500;
  mainTrackDesc.overloadEventThreshold      = 10;
  mainTrackDesc.overloadRestartThreshold    = 5;

  printf( "Setup MAIN track -> " );
  return ( printStatus( mainTrack.setupDccTrack( &mainTrackDesc )));
}

//------------------------------------------------------------------------------------------------------------
// This routine initializes the PROG track object.
//
// ??? define constants such as: SENSE_0R1_OPAMP_11 to set the milliVolts per Amp.
//------------------------------------------------------------------------------------------------------------
uint8_t setupDccTrackProg( ) {

  LcsBaseStationTrackDesc progTrackDesc;

  progTrackDesc.options                     = DT_OPT_SERVICE_MODE_TRACK;

  progTrackDesc.enablePin                   = cdcConfig.DIO_PIN_7;
  progTrackDesc.dccSigPin1                  = cdcConfig.DIO_PIN_4;
  progTrackDesc.dccSigPin2                  = cdcConfig.DIO_PIN_5;
  progTrackDesc.sensePin                    = cdcConfig.ADC_PIN_1;
  progTrackDesc.uartRxPin                   = cdcConfig.UART_RX_PIN_2;

  progTrackDesc.initCurrentMilliAmp         = 500;
  progTrackDesc.limitCurrentMilliAmp        = 500;
  progTrackDesc.maxCurrentMilliAmp          = 1000;
  progTrackDesc.milliVoltPerAmp             = 100 * 11;  // ??? opAmp has Factor eleven ...
  progTrackDesc.startTimeThresholdMillis    = 1000;
  progTrackDesc.stopTimeThresholdMillis     = 500;
  progTrackDesc.overloadTimeThresholdMillis = 500;
  progTrackDesc.overloadEventThreshold      = 10;
  progTrackDesc.overloadRestartThreshold    = 5;

  printf( "Setup PROG track -> " );
  return ( printStatus( progTrack.setupDccTrack( &progTrackDesc )));
}

//------------------------------------------------------------------------------------------------------------
// The base station has also a command interpreter, primarily for the DCC++ commands.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupSerialCommand( ) {

  printf( "Setup Serial Command -> " );
  return ( printStatus( serialCmd.setupSerialCommand( &locoSessions, &mainTrack, &progTrack )));
}

//------------------------------------------------------------------------------------------------------------
// The LCS message interface is initialized in the LCS core library. This routine will set up the receiver
// handler for incoming LCS message that concern the base station.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupMsgInterface( ) {

  printf( "Setup LCS Msg Interface -> " );
  return ( printStatus( msgInterface.setupLcsMsgInterface( &locoSessions, &mainTrack, &progTrack )));
}

//----------------------------------------------------------------------------------------------------------
// After the initial setup of the runtime library, the callback are registered.
//
//----------------------------------------------------------------------------------------------------------
uint8_t registerCallbacks( ) {

    printf( "Registering Callbacks\n" );

    registerLcsMsgCallback( lcsMsgCallback );
    registerCmdCallback( lcsCmdCallback );
    registerInitCallback( lcsInitCallback );
    registerPfailCallback( lcsPfailCallback );
   
   
   // registerReqCallback( lcsReqCallback );
    
    
    registerRepCallback( lcsRepCallback );
    registerEventCallback( lcsEventCallback );
    registerTaskCallback( bsMainTrackCallback, MAIN_TRACK_STATE_TIME_INTERVAL );
    registerTaskCallback( bsProgTrackCallback, PROG_TRACK_STATE_TIME_INTERVAL );
    registerTaskCallback( bsRefreshActiveSessionCallback, SESSION_REFRESH_TASK_INTERVAL );

    return( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// Fire up the base station. First all base station modules are initialized. If this is OK, the DCC tack
// signal generation is enabled, i.e. the interrupt driven DCC packet broadcasting starts. Finally, the 
// track power is turned on and we give control to the LCS runtime for processing events and requests.
//
//----------------------------------------------------------------------------------------------------------
uint8_t startBaseStation( ) {

    uint8_t rStat = ALL_OK; 
    
    if ( rStat == ALL_OK ) rStat = setupSerialCommand( );
    if ( rStat == ALL_OK ) rStat = setupMsgInterface( );
    if ( rStat == ALL_OK ) rStat = setupLocoSessions( );
    if ( rStat == ALL_OK ) rStat = setupDccTrackMain( );
    if ( rStat == ALL_OK ) rStat = setupDccTrackProg( );

    if ( rStat == ALL_OK ) {

        LcsBaseStationDccTrack::startDccProcessing( );

        mainTrack.powerStart( );
        progTrack.powerStart( );

        // ??? bracket so that it is not printed when no console...
        mainTrack.printDccTrackStatus( );
        progTrack.printDccTrackStatus( );
        printf( "Ready...\n" );
    
        startRuntime( );
  }

  return( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// The main program. Setup the runtime, register the callbacks, and get the show on the road.
//
//----------------------------------------------------------------------------------------------------------
int main( ) {

    uint8_t rStat = ALL_OK;

    if ( rStat == ALL_OK ) rStat = initLcsRuntime( );
    if ( rStat == ALL_OK ) rStat = registerCallbacks( );
    if ( rStat == ALL_OK ) return( startBaseStation( ));
}