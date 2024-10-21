//------------------------------------------------------------------------------------------------------------
//
// LCS - Base Station
//
//------------------------------------------------------------------------------------------------------------
// This is the main program for the LCS base station. Every layout would need at least a base station. Its
// primary task is to manage the DCC loco sessions, generate the DCC signals and manage the dual DCC track
// outputs.
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
// Base station global objects.
//
//------------------------------------------------------------------------------------------------------------
CDC::CdcPinConfig               cdcConfig;
LCS::LcsConfig                  lcsConfig;
LcsBaseStationCommand           serialCmd;
LcsBaseStationDccTrack          mainTrack;
LcsBaseStationDccTrack          progTrack;
LcsBaseStationLocoSession       locoSessions;
LcsBaseStationMsgInterface      msgInterface;

//----------------------------------------------------------------------------------------------------------
//
//
// Current mapping: Main Controller Board B.01.00 - PICO - newest version.
//----------------------------------------------------------------------------------------------------------
void setupConfigInfo( ) {

    cdcConfig = CDC::getConfigDefault( );
    lcsConfig = LCS::getConfigDefault( );

    cdcConfig.ADC_PIN_0             = 26;
    cdcConfig.ADC_PIN_1             = 27;

    cdcConfig.PFAIL_PIN             = 5;
    cdcConfig.EXT_INT_PIN           = 22;
    cdcConfig.READY_LED_PIN         = 14;
    cdcConfig.ACTIVE_LED_PIN        = 15;

    cdcConfig.DIO_PIN_0             = 9;
    cdcConfig.DIO_PIN_1             = 8;
    cdcConfig.DIO_PIN_2             = 10;
    cdcConfig.DIO_PIN_3             = 11;
    cdcConfig.DIO_PIN_4             = 21;
    cdcConfig.DIO_PIN_5             = 20;
    cdcConfig.DIO_PIN_6             = 19;
    cdcConfig.DIO_PIN_7             = 18;

    cdcConfig.UART_RX_PIN_1         = 13;
    cdcConfig.UART_RX_PIN_2         = 9;

    cdcConfig.NVM_I2C_SCL_PIN       = 17;
    cdcConfig.NVM_I2C_SDA_PIN       = 16;
    cdcConfig.NVM_I2C_ADR_ROOT      = 0x50;

    cdcConfig.EXT_I2C_SCL_PIN       = 3;
    cdcConfig.EXT_I2C_SDA_PIN       = 2;
    cdcConfig.EXT_I2C_ADR_ROOT      = 0x50;

    cdcConfig.CAN_BUS_RX_PIN        = 0;
    cdcConfig.CAN_BUS_TX_PIN        = 1;
    cdcConfig.CAN_BUS_CTRL_MODE     = CAN_BUS_LIB_PICO_PIO_125K_M_CORE;
    cdcConfig.CAN_BUS_DEF_ID        = 100;

    cdcConfig.NODE_NVM_SIZE         = 8192;
    cdcConfig.EXT_NVM_SIZE          = 4096;

    lcsConfig.options               |= NOPT_SKIP_NODE_ID_CONFIG | NOPT_DEBUG_DURING_SETUP;
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
//
//
//
//----------------------------------------------------------------------------------------------------------
uint8_t lcsInitCallback( uint16_t npId ) {

    printf( "Init Callback: 0x%x\n", npId );
    return( ALL_OK );
}

uint8_t lcsResetCallback( uint16_t npId ) {

    printf( "Reset Callback: 0x%x\n", npId );
    return( ALL_OK );
}

uint8_t lcsPfailCallback( uint16_t npId ) {

    printf( "Pfail Callback: 0x%x\n", npId );
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
// Other Callbacks. All we do is to list their invocation. ( for now )
//
//----------------------------------------------------------------------------------------------------------
uint8_t lcsMsgCallback( uint8_t *msg ) {

    printf( "MsgCallback: " );
    for ( int i = 0; i < 8; i++ ) printf( "0x%2x ");
    printf( "\n" );
    return( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// The LCS core library ends in a loop that manages its internal workings, invoking the callbacks where
// needed. This call back is the general loop callback. It is our chance to do our regular work such as
// checking on short circuit, lost sessions, etc. We will not do all the work in one swoop. There is a simple
// round robin scheme so that on each callback one activity is executed.
//
//------------------------------------------------------------------------------------------------------------
uint8_t lcsTaskCallback( ) {

    printf( "Task Callback...\n" );
    return( ALL_OK );    
}

void lcsLoop( ) {

  static uint8_t nextStep = 0;

  switch ( nextStep ) {

    case 0: mainTrack.runDccTrackStateMachine( );  nextStep = 1; break;
    case 1: progTrack.runDccTrackStateMachine( );  nextStep = 2; break;
    case 2: locoSessions.refreshActiveSessions( ); nextStep = 0; break;
    default: nextStep = 0;
  }
}

//------------------------------------------------------------------------------------------------------------
//
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t lcsReqCallback( uint8_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    printf( "REQ callback: npId: 0x%x, item: %d", npId, item );
    if ( arg1 != nullptr ) printf( ", arg1: %d, ", *arg1 ); else printf( ", arg1: null" );
    if ( arg2 != nullptr ) printf( ", arg2: %d, ", *arg2 ); else printf( ", arg2: null" );
    return( ALL_OK );
}

uint8_t lcsRepCallback( uint8_t npId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

    printf( "REP callback: npId: 0x%x, item: %d", npId, item );
    if ( arg1 != nullptr ) printf( ", arg1: %d, ", *arg1 ); else printf( ", arg1: null" );
    if ( arg2 != nullptr ) printf( ", arg2: %d, ", *arg2 ); else printf( ", arg2: null" );
    return( ALL_OK );
}

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
  CDC::printConfigInfo( &cdcConfig );

  uint8_t rStat = LCS::initRuntime( &lcsConfig, &cdcConfig );

  printf( "LCS Base Station\n" );
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

  mainTrackDesc.enablePin                   =cdcConfig.DIO_PIN_6;
  mainTrackDesc.dccSigPin1                  =cdcConfig.DIO_PIN_2;
  mainTrackDesc.dccSigPin2                  =cdcConfig.DIO_PIN_3;
  mainTrackDesc.sensePin                    =cdcConfig.ADC_PIN_0;
  mainTrackDesc.uartRxPin                   =cdcConfig.UART_RX_PIN_1;

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

  progTrackDesc.enablePin                   =cdcConfig.DIO_PIN_7;
  progTrackDesc.dccSigPin1                  =cdcConfig.DIO_PIN_4;
  progTrackDesc.dccSigPin2                  =cdcConfig.DIO_PIN_5;
  progTrackDesc.sensePin                    =cdcConfig.ADC_PIN_1;
  progTrackDesc.uartRxPin                   =cdcConfig.UART_RX_PIN_2;

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
// 
//
//----------------------------------------------------------------------------------------------------------
uint8_t registerCallbacks( ) {

    printf( "Registering Callbacks\n" );

    registerLcsMsgCallback( lcsMsgCallback );
    registerCmdCallback( lcsCmdCallback );
    registerTaskCallback( lcsTaskCallback, 1000 );
    registerInitCallback( lcsInitCallback );
    registerResetCallback( lcsResetCallback );
    registerPfailCallback( lcsPfailCallback );
    registerReqCallback( lcsReqCallback );
    registerRepCallback( lcsRepCallback );
    registerEventCallback( lcsEventCallback );
    return( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
uint8_t startLcsRuntime( ) {

   uint8_t rStat; 
    
  if ( rStat == ALL_OK ) rStat = setupSerialCommand( );
  if ( rStat == ALL_OK ) rStat = setupMsgInterface( );
  if ( rStat == ALL_OK ) rStat = setupLocoSessions( );
  if ( rStat == ALL_OK ) rStat = setupDccTrackMain( );
  if ( rStat == ALL_OK ) rStat = setupDccTrackProg( );

  if ( rStat == ALL_OK ) {

    LcsBaseStationDccTrack::startDccProcessing( );

    mainTrack.powerStart( );
    progTrack.powerStart( );
    mainTrack.printDccTrackStatus( );
    progTrack.printDccTrackStatus( );

    printf( "Ready...\n" );
    startRuntime( );
  }

  return( LCS::ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
int main( ) {

    initLcsRuntime( );
    registerCallbacks( );
    startLcsRuntime( );
    return( 0 );
}