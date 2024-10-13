//------------------------------------------------------------------------------------------------------------
//
// LCS - Base Station
//
//------------------------------------------------------------------------------------------------------------
// This source file contains ...
//
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

//----------------------------------------------------------------------------------------------------------
// Setup the config data. We first get the defaults for the controller and then set the board specific pin
// numbers and values.
//
//----------------------------------------------------------------------------------------------------------
CDC::CdcPinConfig cfg;


//----------------------------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------------------------
void setupConfigInfo( ) {

  cfg = CDC::getConfigDefault( );

  //--------------------------------------------------------------------------------------------------------
  // Current mapping: Main Controller Board B.01.00 - PICO - newest version.
  //
  //--------------------------------------------------------------------------------------------------------
  cfg.ADC_PIN_0             = 26;
  cfg.ADC_PIN_1             = 27;

  cfg.PWM_PIN_0             = 20;
  cfg.PWM_PIN_1             = 21;

  cfg.PFAIL_PIN             = 7;
  cfg.EXT_INT_PIN           = 22;
  cfg.READY_LED_PIN         = 14;
  cfg.ACTIVE_LED_PIN        = 15;

  cfg.DIO_PIN_0             = 9;
  cfg.DIO_PIN_1             = 8;
  cfg.DIO_PIN_2             = 10;
  cfg.DIO_PIN_3             = 11;
  cfg.DIO_PIN_4             = 21;
  cfg.DIO_PIN_5             = 20;
  cfg.DIO_PIN_6             = 19;
  cfg.DIO_PIN_7             = 18;

  cfg.NVM_I2C_SCL_PIN       = 17;
  cfg.NVM_I2C_SDA_PIN       = 16;
  cfg.NVM_I2C_ADR_ROOT      = 0x50;

  cfg.EXT_I2C_SCL_PIN       = 3;
  cfg. EXT_I2C_SDA_PIN      = 2;
  cfg.EXT_I2C_ADR_ROOT      = 0x50;

  CDC::printConfigInfo( &cfg );
}

//----------------------------------------------------------------------------------------------------------
// Init the CDC and Runtime library...
//
//----------------------------------------------------------------------------------------------------------
uint8_t initLcsRuntime( ) {

  setupConfigInfo( );
 
  uint8_t rStat = LCS::initRuntime( &cfg, LCS::NOPT_DEBUG_DURING_SETUP );

  printf( "LCS Base Station\n" );

  
  if ( rStat != 0 )  printf( "Err code: %d\n", rStat );
  else printf( "OK\n" );

  return( 0 );
}

//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
uint8_t registerCallbacks( ) {


  return( LCS::ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------------------------
uint8_t startLcsRuntime( ) {


  return( LCS::ALL_OK );
}



// ??? we need an idea of system time like DCC. To be broadcasted periodically.




// ??? we also need a broadcast of the layout system capabilities....



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


#if 0

// Code to be reworked....

//------------------------------------------------------------------------------------------------------------
//
// LCS - DCC Base Station
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
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Base Station
// Copyright (C) 2019 - 2023  Helmut Fieres
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
#include "LcsBaseStation.h"

//------------------------------------------------------------------------------------------------------------
// We need to have a second UART port for RailCom.
//------------------------------------------------------------------------------------------------------------
#if not defined( HAVE_HWSERIAL1 )
#error CANNOT COMPILE - NEED A SERIAL PORT ONE
#endif

//----------------------------------------------------------------------------------------------------------
// Setup the config data. We first get the defaults for the controller and then set the board specific pin
// numbers and values.
//
//----------------------------------------------------------------------------------------------------------
CDC::CdcConfigInfo cfg;

void setupConfigInfo( ) {

  cfg = CDC::getConfigDefault( );

  #if defined  ( __AVR_ATmega1284P__ )

  cfg.PFAIL_PIN           = 4;
  cfg.DIO_PIN_0           = 10;
  cfg.DIO_PIN_1           = 11;
  cfg.DIO_PIN_2           = 23;
  cfg.DIO_PIN_3           = 22;
  cfg.DIO_PIN_4           = 21;
  cfg.DIO_PIN_5           = 20;
  cfg.DIO_PIN_6           = 14;
  cfg.DIO_PIN_7           = 15;
  cfg.ADC_PIN_0           = A1; // 25 - reversed on board A.02.03
  cfg.ADC_PIN_1           = A0; // 24 - reversed on board A.02.03
  cfg.READY_LED_PIN       = 19;
  cfg.ACTIVE_LED_PIN      = 18;
  cfg.BUTTON_PIN          = 31;
  cfg.PWM_PIN_0           = cfg.DIO_PIN_6;
  cfg.PWM_PIN_1           = cfg.DIO_PIN_7;
  cfg.UART_RX_PIN_1       = 10;

  cfg.CAN_BUS_SELECT_PIN  = 0;
  cfg.CAN_BUS_RX_PIN      = 0; // ??? currently doubles up as select pin ... until CAN bus lib is changed...
  cfg.CAN_BUS_TX_PIN      = CDC::UNDEFINED_PIN;
  cfg.CAN_BUS_CTRL_MODE   = CAN_BUS_LIB_MCP2515_500K_16MHZ;

  cfg.NVM_SELECT_PIN      = 1;
  cfg.NVM_CHIP_TYPE       = M25LC256;

  #elif defined ( ARDUINO_ARCH_RP2040 )

  #if 0   // mai controller board - old
  cfg.ADC_PIN_0           = 26;
  cfg.ADC_PIN_1           = 27;
  cfg.SPI_SCLK_PIN        = 2;
  cfg.SPI_MOSI_PIN        = 3;
  cfg.SPI_MISO_PIN        = 4;
  cfg.I2C_SCL_PIN_0       = 17;
  cfg.I2C_SDA_PIN_0       = 16;
  cfg.PFAIL_PIN           = 7;
  cfg.BUTTON_PIN          = 12;
  cfg.EXT_INT_PIN         = 22;
  cfg.READY_LED_PIN       = 14;
  cfg.ACTIVE_LED_PIN      = 15;
  cfg.DIO_PIN_0           = 8;
  cfg.DIO_PIN_1           = 9;
  cfg.DIO_PIN_2           = 10;
  cfg.DIO_PIN_3           = 11;
  cfg.DIO_PIN_4           = 21;
  cfg.DIO_PIN_5           = 20;
  cfg.DIO_PIN_6           = 19;
  cfg.DIO_PIN_7           = 18;

  cfg.UART_RX_PIN_1       = 13;

  cfg.NVM_SELECT_PIN      = 5;
  cfg.NVM_CHIP_TYPE       = M25LC128;

  #else

  // Base station board settings: B.01.00

  cfg.ADC_PIN_0           = 26;
  cfg.ADC_PIN_1           = 27;

  cfg.I2C_SCL_PIN_0       = 17;
  cfg.I2C_SDA_PIN_0       = 16;

  cfg.I2C_SCL_PIN_1       = 3;
  cfg.I2C_SDA_PIN_1       = 2;
  cfg.I2C_ADR_1           = 0x50;
  
  cfg.PFAIL_PIN           = 5;
  cfg.EXT_INT_PIN         = 22;
  cfg.READY_LED_PIN       = 14;
  cfg.ACTIVE_LED_PIN      = 15;
 
  cfg.DIO_PIN_2           = 21;
  cfg.DIO_PIN_3           = 20;
  cfg.DIO_PIN_4           = 19;
  cfg.DIO_PIN_5           = 18;
  cfg.DIO_PIN_6           = 6;
  cfg.DIO_PIN_7           = 7;

  cfg.UART_RX_PIN_1       = 13;
  cfg.UART_RX_PIN_2       = 9;

  cfg.NVM_CHIP_TYPE       = M24LC128;

  #endif

  cfg.CAN_BUS_SELECT_PIN  = CDC::UNDEFINED_PIN;
  cfg.CAN_BUS_RX_PIN      = 0;
  cfg.CAN_BUS_TX_PIN      = 1;
  cfg.CAN_BUS_CTRL_MODE   = CAN_BUS_LIB_PICO_PIO_500K_M_CORE;


  #else
#error CANNOT COMPILE - specify the Atmega1284 or PICO LCS main controller board pins
  #endif
}

//------------------------------------------------------------------------------------------------------------
// Base station global objects, we just create them. The setup will be done once all are in place.
//
//------------------------------------------------------------------------------------------------------------

LcsBaseStationCommand         serialCmd;
LcsBaseStationDccTrack        mainTrack;
LcsBaseStationDccTrack        progTrack;
LcsBaseStationLocoSession     locoSessions;
LcsBaseStationMsgInterface    msgInterface;

//------------------------------------------------------------------------------------------------------------
// Some little helper functions.
//
//------------------------------------------------------------------------------------------------------------
void printLcsMsg( uint8_t *msg ) {

  int msgLen = (( msg[0] >> 5 ) + 1 ) % 8;

  for ( int i = 0; i < msgLen; i++ ) {

    INTERFACE.print( msg[i], HEX );
    INTERFACE.print( F(" "));
  }

  INTERFACE.println( );
}

uint8_t printStatus (uint8_t status ) {

  INTERFACE.print( F("Status: "));
  if ( status == ALL_OK ) INTERFACE.println( F("OK"));
  else {

    INTERFACE.print ( F("FAILED: "));
    INTERFACE.println( status );
  }
  return ( status );
}

//----------------------------------------------------------------------------------------------------------
// CDC setup.
//
//----------------------------------------------------------------------------------------------------------
uint8_t setupCdcLib( ) {

  setupConfigInfo( );

  uint8_t  rStat = CDC::init( &cfg );

  INTERFACE.print( "CDC Library init: " );
  INTERFACE.println( rStat );

  return ( printStatus( rStat ));
}

//------------------------------------------------------------------------------------------------------------
// The very first thing to do is to set up the LCS core library. This is done by building the configuration
// descriptor and pass it to the "init" routine of the library. Note, there will only be one object of this
// class.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupLcsLib( ) {

  INTERFACE.println( F("Set up LcsCoreLib"));

  LcsCoreLibConfigDesc lcsDesc;

  lcsDesc.nodeId              = 1;
  lcsDesc.options             = NOPT_SKIP_NODE_ID_CONFIG | NOPT_USE_EXT_NVM;
  lcsDesc.numOfPorts          = 4;
  lcsDesc.numOfEvents         = 32;
  lcsDesc.numOfAttrs          = 128;
  lcsDesc.numOfPeriodicTasks  = 16;

  uint8_t rStat = LcsCoreLib::init( &cfg, &lcsDesc, &lcsLib );

  if ( rStat == ALL_OK ) {

    lcsLib -> registerLcsMsgCallback( busMgtCallback );
    lcsLib -> registerDccMsgCallback( dccMsgCallback );
    lcsLib -> registerCommandCallback( cmdLineCallback );
    lcsLib -> registerPeriodicTask( lcsLoop );

    lcsLib ->registerPortEventCallback( portEventCallback );
    lcsLib-> registerInitCallback( 0, initCallback );
    lcsLib-> registerInfoCallback( 0, infoItemCallback);
    lcsLib-> registerCtrlCallback( 0, ctrlItemCallback);
    lcsLib-> registerReqRepCallback( itemReqCallback );
  }

  INTERFACE.print( F("Set up LcsCoreLib -> "));
  return ( printStatus( rStat ));
}

//------------------------------------------------------------------------------------------------------------
// This routine initializes the Loco Session Map Object.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupLocoSessions( ) {

  LcsBaseStationSessionMapDesc sessionDesc;

  sessionDesc.options     = SM_OPT_ENABLE_REFRESH;
  sessionDesc.maxSessions = 16;

  INTERFACE.print( F( "Setup Session Map -> "));
  return ( printStatus( locoSessions.setupSessionMap( &sessionDesc, lcsLib, &mainTrack, &progTrack )));
}

//------------------------------------------------------------------------------------------------------------
// This routine initializes the MAIN track object.
//
// ??? define constants such as: SENSE_0R1_OPAMP_11 to set the milliVolts per Amp.
//------------------------------------------------------------------------------------------------------------
int setupDccTrackMain( ) {

  LcsBaseStationTrackDesc mainTrackDesc;

  mainTrackDesc.options                     = DT_OPT_RAILCOM | DT_OPT_CUTOUT;

  mainTrackDesc.enablePin                   = cfg.DIO_PIN_6;
  mainTrackDesc.dccSigPin1                  = cfg.DIO_PIN_2;
  mainTrackDesc.dccSigPin2                  = cfg.DIO_PIN_3;
  mainTrackDesc.sensePin                    = cfg.ADC_PIN_0;
  mainTrackDesc.uartRxPin                   = cfg.UART_RX_PIN_1;

  mainTrackDesc.initCurrentMilliAmp         = 500;
  mainTrackDesc.limitCurrentMilliAmp        = 1500;
  mainTrackDesc.maxCurrentMilliAmp          = 2000;
  mainTrackDesc.milliVoltPerAmp             = 100 * 11;  // ??? opAmp has Factor eleven ...
  mainTrackDesc.startTimeThresholdMillis    = 1000;
  mainTrackDesc.stopTimeThresholdMillis     = 500;
  mainTrackDesc.overloadTimeThresholdMillis = 500;
  mainTrackDesc.overloadEventThreshold      = 10;
  mainTrackDesc.overloadRestartThreshold    = 5;

  INTERFACE.print( F("Setup MAIN track -> "));
  return ( printStatus( mainTrack.setupDccTrack( &mainTrackDesc, lcsLib )));
}

//------------------------------------------------------------------------------------------------------------
// This routine initializes the PROG track object.
//
// ??? define constants such as: SENSE_0R1_OPAMP_11 to set the milliVolts per Amp.
//------------------------------------------------------------------------------------------------------------
uint8_t setupDccTrackProg( ) {

  LcsBaseStationTrackDesc progTrackDesc;

  progTrackDesc.options                     = DT_OPT_SERVICE_MODE_TRACK;

  progTrackDesc.enablePin                   = cfg.DIO_PIN_7;
  progTrackDesc.dccSigPin1                  = cfg.DIO_PIN_4;
  progTrackDesc.dccSigPin2                  = cfg.DIO_PIN_5;
  progTrackDesc.sensePin                    = cfg.ADC_PIN_1;
  progTrackDesc.uartRxPin                   = cfg.UART_RX_PIN_2;

  progTrackDesc.initCurrentMilliAmp         = 500;
  progTrackDesc.limitCurrentMilliAmp        = 500;
  progTrackDesc.maxCurrentMilliAmp          = 1000;
  progTrackDesc.milliVoltPerAmp             = 100 * 11;  // ??? opAmp has Factor eleven ...
  progTrackDesc.startTimeThresholdMillis    = 1000;
  progTrackDesc.stopTimeThresholdMillis     = 500;
  progTrackDesc.overloadTimeThresholdMillis = 500;
  progTrackDesc.overloadEventThreshold      = 10;
  progTrackDesc.overloadRestartThreshold    = 5;

  INTERFACE.print( F("Setup PROG track -> "));
  return ( printStatus( progTrack.setupDccTrack( &progTrackDesc, lcsLib )));
}

//------------------------------------------------------------------------------------------------------------
// The base station has also a command interpreter, primarily for the DCC++ commands.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupSerialCommand( ) {

  INTERFACE.print( F("Setup Serial Command -> "));
  return ( printStatus( serialCmd.setupSerialCommand( lcsLib, &locoSessions, &mainTrack, &progTrack )));
}

//------------------------------------------------------------------------------------------------------------
// The LCS message interface is initialized in the LCS core library. This routine will set up the receiver
// handler for incoming LCS message that concern the base station.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupMsgInterface( ) {

  INTERFACE.print( F("Setup LCS Msg Interface -> "));
  return ( printStatus( msgInterface.setupLcsMsgInterface( lcsLib, &locoSessions, &mainTrack, &progTrack )));
}


//----------------------------------------------------------------------------------------------------------
// Callback functions, place holders for testing.
//
//----------------------------------------------------------------------------------------------------------
uint8_t initCallback (uint16_t nodeId, uint8_t portId, uint16_t flags ) {

  INTERFACE.print( F( "initCallback -> node: " ));
  INTERFACE.print( nodeId );
  INTERFACE.print( F( ", port: " ));
  INTERFACE.print( portId );
  INTERFACE.print( F( ", flags: " ));
  INTERFACE.print( flags );
  INTERFACE.println( );
  return ( ALL_OK );
}

uint8_t infoItemCallback( uint8_t portId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

  INTERFACE.print( F( "infoItemCallback -> port: " ));
  INTERFACE.print( portId );
  INTERFACE.print( F( ", item: " ));
  INTERFACE.print( item );
  INTERFACE.print( F( ":" ));
  if ( arg1 != nullptr ) INTERFACE.print( *arg1 ); else INTERFACE.print( F( "null" ));
  INTERFACE.print( F( ":" ));
  if ( arg2 != nullptr ) INTERFACE.print( *arg2 ); else INTERFACE.print( F( "null" ));
  INTERFACE.println( );
  return ( ALL_OK );
}

uint8_t ctrlItemCallback( uint8_t portId, uint8_t item, uint16_t arg1, uint16_t arg2 ) {

  INTERFACE.print (F( "nodeCtrlItemCallback -> port: " ));
  INTERFACE.print( portId );
  INTERFACE.print( F( ", item: " ));
  INTERFACE.print( item );
  INTERFACE.print( F( ":" ));
  INTERFACE.print( arg1 );
  INTERFACE.print(F(":"));
  INTERFACE.print( arg2 );
  INTERFACE.println( );
  return ( ALL_OK );
}

void itemReqCallback( uint16_t nodeId, uint8_t portId, uint8_t item, uint16_t arg1, uint16_t arg2 ) {

  INTERFACE.print(F( "nodeCtrlItemCallback -> node: " ));
  INTERFACE.print( nodeId );
  INTERFACE.print(F(", port: "));
  INTERFACE.print( portId );
  INTERFACE.print(F(", item: "));
  INTERFACE.print(item);
  INTERFACE.print(F( ":" ));
  INTERFACE.print( arg1 );
  INTERFACE.print(F( ":" ));
  INTERFACE.print( arg2 );
  INTERFACE.println( );
}

void portEventCallback( uint16_t nodeId, uint8_t portId, uint8_t eAction, uint16_t eId, uint16_t eData ) {

  INTERFACE.print( F( "portEventCallback -> " ));
  INTERFACE.print( nodeId );
  INTERFACE.print( ":" );
  INTERFACE.print( portId );
  INTERFACE.print( ":" );
  INTERFACE.print( eId );
  INTERFACE.print( ":" );
  INTERFACE.print( eAction );
  INTERFACE.print( ":" );
  INTERFACE.print( eData );
  INTERFACE.println( );
}

//------------------------------------------------------------------------------------------------------------
// The LCS management call back. So far, just show the incoming message...
//
//------------------------------------------------------------------------------------------------------------
void busMgtCallback( uint8_t *msg ) {

  INTERFACE.println( F("busMgtCallback -> " ));
  printLcsMsg( msg );
}

//------------------------------------------------------------------------------------------------------------
// The DCC subsystem messages call back. The callback passed the message to the base station message
// handler.
//
//------------------------------------------------------------------------------------------------------------
void dccMsgCallback( uint8_t *msg ) {

  INTERFACE.println( F("DCC Msg Callback -> " ));
  printLcsMsg( msg );

  msgInterface.handleLcsMsg( msg );
}

//------------------------------------------------------------------------------------------------------------
// The base station has also a command line interpreter. The callback is invoked by the core library when
// there is a command that it does not handle.
//
//------------------------------------------------------------------------------------------------------------
uint8_t cmdLineCallback( char *s ) {

  serialCmd.handleSerialCommand( s );
  return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// The LCS core library ends in a loop that manages its internal workings, invoking the callbacks where
// needed. This call back is the general loop callback. It is our chance to do our regular work such as
// checking on short circuit, lost sessions, etc. We will not do all the work in one swoop. There is a simple
// round robin scheme so that on each callback one activity is executed.
//
//------------------------------------------------------------------------------------------------------------
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
// The Arduino setup routine. Now, put it all together and get the show rolling. The individual software and
// hardware components are initialized. An error in one component, stops the setup process. If all went OK,
// the tracks are put under power. Finally, control is passed to the LCS core library loop. We will never
// return from this call.
//
//------------------------------------------------------------------------------------------------------------
void setup( ) {

  delay( 2000 );
  INTERFACE.begin( 115200 );
  INTERFACE.println( F( "LCS Base Station - setup" ));
  delay( 100 );

  uint8_t ret = setupCdcLib( );

  if ( ret == ALL_OK ) ret = setupLcsLib( );
  if ( ret == ALL_OK ) ret = setupSerialCommand( );
  if ( ret == ALL_OK ) ret = setupMsgInterface( );
  if ( ret == ALL_OK ) ret = setupLocoSessions( );
  if ( ret == ALL_OK ) ret = setupDccTrackMain( );
  if ( ret == ALL_OK ) ret = setupDccTrackProg( );

  if ( ret == ALL_OK ) {

    LcsBaseStationDccTrack::startDccProcessing( );

    mainTrack.powerStart( );
    progTrack.powerStart( );
    mainTrack.printDccTrackStatus( );
    progTrack.printDccTrackStatus( );

    INTERFACE.println( F( "Ready..." ));
    lcsLib->run( );
  }
}

//------------------------------------------------------------------------------------------------------------
// The Arduino loop routine. We never come here.
//
//------------------------------------------------------------------------------------------------------------
void loop( ) { }
#endif