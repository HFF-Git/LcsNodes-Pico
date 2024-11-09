//------------------------------------------------------------------------------------------------------------
//
// LCS Base Station - Serial Command Interface - implementation file
//
//------------------------------------------------------------------------------------------------------------
// The serial command interface is used to directly send commands to the session and DCC track objects. The
// command syntax is patterned after the DCC++ command syntax. Available commands that have a DCC++ counter
// part are implemented exactly after the DCC++ command. The main motivation is to use this interface for
// testing and debugging as well as third party tools that also implement the DCC++ command set to send
// commands to this base station as well when calling the serial IO interface. For the layout control system,
// the approach would rather be to send LCS messages for all tasks.
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Base Station
// Copyright (C) 2019 - 2024  Helmut Fieres
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

using namespace LCS;

//------------------------------------------------------------------------------------------------------------
// The object constructor. Nothing to do here.
//
//------------------------------------------------------------------------------------------------------------
LcsBaseStationCommand::LcsBaseStationCommand( ) { }

//------------------------------------------------------------------------------------------------------------
// The object setup command. We need to remember the other objects we use in handling the commands.
//
//------------------------------------------------------------------------------------------------------------
uint8_t LcsBaseStationCommand::setupSerialCommand(

    LcsBaseStationLocoSession *locoSessions,
    LcsBaseStationDccTrack    *mainTrack,
    LcsBaseStationDccTrack    *progTrack ) {

    this -> locoSessions  = locoSessions;
    this -> mainTrack     = mainTrack;
    this -> progTrack     = progTrack;

    return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// "handleSerialCommand" analyzes the command line and invokes the respective command handler. The first
// character in a command is the command letter. The command is followed by the arguments. For compatibility
// with the DCC++ original command set, each command that is also a DCC++ command is implemented exactly as
// the original. This allows external tools, such as the JMRI Decoder Pro configuration tool to be used.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::handleSerialCommand( char *s ) {

#if 0
    int     charIndex       = 0;
    char    cmdStr[ 256 ]   = { 0 };
    
    while ( s[ charIndex ] != '\0' ) {

        switch ( s[ charIndex ] ) {

            case '<': {
                
                cmdStr[ 0 ] = '\0';
                charIndex ++;

            } break;

            case '>': {

                switch ( s[ 0 ] ) {

                    case 'O': openSessionCmd( cmdStr + 1 ); break;
                    case 'K': closeSessionCmd( cmdStr + 1 ); break;

                    case 't': setThrottleCmd( cmdStr + 1 );  break;
                    case 'f': setFunctionGroupCmd( cmdStr + 1 ); break;
                    case 'v': setFunctionBitCmd( cmdStr + 1 ); break;

                    case 'R': readCVCmd( cmdStr + 1 ); break;
                    case 'W': writeCVByteCmd( cmdStr + 1 ); break;
                    case 'B': writeCVBitCmd( cmdStr + 1 ); break;
                    case 'w': writeCVByteMainCmd( cmdStr + 1 ); break;
                    case 'b': writeCVBitMainCmd( cmdStr + 1 ); break;

                    case 'M': writeDccPacketMainCmd( cmdStr + 1 ); break;
                    case 'P': writeDccPacketProgCmd( cmdStr + 1 ); break;

                    case 'C': setTrackOptionCmd( cmdStr + 1 ); break;
                    case 'Y': printDccLogCommand( cmdStr + 1 ); break;

                    case 'X': emergencyStopCmd( ); break;
                    case '0': turnPowerOffAllCmd( ); break;
                    case '1': turnPowerOnAllCmd( ); break;
                    case '2': turnPowerOnMainCmd( ); break;
                    case '3': turnPowerOnProgCmd( ); break;

                    case 's': printStatusCmd( cmdStr + 1 ); break;
                    case 'S': printBaseStationConfigCmd( ); break;
                    case 'L': printSessionMap( ); break;

                    case 'a': printTrackCurrentCmd( cmdStr + 1 ); break;

                    case '?': printHelpCmd( ); break;

                    case ' ': printf( "\n" ); break;

                    case 'e':
                    case 'E':
                    case 'D':
                    case 'T':
                    case 'Z':
                    case 'Q':
                    case 'F': printf( "<Not implemented>\n" ); break;

                    default: printf( "<Unknown command, use '?' for help>\n" );
                }

                charIndex ++;
            
            } break;

            default: {

                if ( strlen( cmdStr ) < sizeof( cmdStr) ) strncat( cmdStr, &s[ charIndex ], 1 );
                charIndex ++;
            }
        }
    }
#else

    switch ( s[ 0 ] ) {

        case 'O': openSessionCmd( s + 1 ); break;
        case 'K': closeSessionCmd( s + 1 ); break;

        case 't': setThrottleCmd( s + 1 );  break;
        case 'f': setFunctionGroupCmd( s + 1 ); break;
        case 'v': setFunctionBitCmd( s + 1 ); break;

        case 'R': readCVCmd( s + 1 ); break;
        case 'W': writeCVByteCmd( s + 1 ); break;
        case 'B': writeCVBitCmd( s + 1 ); break;
        case 'w': writeCVByteMainCmd( s + 1 ); break;
        case 'b': writeCVBitMainCmd(s + 1 ); break;

        case 'M': writeDccPacketMainCmd( s + 1 ); break;
        case 'P': writeDccPacketProgCmd( s + 1 ); break;

        case 'C': setTrackOptionCmd( s + 1 ); break;
        case 'Y': printDccLogCommand( s + 1 ); break;

        case 'X': emergencyStopCmd( ); break;
        case '0': turnPowerOffAllCmd( ); break;
        case '1': turnPowerOnAllCmd( ); break;
        case '2': turnPowerOnMainCmd( ); break;
        case '3': turnPowerOnProgCmd( ); break;

        case 's': printStatusCmd( s + 1 ); break;
        case 'S': printBaseStationConfigCmd( ); break;
        case 'L': printSessionMap( ); break;

        case 'a': printTrackCurrentCmd( s + 1 ); break;

        case '?': printHelpCmd( ); break;

        case ' ': printf( "\n" ); break;

        case 'e':
        case 'E':
        case 'D':
        case 'T':
        case 'Z':
        case 'Q':
        case 'F': printf( "<Not implemented>\n" ); break;

        default: printf( "<Unknown command, use '?' for help>\n" );
    }

#endif

}

//------------------------------------------------------------------------------------------------------------
// "openSessionCmd" handles the session creation command. This command is used to allocate a loco session.
// We are passed the cab ID and return a session Id.
//
//    <O cabId>
//
//    cabId      -  the requesting cab number, from 1 to MAX_CAB_ID.
//
//    returns: <O sId>
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::openSessionCmd( char *s ) {

    uint16_t  cabId = NIL_CAB_ID;
    uint8_t   sId   = 0;

    if ( sscanf( s, "%hu", &cabId ) != 1 ) return;

    int ret = locoSessions -> requestSession( cabId, LSM_NORMAL, &sId );

    printf( "<O %d>", (( ret == ALL_OK ) ? sId : -1 ));
}

//------------------------------------------------------------------------------------------------------------
// "closeSessionCmd" handles the session release command. The return code is the CabSession error code. A zero
// indicates a successful execution.
//
//    <K sId>
//
//    sId      -  the session number.
//
//    returns: <K status>
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::closeSessionCmd( char *s ) {

    uint8_t sId = NIL_LOCO_SESSION_ID;

    if ( sscanf( s, "%hhu", &sId ) != 1 ) return;

    int ret = locoSessions -> releaseSession( sId );

    printf( "<K %d>", ret );
}

//------------------------------------------------------------------------------------------------------------
// "setThrottleCmd" handles the throttle command. The original DCC++ interface uses both the register Id and
// the cabId. In the new version the sId is sufficient. But just to be compatible with the original
// DCC++ command, we also pass the cabId. It should be either zero or match the cabId in the allocated session.
//
//    <t sId cabId speed direction>
//
//    sId         -  the allocated session number.
//    cabId       -  the Cab Id. The number must match the can number in the session or be zero.
//    speed       -  throttle speed from 0-126, or -1 for emergency stop (resets SPEED to 0)
//    direction   -  the direction: 1=forward, 0=reverse. Setting direction when speed=0 only effects
//                   direction of cab lighting for a stopped train.
//
//    returns: <t sId speed direction >
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::setThrottleCmd( char *s ) {

  uint8_t   sId         = NIL_LOCO_SESSION_ID;
  uint16_t  cabId       = NIL_CAB_ID;
  uint8_t   speed       = 0;
  uint8_t   direction   = 0;

  if ( sscanf( s, "%hhu %hu %hhu %hhu", &sId, &cabId, &speed, &direction ) != 4 ) return;
  if (( cabId != NIL_CAB_ID ) && ( locoSessions -> getSessionIdByCabId( cabId ) != sId )) return;

  locoSessions -> setThrottle( sId, speed, direction );

  printf( "<t %d %d %d>", sId, speed, direction );
}

//------------------------------------------------------------------------------------------------------------
// "setFunctionBitCmd" turns on and off the engine decoder functions F0-F68 (F0 is sometimes called FL). This
// new command directly transmits the function setting to the engine decoder. The command interface is
// handling one function number at a time. The base station will handle the DCC byte generation.
//
//    <v sId funcId val >
//
//    sId     -  the allocated session number, from 1 to MAX_MAIN_REGISTERS.
//    funcId  -  the function number, currently implemented for F0 - F68.
//    val     -  the value to set, 1 or 0.
//
//    returns: NONE.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::setFunctionBitCmd( char *s ) {

    uint8_t sId = NIL_LOCO_SESSION_ID;
    uint8_t funcNum   = 0;
    uint8_t val       = 0;

    if ( sscanf( s, "%hhu %hhu %hhu", &sId, &funcNum, &val ) != 3 ) return;

    locoSessions -> setDccFunctionBit( sId, funcNum, val );
}

//------------------------------------------------------------------------------------------------------------
// "setFunctionGroupCmd" sets the engine decoder functions F0-F68 by group byte using the DCC byte instruction
// format. The user needs to do the calculation as shown in the list below. This command directly transmits
// the command to the engine decoder. This function requires some user math, and is only there for the DCC++
// command interface compatibility.
//
//    <f cabId byte1 [ byte2 ] >
//
//    cabId         -  the cab number
//    byte1         -  see below for encoding
//    byte2         -  see below for encoding
//
//    returns: NONE
//
//    The DCC packet data for setting function groups is defined as follows:
//
//      Group 1:  F0, F4, F3, F2, F1      DCC Command Format: 100DDDDD
//      Group 2:  F8, F7, F6, F5          DCC Command Format: 1011DDDD
//      Group 3:  F12, F11, F10, F9       DCC Command Format: 1010DDDD
//      Group 4:  F20 .. F13              DCC Command Format: 0xDE DDDDDDDD
//      Group 5:  F28 .. F21              DCC Command Format: 0xDF DDDDDDDD
//      Group 6:  F36 .. F29              DCC Command Format: 0xD8 DDDDDDDD
//      Group 7:  F44 .. F37              DCC Command Format: 0xD9 DDDDDDDD
//      Group 8:  F52 .. F45              DCC Command Format: 0xDA DDDDDDDD
//      Group 9:  F60 .. F53              DCC Command Format: 0xDB DDDDDDDD
//      Group 10: F68 .. F61              DCC Command Format: 0xDC DDDDDDDD
//
//    To set functions F0-F4 on (=1) or off (=0):
//
//      BYTE1:  128 + F1*1 + F2*2 + F3*4 + F4*8 + F0*16
//      BYTE2:  omitted
//
//    To set functions F5-F8 on (=1) or off (=0):
//
//      BYTE1:  176 + F5*1 + F6*2 + F7*4 + F8*8
//      BYTE2:  omitted
//
//    To set functions F9-F12 on (=1) or off (=0):
//
//      BYTE1:  160 + F9*1 +F10*2 + F11*4 + F12*8
//      BYTE2:  omitted
//
//    For the remaining groups, the two byte format is used. Byte one is:
//
//          0xde ( 222 ) -> F13-F20
//          0xdf ( 223 ) -> F21-F28
//          0xd8 ( 216 ) -> F29-F36
//          0xd9 ( 217 ) -> F37-F44
//          0xda ( 218 ) -> F45-F52
//          0xdb ( 219 ) -> F53-F60
//          0xdc ( 220 ) -> F61-F68
//
//    Byte two with N being the starting group index is always:
//
//      BYTE2: (FN)*1 + (FN+1)*2 + (FN+2)*4 + (FN+3)*8 + (FN+4)*16 + (FN+5)*32 + (FN+6)*64 + (FN+7)*128
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::setFunctionGroupCmd( char *s ) {

  uint16_t  cabId   = NIL_CAB_ID;
  uint8_t   byte1   = 0;
  uint8_t   byte2   = 0;

    if ( sscanf( s, "%hu %hhu %hhu", &cabId, &byte1, &byte2 ) < 2 ) return;

    uint8_t sId = locoSessions -> getSessionIdByCabId( cabId );

    if ( sId == NIL_LOCO_SESSION_ID ) return;

    if (( byte2 == 0 ) && ( byte1 >= 128 ) && ( byte1 < 160 )) {

        locoSessions -> setDccFunctionGroup( sId, 1, byte1 );
    }
    else if (( byte2 == 0 ) && ( byte1 >= 160 ) && ( byte1 < 176 )) {

        locoSessions -> setDccFunctionGroup( sId, 3, byte1 );
    }
    else if (( byte2 == 0 ) && ( byte1 >= 176 ) && ( byte1 < 192 )) {

        locoSessions -> setDccFunctionGroup( sId, 2, byte1 );
    }
    else if ( byte1 == 0xde ) locoSessions -> setDccFunctionGroup( sId, 4, byte2 );
    else if ( byte1 == 0xdf ) locoSessions -> setDccFunctionGroup( sId, 5, byte2 );
    else if ( byte1 == 0xd8 ) locoSessions -> setDccFunctionGroup( sId, 6, byte2 );
    else if ( byte1 == 0xd9 ) locoSessions -> setDccFunctionGroup( sId, 7, byte2 );
    else if ( byte1 == 0xda ) locoSessions -> setDccFunctionGroup( sId, 8, byte2 );
    else if ( byte1 == 0xdb ) locoSessions -> setDccFunctionGroup( sId, 9, byte2 );
    else if ( byte1 == 0xdc ) locoSessions -> setDccFunctionGroup( sId, 10, byte2 );
}

//------------------------------------------------------------------------------------------------------------
// "readCVCmd" reads a configuration variable from the engine decoder on the programming track. The
// callbacknum and callbacksub parameter are ignored by the base station and just passed back to the caller
// for identification purposes.
//
//    <R cvId [ callbacknum callbacksub ]>
//
//    cvId          -  the configuration variable ID, 1 ... 1024.
//    callbacknum   -  a number echoed back, ignored by the base station
//    callbacksub   -  a number echoed back, ignored by the base station
//
//    returns: <R callbacknum|callbacksub|cvId value>
//
//    where value is 0 - 255 of the CV variable or -1 if the value could not be verified.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::readCVCmd( char *s ) {

    uint16_t  cvId          = NIL_DCC_CV_ID;
    uint8_t   val           = 0;
    int       callbacknum   = 0;
    int       callbacksub   = 0;
    int       ret           = 0;

    if ( sscanf( s, "%hu %d %d", &cvId, &callbacknum, &callbacksub )  < 1 ) return;

    ret = locoSessions -> readCV( cvId, 0, &val );

    printf( "<R %d|%d|%d %d>", callbacknum, callbacksub, cvId, (( ret == ALL_OK ) ? val : -1 ));
}

//------------------------------------------------------------------------------------------------------------
// "writeCVByteCmd" writes a data byte to the engine decoder on the programming track and then verifies it.
// The callbacknum and callbacksub parameter are ignored by the base station and just passed back to the
// caller for identification purposes.
//
//    <W cvId val [ callbacknum callbacksub ]>
//
//    cvId          -  the configuration variable ID, 1 ... 1024.
//    val           -  the data byte.
//    callbacknum   -  a number echoed back, ignored by the base station
//    callbacksub   -  a number echoed back, ignored by the base station
//
//    returns: <W callbacknum|callbacksub|cvId Value>
//
//    where Value is 0 - 255 of the CV variable or -1 if the verification failed.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::writeCVByteCmd( char *s ) {

  uint16_t  cvId          = NIL_DCC_CV_ID;
  uint8_t   val           = 0;
  int       callbacknum   = 0;
  int       callbacksub   = 0;
  int       ret           = 0;

  if ( sscanf( s, "%hu %hhu %d %d", &cvId, &val, &callbacknum, &callbacksub ) < 2 ) return;

  ret = locoSessions -> writeCVByte( cvId, val );

  printf( "<W %d|%d|%d %d>", callbacknum, callbacksub, cvId, (( ret == ALL_OK ) ? val : -1 ));
}

//------------------------------------------------------------------------------------------------------------
// "writeCVBitCmd" writes a bit to the engine decoder on the programming track and then verifies the
// operation. The callbacknum and callbacksub parameter are ignored by the base station and just passed back
// to the caller for identification purposes.
//
//    <B cvId bitPos bitVal callbacknum callbacksub>
//
//    cvId          -  the configuration variable ID, 1 ... 1024.
//    bitPos        -  the bit position of the bit, 0 .. 7.
//    bitVal        -  the data bit.
//    callbacknum   -  a number echoed back, ignored by the base station
//    callbacksub   -  a number echoed back, ignored by the base station
//
//    returns: <B callbacknum|callbacksub|cvId bitPos Value>
//
//    where Value is 0 or 1 of the bit or -1 if the verification failed.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::writeCVBitCmd( char *s ) {

  uint16_t  cvId          = NIL_DCC_CV_ID;
  uint8_t   bitPos        = 0;
  uint8_t   bitVal        = 0;
  int       callbacknum   = 0;
  int       callbacksub   = 0;
  int       ret           = 0;

  if ( sscanf( s, "%hu %hhu %hhu %d %d", &cvId, &bitPos, &bitVal, &callbacknum, &callbacksub ) != 5 ) return;

  ret = locoSessions -> writeCVBit( cvId, bitPos, bitVal );

  printf( "<B %d|%d|%d|%d %d>", callbacknum, callbacksub, cvId, bitPos, (( ret == ALL_OK ) ? bitVal : -1 ));
}

//------------------------------------------------------------------------------------------------------------
// "writeCVByteMainCmd" writes a data byte to the engine decoder on the main track, without any verification.
// To be compatible with the DCC++ command set, the command is using the cabId to identify the loco we talk
// about.
//
//    <w cabId cvId val >
//
//    cabId       -  the cabId number.
//    cvId        -  the configuration variable ID, 1 ... 1024.
//    val         -  the data byte.
//
//    returns: NONE
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::writeCVByteMainCmd( char *s ) {

  uint16_t  cabId = NIL_CAB_ID;
  uint16_t  cvId  = NIL_DCC_CV_ID;
  uint8_t   val   = 0;

  if ( sscanf( s, "%hu %hu %hhu", &cabId, &cvId, &val ) != 3 ) return;

  locoSessions -> writeCVByteMain( locoSessions -> getSessionIdByCabId( cabId ), cvId, val );
}

//------------------------------------------------------------------------------------------------------------
// "writeCVBitMainCmd" writes a data byte to the engine decoder on the main track, without any verification.
// To be compatible with the DCC++ command set, the command is using the cabId to identify the loco we talk
// about.
//
//    <b cabId cvId bitPos bitVal >
//
//    cabId      -  the cabId number.
//    cvId       -  the configuration variable ID, 1 ... 1024.
//    bitPos     -  the bit position of the bit, 0 .. 7.
//    bitVal     -  the data bit.
//
//    returns: NONE
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::writeCVBitMainCmd( char *s ) {

  uint16_t  cabId = NIL_CAB_ID;
  uint16_t  cvId    = NIL_DCC_CV_ID;
  uint8_t   bitPos = 0;
  uint8_t   bitVal = 0;

  if ( sscanf(s, "%hu %hu %hhu %hhu", &cabId, &cvId, &bitPos, &bitVal ) != 4 ) return;

  locoSessions -> writeCVBitMain( locoSessions -> getSessionIdByCabId( cabId ), cvId, bitPos, bitVal );
}

//------------------------------------------------------------------------------------------------------------
// "writeDccPacketMainCmd" writes a DCC packet to the main operations track. This is for testing and debugging
// and you better know the DCC packet standard by heart :-). The DCC standards define packets up to 15 data
// bytes payload.
//
//    <M byte1 byte2 [ byte3 ... byte10 ]>
//
//    byte1 .. byte10   - the packet data in hexadecimal
//
//    returns: NONE
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::writeDccPacketMainCmd( char *s ) {

  uint8_t b[ 16 ] = { 0 };
  uint8_t nBytes  = sscanf( s,
                            "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu"
                            "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu",
                            b, b + 1, b + 2, b + 3, b + 4, b + 5, b + 6, b + 7,
                            b + 8, b + 9, b + 10, b + 11, b + 12, b + 13, b + 14, b + 15 );

  if ( nBytes >= 3 && nBytes <= 10 ) locoSessions -> writeDccPacketMain( b, nBytes, 0 );
}

//------------------------------------------------------------------------------------------------------------
// "writeDccPacketProgCmd" writes a DCC packet to the programming track. This is for testing and debugging and
// you better know the DCC packet standard by heart :-). The DCC standards define packets up to 15 data
// bytes payload.
//
///    <P byte1 byte2 [ byte3 ... byte10 ]>
//
//    byte1 .. byte10   - the packet data in hexadecimal
//
//    returns: NONE
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::writeDccPacketProgCmd( char *s ) {

  uint8_t b[ 16 ] = { 0 };
  uint8_t nBytes  = sscanf( s,
                            "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu"
                            "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu",
                            b, b + 1, b + 2, b + 3, b + 4, b + 5, b + 6, b + 7,
                            b + 8, b + 9, b + 10, b + 11, b + 12, b + 13, b + 14, b + 15 );

  if ( nBytes >= 3 && nBytes <= 10 ) locoSessions -> writeDccPacketProg( b, nBytes, 0 );
}

//------------------------------------------------------------------------------------------------------------
// "emergencyStopCmd" handles the emergencyStop command. This new command causes the base station to send out
// the emergency stop broadcast DCC command.
//
//    <X>
//
//    returns: <X>
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::emergencyStopCmd( ) {

  locoSessions -> emergencyStopAll( );
  printf( "<X>" );
}

//------------------------------------------------------------------------------------------------------------
// "turnPowerOnXXX" and "turnPowerOff" enables/disables the main and/or the programming track.
//
//    <0> - turn operations and programming track power off
//    <1> - turn operations and programming track power on
//    <2> - turn operations track power on
//    <3> - turn programming track power on
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::turnPowerOnAllCmd( ) {

  mainTrack -> powerStart( );
  progTrack -> powerStart( );
  printf( "<p1>" );
}

void LcsBaseStationCommand::turnPowerOffAllCmd( ) {

  mainTrack -> powerStop( );
  progTrack -> powerStop( );
  printf( "<p0>" );
}

void LcsBaseStationCommand::turnPowerOnMainCmd( ) {

  mainTrack -> powerStart( );
  printf( "<p1 MAIN>" );
}

void LcsBaseStationCommand::turnPowerOnProgCmd( ) {

  progTrack -> powerStart( );
  printf( "<p1 PROG>" );
}

//------------------------------------------------------------------------------------------------------------
// "setTrackOptionCmd" turns on and off capabilities of the operations or service track.
//
//    <C option>
//
//    option   - the option value.
//
//        1 -> set main track Cutout mode on.
//        2 -> set main track Cutout mode off.
//        3 -> set main track Railcom mode on.
//        4 -> set main track Railcom mode off.
//
//        10 -> set service track into operations mode.
//        11 -> set service track into service mode.
//
//    returns: NONE
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::setTrackOptionCmd( char *s ) {

  uint8_t option = 0;

  if ( sscanf( s, "%hhu", &option ) == 1 ) {

    switch ( option ) {

      case 1: mainTrack -> cutoutOn( );   break;
      case 2: mainTrack -> cutoutOff( );  break;
      case 3: mainTrack -> railComOn( );  break;
      case 4: mainTrack -> railComOff( ); break;

      case 10: progTrack -> serviceModeOff( );  break;
      case 11: progTrack -> serviceModeOn( ); break;
    }
  }
}

//------------------------------------------------------------------------------------------------------------
// "printStatusCmd" list information about the base station. Using just a "s" for a summary status is always
// a good idea to do this just as a first basic test if things are running at all. The level is a positive
// integer that specifies the information items to be listed.
//
//    <s [ opt ]> - the kind of status to display.
//
//    returns:  series of status information that can be read by an interface to determine status of the base
//              station and important settings
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::printStatusCmd( char *s ) {

  uint8_t opt = 0;

  if ( sscanf( s, "%hhu", &opt ) > 0 ) {

    switch ( opt ) {

      case 0: printVersionInfo( );      break;
      case 1: printConfiguration( );    break;
      case 2: printSessionMap( );       break;
      case 3: printTrackStatusMain( );  break;
      case 4: printTrackStatusProg( );  break;

      case 9: {

          printConfiguration( );
          printSessionMap( );
          printTrackStatusMain( );
          printTrackStatusProg( );

        } break;

      default: printVersionInfo( );
    }
  } else printVersionInfo( );
}

//------------------------------------------------------------------------------------------------------------
// "printBaseStationConfigCmd" list information about the base in a DCC++ compatible way.
//
//    <S> - the basestation configuration.
//
//    returns:  series of status information that can be read by an interface to determine status of the base
//              station and important settings
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::printBaseStationConfigCmd( ) {

  printConfiguration( );
}

//------------------------------------------------------------------------------------------------------------
// "printConfiguration" lists out the key hardware and software settings. Also very useful as the first
// trouble shooting task.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::printConfiguration( ) {

  printVersionInfo( );
  locoSessions -> printSessionMapConfig( );
  mainTrack -> printDccTrackConfig( );
  progTrack -> printDccTrackConfig( );
}

//------------------------------------------------------------------------------------------------------------
// "printVersionInfo" list out the Arduino type and software version of this program.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::printVersionInfo( ) {

  printf( "<\nLCS Base Station / Version: tbd / %s %s >\n", __DATE__, __TIME__  );
}

//------------------------------------------------------------------------------------------------------------
// "printSessionMap" list out the active session table content.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::printSessionMap( ) {

  locoSessions -> printSessionMapInfo( );
}

//------------------------------------------------------------------------------------------------------------
// "printTrackStatusMain" lists out the current MAIN track status
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::printTrackStatusMain( ) {

  mainTrack -> printDccTrackStatus( );
}

//------------------------------------------------------------------------------------------------------------
// "printTrackStatusProg" lists out the current PROG track status
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::printTrackStatusProg( ) {

  progTrack -> printDccTrackStatus( );
}

//------------------------------------------------------------------------------------------------------------
// "printTrackCurrentCmd" reads the actual current being drawn on the main operations track.
//
//    <a [ track ]>
//
// where "track" == 0 or omitted is the MAIN track, "track" == 1 is the PROG track.
//
//    returns: <a current>, where current is the actual power consumption in milliAmps.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::printTrackCurrentCmd( char *s ) {

  int opt = -1;

  sscanf( s, "%d", &opt );

  printf( "<a " );

  switch ( opt ) {

    case 0: printf( "%d", mainTrack -> getActualCurrent( )); break;
    case 1: printf( "%d", progTrack -> getActualCurrent( )); break;
    case 2: printf( "%d %d", mainTrack -> getActualCurrent( ), progTrack -> getActualCurrent( )); break;

    case 10: printf( "%d", mainTrack -> getRMSCurrent( )); break;
    case 11: printf( "%d", progTrack -> getRMSCurrent( )); break;
    case 12: printf( "%d %d", mainTrack -> getRMSCurrent( ), progTrack -> getRMSCurrent( )); break;

    default: printf( "%d", mainTrack -> getRMSCurrent( ));
  }

  printf( ">" );
}

//------------------------------------------------------------------------------------------------------------
// "printDccLogCommandCommand" is the command to manage the DCC log for tracing and debugging purposes.
//
//    <Y [ opt ]> where "opt" is the command to execute from the DCC Log function.
//
//      Main track: 
//
//      0 - disable DCC logging
//      1 - enable DCC logging
//      2 - start DCC logging
//      3 - stop DCC logging
//      4 - list log entries
//
//      Prog track:
//
//      10 - disable DCC logging
//      11 - enable DCC logging
//      12 - start DCC logging
//      13 - stop DCC logging
//      14 - list log entries
//
//      RailCom:
//
//      20 - show real time RailCom buffer, experimental
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::printDccLogCommand( char *s ) {

    int opt = -1;

    sscanf( s, "%d", &opt );

   printf( "<Y %d ", opt );

    switch ( opt ) {

        case 0:     mainTrack -> enableLog( false );  break;
        case 1:     mainTrack -> enableLog( true );   break;
        case 2:     mainTrack -> beginLog( );         break;
        case 3:     mainTrack -> endLog( );           break;
        case 4:     mainTrack -> printLog( );         break;

        case 10:    progTrack -> enableLog( false );  break;
        case 11:    progTrack -> enableLog( true );   break;
        case 12:    progTrack -> beginLog( );         break;
        case 13:    progTrack -> endLog( );           break;
        case 14:    progTrack -> printLog( );         break;

        case 20: {

            uint8_t buf[ 16 ];

            mainTrack -> getRailComMsg( buf, sizeof( buf ));

            printf( "RC: " );
            for ( uint8_t i = 0; i < 8; i++ ) printf( "0x%x ", buf[ i ]);

        } break;

        default: ;
    } 
    
    printf( ">" );
}

//------------------------------------------------------------------------------------------------------------
// "printHelp" lists a short version of all the command.
//
//------------------------------------------------------------------------------------------------------------
void LcsBaseStationCommand::printHelpCmd( ) {

    printf( "\nCommands:\n" );

    printf( "<O cabId>                              - allocate a session for the cab\n" );
    printf( "<K sId>                                - release a session\n" );
    printf( "<t sId cabId speed dir>                - set cab speed / direction\n" );
    printf( "<f cabId funcId val >                  - set cab function value, group DCC format\n" );
    printf( "<v sId funcId val >                    - set cab function value, individual\n" );
    printf( "<R cvId callbacknum callbacksub >      - read CV byte\n" );
    printf( "<W cvId val callbacknum callbacksub>   - write CV byte on programming track\n" );
    printf( "<B cvId bitPos bitVal callbacknum callbacksub> - write CV bit on programming track\n" );
    printf( "<w cabId cvId val > - write CV byte on operations track\n" );
    printf( "<b cabId cvId bitPos bitVal > - write CV bit on operations track\n" );
    printf( "<M sId byte1 byte2 [ byte3 ... byte10 ]> - send DCC packet on operations track to Reg n\n" );
    printf( "<P sId byte1 byte2 [ byte3 ... byte10 ]> - send DCC packet on programming track to Reg n\n" );

    printf( "<C track [option] - set track option, track = 0 -> MAIN, track = 1 -> PROG\n" );
    printf( "            " " - 1  - set main track cutout on\n" );
    printf( "            " " - 2  - set main track cutout off\n" );
    printf( "            " " - 3  - set main track RailCom on\n" );
    printf( "            " " - 4  - set main track RailCom off\n" );
    printf( "            " " - 10 - set prog track in operations mode\n" );
    printf( "            " " - 11 - set prog track in service mode\n" );

    printf( "<X>  - emergency stop all\n" );

    printf( "<0> - turn operations and programming track power off\n" );
    printf( "<1> - turn operations and programming track power on\n" );
    printf( "<2> - turn operations track power on\n" );
    printf( "<3> - turn programming track power on\n" );

    printf( "<a [ opt ]>    - list current consumption, default is RMS for MAIN\n" );
    printf( "             " " - opt 0  - actual - MAIN\n" );
    printf( "             " " - opt 1  - actual - PROG\n" );
    printf( "             " " - opt 2  - actual - both\n" );
    printf( "             " " - opt 10 - RMS - MAIN\n" );
    printf( "             " " - opt 11 - RMS - PROG\n" );
    printf( "             " " - opt 12 - RMS - both\n" );

    printf( "<C <option>> - turn on/off the Railcom option on the main track( 0 - off, 1 - on)\n" );

    printf( "<s [ level ]> - list status at detail level, default is summary\n" );
    printf( "             " " - level 0 - summary\n" );
    printf( "             " " - level 1 - configuration\n" );
    printf( "             " " - level 2 - session map\n" );
    printf( "             " " - level 3 - main track current\n" );
    printf( "             " " - level 4 - prog track current\n" );
    printf( "             " " - level 9 - all of the above\n" );

    printf( "<S> - list base station configuration\n" );
    printf( "<L> - list base station session table" );

    printf( "<Y [ opt ]> - DCC log options ( used for debugging and tracing )\n" );
    printf( "             " " - level 0 - disable logging\n" );
    printf( "             " " - level 1 - enable logging\n" );
    printf( "             " " - level 2 - begin logging\n" );
    printf( "             " " - level 3 - end logging\n" );
    printf( "             " " - level 4 - print logging data\n" );

    printf( "<?> - list this help\n" );

    printf( "\n" );
}
