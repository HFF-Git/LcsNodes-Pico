//----------------------------------------------------------------------------------------
//
// LCS Base Station - Serial Command Interface - implementation file
//
//----------------------------------------------------------------------------------------
// The serial command interface is used to directly send commands to the session 
// and DCC track objects. The command syntax is patterned after the DCC++ 
// ( DCC_EX ) command syntax. Available commands that have a DCC++ ( DCC-EX ) 
// counter part are implemented exactly as specified. The main motivation is to
// support applications that DCC++ ( DCC-EX ) commands to control parts of a 
// layout by sending commands to this base station.
//
// Note that only layout commands concerning track power and locomotive control
// are implemented. In particular, DCC-EX commands to define and manage 
// accessories such as turnouts and signals are not implemented. For our layout 
// control system, the approach would rather be to send LCS messages from a 
// layout management application to the base station to control accessories. 
//
// ??? as the syntax is not really regular, it becomes harder and harder to 
// just sscanf as the parser. Sigh.
//
//----------------------------------------------------------------------------------------
//
// LCS - Base Station
// Copyright (C) 2020 - 2026  Helmut Fieres
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
//----------------------------------------------------------------------------------------
#include "LcsBaseStation.h"

//----------------------------------------------------------------------------------------
// External global variables.
//
//----------------------------------------------------------------------------------------
extern uint16_t         debugMask;
extern LcsLocoSessions  *locoSessions;
extern LcsDccTrack      *mainTrack;
extern LcsDccTrack      *progTrack; 

//----------------------------------------------------------------------------------------
// The local namespace contains the routines that actually produce most of the 
// formatted string. The routines are all built with the "sprintf" function and 
// return the numbers of characters generated in the passed line buffer.
//
//----------------------------------------------------------------------------------------
namespace {

//----------------------------------------------------------------------------------------
// "printVersionInfo" list out the Arduino type and software version of this program.
//
//----------------------------------------------------------------------------------------
void printVersionInfo( uint8_t chan ) {

    usbIoPrintf( chan, "<\nLCS Base Station / Version: tbd / %s %s >\n", 
            __DATE__, __TIME__  );
}

//----------------------------------------------------------------------------------------
// "turnPowerOnOff" enables/disables the main and/or the programming track.
// The JOIN option will turn on the programming track if the main track is on. 
// The JOIN option is ignored if the main track is off. The JOIN option is used
// to allow programming of decoders on the programming track while the main 
// track is powered on.
//
// <1 [ MAIM | PROG ]> - turn operations and programming track power on.
// <0 [ MAIM | PROG | JOIN ]> - turn operations and programming track power off.
//
// returns: < cmd arg >
//
// Note: The JOIN option controls the setting of the PROG track. It switches to 
// MAIN mode. Any loco command, t or F, will reset the PROG track back to 
// service track mode. Currently nit implemented.
// 
// ???: apparently, DCC-EX does not return anything for the commands. A broadcast
// command "<pOnOff [track]>" is triggered.
//
//----------------------------------------------------------------------------------------
void turnPowerOnOffCmd( uint8_t chan, char *s ) {

    uint8_t  powerOnOff   = 0;
    char     trackStr[ 16 ] = { 0 };

    if ( sscanf( s, "%hhu %s", &powerOnOff, trackStr ) < 1 ) return;

    if ( powerOnOff == 1 ) {

        if ( strcmp( trackStr, "MAIN" ) == 0 ) {

            mainTrack -> powerEnable( true );
            usbIoPrintf( chan, "<1 MAIN>" );
        }
        else if ( strcmp( trackStr, "PROG" ) == 0 ) {

            progTrack -> powerEnable( true );
            usbIoPrintf( chan, "<1 PROG>" );
        }
        else if ( strcmp( trackStr, "JOIN" ) == 0 ) {

            usbIoPrintf( chan,  "<X>" );
        }
        else {

            mainTrack -> powerEnable( true );
            progTrack -> powerEnable( true );
            usbIoPrintf( chan,  "<1>" );
        }
    }
    else {

        if ( strcmp( trackStr, "MAIN" ) == 0 ) {
            
            mainTrack -> powerEnable( false );
            usbIoPrintf( chan, "<0 MAIN>" );
        }
        else if ( strcmp( trackStr, "PROG" ) == 0 ) {
            
            progTrack -> powerEnable( false );
            usbIoPrintf( chan, "<0 PROG>" );
        }
        else {
            
            mainTrack -> powerEnable( false );
            progTrack -> powerEnable( false );
            usbIoPrintf( chan, "<0>" );
        }
    }
}

//----------------------------------------------------------------------------------------
// "setThrottleCmd" handles the throttle command. 
// 
//    <t cabId [ speed direction ]>
//    <t cabId reg speed direction> 
//
//    cabId     -   the Cab Id. 
//
//.   regId     -   register id, deprecated, not used.
//
//    speed     -   throttle speed from 0-126, or -1 for emergency stop 
//                  ( resets speed to 0 )
//
//    direction -   the direction: 1=forward, 0=reverse. Setting direction when 
//                  speed=0 only effects direction of cab lighting for a stopped 
//                  train.
//
//    returns: none
//
// There are two formats. Historically there was a "reg" argument. This form 
// is deprecated.
//
// DCC-EX states that as a result of any Loco Command this will be broadcasted.
// What exactly is this broadcast good for?
// 
//    <l cab reg speedByte functionMap>   
//
// Also, if we in JOIN mode, the mode of the PROG track will be reset to PROG.
//
//----------------------------------------------------------------------------------------
void setThrottleCmd( uint8_t chan, char *s ) {

    uint16_t  cabId         = NIL_CAB_ID;
    uint8_t   regId         = 0;
    uint8_t   speed         = 0;
    uint8_t   direction     = 0;
    int       args          = 0;

    args = sscanf( s, "%hu %hhu %hhu %hhu", &cabId, &regId, &speed, &direction );
    
    if ( args < 1 ) return;

    if ( args < 4 ) {

        args = sscanf( s, "%hu %hhu %hhu", &cabId, &speed, &direction );  
    }

    if (( cabId >= MIN_CAB_ID ) && ( cabId <= MAX_CAB_ID )) {

        uint8_t rStat = locoSessions -> setThrottle( cabId, speed, direction );
        if ( rStat != LCS_OK ) usbIoPrintf( chan, "<X>" );
    }
    else if ( cabId == 0 ) {

        // ??? communicate a refresh ...
    }

    // ??? reset the JOIN mode if in JOIN mode.
}

//----------------------------------------------------------------------------------------
// "setFunctionBitCmd" turns on and off the engine decoder functions F0-F68. 
// This command directly transmits the function setting to the engine decoder. 
// The command interface is handling one function number at a time. The base 
// station will handle the DCC byte generation.
//
//    <F cabId funcId val >
//
//    cabId   -  the cabId.
//    funcId  -  the function number, currently implemented for F0 - F68.
//    val     -  the value to set, 1 or 0.
//
// DCC-EX states that as a result of any Loco Command this will be broadcasted.
// What exactly is this broadcast good for?
// 
//    <l cab reg speedByte functionMap>  
//
// Also, if we in JOIN mode, the mode of the PROG track will be reset to PROG.
//
//----------------------------------------------------------------------------------------
void setFunctionBitCmd( uint8_t chan, char *s ) {

    uint16_t cabId      = NIL_CAB_ID;
    uint8_t  funcNum    = 0;
    uint8_t  val        = 0;

    if ( sscanf( s, "%hu %hhu %hhu", &cabId, &funcNum, &val ) != 3 ) return;

    uint8_t rStat = locoSessions -> setDccFunctionBit( 0, funcNum, val );
    if ( rStat != LCS_OK ) usbIoPrintf( chan, "<X>" );

    // ??? reset the JOIN mode if in JOIN mode.
}

//----------------------------------------------------------------------------------------
// "setFunctionGroupCmd" sets the engine decoder functions F0-F68 by group byte 
// using the DCC byte instruction format. The user needs to do the calculation 
// as shown in the list below. This command directly transmits the command to 
// the engine decoder. This function requires some user math, and is only there 
// for the DCC++ command interface compatibility.
//
// NOTE: this command is deprecated. We just keep it for compatibility with the 
// DCC++ command set. The preferred way to set functions is to use the 
// "setFunctionBitCmd" command which is easier to use and does not require the
// user to do any math.
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
//      BYTE2: (FN)*1 + (FN+1)*2 + (FN+2)*4 + (FN+3)*8 + (FN+4)*16 + 
//             (FN+5)*32 + (FN+6)*64 + (FN+7)*128
//
// DCC-EX states that as a result of any Loco Command this will be broadcasted.
// What exactly is this broadcast good for?
// 
//    <l cab reg speedByte functionMap>  
//
// Also, if we in JOIN mode, the mode of the PROG track will be reset to PROG.
//
//----------------------------------------------------------------------------------------
void setFunctionGroupCmd( uint8_t chan, char *s ) {

  uint16_t  cabId   = NIL_CAB_ID;
  uint8_t   byte1   = 0;
  uint8_t   byte2   = 0;
  uint8_t   rStat   = LCS_OK;

    if ( sscanf( s, "%hu %hhu %hhu", &cabId, &byte1, &byte2 ) < 2 ) return;

    if ( cabId == NIL_CAB_ID ) return;

    if (( byte2 == 0 ) && ( byte1 >= 128 ) && ( byte1 < 160 )) {

        rStat = locoSessions -> setDccFunctionGroup( cabId, 1, byte1 );
    }
    else if (( byte2 == 0 ) && ( byte1 >= 160 ) && ( byte1 < 176 )) {

        rStat = locoSessions -> setDccFunctionGroup( cabId, 3, byte1 );
    }
    else if (( byte2 == 0 ) && ( byte1 >= 176 ) && ( byte1 < 192 )) {

        rStat = locoSessions -> setDccFunctionGroup( cabId, 2, byte1 );
    }
    else if ( byte1 == 0xde ) { 
        
        rStat = locoSessions -> setDccFunctionGroup( cabId, 4, byte2 );
    }
    else if ( byte1 == 0xdf ) { 

        rStat = locoSessions -> setDccFunctionGroup( cabId, 5, byte2 );
    }
    else if ( byte1 == 0xd8 ) { 

        rStat = locoSessions -> setDccFunctionGroup( cabId, 6, byte2 );
    }
    else if ( byte1 == 0xd9 ) { 

        rStat = locoSessions -> setDccFunctionGroup( cabId, 7, byte2 );
    }
    else if ( byte1 == 0xda ) { 

        rStat = locoSessions -> setDccFunctionGroup( cabId, 8, byte2 );
    }
    else if ( byte1 == 0xdb ) { 

        rStat = locoSessions -> setDccFunctionGroup( cabId, 9, byte2 );
    }
    else if ( byte1 == 0xdc ) { 

        rStat = locoSessions -> setDccFunctionGroup( cabId, 10, byte2 );
    }

    if ( rStat != LCS_OK ) usbIoPrintf( chan, "<X>" );

    // ??? reset the JOIN mode if in JOIN mode.
}

//----------------------------------------------------------------------------------------
// "emergencyStopCmd" handles the emergencyStop command. This new command causes 
// the base station to send out the emergency stop broadcast DCC command.
//
//    <!>
//
//    returns: none
//
//----------------------------------------------------------------------------------------
void emergencyStopCmd( uint8_t chan ) {

    locoSessions -> emergencyStopAll( );
}

//----------------------------------------------------------------------------------------
// Turnout command. 
//
//  <T> -   request a list of all turnouts. Reply: <T id state>
//          state = 0 -> thrown, state = 1, closed,
//
//  <T id state> - set the turnout. 
//          state = 0, C, 1, T or X for "examine". 
//
//  returns: <H id state>
//
//  Apparently, DCC-EX keeps a list of defined turnouts. We do not do this.
//  Unclear what list commands should do in this case.
//
//----------------------------------------------------------------------------------------
void turnoutCmd( uint8_t chan, char *s ) {


    uint16_t id = 0;
    char state[ 2 ];
    int args = sscanf( s, "%hu %s", &id, state );

    if ( args == 0 ) {

    }
    else if ( args == 2 ) {

        if ( strlen( state ) != 1 ) return;

        if (( state[ 0 ] == '1' ) or ( state[ 0 ] == 'T' )) {

            // set turnout to thrown ...
            // response: "H id state"
        }
        else if (( state[ 0 ] == '0' ) or ( state[ 0 ] == 'C' )) {

            // set turnout to close ...
            // response: "H id state"
        }
        else if ( state[ 0 ] == 'X' ) {

            // examine...
            // response: "H id state"
        }
        else {

            usbIoPrintf( chan, "<X>" );
        }
    }
}

//----------------------------------------------------------------------------------------
// "writeCVByteMainCmd" writes a data byte to the engine decoder on the main 
// track, without any verification. To be compatible with the DCC++ command set, 
// the command is using the cabId to identify the loco we talk about.
//
//    <w cabId cvId val >
//
//    cabId       -  the cabId number.
//    cvId        -  the configuration variable ID, 1 ... 1024.
//    val         -  the data byte.
//
//    returns: NONE
//
//----------------------------------------------------------------------------------------
void writeCVByteMainCmd( uint8_t chan, char *s ) {

    uint16_t  cabId = NIL_CAB_ID;
    uint16_t  cvId  = NIL_DCC_CV_ID;
    uint8_t   val   = 0;

    if ( sscanf( s, "%hu %hu %hhu", &cabId, &cvId, &val ) != 3 ) return;

    locoSessions -> writeCVByteMain( cabId, cvId, val );
}

//----------------------------------------------------------------------------------------
// "writeCVBitMainCmd" writes a data byte to the engine decoder on the main track, 
// without any verification. To be compatible with the DCC++ command set, the 
// command is using the cabId to identify the loco we talk about.
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
//----------------------------------------------------------------------------------------
void writeCVBitMainCmd( uint8_t chan, char *s ) {

    uint16_t  cabId = NIL_CAB_ID;
    uint16_t  cvId    = NIL_DCC_CV_ID;
    uint8_t   bitPos = 0;
    uint8_t   bitVal = 0;

    if ( sscanf(s, "%hu %hu %hhu %hhu", 
                &cabId, &cvId, &bitPos, &bitVal ) != 4 ) return;

    locoSessions -> writeCVBitMain( cabId, cvId, bitPos, bitVal );
}

//----------------------------------------------------------------------------------------
// "readCVCmd" reads a configuration variable from the engine decoder on the 
// programming track. The callbacknum and callbacksub parameter are ignored 
// by the base station and just passed back to the caller for identification 
// purposes.
//
//    <R>
//    <R cvId>
//
//    cvId  -  the configuration variable ID, 1 ... 1024.
//
//    returns: <r cvId value>
//
//----------------------------------------------------------------------------------------
void readCVCmd( uint8_t chan, char *s ) {

    uint16_t    cvId    = NIL_DCC_CV_ID;
    uint8_t     val     = 0;
    int         ret     = 0;
    int         len     = sscanf( s, "%hu", &cvId ); 

    if ( len  < 1 ) return;

    if ( len == 0 ) {

        usbIoPrintf( chan, "<r -1>" ); // ??? to do ...
    }

    if ( len == 1 ) {

        ret = locoSessions -> readCV( cvId, 0, &val );
        usbIoPrintf( chan, "<R %d|%d>", cvId, (( ret == LCS_OK ) ? val : -1 ));
    }
}

//----------------------------------------------------------------------------------------
// "writeCVByteCmd" writes a data byte to the engine decoder on the programming 
// track and then verifies it. The callbacknum and callbacksub parameter are 
// ignored and just passed back to the caller for identification purposes.
//
//    <W cvId val>
//    <W address>
//
//    cvId          -  the configuration variable ID, 1 ... 1024.
//    val           -  the data byte.
//    address       -  the decoder address.
//
//    returns: <w cvId value> or  <w cvId -1>
//    returns: <w address> or <w -1>
//
//----------------------------------------------------------------------------------------
void writeCVByteCmd( uint8_t chan, char *s ) {

    uint16_t  cvId          = NIL_DCC_CV_ID;
    uint8_t   val           = 0;
    int       ret           = 0;

    if ( sscanf( s, "%hu %hhu %d %d", &cvId, &val ) < 2 ) return;

    ret = locoSessions -> writeCVByte( cvId, val );
    usbIoPrintf( chan, "<W %d %d>", cvId, (( ret == LCS_OK ) ? val : -1 ));
}

//----------------------------------------------------------------------------------------
// "writeCVBitCmd" writes a bit to the engine decoder on the programming track
// and then verifies the operation. The callbacknum and callbacksub parameter are 
// ignored and just passed back to the caller for identification purposes.
//
//    <B cvId bitPos bitVal>
//
//    cvId          -  the configuration variable ID, 1 ... 1024.
//    bitPos        -  the bit position of the bit, 0 .. 7.
//    bitVal        -  the data bit.
//
//    returns: <B cvId bitPos Value>
//
//    where Value is 0 or 1 of the bit or -1 if the verification failed.
//
//----------------------------------------------------------------------------------------
void writeCVBitCmd( uint8_t chan, char *s ) {

    uint16_t  cvId          = NIL_DCC_CV_ID;
    uint8_t   bitPos        = 0;
    uint8_t   bitVal        = 0;
    int       ret           = 0;

    if ( sscanf( s, "%hu %hhu %hhu", &cvId, &bitPos, &bitVal ) != 5 ) return;

    ret = locoSessions -> writeCVBit( cvId, bitPos, bitVal );

    usbIoPrintf( chan, "<B %d|%d|%d>", cvId, bitPos, 
           (( ret == LCS_OK ) ? bitVal : -1 ));
}

//----------------------------------------------------------------------------------------
// The "V" command verifies a CV bit or variable by comparing the data read
// with the value passed to the command.
//
//  <V cvId bitPos bitVal>
//  <V cvId val>
//
//  cvId          -  the configuration variable ID, 1 ... 1024.
//  bitPos        -  the bit position of the bit, 0 .. 7.
//  bitVal        -  the data bit.
//  val           - the CV value.
//
//  returns: 0 | 1 or <V -1>
//  returns: <v cvId val> or <v cv -1>
//
//----------------------------------------------------------------------------------------
void verifyCvCmd( uint8_t chan, char *s ) {

    uint16_t    cvId    = NIL_DCC_CV_ID;
    uint8_t     bitPos  = 0;
    uint8_t     testVal = 0;
    int         ret     = 0;

    if ( sscanf( s, "%hu  %hhu %hhu", &cvId, &bitPos, &testVal ) != 3 )  return;

    // ??? to do ....

    // ret = locoSessions -> verifyCvVal( cvId, testVal );
    // ret = locoSessions -> verifyCvBit( cvId, bitPos, testVal );
}

//----------------------------------------------------------------------------------------
// "writeDccPacketMainCmd" writes a DCC packet to the main operations track. This
// is for testing and debugging and you better know the DCC packet standard by 
// heart :-). The DCC standards define packets up to 15 data bytes payload.
//
//    <M byte1 byte2 [ byte3 ... byte10 ]>
//
//    byte1 .. byte10   - the packet data in hexadecimal
//
//    returns: NONE
//
//----------------------------------------------------------------------------------------
void writeDccPacketMainCmd( uint8_t chan, char *s ) {

    uint8_t b[ 16 ] = { 0 };
    uint8_t nBytes  = sscanf( s,
                    "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu"
                    "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu",
                    b, b + 1, b + 2, b + 3, b + 4, b + 5, b + 6, b + 7,
                    b + 8, b + 9, b + 10, b + 11, b + 12, b + 13, b + 14, b + 15 );

    
    if ( nBytes >= 3 && nBytes <= 16 ) 
        locoSessions -> writeDccPacketMain( b, nBytes );
}

//----------------------------------------------------------------------------------------
// "writeDccPacketProgCmd" writes a DCC packet to the programming track. This is
// for testing and debugging and you better know the DCC packet standard by heart :-). 
// The DCC standards define packets up to 15 data bytes payload.
//
///    <P byte1 byte2 [ byte3 ... byte10 ]>
//
//    byte1 .. byte10   - the packet data in hexadecimal
//
//    returns: NONE
//
//----------------------------------------------------------------------------------------
void writeDccPacketProgCmd( uint8_t chan, char *s ) {

    uint8_t b[ 16 ] = { 0 };
    uint8_t nBytes  = sscanf( s,
                    "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu"
                    "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu",
                    b, b + 1, b + 2, b + 3, b + 4, b + 5, b + 6, b + 7,
                    b + 8, b + 9, b + 10, b + 11, b + 12, b + 13, b + 14, b + 15 );

    if ( nBytes >= 3 && nBytes <= 16 ) 
        locoSessions -> writeDccPacketProg( b, nBytes );
}

//----------------------------------------------------------------------------------------
// The "D" command is the entry point to a lot of commands in DCC-EX. We 
// implemented a subset of these commands.
// 
//  <D option args>>
//
// returns: none
// 
// Note: the D command in DCC-EX has a ton of options and qualifiers. For now,
// we only recognize the base station reset.
//
// Note: this is a rather complex command, we perhaps need to read in string 
// and analyze hen, vs. let sscanf handle it. 
//
//----------------------------------------------------------------------------------------
void handleDCmd( uint8_t chan, char *s ) {

    char argStr1[ 16 ] = { 0 };
    char argStr2[ 16 ] = { 0 };
    char argStr3[ 16 ] = { 0 };


    if ( sscanf( s, "%s %s %s", argStr1, argStr2, argStr3 ) < 1 ) return;

    if ( strcmp( argStr1, "RESET" ) == 0 ) {

        // ??? reset the base station ...
    }
    else if ( strcmp( argStr1, "ACK" ) == 0 ) {


        printf( "<X>" );
    }
    else if ( strcmp( argStr1, "PROGBOOST" ) == 0 ) {

        printf( "<X>" );
    }
}

//----------------------------------------------------------------------------------------
// The "J" command handles a variety of info commands.
//
//  <J ... >
//
//  <J T id> - details of a turnout.
//  <J T> - return a list of defined turnouts.
//
//----------------------------------------------------------------------------------------
void handleJCmd( uint8_t chan, char *s ) {


    

    printf( "<X>" ); // for now ...
}

//----------------------------------------------------------------------------------------
// The "Q" command request a list of all sensors and their state.
//
//  <Q>
//
//  returns: <q id> for each defined sensors.
//
// I am not sure what this command will really do. We do not define sensors
// in the base station. Also, are sensor the track occupancy sensors ?
//
//----------------------------------------------------------------------------------------
void statusOfSensorsCmd( uint8_t chan, char *s ) {

     printf( "<X>" ); // for now ...
}

//----------------------------------------------------------------------------------------
// The "S" command request a list of all defined sensors.
// 
// Unclear what to return ...
//
//----------------------------------------------------------------------------------------
void numOfDefinedSensorsCmd( uint8_t chan, char *s ) {

     printf( "<X>" ); // for now ...
}

//----------------------------------------------------------------------------------------
// "baseStationOptCmd" is the command to manage the base station specific command
// options not defined in DCC-EX.
//
//  <Y [ opt ]> where "opt" is the option to execute.
//
//  Main track: 
//
//      1 - set PLAIN mode on main track
//      1 - enable CUTOUT on main track
//      3 - enable RAILCOM on main track
//
//      5 - disable DCC logging
//      6 - enable DCC logging
//      7 - start DCC logging
//      8 - stop DCC logging
//      9 - list log entries
//
//  Prog track:
//
//      10 - disable DCC logging
//      11 - enable DCC logging
//      12 - start DCC logging
//      13 - stop DCC logging
//      14 - list log entries
//
//  RailCom:
//
//      20 - show real time RailCom buffer, experimental
//
// Note: this is not a DDC-EX command.
//
//----------------------------------------------------------------------------------------
void baseStationOptCmd( uint8_t chan, char *s ) {

    int opt = -1;

    sscanf( s, "%d", &opt );

    printf( "<Y %d ", opt );

    switch ( opt ) {

        case 1:     mainTrack -> setTrackMode( DT_M_PLAIN );  break;
        case 2:     mainTrack -> setTrackMode( DT_M_CUTOUT ); break;
        case 3:     mainTrack -> setTrackMode( DT_M_RAILCOM );  break;

        case 5:     mainTrack -> enableLog( false );  break;
        case 6:     mainTrack -> enableLog( true );   break;
        case 7:     mainTrack -> beginLog( );         break;
        case 8:     mainTrack -> endLog( );           break;
        case 9:     mainTrack -> printLog( );         break;

        case 10:    progTrack -> enableLog( false );  break;
        case 11:    progTrack -> enableLog( true );   break;
        case 12:    progTrack -> beginLog( );         break;
        case 13:    progTrack -> endLog( );           break;
        case 14:    progTrack -> printLog( );         break;

        case 20: {

            uint8_t buf[ 16 ];

            mainTrack -> getRailComMsg( buf, sizeof( buf ));

            usbIoPrintf( chan, "RC: " );
            for ( uint8_t i = 0; i < 8; i++ ) usbIoPrintf( chan, "0x%x ", buf[ i ]);

        } break;

        default: ;
    } 
    
    printf( ">" );
}

//----------------------------------------------------------------------------------------
// "printTrackCurrentCmd" reads the actual current being drawn on the DCC main 
// operations track. 
//
//    <c>
//
//    returns:  <c "CurrentMAIN" current C "Milli" "0" max_ma "1" trip_ma>
//              ( "CurrentMAIN" - text for software like JMRI )
//              ( current - current in milliAmps )
//              ( C - signifies this is a current meter )
//              ( "0" - first parameter )
//              ( max_ma - maximum current handling of the track )
//              ( "1" - first parameter )
//              ( trip_ma - over current value configured )
//              
//  ??? need to fetch the data.
// 
//----------------------------------------------------------------------------------------
void currentOnTrackCmd( uint8_t chan, char *s ) {

    usbIoPrintf( chan, "CurrentMAIN %d C Milli 0 %d 1 %d>",
            mainTrack -> getActualCurrent( ), 0, 0 );  // ??? fix ...
}

//----------------------------------------------------------------------------------------
// The "#" command returns the number of supported cabs. We do not have such a 
// concept.
//
//
//----------------------------------------------------------------------------------------
void numOfCabSupportedCmd ( uint8_t chan, char *s ) {

    // ??? what to return ? The number of our cache ?
}

//----------------------------------------------------------------------------------------
// "printStatusCmd" list information about the base station. Using just a "s" 
// for a summary status is always a good idea to do this just as a first basic
// test if things are running at all. The level is a positive integer that 
// specifies the information items to be listed.
//
//    <s [ opt ]> - the kind of status to display.
//
// The command "<s>" without any argument, is the DCC-EX command. We do however
// allow options for more details of our base station.     
//
//----------------------------------------------------------------------------------------
void printStatusCmd( uint8_t chan, char *s ) {

    uint8_t opt = 0;

    if ( sscanf( s, "%hhu", &opt ) > 0 ) {

        switch ( opt ) {

            case 1: {

                printVersionInfo( chan );
                locoSessions -> printActiveCabMap( );
                mainTrack -> printDccTrackConfig( );
                progTrack -> printDccTrackConfig( );
            
            } break;
           
            case 2: { 
                
                mainTrack -> printDccTrackStatus( );  
                
            } break;

            case 3: { 
                
                progTrack -> printDccTrackStatus( );  
                
            } break;

            case 4: {

                locoSessions -> printActiveCabMap( );

            } break;

            default: printVersionInfo( chan );
        }
    } 
    else printVersionInfo( chan );
}

//----------------------------------------------------------------------------------------
// "printHelp" lists a short version of all the command.
//
// ??? cleanup ...
// ??? better way to print, perhaps in two calls...
//----------------------------------------------------------------------------------------
void printHelpCmd( uint8_t chan ) {

    constexpr char fmtStr[ ] = "%24s - %32s";

    usbIoPrintf( chan, "%24s", "\nCommands:\n" );

    usbIoPrintf( chan, fmtStr,
                 "<0> MAIN | PROG", 
                 "turn track power off\n" );

    usbIoPrintf( chan, fmtStr,
                 "<1> MAIN | PROG", 
                 "turn track power off\n" );

    usbIoPrintf( chan, fmtStr,
                 "<1> JOIN", 
                 "join MAIN and PROG track\n" );            

    usbIoPrintf( chan, fmtStr, 
                 "<t cabId speed dir>"
                 "set cab speed / direction\n" );

    usbIoPrintf( chan, fmtStr,
                 "<f cabId funcId val>"
                "set cab function value, group DCC format\n" );

    usbIoPrintf( chan, fmtStr,
                 "<F cabId funcId val>"
                 "set cab function value, individual\n" );

    usbIoPrintf( chan, fmtStr, "<!>", "emergency stop all\n" );

    usbIoPrintf( chan, fmtStr,
                 "<T id state>"
                 "set Turnout ( state = 0, 1, T, C, X )\n" );

    usbIoPrintf( chan, fmtStr,
                 "<R cvId>"
                 "read CV byte\n" );

    usbIoPrintf( chan, fmtStr,
                 "<R>"
                 "read decoder address\n" );

    usbIoPrintf( chan, fmtStr,
                 "<W cvId val>"
                 "write CV byte on programming track\n" );

    usbIoPrintf( chan, fmtStr,
                 "<W address>"
                 "write decoder address\n" );
    
    usbIoPrintf( chan, fmtStr,
                 "<B cvId bitPos bitVal> "
                 "write CV bit on programming track\n" );

    usbIoPrintf( chan, fmtStr,
                 "<V cvId bitPos bitVal> "
                 "verify CV bit on programming track\n" );

    usbIoPrintf( chan, fmtStr,
                 "<V cvId cvVal> "
                 "verify CV value on programming track\n" );

    usbIoPrintf( chan, fmtStr,
                 "<w cabId cvId val > "
                 "write CV byte on operations track\n" );
    
    usbIoPrintf( chan, fmtStr,
                 "<b cabId cvId bitPos bitVal > "
                 "write CV bit on operations track\n" );

    usbIoPrintf( chan, fmtStr,
                 "<M byte1 byte2 [ byte3 ... byte10 ]>",
                 "send DCC packet on operations track\n" );

    usbIoPrintf( chan, fmtStr,
                 "<P byte1 byte2 [ byte3 ... byte10 ]> "
                 "send DCC packet on programming track\n" );

    usbIoPrintf( chan, fmtStr,
                 "<s [ opt ]>",
                 "list status information\n" );

    usbIoPrintf( chan, fmtStr, "",  "   0 - summary\n" );
    usbIoPrintf( chan, fmtStr, "",  "   1 - configuration\n" );
    usbIoPrintf( chan, fmtStr, "",  "   2 - session map\n" );
    usbIoPrintf( chan, fmtStr, "",  "   3 - main track current\n" );
    usbIoPrintf( chan, fmtStr, "",  "   4 - prog track current\n" );
    usbIoPrintf( chan, fmtStr, "",  "   9 - all of the above\n" );

    usbIoPrintf( chan, fmtStr,
                 "<Y <option>>",
                 "set LCS base station options\n" );

    usbIoPrintf( chan, fmtStr, "", "   1  - set main track PLAIN\n" );
    usbIoPrintf( chan, fmtStr, "", "   2  - enable main track CUTOUT\n" );
    usbIoPrintf( chan, fmtStr, "", "   3  - enable main track RAILCOM\n" );

    usbIoPrintf( chan, fmtStr, "", "   5  - disable main track logging\n" );
    usbIoPrintf( chan, fmtStr, "", "   6  - enable main track logging\n" );
    usbIoPrintf( chan, fmtStr, "", "   7  - begin main track logging\n" );
    usbIoPrintf( chan, fmtStr, "", "   8  - end main track logging\n" );
    usbIoPrintf( chan, fmtStr, "", "   9  - print main track logging data\n" );
    usbIoPrintf( chan, fmtStr, "", "   10 - disable prog track logging\n" );
    usbIoPrintf( chan, fmtStr, "", "   11 - enable prog track logging\n" );
    usbIoPrintf( chan, fmtStr, "", "   12 - begin prog track logging\n" );
    usbIoPrintf( chan, fmtStr, "", "   13 - end prog track logging\n" );
    usbIoPrintf( chan, fmtStr, "", "   14 - print prog track logging data\n" );

    usbIoPrintf( chan, "<?> - list this help\n" );

    usbIoPrintf( chan, "\n" );
}

} // namespace


//----------------------------------------------------------------------------------------
// "initSerialCommand" initializes the command interface. Right now, we only 
// ensure that session and track objects have been created.
//
//----------------------------------------------------------------------------------------
uint8_t initSerialCommand( ) {

    if (( locoSessions == nullptr ) ||
        ( mainTrack    == nullptr ) ||
        ( progTrack    == nullptr )) return( ERR_BASE_STATION_CMD_INIT );

    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "executeSerialCommand" ....
//
//
//----------------------------------------------------------------------------------------
uint8_t executeSerialCommand( uint8_t chan, char *cmdStr ) {

    switch ( cmdStr[ 0 ] ) {

        case 't': setThrottleCmd( chan, cmdStr + 1 );  break;
        case 'F': setFunctionBitCmd( chan, cmdStr + 1 ); break;
        case 'f': setFunctionGroupCmd( chan, cmdStr + 1 ); break;

        case 'T': turnoutCmd( chan, cmdStr + 1 ); break;

        case 'R': readCVCmd( chan, cmdStr + 1 ); break;
        case 'W': writeCVByteCmd( chan, cmdStr + 1 ); break;
        case 'B': writeCVBitCmd( chan, cmdStr + 1 ); break;
        case 'V': verifyCvCmd( chan, cmdStr + 1 ); break;
        case 'w': writeCVByteMainCmd( chan, cmdStr + 1 ); break;
        case 'b': writeCVBitMainCmd( chan, cmdStr + 1 ); break;
       
        case 'M': writeDccPacketMainCmd( chan, cmdStr + 1 ); break;
        case 'P': writeDccPacketProgCmd( chan, cmdStr + 1 ); break;

        case '0': turnPowerOnOffCmd( chan, cmdStr + 1 ); break;
        case '1': turnPowerOnOffCmd( chan, cmdStr + 1); break;
        case '!': emergencyStopCmd( chan ); break;
        case 'c': currentOnTrackCmd( chan, cmdStr + 1 ); break;

        case 'D': handleDCmd( chan, cmdStr + 1 ); break;
        case 'J': handleJCmd( chan, cmdStr + 1 ); break;
        case 'Q': statusOfSensorsCmd( chan, cmdStr + 1 ); break;
        case 'S': numOfDefinedSensorsCmd( chan, cmdStr + 1 ); break;

        case '?': printHelpCmd( chan ); break;
        case 's': printStatusCmd( chan, cmdStr + 1 ); break;
        case '#': numOfCabSupportedCmd( chan, cmdStr + 1 ); break;

        case 'Y': baseStationOptCmd( chan, cmdStr + 1 ); break;
        
        case ' ': usbIoPrintf( chan, "\n" ); break;

        default:  usbIoPrintf( chan, 
                            "<Unknown command, use '?' for help>\n" );
    }

    return( LCS_OK );
}


//----------------------------------------------------------------------------------------
// "handleSerialCommand" analyzes the command line and invokes the respective 
// command handler. The first character in a command is the command letter. The 
// command is followed by the arguments. 
//
// For compatibility with the DCC++ original command set, each command that is 
// also a DCC++ / DCC-EX command is implemented exactly as the original. This 
// allows external tools, such as the JMRI Decoder Pro configuration tool to be
// used. 
//
// The command handler supports command sequences "<" ... ">" in one line which
// are processed once the carriage return is hit. The channel parameter defines
// to which USB channel the output is sent.
//
//----------------------------------------------------------------------------------------
uint8_t handleSerialCommand( uint8_t chan, char *s ) {

    int     charIndex       = 0;
    char    cmdStr[ 256 ]   = { 0 };
    
    while ( s[ charIndex ] != '\0' ) {

        switch ( s[ charIndex ] ) {

            case '<': {

                cmdStr[ 0 ] = '\0';
                charIndex ++;

            } break;

            case '>': {

                executeSerialCommand( chan, cmdStr + 1 );
                charIndex ++;
            
            } break;

            default: {

                if ( strlen( cmdStr ) < sizeof( cmdStr) ) 
                    strncat( cmdStr, &s[ charIndex ], 1 );
                charIndex ++;
            }
        }
    }

    return( LCS_OK );
}

