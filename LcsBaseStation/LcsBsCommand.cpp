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


// ??? if we make the commands to be called from the console callback and the DCC-EX
// command callback we need a way to say which channel to use for printf....




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
void printVersionInfo( ) {

    printf( "<\nLCS Base Station / Version: tbd / %s %s >\n", 
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
// ??? the JOIN option controls the setting of the PROG track. It switches to 
// MAIN mode. Any loco command, t or F, will reset the PROG track back to 
// service track mode. To do ...
//
//----------------------------------------------------------------------------------------
void turnPowerOnOffCmd( char *s ) {

    uint8_t  powerOnOff   = 0;
    char     trackStr[ 16 ] = { 0 };

    if ( sscanf( s, "%hhu %s", &powerOnOff, trackStr ) < 1 ) return;

    if ( powerOnOff == 1 ) {

        if ( strcmp( trackStr, "MAIN" ) == 0 ) {

            mainTrack -> powerEnable( true );
            printf( "<1 MAIN>" );
        }
        else if ( strcmp( trackStr, "PROG" ) == 0 ) {

            progTrack -> powerEnable( true );
            printf( "<1 PROG>" );
        }
        else if ( strcmp( trackStr, "JOIN" ) == 0 ) {

           // ??? how to best handle this ?
        }
        else {

            mainTrack -> powerEnable( true );
            progTrack -> powerEnable( true );
            printf( "<1>" );
        }
    }
    else {

        if ( strcmp( trackStr, "MAIN" ) == 0 ) {
            
            mainTrack -> powerEnable( false );
            printf( "<0 MAIN>" );
        }
        else if ( strcmp( trackStr, "PROG" ) == 0 ) {
            
            progTrack -> powerEnable( false );
            printf( "<0 PROG>" );
        }
        else {
            
            mainTrack -> powerEnable( false );
            progTrack -> powerEnable( false );
            printf( "<0>" );
        }
    }
}

//----------------------------------------------------------------------------------------
// "setTrackOptionCmd" turns on and off capabilities of the operations or service
// track.
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
//    returns: NONE
//
// Note: this is not a DCC-EX command. I use it for my options. Perhaps one
// day, we replace it by an attribute setting in the base station.
//
//----------------------------------------------------------------------------------------
void setTrackOptionCmd( char *s ) {

    uint8_t option = 0;

    if ( sscanf( s, "%hhu", &option ) == 1 ) {

        switch ( option ) {

            case 1: mainTrack -> setTrackMode( DT_M_CUTOUT ); break;
            case 2: mainTrack -> setTrackMode( DT_M_PLAIN );  break;
            case 3: mainTrack -> setTrackMode( DT_M_RAILCOM );  break;
            case 4: mainTrack -> setTrackMode( DT_M_PLAIN ); break;
            default: ;
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
//----------------------------------------------------------------------------------------
void setThrottleCmd( char *s ) {

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
        if ( rStat != LCS_OK ) printf( "<X>" );
    }
    else if ( cabId == 0 ) {

        // ??? communicate a refresh ...
    }
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
//    returns: NONE.
//
//----------------------------------------------------------------------------------------
void setFunctionBitCmd( char *s ) {

    uint16_t cabId      = NIL_CAB_ID;
    uint8_t  funcNum    = 0;
    uint8_t  val        = 0;

    if ( sscanf( s, "%hu %hhu %hhu", &cabId, &funcNum, &val ) != 3 ) return;

    uint8_t rStat = locoSessions -> setDccFunctionBit( 0, funcNum, val );
    if ( rStat != LCS_OK ) printf( "<X>" );
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
//----------------------------------------------------------------------------------------
void setFunctionGroupCmd( char *s ) {

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

    if ( rStat != LCS_OK ) printf( "<X>" );
}

//----------------------------------------------------------------------------------------
// "emergencyStopCmd" handles the emergencyStop command. This new command causes 
// the base station to send out the emergency stop broadcast DCC command.
//
//    <!>
//
//    returns: <X>
//
//----------------------------------------------------------------------------------------
void emergencyStopCmd( ) {

    locoSessions -> emergencyStopAll( );
    printf( "<!>" );
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
void writeCVByteMainCmd( char *s ) {

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
void writeCVBitMainCmd( char *s ) {

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
//    <R cvId [ callbacknum callbacksub ]>
//
//    cvId          -  the configuration variable ID, 1 ... 1024.
//    callbacknum   -  a number echoed back, ignored by the base station
//    callbacksub   -  a number echoed back, ignored by the base station
//
//    returns: <R callbacknum|callbacksub|cvId value>
//
//    where value is 0 - 255 of the CV variable or -1 if the value could not
//    be verified.
//
// ??? callback is deprecated. Cross check and take it out ?
//
//----------------------------------------------------------------------------------------
void readCVCmd( char *s ) {

    uint16_t  cvId          = NIL_DCC_CV_ID;
    uint8_t   val           = 0;
    int       callbacknum   = 0;
    int       callbacksub   = 0;
    int       ret           = 0;

    if ( sscanf( s, "%hu %d %d", &cvId, &callbacknum, &callbacksub )  < 1 ) return;

    ret = locoSessions -> readCV( cvId, 0, &val );

    printf( "<R %d|%d|%d %d>", callbacknum, callbacksub, cvId, 
            (( ret == LCS_OK ) ? val : -1 ));
}

//----------------------------------------------------------------------------------------
// "writeCVByteCmd" writes a data byte to the engine decoder on the programming 
// track and then verifies it. The callbacknum and callbacksub parameter are 
// ignored and just passed back to the caller for identification purposes.
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
// ??? callback is deprecated. Cross check and take it out ?
//
//----------------------------------------------------------------------------------------
void writeCVByteCmd( char *s ) {

    uint16_t  cvId          = NIL_DCC_CV_ID;
    uint8_t   val           = 0;
    int       callbacknum   = 0;
    int       callbacksub   = 0;
    int       ret           = 0;

    if ( sscanf( s, "%hu %hhu %d %d", 
                 &cvId, &val, &callbacknum, &callbacksub ) < 2 ) return;

    ret = locoSessions -> writeCVByte( cvId, val );

    printf( "<W %d|%d|%d %d>", callbacknum, callbacksub, cvId, 
           (( ret == LCS_OK ) ? val : -1 ));
}

//----------------------------------------------------------------------------------------
// "writeCVBitCmd" writes a bit to the engine decoder on the programming track
// and then verifies the operation. The callbacknum and callbacksub parameter are 
// ignored and just passed back to the caller for identification purposes.
//
//    <B cvId bitPos bitVal [ callbacknum callbacksub ]>
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
// ??? callback is deprecated. Cross check and take it out ?
//
//----------------------------------------------------------------------------------------
void writeCVBitCmd( char *s ) {

    uint16_t  cvId          = NIL_DCC_CV_ID;
    uint8_t   bitPos        = 0;
    uint8_t   bitVal        = 0;
    int       callbacknum   = 0;
    int       callbacksub   = 0;
    int       ret           = 0;

    if ( sscanf( s, "%hu %hhu %hhu %d %d", 
                 &cvId, &bitPos, &bitVal, &callbacknum, &callbacksub ) != 5 ) 
        return;

    ret = locoSessions -> writeCVBit( cvId, bitPos, bitVal );

    printf( "<B %d|%d|%d|%d %d>", callbacknum, callbacksub, cvId, bitPos, 
           (( ret == LCS_OK ) ? bitVal : -1 ));
}

//----------------------------------------------------------------------------------------
//
//  <v cvId val>
//
//  cvId          -  the configuration variable ID, 1 ... 1024.
//  val           -  the data data value to expect.
//
//  returns: 
//
//      
//----------------------------------------------------------------------------------------
void verifyCvCmd( char *s ) {

    uint16_t  cvId      = NIL_DCC_CV_ID;
    uint8_t   testVal   = 0;
    int       ret       = 0;

    if ( sscanf( s, "%hu %hhu", &cvId, &testVal ) != 2 )  return;

    // ret = locoSessions -> verifyCv( cvId, bitPos, bitVal );

}

//----------------------------------------------------------------------------------------
//
//  <V cvId bitPos bitVal>
//
//  cvId          -  the configuration variable ID, 1 ... 1024.
//  bitPos        -  the bit position of the bit, 0 .. 7.
//  bitVal        -  the data bit.
//
//  returns: 
//
//
//----------------------------------------------------------------------------------------
void verifyCvBitCmd( char *s ) {

    uint16_t    cvId    = NIL_DCC_CV_ID;
    uint8_t     bitPos  = 0;
    uint8_t     testVal = 0;
    int         ret     = 0;

    if ( sscanf( s, "%hu  %hhu %hhu", &cvId, &bitPos, &testVal ) != 3 )  return;

    // ret = locoSessions -> verifyCvBit( cvId, bitPos, bitVal );

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
void writeDccPacketMainCmd( char *s ) {

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
void writeDccPacketProgCmd( char *s ) {

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
//----------------------------------------------------------------------------------------
void handleDCmd( char *s ) {

    char argStr[ 16 ] = { 0 };

    if ( sscanf( s, "%s", argStr ) < 1 ) return;

    if ( strcmp( argStr, "RESET" ) == 0 ) {

        // ??? reset the base station ...
    }
    else {

        printf( "<X>" );
    }
}

//----------------------------------------------------------------------------------------
// "dccLogCommand" is the command to manage the DCC log for tracing and debugging
// purposes.
//
//  <Y [ opt ]> where "opt" is the command to execute from the DCC Log function.
//
//  Main track: 
//
//      0 - disable DCC logging
//      1 - enable DCC logging
//      2 - start DCC logging
//      3 - stop DCC logging
//      4 - list log entries
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
void dccLogCommand( char *s ) {

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
//----------------------------------------------------------------------------------------
void currentOnTrackCmd( char *s ) {

    printf( "CurrentMAIN %d C Milli 0 %d 1 %d>",
            mainTrack -> getActualCurrent( ), 0, 0 );  // ??? fix ...
}

//----------------------------------------------------------------------------------------
// "printStatusCmd" list information about the base station. Using just a "s" 
// for a summary status is always a good idea to do this just as a first basic
// test if things are running at all. The level is a positive integer that 
// specifies the information items to be listed.
//
//    <s [ opt ]> - the kind of status to display.
//
//    returns:  series of status information that can be read by an interface 
//              to determine status of the base station and important settings
//
// ??? changes... some prints change...
//----------------------------------------------------------------------------------------
void printStatusCmd( char *s ) {

    uint8_t opt = 0;

    if ( sscanf( s, "%hhu", &opt ) > 0 ) {

        switch ( opt ) {

            case 1: {

                printVersionInfo( );
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

            default: printVersionInfo( );
        }
    } 
    else printVersionInfo( );
}

//----------------------------------------------------------------------------------------
// "printHelp" lists a short version of all the command.
//
// ??? cleanup ...
//----------------------------------------------------------------------------------------
void printHelpCmd( ) {

    printf( "\nCommands:\n" );

    printf( "<t cabId speed dir>                "
            "- set cab speed / direction\n" );
    printf( "<f cabId funcId val >                  "
            "- set cab function value, group DCC format\n" );
    printf( "<F cabId funcId val >                    "
            "- set cab function value, individual\n" );
    printf( "<R cvId callbacknum callbacksub >      "
            "- read CV byte\n" );
    printf( "<W cvId val callbacknum callbacksub>   "
            "- write CV byte on programming track\n" );
    printf( "<B cvId bitPos bitVal callbacknum callbacksub> "
            "- write CV bit on programming track\n" );
    printf( "<w cabId cvId val > "
            "- write CV byte on operations track\n" );
    printf( "<b cabId cvId bitPos bitVal > "
            "- write CV bit on operations track\n" );
    printf( "<M sId byte1 byte2 [ byte3 ... byte10 ]> "
            "- send DCC packet on operations track to Reg n\n" );
    printf( "<P sId byte1 byte2 [ byte3 ... byte10 ]> "
            "- send DCC packet on programming track to Reg n\n" );

    printf( "<C track [option] "
            "- set track option, track = 0 -> MAIN, track = 1 -> PROG\n" );
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

    printf( "<a [ opt ]>  " 
            " - list current consumption, default is RMS for MAIN\n" );

    printf( "             " "   - opt 0  - actual - MAIN\n" );
    printf( "             " "   - opt 1  - actual - PROG\n" );
    printf( "             " "   - opt 2  - actual - both\n" );
    printf( "             " "   - opt 10 - RMS - MAIN\n" );
    printf( "             " "   - opt 11 - RMS - PROG\n" );
    printf( "             " "   - opt 12 - RMS - both\n" );

    printf( "<C <option>> "
            "- turn on/off the Railcom option on the main track( 0 - off, 1 - on)\n" );

    printf( "<s [ level ]>" " - list status at detail level, default is summary\n" );
    printf( "             " "   - level 0 - summary\n" );
    printf( "             " "   - level 1 - configuration\n" );
    printf( "             " "   - level 2 - session map\n" );
    printf( "             " "   - level 3 - main track current\n" );
    printf( "             " "   - level 4 - prog track current\n" );
    printf( "             " "   - level 9 - all of the above\n" );

    printf( "<S> - list base station configuration\n" );
    printf( "<L> - list base station session table" );

    printf( "<Y [ opt ]> - DCC log options ( used for debugging and tracing )\n" );
    printf( "             " "   - 0  - disable main track logging\n" );
    printf( "             " "   - 1  - enable main track logging\n" );
    printf( "             " "   - 2  - begin main track logging\n" );
    printf( "             " "   - 3  - end main track logging\n" );
    printf( "             " "   - 4  - print main track logging data\n" );
    printf( "             " "   - 10 - disable prog track logging\n" );
    printf( "             " "   - 11 - enable prog track logging\n" );
    printf( "             " "   - 12 - begin prog track logging\n" );
    printf( "             " "   - 13 - end prog track logging\n" );
    printf( "             " "   - 14 - print prog track logging data\n" );

    printf( "<?> - list this help\n" );

    printf( "\n" );
}

} // namespace


//----------------------------------------------------------------------------------------
// "initSerialCommand" initializes the command interface. Right now, we only make
// sure that session and track objects have been created.
//
//----------------------------------------------------------------------------------------
uint8_t initSerialCommand( ) {

    if (( locoSessions == nullptr ) ||
        ( mainTrack    == nullptr ) ||
        ( progTrack    == nullptr )) return( ERR_BASE_STATION_CMD_INIT );

    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "handleSerialCommand" analyzes the command line and invokes the respective 
// command handler. The first character in a command is the command letter. The 
// command is followed by the arguments. For compatibility with the DCC++ original
// command set, each command that is also a DCC++ command is implemented exactly 
// as the original. This allows external tools, such as the JMRI Decoder Pro 
// configuration tool to be used. The command handler supports command sequences
// "<" ... ">" in one line which are processed once the carriage return is hit.
//
//----------------------------------------------------------------------------------------
void handleSerialCommand( char *s ) {

    int     charIndex       = 0;
    char    cmdStr[ 256 ]   = { 0 };
    
    while ( s[ charIndex ] != '\0' ) {

        switch ( s[ charIndex ] ) {

            case '<': {

                cmdStr[ 0 ] = '\0';
                charIndex ++;

            } break;

            case '>': {

                switch ( cmdStr[ 0 ] ) {

                    case 't': setThrottleCmd( cmdStr + 1 );  break;
                    case 'F': setFunctionBitCmd( cmdStr + 1 ); break;
                    case 'f': setFunctionGroupCmd( cmdStr + 1 ); break;

                    case 'R': readCVCmd( cmdStr + 1 ); break;
                    case 'W': writeCVByteCmd( cmdStr + 1 ); break;
                    case 'B': writeCVBitCmd( cmdStr + 1 ); break;
                    case 'V': verifyCvCmd( cmdStr + 1 ); break;
                    case 'w': writeCVByteMainCmd( cmdStr + 1 ); break;
                    case 'b': writeCVBitMainCmd( cmdStr + 1 ); break;
                    case 'v': verifyCvBitCmd( cmdStr + 1 ); break;

                    case 'M': writeDccPacketMainCmd( cmdStr + 1 ); break;
                    case 'P': writeDccPacketProgCmd( cmdStr + 1 ); break;

                    case '0': turnPowerOnOffCmd( cmdStr + 1 ); break;
                    case '1': turnPowerOnOffCmd( cmdStr + 1); break;
                    case '!': emergencyStopCmd( ); break;
                    case 'c': currentOnTrackCmd( cmdStr + 1 ); break;

                    case '?': printHelpCmd( ); break;
                    case 's': printStatusCmd( cmdStr + 1 ); break;
                 
                    case 'C': setTrackOptionCmd( cmdStr + 1 ); break;
                    case 'Y': dccLogCommand( cmdStr + 1 ); break;
                    
                    case ' ': printf( "\n" ); break;

                    default:  printf( "<Unknown command, use '?' for help>\n" );
                }

                charIndex ++;
            
            } break;

            default: {

                if ( strlen( cmdStr ) < sizeof( cmdStr) ) 
                    strncat( cmdStr, &s[ charIndex ], 1 );
                charIndex ++;
            }
        }
    }
}

//----------------------------------------------------------------------------------------
// ??? "handleSerialCommand" implements the commands the base station command
// interface supports. We stick to DCC-EX as best as we can. In addition there
// needs to be a "handleDccEXCommand" routine that offers the DCC-EX commands
// used by RocRail. The command set is a subset of the general commands of the 
// base station plus some additional commands from the DCC-EX set to support.
//
// The command handler supports the same command sequences "<" ... ">" in one 
// line which are processed once the carriage return is hit.

void handleDccExCommands( char *s ) {


}

