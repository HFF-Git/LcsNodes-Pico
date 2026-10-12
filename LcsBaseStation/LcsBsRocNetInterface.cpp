//----------------------------------------------------------------------------------------
//
// LCS Base Station - RocRail RocNet RASCII - implementation file
//
//----------------------------------------------------------------------------------------
// The base station supports a RocRail connection. There are quite a few options
// what RocRail accepts as a base station. One popular one is DCC-EX commands. 
// Another one is the RocNet message format and its ASCCII cousin RASCII. This
// file implements the latter. RocRail Protocol Version 2 is required. 
//
// From the LCS command callback, we are passed the RocNet message as an ASCII
// string. All bytes are two hex digits in ASCII.
//
//  "@" <len> <cmd/type> <payload> 
//
// ??? this a bit more complex. There are headers !!!!!!!
//
// The commands supported so far: 
// 
//  Loco command:
//      "@" <len=nn> <cmd=01> <highAdr=nn> <lowAdr=00> <speed=nn> <dir=nn> 
//                            <F0_F4> <F5_F12> <F13_F20> <F21_F28> ...
//      ( len=number of payload + cmd )
//      ( len depends on function map )
//      ( speed 0 -- 127, dir 0 or 1 )
//
// Turnout command:
//      "@" <len=05> <cmd=02> <highAdr=nn> <lowAdr=nn> <port=nn> <stat=nn>
//      ( adr is 16-bit address, port on address )(red)
//      ( state=0 is normal, state=1 is thrown )(green)
//
// Signal command: 
//      ??? 
//
// Track occupancy command:
//
//      "@" <len=04> <cmd=05> <module=nnn> <port=nnn> <state=nn> 
//      ( only 255 modules, 255 port )#
//      ( 0 - free, 1 - occupied ) 
//
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
// A little helper, convert HEX ASCII to int.
//
//----------------------------------------------------------------------------------------
int hexToBin( char c ) {

    if (c >= '0' && c <= '9') return ( c - '0' );
    if (c >= 'A' && c <= 'F') return ( c - 'A' + 10 );
    if (c >= 'a' && c <= 'f') return ( c - 'a' + 10 );
    return -1;
}

//----------------------------------------------------------------------------------------
// RocNet messages start with a "@" followed by a series of bytes encoded as 
// two ASCII hex numbers each. This routines returns a byte array of the message
// and its length.
// 
//----------------------------------------------------------------------------------------
uint8_t convertRocNetMsg( const char *msg, uint8_t *bMsg, size_t bLen ) {

    if (( bMsg == nullptr ) || ( bMsg[ 0 ] != '@' )) return( -1 );
    msg ++;

    size_t count = 0;

    while ( *msg != '\0' ) {

        if (( msg[ 1 ] == '\0' ) || ( count >= bLen )) return( -1 );;

        int hi = hexToBin(msg[ 0 ]);
        int lo = hexToBin(msg[ 1 ]);
        if (( hi < 0 ) || ( lo < 0 )) return( -1 );

        bMsg[ count++ ] = static_cast<uint8_t>(( hi << 4 ) | lo );
        msg += 2;
    }

    return( count );
}

} // namespace

//----------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------
uint8_t initRocNetInterface( ) {

    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------
void handleRocNetMsg( const char *msg  ) {

    uint8_t rocNetMsg[ 128 ];
    uint8_t msgLen;

    msgLen = convertRocNetMsg( msg, rocNetMsg, sizeof( rocNetMsg ));

    // ??? now we have the message. first 8 bytes are the header.
    // rest is payload.



   
    return;
}





//----------------------------------------------------------------------------------------
//
//
// ??? this a bit more complex. There are headers !!!!!!!
//----------------------------------------------------------------------------------------
uint8_t handleRocNetCommand( const char *msg ) {

    // Check the start character.
    if ( msg == nullptr || msg[0] != '@' ) return ( 99 );

    // Decode the payload length (two hexadecimal characters).
    char lenStr[ 3 ] = { msg[ 1 ], msg[ 2 ], '\0' };
    char *end;
    unsigned long dataLength = strtoul(lenStr, &end, 16);

    if ( end != lenStr + 2 || dataLength == 0 || dataLength > 32 )
        return ( 99 );

    // Check that the complete payload is present.
    // Each payload byte is represented by two hexadecimal characters.
    if ( strlen( msg ) < 3 + dataLength * 2 ) return ( 99 );

    // Decode the payload into binary bytes.
    uint8_t payloadBytes[ 32 ];

    for ( size_t i = 0; i < dataLength; i++ ) {
        
        char hexPair[3] = { msg[3 + i * 2], msg[4 + i * 2], '\0' };

        unsigned long value = strtoul(hexPair, &end, 16);

        if (end != hexPair + 2 ) return ( 99 );

        payloadBytes[i] = static_cast<uint8_t>(value);
    }

    // Dispatch the message according to its command type.
    uint8_t cmdType = payloadBytes[ 0 ];

    if ( cmdType == 0x01 ) {

        // LOK-BEFEHL: at least 5 payload bytes are required.
        if (dataLength < 5)
            return false;

        uint16_t dccAddr =
            (static_cast<uint16_t>(payloadBytes[1]) << 8) |
            payloadBytes[2];

        uint8_t speed     = payloadBytes[3];
        uint8_t direction = payloadBytes[4];

        uint8_t f0_f4   = (dataLength >= 6) ? payloadBytes[5] : 0;
        uint8_t f5_f12  = (dataLength >= 7) ? payloadBytes[6] : 0;
        uint8_t f13_f20 = (dataLength >= 8) ? payloadBytes[7] : 0;
        uint8_t f21_f28 = (dataLength >= 9) ? payloadBytes[8] : 0;


        // updateDccSignal(dccAddr, speed, direction,
        //                f0_f4, f5_f12, f13_f20, f21_f28);

    } else if (cmdType == 0x02) {
        
        // WEICHE / SIGNAL: at least 5 payload bytes are required.
        if (dataLength < 5)
            return false;

        uint16_t accessoryAddr =
            (static_cast<uint16_t>(payloadBytes[1]) << 8) |
            payloadBytes[2];

        uint8_t port  = payloadBytes[3];
        uint8_t state = payloadBytes[4];

        // sendToCanBus(accessoryAddr, port, state);
    }

    return true;
}
