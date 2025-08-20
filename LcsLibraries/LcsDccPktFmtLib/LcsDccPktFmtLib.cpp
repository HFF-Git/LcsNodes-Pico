//----------------------------------------------------------------------------------------
//
// LCS Dcc Packet Formatter - implementation file
//
///---------------------------------------------------------------------------------------
// This file contains the DCC formatter methods. The routines are used to display a DCC 
// packet in human readable format. There are methods that analyze a DCC packet for 
// length, checksum and instruction type. The formatting routines will build a string 
// with a binary, hexadecimal or content formatted data.
//
///---------------------------------------------------------------------------------------
//
// LCS - DCC Packet Formatter
// Copyright (C) 2021 - 2024  Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details. You should
// have received a copy of the GNU General Public License along with this program. 
// If not, see <http://www.gnu.org/licenses/>.
//
//----------------------------------------------------------------------------------------
#include "LcsDccPktFmtLib.h"
#include <pico/stdio.h>
#include <pico/stdlib.h>

///---------------------------------------------------------------------------------------
// The local namespace contains the routines that actually produce most of the formatted
// string. The routines are all built with the "sprintf" function and return the numbers
// of characters generated in the passed line buffer.
//
///---------------------------------------------------------------------------------------
namespace {

///---------------------------------------------------------------------------------------
// A DCC packets is at least two bytes long. The maximum size is 12 bytes for an XPOM 
// packet. The NMRA folks currently set a maximum of 6 bytes though, which was the limit
// before XPOM support.
//
///---------------------------------------------------------------------------------------
const uint8_t     MIN_DCC_PACKET_SIZE = 2;
const uint8_t     MAX_DCC_PACKET_SIZE = 12;

///---------------------------------------------------------------------------------------
// The string buffer needs to be large enough to accommodate the string produced. A DCC
// packet can be up to 11 bytes long, including the checksum. We will just check that 
// the buffer is large enough using the maximum sizes possible, although the maximum size
// is rarely needed.
//
//  HEX packet format:        11 * 5 chars + 3 = 58 chars
//  BIN packet format:        11 * 9 chars + 3 = 102 chars
//  Formatted packet format:  64 chars
//
///---------------------------------------------------------------------------------------
const uint8_t DCC_PACKET_BUF_IN_FMT = 64;
const uint8_t DCC_PACKET_BUF_IN_HEX = 58;
const uint8_t DCC_PACKET_BUF_IN_BIN = 102;

///---------------------------------------------------------------------------------------
// Some utility functions.
//
///---------------------------------------------------------------------------------------
bool isInRangeU( uint8_t val, uint8_t lower, uint8_t upper ) {

    return (( val >= lower ) && ( val <= upper ));
}

uint8_t bitRead( uint8_t arg, uint8_t pos ) {

    return ( arg >> ( pos % 8 )) & 1;
}

int valueToTokenStr( const char *buf, uint8_t val,
                     const char *trueStr, const char *falseStr ) {

    return ( sprintf((char *) buf, ( val ? ((char *) trueStr ) : ((char *) falseStr ))));
} 

uint8_t dccPacketInstrByte( uint8_t *dccPkt ) {

    if      ( isInRangeU( dccPkt[ 1 ],   1, 127 )) return ( dccPkt[ 2 ] );
    else if ( isInRangeU( dccPkt[ 1 ], 192, 231 )) return ( dccPkt[ 3 ] );
    else return ( 0 );
}

uint8_t dccPacketAddressLength( uint8_t *dccPkt ) {

    if      ( isInRangeU( dccPkt[ 1 ],   1, 127 )) return ( 1 );
    else if ( isInRangeU( dccPkt[ 1 ], 128, 231 )) return ( 2 );
    else return ( 0 );
} 

bool validDccPkt( uint8_t *dccPkt ) {

    if ( isInRangeU( dccPkt[ 0 ], MIN_DCC_PACKET_SIZE, MAX_DCC_PACKET_SIZE )) {

        uint8_t dccPktLen = dccPkt[ 0 ];
        uint8_t chkSum    = 0;

        for ( uint8_t n = 1; n < dccPktLen; n++ ) chkSum ^= dccPkt[ n ];
        return ( chkSum == dccPkt[ dccPktLen ] );
    }
    else return ( false );
} 

bool idlePacket( uint8_t *dccPkt ) {

    return (( dccPkt[ 0 ] == 3 ) && ( dccPkt[ 1 ] == 0xff ) &&
            ( dccPkt[ 2 ] == 0 ) && ( dccPkt[ 3 ] == 0xff ));
}

bool resetPacket( uint8_t *dccPkt ) {

    return (( dccPkt[ 0 ] == 3 ) && ( dccPkt[ 1 ] == 0 ) &&
            ( dccPkt[ 2 ] == 0 ) && ( dccPkt[ 3 ] == 0 ));
} 

///---------------------------------------------------------------------------------------
// "speed28ToStr" returns a string with the speed information decoded for the 28 speed 
// step model. The function returns the number of characters in the string buffer.
//
///---------------------------------------------------------------------------------------
int speed28ToStr( char *buf, uint8_t dataByte ) {

    if      (( dataByte & 0b00001111 ) == 0 )  return ( sprintf( buf, "Stop " ));
    else if (( dataByte & 0b00001111 ) == 1 )  return ( sprintf( buf, "Estp " ));
    else {

        int speed   = (( dataByte & 0b00001111 ) << 1 ) - 3 + bitRead( dataByte, 4 );
        int dir     = ( bitRead( dataByte, 5 ));

        if ( dir ) return ( sprintf( buf, "Fwd %02d ", speed ));
        else       return ( sprintf( buf, "Rev %02d ", speed ));
    }
}

///---------------------------------------------------------------------------------------
// "speed128ToStr" returns a string with the speed information decoded for the 128 speed 
// step model. The function returns the number of characters in the string buffer.
//
///---------------------------------------------------------------------------------------
int speed128ToStr( char *buf, uint8_t dataByte ) {

    int speed   = dataByte & 0b01111111;
    int dir     = bitRead( dataByte, 7 );

    if      ( speed == 0 )  return ( sprintf( buf, "Stop " ));
    else if ( speed == 1 )  return ( sprintf( buf, "Estp " ));
    else if ( speed < 127 ) {

        if ( dir ) return ( sprintf( buf, "Fwd %03d ", speed ));
        else       return ( sprintf( buf, "Rev %03d ", speed ));
    }
    else return ( 0 );
} 

///---------------------------------------------------------------------------------------
// "decoderControlStr" implements the instruction group zero ( 000x-xxxx ) formatting. 
// There are commands to reset the controller as well as managing the consist function
// in this group. The function returns the number of characters in the string buffer.
//
///---------------------------------------------------------------------------------------
int decoderControlStr( char *buf, uint8_t *dccPkt ) {

    uint8_t dccPktLen = dccPkt[ 0 ];
    uint8_t dccInstr  = dccPacketInstrByte( dccPkt );

    switch ( dccInstr & 0b00011111 ) {

        case 0b00000: return ( sprintf( buf, "Decoder Reset " ));
        case 0b00001: return ( sprintf( buf, "Decoder Hard Reset " ));

        case 0b00010:
        case 0b00011: return ( sprintf( buf, "Factory test" ));

        case 0b01010: return ( sprintf( buf, "Clear CV #29 Bit 5 " ));
        case 0b01011: return ( sprintf( buf, "Set CV #29 Bit 5 " ));

        case 0b01111: return ( sprintf( buf, "Decoder Ack Request " ));

        case 0b10010: {

            uint8_t consistAdr = dccPkt[dccPktLen - 1] & 0b01111111;
            if ( consistAdr == 0 )  return ( sprintf( buf, "deactivate " ));
            else          return ( sprintf( buf, "activate normal %d ", consistAdr ));
        }

        case 0b10011: {

            uint8_t consistAdr = dccPkt[dccPktLen - 1] & 0b01111111;
            if ( consistAdr == 0 )  return ( sprintf( buf, "deactivate " ));
            else        return ( sprintf( buf, "activate opposite %d ", consistAdr ));
        }

        default: return ( sprintf( buf, "Reserved: 0x%x02 ", dccInstr ));
    }
}

///---------------------------------------------------------------------------------------
// "decoderExtendedInstructionsStr" implements the instruction group one ( 001x-xxxx ) 
// formatting. There are all kinds of instructions in this group to control the loco 
// speed, direction and functions. The function returns the number of characters in the
// string buffer.
//
///---------------------------------------------------------------------------------------
int decoderExtendedInstructionsStr( char *buf, uint8_t *dccPkt ) {

    uint8_t dccPktLen = dccPkt[ 0 ];
    uint8_t dccInstr  = dccPacketInstrByte( dccPkt );

    switch ( dccInstr & 0b00011111 ) {

        case 0b11100: {

            int cursor   = 0;
            int pktIndex = (( dccPkt[1] & 0b10000000 ) ? 4 : 3 );

            cursor += speed128ToStr( buf + cursor, dccPkt[pktIndex] );
            pktIndex ++;

            cursor += sprintf( buf + cursor, " F0: 0x%02x ", ( dccPkt[pktIndex] ));
            pktIndex ++;

            if ( pktIndex < dccPktLen ) {

                cursor += sprintf( buf + cursor, " F8: 0x%02x ", ( dccPkt[pktIndex] ));
                pktIndex ++;
            }

            if ( pktIndex < dccPktLen ) {

                cursor += sprintf( buf + cursor, " F16: 0x%02x ", ( dccPkt[pktIndex] ));
                pktIndex ++;
            }

            if ( pktIndex < dccPktLen ) {

                cursor += sprintf( buf + cursor, " F24: 0x%02x ", ( dccPkt[pktIndex] ));
            }

            return ( cursor );
        }

        case 0b11101: return ( sprintf( buf, "Analog  0x%02x:0x%02x ", 
                                        dccPkt[dccPktLen - 2], dccPkt[dccPktLen - 1] ));
        case 0b11110: return ( sprintf( buf, "Special Op Mode 0x%02x ", 
                                        dccPkt[dccPktLen - 1] ));
        case 0b11111: return ( speed128ToStr( buf, dccPkt[dccPktLen - 1] ));
        default:      return ( sprintf( buf, "Reserved: 0x%x02 ", dccInstr ));
    }
} 
  
///---------------------------------------------------------------------------------------
// This function formats the F0 to F4 function setting instruction. Some older version 
// used F0 as the lights on function. We show the bits in the format "L F4..F1". The 
// function returns the number of characters in the string buffer.
//
///---------------------------------------------------------------------------------------
int decoderF0F4GroupStr( char *buf, uint8_t instrByte ) {

    return ( sprintf( buf, "L F4-F1 0x%02x ", ( instrByte & 0b00011111 )));
} 

///---------------------------------------------------------------------------------------
// This function formats the F5 to F12 function setting instruction. The order is 
// "F8 - F5" and "F12 - F9". Instruction bit 4 select the respective group. The function
// returns the number of characters in the text buffer.
//
///---------------------------------------------------------------------------------------
int decoderF5F12GroupStr( char *buf, uint8_t instrByte ) {

    if ( bitRead( instrByte, 4 ))  
        return ( sprintf( buf, "F8-F5 0x%02x ", ( instrByte & 0b00001111 )));
    else                           
        return ( sprintf( buf, "F12-F9 0x%02x ", ( instrByte & 0b00001111 )));
} 

///---------------------------------------------------------------------------------------
// "decoderExtendedAttributesStr" implements the instruction group six ( 110x-xxxx ) 
// formatting. This group  contains binary state commands, function setting commands and 
// command to set model time. The function returns the number of characters in the 
// string buffer.
//
///---------------------------------------------------------------------------------------
int decoderExtendedAttributesStr( char *buf, uint8_t *dccPkt ) {

    int     cursor    = 0;
    uint8_t dccPktLen = dccPkt[ 0 ];
    uint8_t dccInstr  = dccPacketInstrByte( dccPkt );

    switch ( dccInstr & 0b00011111 ) {

        case 0b00001: {

            return ( sprintf( buf + cursor, "ModelTime 0x%02x 0x%02x 0x%02x ",
                              dccPkt[dccPktLen - 3], 
                              dccPkt[dccPktLen - 2], 
                              dccPkt[dccPktLen - 1]));
        }

        case 0b00010: {

            return ( sprintf( buf + cursor, "SysTime 0x%02x 0x%02x 0x%02x 0x%02x ",
                              dccPkt[dccPktLen - 4], dccPkt[dccPktLen - 3],
                              dccPkt[dccPktLen - 2], dccPkt[dccPktLen - 1]));
        }

        case 0b00000: {

            cursor += sprintf( buf + cursor, "BinStateLong %d ", 
                                256 * dccPkt[dccPktLen - 1] +
                                ( dccPkt[dccPktLen - 2] & 0b01111111 ));

            cursor += valueToTokenStr(  buf + cursor, 
                                        bitRead( dccPkt[dccPktLen - 2], 7 ), 
                                        "On ", "Off " );
            return ( cursor );
        }

        case 0b11101: {

            cursor += sprintf( buf + cursor, "BinStateShort %d ", 
                                ( dccPkt[dccPktLen - 1] & 0b01111111 ));

            cursor += valueToTokenStr( buf + cursor, 
                                        bitRead( dccPkt[dccPktLen - 1], 7 ), 
                                        "On ", "Off " );
            return ( cursor );
        }

        case 0b11110: 
            return ( sprintf( buf, " F20-F13 0x%02x ", ( dccPkt[dccPktLen - 1] )));
        case 0b11111: 
            return ( sprintf( buf, " F28-F21 0x%02x ", ( dccPkt[dccPktLen - 1] )));
        case 0b11000: 
            return ( sprintf( buf, " F29-F36 0x%02x ", ( dccPkt[dccPktLen - 1] )));
        case 0b11001: 
            return ( sprintf( buf, " F37-F44 0x%02x ", ( dccPkt[dccPktLen - 1] )));
        case 0b11010: 
            return ( sprintf( buf, " F45-F52 0x%02x ", ( dccPkt[dccPktLen - 1] )));
        case 0b11011: 
            return ( sprintf( buf, " F53-F60 0x%02x ", ( dccPkt[dccPktLen - 1] )));
        case 0b11100: 
            return ( sprintf( buf, " F61-F68 0x%02x ", ( dccPkt[dccPktLen - 1] )));
        default:      
            return ( sprintf( buf, " Reserved: 0x%02x ", dccInstr ));
    }
 } 

///---------------------------------------------------------------------------------------
// "decoderOtmCvProgrammingStr" implements the instruction group seven ( 1110-xxxx ) 
// formatting for on the main track programming. There is a long and a short form of 
// this command. The short form only applies to the operations mode, only for locomotives, 
// and only a few CVs can be accessed. Also, consist addresses are not allowed. The short
// form is two bytes in length. The long form applies to both locomotives and accessory
// decoders. This form is three bytes in length.
//
// In addition, there is the XPOM instruction, which allows in operations mode only a 
// quick access to the range of CVs that are otherwise only accessible via CV31/CV32. 
// XPOM allows to access a linear address range of 16Mb and the access of up to 4 data 
// bytes in one instruction. The instruction looks identical to the CV long access form,
// except for the length of the packet. If the instruction part is greater than 3 bytes,
// i.e 4 or 5 bytes with the address, we will decode an XPOM instruction. The function 
// returns the number of characters in the string buffer.
//
///---------------------------------------------------------------------------------------
int decoderOtmCvProgrammingStr( char *buf, uint8_t *dccPkt ) {

    uint8_t dccPktLen = dccPkt[ 0 ];
    uint8_t dccInstr  = dccPacketInstrByte( dccPkt );
    uint8_t dccAdrLen = dccPacketAddressLength( dccPkt );

    if ( dccInstr & 00010000 ) {

        switch ( dccInstr & 0b00011111 ) {

            case 0b10010:  
                return ( sprintf( buf, "CV 23 %d", ( dccPkt[ dccPktLen - 1 ] )));
            
            case 0b10011:  
                return ( sprintf( buf, "CV 24 %d", ( dccPkt[ dccPktLen - 1 ] )));
            
            case 0b10100: 
                return ( sprintf( buf, "CV 17,18 %d %d",
                            ( dccPkt[ dccPktLen - 2 ] ), ( dccPkt[ dccPktLen - 1 ] )));
            
            case 0b10101:  
                return ( sprintf( buf, "CV 31,32 %d %d",
                            ( dccPkt[ dccPktLen - 2 ] ), ( dccPkt[ dccPktLen - 1 ] )));
             
            default:       
                return ( sprintf( buf, "Reserved: %x02 ", dccInstr ));
        }
    }
    else if (( dccPktLen - dccAdrLen ) < 5 ) {

        int  cvAddress  = 256 * ( dccInstr & 0b00000011 ) + dccPkt[ dccPktLen - 2 ] + 1;
        uint8_t cvMode     = ( dccInstr & 0b00001100 ) >> 2;
        uint8_t cvData     = dccPkt[ dccPktLen - 1 ];

        switch ( cvMode ) {

            case 1:  return ( sprintf( buf, "CV %d Verify 0x%02x", cvAddress, cvData ));
            case 3:  return ( sprintf( buf, "CV %d Write 0x%02x", cvAddress, cvData ));

            case 2: {

                if ( dccPkt[ dccPktLen - 2 ] & 0b00010000 ) {

                    return ( sprintf( buf, "CV %d Bit write %d=%d", cvAddress,
                                    ( cvData & 0x07 ), (( cvData & 0x08 ) >> 3 )));
                }
                else {

                    return ( sprintf( buf, "CV %d Bit verify %d=%d", cvAddress,
                                    ( cvData & 0x07 ), (( cvData & 0x08 ) >> 3 )));
                }
            }

            default: return ( 0 );
        }
    }
    else {

        unsigned long cvAddress =
            ((( dccPkt[ dccAdrLen + 1 ] * 256 ) + 
            dccPkt[ dccAdrLen + 2 ] ) * 256 ) + dccPkt[ dccAdrLen + 3 ];

        uint8_t       cvMode    = ( dccInstr & 0b00001100 ) >> 2;
        uint8_t       cvData    = dccPkt[ dccPktLen - 1 ];
        uint8_t       seqNum    = ( dccInstr & 0b00000011 );
        uint8_t       cursor    = 0;

        switch ( cvMode ) {

            case 1:  {

                return ( sprintf( buf + cursor, "CV %lu : %d, Read via RailCom ", 
                             cvAddress, seqNum ));
            }

            case 3:  {

                cursor += sprintf( buf + cursor, "CV %lu : %d, Write ", 
                                   cvAddress, seqNum );

                for ( uint8_t i = dccAdrLen + 5; i < dccPktLen - 1; i++ ) {

                    cursor += sprintf( buf + cursor, "0x%02x ", dccPkt[ i ] );
                }

                return ( cursor );
            }

            case 2: {

                if ( dccPkt[ dccPktLen - 1 ] & 0b00010000 ) {

                    return ( sprintf( buf + cursor,  "CV %lu Bit write %d=%d",
                            cvAddress, ( cvData & 0x07 ), (( cvData & 0x08 ) >> 3 )));
                }
                else {

                    return ( sprintf( buf + cursor, "CV %lu Bit verify %d=%d",
                            cvAddress, ( cvData & 0x07 ), (( cvData & 0x08 ) >> 3 )));
                }
            }

            default: return ( 0 );
        }
    }
} 

///---------------------------------------------------------------------------------------
// "decoderSvcModeCvProgrammingStr" implements the instruction group seven ( 0111-xxxx )
// formatting for the programming track programming. Programming on the programming track
// uses no address and hence there is an overlap in the meaning of the first  uint8_t. 
// This routine is called when we are in SHOW_SCV_MODE. There is also an older access 
// mode called register mode access. Although it should not be used anymore, we will 
// still decode it. The old instruction is two bytes, the newer long form is three bytes
// in length. The function returns the number of characters in the string buffer.
//
///---------------------------------------------------------------------------------------
int decoderSvcModeCvProgrammingStr( char *buf, uint8_t *dccPkt ) {

    uint8_t dccPktLen = dccPkt[ 0 ];

    if ( dccPktLen == 2 ) {

        int       cvAddress     = ( dccPkt[ 1 ] & 0b00000111 ) + 1;
        uint8_t   cvMode        = ( dccPkt[ 1 ] & 0b00000100 ) >> 3;
        uint8_t   cvData        = dccPkt[ dccPktLen - 1 ];

        if ( cvMode == 1 )  
            return ( sprintf( buf, "CV %d Write 0x%02x ", cvAddress, cvData ));
        else       
            return ( sprintf( buf, "CV %d Verify 0x%02x ", cvAddress, cvData ));
    }
    else {

        int       cvAddress     = 256 * ( dccPkt[ 1 ] & 0b00000011 ) + dccPkt[ 2 ] + 1;
        uint8_t   cvMode        = ( dccPkt[ 1 ] & 0b00001100 ) >> 2;
        uint8_t   cvData        = dccPkt[ 3 ];

        switch ( cvMode ) {

            case 1:  return ( sprintf( buf, "CV %d Verify 0x%02x ", cvAddress, cvData ));
            case 3:  return ( sprintf( buf, "CV %d Write 0x%02x ", cvAddress, cvData ));

            case 2: {

                if ( dccPkt[ 3 ] & 0b00010000 ) {

                    return ( sprintf( buf, "CV %d Bit write %d=%d ",
                            cvAddress, ( cvData & 0x07 ),  (( cvData & 0x08 ) >> 3 )));
                }
                else {

                    return ( sprintf( buf, "CV %d Bit verify %d=%d ",
                            cvAddress, ( cvData & 0x07 ), (( cvData & 0x08 ) >> 3 )));
                }
            }

            default: return ( 0 );
        }
    }
} 

///---------------------------------------------------------------------------------------
// "decoderAccessoryStr" implements the formatting of an accessory decoder command. 
// Accessory decoders use the address bytes to encoder also part of the command in the
// second address  uint8_t. Bit 7 selects between a simple and an extended accessory 
// decoder. Bit 3 is the activation bit, bit 0 the pair selection bit. The function 
// returns the number of characters in the string buffer.
//
///---------------------------------------------------------------------------------------
int decoderAccessoryStr( char *buf, uint8_t *dccPkt ) {

    int       cursor  = 0;
    uint8_t   byte1   = dccPkt[ 1 ];
    uint8_t   byte2   = dccPkt[ 2 ];
    uint16_t  address = ((( ~ byte2 & 0x70 ) >> 4 ) * 256 ) + 
                        (( byte1 & 0x3F ) << 2 ) + (( byte2 & 0x06 ) >> 1 );

    if ( dccPkt[2] & 0b10000000 ) {

        cursor += sprintf( buf + cursor, "Acc %d:%d ", address, bitRead( dccPkt[2], 0 ));
        cursor += valueToTokenStr( buf + cursor, bitRead( dccPkt[2], 3 ), " On ", " Off " );
    }
    else {

        cursor += sprintf( buf + cursor, "Acc %d Ext 0x%02x ", address, dccPkt[3] );
    }

    return ( cursor );
} 

///---------------------------------------------------------------------------------------
// "decoderLocoStr" formats a DCC packet for a loco. All we do in this function is to 
// branch to the instruction group handler. The function returns the number of characters
// in the string buffer.
//
///---------------------------------------------------------------------------------------
int decoderLocoStr( char *buf, uint8_t *dccPkt ) {

    int         cursor      = 0;
    uint8_t     byte1       = dccPkt[ 1 ];
    uint8_t     byte2       = dccPkt[ 2 ];
    uint8_t     dccInstr    = dccPacketInstrByte( dccPkt );

    if ( isInRangeU( byte1, 1, 127 )) {

        cursor += sprintf( buf + cursor, "Loc %d ", byte1 & 0b01111111 );
    }
    else if ( isInRangeU( byte1, 192, 231 )) {

        cursor += sprintf( buf + cursor, 
                            "Loc %d ", (( byte1 & 0b00111111 ) * 256 ) + byte2 );
    }

    switch ( dccInstr >> 5 ) {

        case 0:   cursor += decoderControlStr( buf + cursor, dccPkt ); break;
        case 1:   cursor += decoderExtendedInstructionsStr( buf + cursor, dccPkt ); break;

        case 2:
        case 3:   cursor += speed28ToStr( buf + cursor, dccInstr ); break;

        case 4:   cursor += decoderF0F4GroupStr( buf + cursor, dccInstr ); break;
        case 5:   cursor += decoderF5F12GroupStr( buf + cursor, dccInstr ); break;
        case 6:   cursor += decoderExtendedAttributesStr( buf + cursor, dccPkt ); break;
        case 7:   cursor += decoderOtmCvProgrammingStr( buf + cursor, dccPkt ); break;
    }

    return ( cursor );
} 

///---------------------------------------------------------------------------------------
// "dccOperationsPacketStr" formats an operations packet. An operations packet has an 
// address followed by the command and perhaps arguments. The function returns the number
// of characters in the string buffer.
//
///---------------------------------------------------------------------------------------
int dccOperationsPacketStr( char *buf, uint8_t *dccPkt ) {

    int     cursor  = 0;
    uint8_t fByte   = dccPkt[ 1 ];

    if (( isInRangeU( fByte, 1, 127 )) || ( isInRangeU( fByte, 192, 231 ))) {

        cursor += decoderLocoStr( buf + cursor, dccPkt );
    }
    else if ( isInRangeU( fByte, 128, 191 )) {

         cursor += decoderAccessoryStr( buf + cursor, dccPkt );
    }
    else if ( isInRangeU( fByte, 232, 254 )) {

        // ??? decode 254,  check RailCommunity docs standard....

        cursor += sprintf( buf + cursor, "Reserved address 0x%02x ", fByte );
    }


    // ??? also, need to format the base station attributes, system time data etc ...


    return ( cursor );
}

///---------------------------------------------------------------------------------------
// "dccPacketToStr" builds a string for the DCC packet passed. There are two basic modes.
// The first is the regular DCC packet decoding on the main track, i.e. the first uint8_t
// is part of an address. Decoding will branch based on the address range found. The 
// second mode is the service mode for decoding DCC packets on the programming track, 
// where there is no address. For both modes, there is an option to list the packet in 
// HEX and BINARY. The function returns the number of characters to the passed buffer 
// appended.
//
///---------------------------------------------------------------------------------------
int dccPacketToStr( char *buf, uint8_t *dccPkt, bool svcMode = false ) {

    int cursor  = 0;

    if ( ! isInRangeU( dccPkt[ 0 ], MIN_DCC_PACKET_SIZE, MAX_DCC_PACKET_SIZE )) {

        cursor += sprintf( buf + cursor, "** Invalid DCC packet ** ");
    }
    else if ( ! validDccPkt( dccPkt )) {

        cursor += sprintf( buf + cursor, "** Invalid DCC packet checksum ** ");
    }
    else if ( idlePacket( dccPkt )) {

        cursor += sprintf( buf + cursor, "Idle ");
    }
    else if ( resetPacket( dccPkt )) {

        if ( dccPkt[ 2 ] == 0 )  
            cursor += sprintf( buf + cursor, "Reset ");
        else  
            cursor += sprintf( buf + cursor, "RailCom: 0x%02x ", dccPkt[2] );
    }
    else {

        if ( svcMode )  cursor += decoderSvcModeCvProgrammingStr( buf + cursor, dccPkt );
        else            cursor += dccOperationsPacketStr( buf + cursor, dccPkt );
    }

    return ( cursor );
}

///---------------------------------------------------------------------------------------
// "dccPacketHexStr" returns a string with the DCC packet in hex format, enclosed by 
// brackets. The function returns the number of characters put into the string buffer.
// If there is an error, the return code is a -1.
///---------------------------------------------------------------------------------------
int dccPacketHexStr( char *buf, uint8_t *dccPacket ) {

    int     cursor = sprintf( buf, "(" );
    uint8_t pktLen = dccPacket[0];

    for ( uint8_t n = 1; n <= pktLen; n++ ) {

        cursor += sprintf( buf + cursor, "0x%02x", dccPacket[n] );
        if ( n < pktLen )  cursor += sprintf( buf + cursor, " " );
    }

    cursor += sprintf( buf + cursor, ")" );
    return ( cursor );
}

///---------------------------------------------------------------------------------------
// "dccPacketBinStr" returns a string with the DCC packet in binary format, enclosed by
// brackets. The function returns the number of characters put into the string buffer. 
// If there is an error, the return code is a -1.
//
///---------------------------------------------------------------------------------------
int dccPacketBinStr( char *buf, uint8_t *dccPacket ) {

    int     cursor = sprintf( buf, "(" );
    uint8_t pktLen = dccPacket[0];

    for (  uint8_t n = 1; n <= pktLen; n++ ) {

        for ( int i = 7; i >= 0; i-- ) {

            cursor += valueToTokenStr( buf + cursor, 
                                            bitRead( dccPacket[ n ], i ), "1", "0" );
        }

         if ( n < pktLen )  cursor += sprintf( buf + cursor, " " );
    }

    cursor += sprintf( buf + cursor, ")" );
    return ( cursor );
}

}; // namespace

//========================================================================================
//========================================================================================
//
// Class part.
//
//========================================================================================
// "isXXX" methods to analyze the DCC packet. A monitor program would use them to get an
// idea what packet is at hand. We analyze the overall length, the checksum, and decode
// the instruction  uint8_t.
//
///---------------------------------------------------------------------------------------
bool LcsDccPacketFormatter::isValidDccPacket( uint8_t *dccPacket ) {

    return ( validDccPkt( dccPacket ));
}

bool LcsDccPacketFormatter::isIdlePacket( uint8_t *dccPacket ) {

    return ( idlePacket( dccPacket ));
}

bool LcsDccPacketFormatter::isResetPacket( uint8_t *dccPacket ) {

    return ( resetPacket( dccPacket ));
}

bool LcsDccPacketFormatter::isOpsModeLocPkt( uint8_t *dccPacket ) {

    uint8_t fByte = dccPacket[1];

    return (( isValidDccPacket( dccPacket )) &&
             (( isInRangeU( fByte, 1, 127 )) || ( isInRangeU( fByte, 192, 231 ))));
}

bool LcsDccPacketFormatter::isOpsModeAccPkt( uint8_t *dccPacket ) {

    return ( isValidDccPacket( dccPacket ) && ( isInRangeU( dccPacket[1], 128, 191 )));
}

bool LcsDccPacketFormatter::isSvcModePacket( uint8_t *dccPacket ) {

    return (( isValidDccPacket( dccPacket )) && 
                    (( dccPacket[ 1 ] & 0b01110000 ) == 0b01110000 ));
}

///---------------------------------------------------------------------------------------
// "formatDccPacketHex" returns a string with the DCC packet in hex format, enclosed by 
// brackets. The function returns the number of characters put into the string buffer. 
// If there is an error, the return code is a -1.
//
///---------------------------------------------------------------------------------------
int LcsDccPacketFormatter::formatDccPacketHex(  char *buf, 
                                                uint16_t bufLen, 
                                                uint8_t *dccPacket ) {

    return (
        ( bufLen >= DCC_PACKET_BUF_IN_HEX ) ? ( dccPacketHexStr( buf, dccPacket )) : -1 );
}

///---------------------------------------------------------------------------------------
// "formatDccPacketBin" returns a string with the DCC packet in binary format, enclosed 
// by brackets. The function returns the number of characters put into the string buffer.
// If there is an error, the return code is a -1.
//
///---------------------------------------------------------------------------------------
int LcsDccPacketFormatter::formatDccPacketBin(  char *buf, 
                                                uint16_t bufLen, 
                                                uint8_t *dccPacket ) {

    return (
        ( bufLen >= DCC_PACKET_BUF_IN_BIN ) ? ( dccPacketBinStr( buf, dccPacket )) : -1 );
}

///---------------------------------------------------------------------------------------
// "formatDccPacketOpsMode" formats a DCC packet as an operations mode packet. If there
// is an error, the return code is a -1.
//
///---------------------------------------------------------------------------------------
int LcsDccPacketFormatter::formatDccPacketOpsMode(  char *buf, 
                                                    uint16_t bufLen, 
                                                    uint8_t *dccPacket ) {

    return (
        ( bufLen >= DCC_PACKET_BUF_IN_FMT ) ? ( dccPacketToStr( buf, dccPacket )) : -1 );
}

///---------------------------------------------------------------------------------------
// "formatDccPacketSvcMode" formats a DCC packet as an service mode packet. If there is
// an error, the return code is a -1.
//
///---------------------------------------------------------------------------------------
int LcsDccPacketFormatter::formatDccPacketSvcMode(  char *buf, 
                                                    uint16_t bufLen, 
                                                    uint8_t *dccPacket ) {

    return (
        ( bufLen  >= DCC_PACKET_BUF_IN_FMT ) ? 
        ( dccPacketToStr( buf, dccPacket, true )) : -1 );
}
