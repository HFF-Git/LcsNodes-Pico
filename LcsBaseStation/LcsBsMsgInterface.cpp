//----------------------------------------------------------------------------------------
//
// LCS Base Station - LCS Msg Interface - implementation file.
//
//----------------------------------------------------------------------------------------
// ...
//
//
//----------------------------------------------------------------------------------------
//
// LCS Base Station - LCS Msg Interface - implementation file.
// Copyright (C) 2020 - 2026  Helmut Fieres
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
#include "LcsBaseStation.h"

//----------------------------------------------------------------------------------------
// External global variables.
//
//----------------------------------------------------------------------------------------
extern uint16_t debugMask;
extern LcsLocoSessions  *locoSessions;
extern LcsDccTrack      *mainTrack;
extern LcsDccTrack      *progTrack; 

//----------------------------------------------------------------------------------------
// The base station message interface local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

//----------------------------------------------------------------------------------------
// "setupDebugEnabled" and "retStat" are the debug support routines. We can 
// easily check whether debug is enabled at all. The return status routine will
// print out a return status message when debugging is enabled. The macro 
// "RET_STAT" is a nice helper that adds the function name to the message.
// 
//----------------------------------------------------------------------------------------
inline bool msgDebugEnabled( ) {

    return (( debugMask & DBG_BS_ENABLED ) && 
            ( debugMask & DBG_BS_LCS_MSG_INTERFACE )); 
}

inline void enterFunc( char *name ) {

    if ( msgDebugEnabled( )) usbIoPrintf( CDC_USB_CONSOLE, "--> %s\n", name );
}

inline uint8_t retStat( char *name, uint8_t errId ) {

    if ( msgDebugEnabled( )) {

        if ( errId == LCS_OK ) usbIoPrintf( CDC_USB_CONSOLE, "<-- %s: OK\n", name );
        else            usbIoPrintf( CDC_USB_CONSOLE, "<-- %s: %d\n", name, errId );
    }

    return ( errId );
}

#define ENTER_FUNC() enterFunc((char *) __func__)
#define RET_STAT(x) retStat((char *) __func__, ( x ))

//----------------------------------------------------------------------------------------
// Some helper functions.
//
//----------------------------------------------------------------------------------------
void printLcsMsg( uint8_t *msg ) {

    int msgLen = (( msg[ 0 ] >> 5 ) + 1 ) % 8;

    usbIoPrintf( CDC_USB_CONSOLE, "[" ); 

    for ( int i = 0; i < msgLen; i++ ) { 
        
        usbIoPrintf( CDC_USB_CONSOLE, "0x%x ", msg[ i ] );
    }

    usbIoPrintf( CDC_USB_CONSOLE, "]" ); 
} 

}; // namespace


//----------------------------------------------------------------------------------------
// "initLcsMsgInterface" initializes the LCS message interface. Right now, we only 
// make sure that session and track objects have been created.
//
//----------------------------------------------------------------------------------------
uint8_t initLcsMsgInterface( ) {

    if (( locoSessions == nullptr ) ||
        ( mainTrack    == nullptr ) ||
        ( progTrack    == nullptr )) return( ERR_BASE_STATION_MSG_INIT );

    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "handleLcsMsg" is the registered message handler for the LCS core library to 
// invoke for an incoming LCS DCC type message. Essentially, this routine is a 
// big switch statement.
//
//----------------------------------------------------------------------------------------
void handleLcsMsg( uint8_t *msg ) {

    if ( msgDebugEnabled( )) {

        usbIoPrintf( CDC_USB_CONSOLE, "LCS-RECV: " );
        printLcsMsg( msg );
        usbIoPrintf( CDC_USB_CONSOLE, "\n" );
    }

    switch ( msg[ 0 ] ) {

        //--------------------------------------------------------------------------------
        // LCS_OP_REQ_LOC request. We will return the Loco Data from the base
        // station dictionary, if the loco exists.
        //
        // ??? really no difference between REQ and QRY ?
        //--------------------------------------------------------------------------------
        case LCS_OP_QRY_LOC: 
        case LCS_OP_REQ_LOC: {

            uint16_t  cabId   = msg[ 1 ] * 256 + msg[ 2 ];
            uint8_t   flags   = msg[ 3 ];
            uint8_t   rStat   = LCS_OK;

            if ( msgDebugEnabled( ))  {
                
                usbIoPrintf( CDC_USB_CONSOLE, "LCS_OP_REQ_LOC, cabId: %d, "
                        "Flags: 0x%x,  -> Ret: %d, sId: %d\n", 
                        cabId, flags, rStat );
            }

            if ( rStat != LCS_OK ) {

                sendDccAck( ERR_LOCO_SESSION_ALLOCATE, 
                            highByte( cabId ), 
                            lowByte( cabId ));
            }

            LcsCabEntry *cPtr = locoSessions -> lookupCabEntry( cabId );

            if ( cPtr == nullptr ) {

                sendDccAck( ERR_LOCO_SESSION_ALLOCATE, 
                            highByte( cabId ), 
                            lowByte( cabId ));
            }

            sendRepLoc( cabId,
                        cPtr -> speedInfo & 0xFF,
                        cPtr -> d.funcMap[ 0 ],
                        cPtr -> d.funcMap[ 1 ],
                        cPtr -> d.funcMap[ 2 ] );

        } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_SET_LSPD request. Hopefully the most used message you will 
        // see. After all, we want to control engines. :-)
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_SET_LSPD: {

            uint16_t  cabId   = msg[ 1 ] * 256 + msg[ 2 ];
            uint8_t   spDir   = 0;
            uint8_t   rStat   = LCS_OK;

            rStat = locoSessions -> setThrottle( cabId, spDir & 0x7f, 
                                                   ( spDir & 0x80 ) >> 7 );

            if ( rStat != LCS_OK ) {
                
                sendDccAck( rStat, highByte( cabId ), lowByte( cabId ) );
            }

            if ( msgDebugEnabled( )) {

                usbIoPrintf( CDC_USB_CONSOLE, 
                             "LCS_OP_SET_LSPD, cabId: %d, spDir: 0x%x -> Ret: %d\n", 
                             cabId, spDir, rStat );
            }

        } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_SET_LMOD request.
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_SET_LMOD: {

            uint16_t  cabId   = msg[ 1 ] * 256 + msg[ 2 ];
            uint8_t   flags   = msg[ 3 ];
            uint8_t   rStat   = LCS_OK;

            // ??? what to call ? 
            // ??? return status

            if ( msgDebugEnabled( )) {

                usbIoPrintf( CDC_USB_CONSOLE, "LCS_OP_SET_LMOD, sId: %d, "
                        "flags: 0x%x -> Ret: %d\n", cabId, flags, rStat );
            }

      } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_LOC_FGRP request. This command is used to request the setting of a 
        // function group using the NMRA DCC function data byte layout.
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_LOC_FGRP: {

            uint16_t  cabId   = msg[ 1 ] * 256 + msg[ 2 ];
            uint8_t   fGroup  = msg[ 3 ];
            uint8_t   dccByte = msg[ 4 ];
            uint8_t   rStat   = LCS_OK;

            rStat = locoSessions -> setDccFunctionGroup( cabId, fGroup, dccByte );

            if ( rStat != LCS_OK ) {
                
                sendDccAck( rStat, highByte( cabId ), lowByte( cabId ) );
            }

            if ( msgDebugEnabled( )) {

                usbIoPrintf( CDC_USB_CONSOLE, 
                             "LCS_OP_LOC_FGRP, cabId: %d, fGroup: %d, "
                             "dccByte: 0x%x -> Ret: %d\n",
                             cabId, fGroup, dccByte, rStat );
            }
        
        } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_LOC_FON and LCS_OP_LOC_FOF request. These messages set or 
        // clear a function flag identified by the function number.
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_LOC_FON:
        case LCS_OP_LOC_FOFF: {

            uint16_t  cabId   = msg[ 1 ] * 256 + msg[ 2 ];
            uint8_t   fNum    = msg[ 3 ];
            uint8_t   rStat   = LCS_OK;
           
            rStat = locoSessions -> setDccFunctionBit( cabId, fNum, 
                                       (( msg[0] == LCS_OP_LOC_FON ) ? 1 : 0 ));

            if ( rStat != LCS_OK ) {
                
                sendDccAck( rStat, highByte( cabId ), lowByte( cabId ) );
            }

            if ( msgDebugEnabled( )) { 

                usbIoPrintf( CDC_USB_CONSOLE, 
                             "LCS_OP_LOC_FON/FOF, sId: %d, fNum: %d -> "
                             "Ret: %d\n", cabId, fNum, rStat ); 
            }
        
        } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_KEEP_LOC request. This command is send by the cab handheld on
        // a regular base to notify the base station that the session is still
        // alive, when no other DCC command is transmitted. Any other DCC command
        // received from the handheld will set the flag.
        //
        // ??? not sure we keep this ?
        //--------------------------------------------------------------------------------
        case LCS_OP_KEEP_LOC: {

            uint16_t  cabId   = msg[ 1 ] * 256 + msg[ 2 ];
            uint8_t   rStat   = LCS_OK;

            rStat = locoSessions ->markCabAlive( cabId );

            if ( rStat != LCS_OK ) {
                
                sendDccAck( rStat, highByte( cabId ), lowByte( cabId ) );
            }

            if ( msgDebugEnabled( )) { 

                usbIoPrintf( CDC_USB_CONSOLE, 
                             "LCS_OP_KEEP_LOC, sId: %d -> ret: %d\n", 
                             cabId, rStat );
            }

        } break;

        //--------------------------------------------------------------------------------
        //
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_QRY_LCON: {

            // ??? to do ...

        } break;

        //--------------------------------------------------------------------------------
        //
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_SET_LCON: {

            // ??? to do ...

        } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_SET_CVM request. This command is an on the track CV programming
        // command. The base station will send a CV byte to the specific loco on 
        // the main track.
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_SET_CVM: {

            uint16_t  cabId = msg[ 1 ] * 256 + msg[ 2 ];
            uint16_t  cvId  = msg[ 3 ] * 256 + msg[ 4 ];
            uint8_t   mode  = msg[ 5 ];
            uint8_t   val   = msg[ 6 ];
            uint8_t   rStat = LCS_OK;

            rStat = locoSessions -> writeCVMain( cabId, cvId, mode, val );
            
            if ( rStat != LCS_OK ) {
                
                sendDccAck( rStat, highByte( cabId ), lowByte( cabId ) );
            }

            if ( msgDebugEnabled( )) {

                usbIoPrintf( CDC_USB_CONSOLE, 
                             "LCS_OP_SET_CVM, cabId: %d -> ret: %d\n", 
                             cabId, rStat );
            }
            
        } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_SET_CVS request. This command writes a value to the CV in 
        // service mode, a separate programming track. The session number is
        // not used, any session number will do. The mode byte specifies the 
        // service mode:
        //
        //  0 - Direct Byte
        //  1 - Direct Bit
        //  2 - Page Mode
        //  3 - Register Mode
        //  4 - Address Only Mode
        //
        // We only support mode 0 and 1. The rest is kind of deprecated and 
        // should not be used in new designs. Mode 1 encodes the bit and the 
        // bit position in the byte as ‘111CDBBB’ where C is here is always 1 
        // as only ‘writes’ are possible in OTM programming. D is the bit value, 
        // either  0 or 1 and BBB is the bit position in the CV byte. 000 to
        // 111 for bits 0 to 7.
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_SET_CVS: {

            uint16_t  cvId  = msg[ 1 ] * 256 + msg[ 2 ];
            uint8_t   mode  = msg[ 3 ];
            uint8_t   val   = msg[ 4 ];
            uint8_t   rStat = LCS_OK;

            rStat   = locoSessions -> writeCV( cvId, mode, val );

            if ( rStat != LCS_OK ) {
                
                sendDccAck( rStat, highByte( cvId ), lowByte( cvId ) );
            }

            if ( msgDebugEnabled( )) {
            
                usbIoPrintf( CDC_USB_CONSOLE, 
                             "LCS_OP_SET_CVS, cvId: %d, ret: %d\n", 
                             cvId, rStat );
            }
            
        } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_REQ_CVS request. This command requests a CV read in service 
        // mode on the programming track. The session number is not used, any 
        // session number will do. The mode byte specifies the service mode:
        //
        //  0 - Direct Byte
        //  1 - Direct Bit
        //  2 - Page Mode
        //  3 - Register Mode
        //  4 - Address Only Mode
        //
        // Upon successful execution, the base station sends a LCS_OP_REP_CVS 
        // command with the requested data.
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_REQ_CVS: {

            uint16_t  cvId  = msg[ 1 ] * 256 + msg[ 2 ];
            uint8_t   mode  = msg[ 3 ];
            uint8_t   val   = 0;
            uint8_t   rStat = LCS_OK;

            rStat = locoSessions -> readCV( cvId, mode, &val );

            if ( rStat == LCS_OK ) {

                rStat = sendRepLocCvProg( cvId, val );
            }
            else sendDccAck( rStat, highByte( cvId ), lowByte( cvId ) );
            
            if ( msgDebugEnabled( )) {
            
                usbIoPrintf( CDC_USB_CONSOLE, 
                             "LCS_OP_REQ_CVS, cvId: %d, val: 0x%2x, ret: %d\n", 
                             cvId, val, rStat );
            }

        } break;

        //--------------------------------------------------------------------------------
        //
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_SET_BACC: {

            // ??? to do ...

        } break;

        //--------------------------------------------------------------------------------
        //
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_SET_EACC: {

            // ??? to do ...

        } break;

        //--------------------------------------------------------------------------------
        //
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_SET_MPOM: {

            // ??? to do ...

        } break;

        //--------------------------------------------------------------------------------
        //
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_REQ_MPOM: {

            // ??? to do ...

        } break;

        //--------------------------------------------------------------------------------
        //
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_REP_MPOM: {

            // ??? to do ...

        } break;

        //--------------------------------------------------------------------------------
        //
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_SET_APOM: {

            // ??? to do ...

        } break;

        //--------------------------------------------------------------------------------
        //
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_REQ_APOM: {

            // ??? to do ...

        } break;

        //--------------------------------------------------------------------------------
        //
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_REP_APOM: {

            // ??? to do ...

        } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_SEND_DCCx request. This command sends a DCC packet exactly as 
        // passed. There are three to six bytes in the package. As the base 
        // station does no checking and just send out the byte sequence, care 
        // should thus be taken that it is a valid packet. Better put the DCC 
        // standard under your night pillow.
        //
        // NMRA defines a maximum packet size of 6 bytes. The RailCommunity 
        // defines up to ten data bytes plus a checksum byte. So far, the LCS
        // bus does not offer a message type for this extended packet length.
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_SEND_DCC3: {

            uint8_t rStat = LCS_OK;

            rStat = locoSessions -> writeDccPacketMain( msg, 3 );

            if ( msgDebugEnabled( )) {
            
                usbIoPrintf( CDC_USB_CONSOLE, 
                             "LCS_OP_SEND_DCC3, byte1: 0x%2x, byte2: 0x%2x, "
                             "byte3: 0x%2x, rStat: %d\n", 
                             msg[ 0 ], msg[ 1 ], msg[ 2 ], rStat );
            }
            
        } break;

        case LCS_OP_SEND_DCC4: {
            
            uint8_t rStat = LCS_OK;

            rStat = locoSessions -> writeDccPacketMain( msg, 4 );

            if ( msgDebugEnabled( )) {
            
                usbIoPrintf( CDC_USB_CONSOLE, 
                             "LCS_OP_SEND_DCC4, byte1: 0x%2x, byte2: 0x%2x, "
                             "byte3: 0x%2x, byte4: 0x%2x, rStat: %d\n", 
                             msg[ 0 ], msg[ 1 ], msg[ 2 ], msg[ 3 ], rStat );
            }
            
        } break;

        case LCS_OP_SEND_DCC5: {
            
            uint8_t rStat = LCS_OK;

            rStat = locoSessions -> writeDccPacketMain( msg, 5 );

            if ( msgDebugEnabled( )) {
            
                usbIoPrintf( CDC_USB_CONSOLE, 
                             "LCS_OP_SEND_DCC5, byte1: 0x%2x, byte2: 0x%2x, "
                             "byte3: 0x%2x, byte4: 0x%2x, byte5: 0x%2x, rStat: %d\n", 
                             msg[ 0 ], msg[ 1 ], msg[ 2 ], msg[ 3 ], msg[ 4 ], rStat );
            }
            
        } break;

       case LCS_OP_SEND_DCC6: {
            
            uint8_t rStat = LCS_OK;

            rStat = locoSessions -> writeDccPacketMain( msg, 6 );

            if ( msgDebugEnabled( )) {
            
                usbIoPrintf( CDC_USB_CONSOLE, 
                             "LCS_OP_SEND_DCC6, byte1: 0x%2x, byte2: 0x%2x, "
                             "byte3: 0x%2x, byte4: 0x%2x, byte5: 0x%2x, "
                             "byte6: 0x%2x, rStat: %d\n", 
                             msg[ 0 ], msg[ 1 ], msg[ 2 ], msg[ 3 ], 
                             msg[ 4 ], msg[ 5 ], rStat );
            }
            
        } break;

        //--------------------------------------------------------------------------------
        //
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_TON: {

            // ??? to do ...

        } break;

        //--------------------------------------------------------------------------------
        //
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_TOF: {

            // ??? to do ...

        } break;

        //--------------------------------------------------------------------------------
        //
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_ESTP: {

            // ??? to do ...

        } break;

        //--------------------------------------------------------------------------------
        // Node Item and Port Item cases.
        //
        //--------------------------------------------------------------------------------
        // ??? make calls to the nodeManagement class ? or just do them here ?   
        // ??? separate callbacks for DCC calls and NodeMgt calls ?
        
        default: { }
    } 
}
