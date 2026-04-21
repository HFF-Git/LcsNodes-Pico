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
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details. You 
// should have received a copy of the GNU General Public License along with this 
// program. If not, see <http://www.gnu.org/licenses/>.
//
//----------------------------------------------------------------------------------------
#include "LcsBaseStation.h"

//----------------------------------------------------------------------------------------
// Namespaces.
//
//----------------------------------------------------------------------------------------
using namespace LCS;

//----------------------------------------------------------------------------------------
// External global variables.
//
//----------------------------------------------------------------------------------------
extern uint16_t debugMask;

//----------------------------------------------------------------------------------------
// The base station message interface local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

//----------------------------------------------------------------------------------------
// Some helper functions.
//
//----------------------------------------------------------------------------------------
void printLcsMsg( uint8_t *msg ) {

    int msgLen = (( msg[ 0 ] >> 5 ) + 1 ) % 8;

    printf( "[" ); 

    for ( int i = 0; i < msgLen; i++ ) printf( "0x%x ", msg[ i ] );
    printf( "]" ); 
} 

  
}; // namespace

//========================================================================================
//========================================================================================
//
// Object part.
//
//========================================================================================
//========================================================================================

//----------------------------------------------------------------------------------------
//  The object constructor. Nothing really to do right now.
//
//----------------------------------------------------------------------------------------
LcsBaseStationMsgInterface::LcsBaseStationMsgInterface( ) { }

//----------------------------------------------------------------------------------------
// Set up the base station LCS message interface. We store away the coreLib, 
// locoSession and the two DCC track object references.
//
//----------------------------------------------------------------------------------------
uint8_t LcsBaseStationMsgInterface::setupLcsMsgInterface(

    LcsBaseStationLocoSession   *locoSessions,
    LcsBaseStationDccTrack      *mainTrack,
    LcsBaseStationDccTrack      *progTrack

    ) {

    if (( locoSessions == nullptr ) || 
        ( mainTrack == nullptr )  || 
        ( progTrack == nullptr ))
        return ( ERR_MSG_INTERFACE_SETUP );

    this -> locoSessions  = locoSessions;
    this -> mainTrack     = mainTrack;
    this -> progTrack     = progTrack;

    return ( LCS_OK );
} 

//----------------------------------------------------------------------------------------
// "handleLcsMsg" is the registered message handler for the LCS core library to 
// invoke for an incoming LCS DCC type message. Essentially, this routine is a big
// switch statement.
//
//----------------------------------------------------------------------------------------
void LcsBaseStationMsgInterface::handleLcsMsg( uint8_t *msg ) {

    if (( debugMask & DBG_BS_CONFIG ) && 
        ( debugMask & DBG_BS_LCS_MSG_INTERFACE )) {

        printf( "LCS-RECV: " );
        for ( int i = 0; i < (( msg[ 0 ] >> 5 ) + 1 ); i++ ) printf( "0x%2x ");
        printf( "\n" );
    }

    switch ( msg[ 0 ] ) {

        //--------------------------------------------------------------------------------
        // LCS_OP_REQ_LOC request.
        //
        // ??? The meaning will change if we auto-allocate a session. We should still
        // add a session entry on behalf of the loco and return a REP_LOC message.
        // One day, we could return data from a dictionary about default loco 
        // settings. If the session entry already exist, we just return the actual
        // data. That will do for SHARE and perhaps also STEAL.
        //
        // ??? need to implement a protocol between handhelds for STEAL and SHARE.
        //--------------------------------------------------------------------------------
        case LCS_OP_REQ_LOC: {

            uint16_t  cabId   = msg[ 1 ] * 256 + msg[ 2 ];
            uint8_t   flags   = msg[ 3 ];
            uint8_t   sId     = 0;
            int       ret     = locoSessions -> requestSession( cabId, flags, &sId );

            if (( debugMask & DBG_BS_CONFIG ) && 
                ( debugMask & DBG_BS_LCS_MSG_INTERFACE )) {
                
                printf( "LCS_OP_REQ_LOC, cabId: %d, "
                        "Flags: 0x%x,  -> Ret: %d, sId: %d\n", 
                        cabId, flags, ret , sId );
            }

            if ( ret == LCS_OK ) {

                SessionMapEntry *smePtr = locoSessions -> getSessionMapEntryPtr( sId );

                if ( smePtr != NULL ) {

                    // ??? FIX ...

                    #if 0
                    sendRepLoc( sId,
                                cabId,
                                ((( smePtr -> direction ) ? 0x80 : 0 ) | 
                                 ( smePtr -> speed & 0x7F )),
                                smePtr -> functions[ 0 ],
                                smePtr -> functions[ 1 ],
                                smePtr -> functions[ 2 ] );
                    #endif
                }
                else sendDccErr(  ERR_LOCO_SESSION_ALLOCATE, 
                                  highByte( cabId ), 
                                  lowByte( cabId ));
            }
            else sendDccErr( ERR_LOCO_SESSION_ALLOCATE, 
                             highByte( cabId ), 
                             lowByte( cabId ));

        } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_QRY_LOC request. The query request obtains the current session data. 
        // The reply command is the REP-LOC command, which sends the current sessionId 
        // and speed, direction and function data for F0 to F12.
        //
        //
        // ??? we can combine this with an RLOC message. Since there are only
        // sessions on demand, there is no point to query that data.
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_QRY_LOC: {

            uint8_t         sId     = msg[ 1 ];
            SessionMapEntry *smePtr = locoSessions -> getSessionMapEntryPtr( sId );

            if (( debugMask & DBG_BS_CONFIG ) && 
                ( debugMask & DBG_BS_LCS_MSG_INTERFACE )) {
            
                printf( "LCS_OP_QRY_LOC: %d\n", sId );
            }

            if ( smePtr != NULL ) {

                #if 0 // ??? FIX .... 
                sendRepLoc( sId,
                            smePtr -> cabId,
                            ((( smePtr -> direction ) ? 0x80 : 0 ) | 
                             ( smePtr -> speed & 0x7F )),
                            smePtr -> functions[ 0 ],
                            smePtr -> functions[ 1 ],
                            smePtr -> functions[ 2 ] );
                #endif
            }
            else sendDccErr( ERR_SESSION_NOT_FOUND, sId );

        } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_REL_LOC request. The session is released and the session map entry
        // deallocated. If all works fine we broadcast the session cancelled message
        // to other nodes.
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_REL_LOC: {

            uint8_t sId = msg[ 1 ];
            int     ret = locoSessions -> releaseSession( sId );

            if (( debugMask & DBG_BS_CONFIG ) && 
                ( debugMask & DBG_BS_LCS_MSG_INTERFACE )) {
                
                printf( "LCS_OP_REL_LOC: %d -> Ret: %d\n", sId, ret );
            }

            if ( ret == LCS_OK ) sendDccErr( ERR_LOCO_SESSION_CANCELLED );
            else                 sendDccErr( ERR_SESSION_NOT_FOUND, sId );

        }  break;

        //--------------------------------------------------------------------------------
        // LCS_OP_SET_LSPD request. Hopefully the most used message you will see. After
        // all, we want to control engines. :-)
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_SET_LSPD: {

            uint8_t sId   = msg[ 1 ];
            uint8_t spDir = msg[ 2 ];
            int     ret   = locoSessions -> markSessionAlive( sId );

            if ( ret == LCS_OK ) 
                ret = locoSessions -> setThrottle( sId, spDir & 0x7f, 
                                                   ( spDir & 0x80 ) >> 7 );

            if ( ret != LCS_OK ) sendDccErr( ret, sId );

            if (( debugMask & DBG_BS_CONFIG ) && 
                ( debugMask & DBG_BS_LCS_MSG_INTERFACE )) {

                printf( "LCS_OP_SET_LSPD, sId: %d, spDir: 0x%x -> Ret: %d\n", 
                        sId, spDir, ret );
            }

        } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_SET_LMOD request.
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_SET_LMOD: {

            uint8_t sId   = msg[ 1 ];
            uint8_t flags = msg[ 2 ];
            int     ret   = locoSessions -> markSessionAlive( sId );

            if ( ret == LCS_OK ) ret = locoSessions -> updateSession( sId, flags );
            if ( ret != LCS_OK ) sendDccErr( ret, sId );

            if (( debugMask & DBG_BS_CONFIG ) && 
                ( debugMask & DBG_BS_LCS_MSG_INTERFACE )) {

                printf( "LCS_OP_SET_LMOD, sId: %d, "
                        "flags: 0x%x -> Ret: %d\n", sId, flags, ret );
            }

      } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_LOC_FGRP request. This command is used to request the setting of a 
        // function group using the NMRA DCC function data byte layout.
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_LOC_FGRP: {

            uint8_t sId     = msg[ 1 ];
            uint8_t fGroup  = msg[ 2 ];
            uint8_t dccByte = msg[ 3 ];
            int     ret     = locoSessions -> markSessionAlive( sId );

            if ( ret == LCS_OK ) 
               ret = locoSessions -> setDccFunctionGroup( sId, fGroup, dccByte );
            if ( ret != LCS_OK ) sendDccErr( ret, sId );

            if (( debugMask & DBG_BS_CONFIG ) && 
                ( debugMask & DBG_BS_LCS_MSG_INTERFACE )) {

                printf( "LCS_OP_LOC_FGRP, sId: %d, fGroup: %d, "
                        "dccByte: 0x%x -> Ret: %d\n",
                        sId, fGroup, dccByte, ret );
            }
        
        } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_LOC_FON and LCS_OP_LOC_FOF request. These messages set or clear a
        // function flag identified by the function number.
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_LOC_FON:
        case LCS_OP_LOC_FOFF: {

            uint8_t sId   = msg[ 1 ];
            uint8_t fNum  = msg[ 2 ];
            int     ret   = locoSessions -> markSessionAlive( sId );

            if ( ret == LCS_OK )
            ret = locoSessions -> setDccFunctionBit( sId, fNum, 
                                            (( msg[0] == LCS_OP_LOC_FON ) ? 1 : 0 ));

            if ( ret != LCS_OK ) sendDccErr( ret, sId );

            if (( debugMask & DBG_BS_CONFIG ) && 
                ( debugMask & DBG_BS_LCS_MSG_INTERFACE )) {

                printf( "LCS_OP_LOC_FON/FOF, sId: %d, fNum: %d -> "
                        "Ret: %d\n", sId, fNum, ret ); 
            }
        
        } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_KEEP_LOC request. This command is send by the cab handheld on a 
        // regular base to notify the  base station that the session is still alive,
        // when no other DCC command is transmitted. Any other DCC command received
        // from the handheld will set the flag.
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_KEEP_LOC: {

            uint8_t sId = msg[ 1 ];
            int     ret = locoSessions -> markSessionAlive( sId );

            if ( ret != LCS_OK ) sendDccErr( ret, sId );

            if (( debugMask & DBG_BS_CONFIG ) && 
                ( debugMask & DBG_BS_LCS_MSG_INTERFACE )) {

                printf( "LCS_OP_KEEP_LOC, sId: %d -> ret: %d\n", sId, ret );
            }

        } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_SET_CVM request. This command is an on the track CV programming
        // command. The base station will send a CV byte to the specific loco on 
        // the main track.
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_SET_CVM: {

            uint8_t   sId   = msg[ 1 ];
            uint16_t  cvId  = msg[ 2 ] * 256 + msg[ 3 ];
            uint8_t   mode  = msg[ 4 ];
            uint8_t   val   = msg[ 5 ];
            int       ret   = locoSessions -> markSessionAlive( sId );

            if ( ret == LCS_OK ) 
               ret = locoSessions -> writeCVMain( sId, cvId, mode, val );
            if ( ret != LCS_OK ) sendDccErr( ret, sId );

            if (( debugMask & DBG_BS_CONFIG ) && 
                ( debugMask & DBG_BS_LCS_MSG_INTERFACE )) {

                printf( "LCS_OP_SET_CVM, sId: %d -> ret: %d\n", sId, ret );
            }
            
        } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_SET_CVS request. This command writes a value to the CV in service 
        // mode, a separate programming track. The session number is not used, any
        // session number will do. The mode byte specifies the service mode:
        //
        //  0 - Direct Byte
        //  1 - Direct Bit
        //  2 - Page Mode
        //  3 - Register Mode
        //  4 - Address Only Mode
        //
        // We only support mode 0 and 1. The rest is kind of deprecated and should 
        // not be used in new designs. Mode 1 encodes the bit and the bit position in
        // the byte as ‘111CDBBB’ where C is here is always 1 as only ‘writes’ are
        // possible in OTM programming. D is the bit value, either  0 or 1 and BBB is 
        // the bit position in the CV byte. 000 to 111 for bits 0 to 7.
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_SET_CVS: {

            uint16_t  cvId  = msg[ 1 ] * 256 + msg[ 2 ];
            uint8_t   mode  = msg[ 3 ];
            uint8_t   val   = msg[ 4 ];
            int       ret   = locoSessions -> writeCV( cvId, mode, val );

            if ( ret != LCS_OK ) sendDccErr( ret );

            if (( debugMask & DBG_BS_CONFIG ) && 
                ( debugMask & DBG_BS_LCS_MSG_INTERFACE )) {

                printf( "LCS_OP_SET_CVS, ret: %d\n", ret );
            }
            
        } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_REQ_CVS request. This command requests a CV read in service mode
        // on the programming track. The session number is not used, any session 
        // number will do. The mode byte specifies the service mode:
        //
        //  0 - Direct Byte
        //  1 - Direct Bit
        //  2 - Page Mode
        //  3 - Register Mode
        //  4 - Address Only Mode
        //
        // Upon successful execution, the base station will send a LCS_OP_REP_CVS 
        // command with the requested data.
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_REQ_CVS: {

            uint16_t  cvId  = msg[ 1 ] * 256 + msg[ 2 ];
            uint8_t   mode  = msg[ 3 ];
            uint8_t   val   = 0;
            int       ret   = locoSessions -> readCV( cvId, mode, &val );

            if ( ret == LCS_OK )  sendRepLocCvProg( cvId, val );
            else                  sendDccErr( ret );

            if (( debugMask & DBG_BS_CONFIG ) && 
                ( debugMask & DBG_BS_LCS_MSG_INTERFACE )) {

                printf( "LCS_OP_REQ_CVS, ret: %d\n", ret );
            }

         } break;

        //--------------------------------------------------------------------------------
        // LCS_OP_SEND_DCCx request. This command sends a DCC packet exactly as 
        // passed. There are three to six bytes in the package. As the base station
        // will do no checking and just send out the byte sequence, care should thus
        // be taken that it is a valid packet. Better put the DCC standard under your
        // night pillow.
        //
        // NMRA defines a maximum packet size of 6 bytes. The RailCommunity defines
        // up to ten data bytes plus a checksum byte. So far, the LCS bus does not
        // offer a message type for this extended packet length.
        //
        //--------------------------------------------------------------------------------
        case LCS_OP_SEND_DCC3: {
            
            uint8_t ret = locoSessions -> writeDccPacketMain( &msg[ 2 ], 3, msg[ 1 ] ); 

            if (( debugMask & DBG_BS_CONFIG ) && 
                ( debugMask & DBG_BS_LCS_MSG_INTERFACE )) {

                printf( "LCS_OP_SEND_DCC3, ret: %d\n", ret );
            }
            
        } break;

        case LCS_OP_SEND_DCC4: {
            
            uint8_t ret = locoSessions -> writeDccPacketMain( &msg[ 2 ], 4, msg[ 1 ] ); 

            if (( debugMask & DBG_BS_CONFIG ) && 
                ( debugMask & DBG_BS_LCS_MSG_INTERFACE )) {

                printf( "LCS_OP_SEND_DCC4, ret: %d\n", ret );
            }
            
        } break;

        case LCS_OP_SEND_DCC5: {
            
            uint8_t ret = locoSessions -> writeDccPacketMain( &msg[ 2 ], 5, msg[ 1 ] ); 

            if (( debugMask & DBG_BS_CONFIG ) && 
                ( debugMask & DBG_BS_LCS_MSG_INTERFACE )) {

                printf( "LCS_OP_SEND_DCC5, ret: %d\n", ret );
            }
            
        } break;

       
       case LCS_OP_SEND_DCC6: {
            
            uint8_t ret = locoSessions -> writeDccPacketMain( &msg[ 2 ], 6, msg[ 1 ] ); 

            if (( debugMask & DBG_BS_CONFIG ) && 
                ( debugMask & DBG_BS_LCS_MSG_INTERFACE )) {

                printf( "LCS_OP_SEND_DCC6, ret: %d\n", ret );
            }
            
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
