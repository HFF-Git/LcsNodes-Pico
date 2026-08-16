///---------------------------------------------------------------------------------------
//
// LCS - DCC Track Manager - Include file
//
///---------------------------------------------------------------------------------------
//
//
///---------------------------------------------------------------------------------------
//
// LCS - Driver Library Code for Occupancy Detect boards
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
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------
#pragma once

#include "LcsUtilLib.h"
#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
using namespace CDC;
using namespace LCS;

//----------------------------------------------------------------------------------------
// The DCC Track library maintains a set of debug flags. The following debug 
// flags are defined:
//
//  DBG_DCC_TRACK_ENABLED         - DEBUG messages enabled.
//  DBG_DCC_TRACK_CONFIG          - debug DCC track configuration
//  DBG_DCC_TRACK_POWER_MGMT      - debug power management 
//  DBG_DCC_TRACK_DCC_ACK_DETECT  - debug display decoder ACK     
//  DBG_DCC_TRACK_RAILCOM         - debug RailCom function.
//
//----------------------------------------------------------------------------------------
enum DccTrackDebugFlags : uint16_t {

    DBG_DCC_TRACK_ENABLED          = 1 << 15,
    DBG_DCC_TRACK_CONFIG           = 1 << 0,   
    DBG_DCC_TRACK_POWER_MGMT       = 1 << 1, 
    DBG_DCC_TRACK_DCC_ACK_DETECT   = 1 << 2,     
    DBG_DCC_TRACK_RAILCOM          = 1 << 3   
};

//----------------------------------------------------------------------------------------
// Base station errors. Note that they need to be in the assigned to the user 
// number range of errors defined in the LCS runtime library. 
//
//----------------------------------------------------------------------------------------
enum DccTrackErrors : uint8_t {

    ERR_DCC_TRACK_ERR_BASE          = ERR_USER_SPECIFIC_BASE,
    ERR_DCC_RNUM_CONFIG_ERR         = ERR_DCC_TRACK_ERR_BASE + 1,
    ERR_DCC_ENABLE_RNUM_ERR         = ERR_DCC_TRACK_ERR_BASE + 2,
    ERR_DCC_CONTROL_RNUM_ERR        = ERR_DCC_TRACK_ERR_BASE + 3,
    ERR_DCC_SENSE_RNUM_ERR          = ERR_DCC_TRACK_ERR_BASE + 4,
    ERR_DCC_UART_RNUM_ERR           = ERR_DCC_TRACK_ERR_BASE + 5,


    ERR_DCC_TRACK_CONFIG            = ERR_DCC_TRACK_ERR_BASE + 1,


};

//----------------------------------------------------------------------------------------
// DCC packet definition. A DCC packet is the payload data without the checksum. 
// Besides the length in bytes and the buffer, there is a repeat counter to specify
// how often this packet will be repeatedly transmitted after the first transmission.
// Currently, a DCC packet is at most 15 bytes long, excluding the checksum byte. 
// This is true for XPOM and DCC-A support, otherwise it is historically a maximum 
// of 6 bytes.
//
//----------------------------------------------------------------------------------------
const uint8_t DCC_PACKET_SIZE = 16;

struct DccPacket {

    uint8_t len;
    uint8_t repeat;
    uint8_t buf[ DCC_PACKET_SIZE ];
};

//----------------------------------------------------------------------------------------
// Setup options to set for the DCC track. They are set when the track object is
// created.
//
//  DT_OPT_SERVICE_MODE_TRACK  - The track is a DCC PROG track, else a MAIN track.
//  DT_OPT_CUTOUT              - The track is configured to emit a cutout.
//  DT_OPT_RAILCOM             - The track support Railcom detection.
//
//----------------------------------------------------------------------------------------
enum DccTrackOptions : uint16_t {

    DT_OPT_NIL                  = 0,

    DT_OPT_SERVICE_MODE_TRACK   = 1 << 0,
    
    DT_OPT_ACC_DETECT           = 1 << 3
};

//----------------------------------------------------------------------------------------
// A track can operate in several modes. The modes influence in what operating
// mode the track works.
// 
//  DT_M_PLAIN          -   the track is in DCC main track mode. No cutout, no
//                          railcom support, no Acc-Detect function. The default
//                          packet is the IDLE packet.
//
//  DT_M_CUTOUT         -   the track is in DCC main track mode and has the 
//                          cutout generation enabled.
//
//  DT_M_RAILCOM        -   the track is in DCC main track mode and has the 
//                          cutout generation and railcom detection enabled.
//
//  DT_M_ACC_DETECT     -   the track is in DCC programming mode. The decoder
//                          acknowledge function is enabled, cutout and RailCom
//                          are disabled. The default packet is the RESET packet. 
//
//----------------------------------------------------------------------------------------
enum DccTrackMode : uint8_t {

    DT_M_NIL                    = 0,
    DT_M_TRACK_OFF              = 1,
    DT_M_PLAIN                  = 2,
    DT_M_CUTOUT                 = 3,
    DT_M_RAILCOM                = 4,
    DT_M_ACC_DETECT             = 5
};

//----------------------------------------------------------------------------------------
// The DCC track object has a set of flags to indicate its current status.
//
//  DT_F_POWER_ON             - The track is under power.
//  DT_F_POWER_SHORT_CIRCUIT  - The H-Bridge signaled a short circuit condition. 
//  DT_F_POWER_OVERLOAD       - An overload situation was detected.
//  DT_F_CUTOUT_MODE_ON       - The track has the cutout generation enabled.
//  DT_F_RAILCOM_MODE_ON      - The track has the railcom detect enabled.
//  DT_F_RAILCOM_MSG_PENDING  - A Railcom received datagram is indicated.
//  DT_F_LOG_ENABLED          - Internal log facility is enabled.
//  DT_F_LOG_ACTIVE           - Internal log facility is active.
//  DT_F_CONFIG_ERROR         - The configuration descriptor has invalid options.
//
//----------------------------------------------------------------------------------------
enum DccTrackFlags : uint16_t {

    DT_F_NIL                    = 0,
    DT_F_POWER_ON               = 1 << 0,
    DT_F_POWER_SAMPLE_PENDING   = 1 << 1,
    DT_F_POWER_OVERLOAD         = 1 << 2,
    DT_F_CUTOUT_ON              = 1 << 4,
    DT_F_RAILCOM_ON             = 1 << 5,
    DT_F_ACC_DETECT_ON          = 1 << 6,
    DT_F_DCC_PACKET_PENDING     = 1 << 7,
    DT_F_RAILCOM_MSG_PENDING    = 1 << 8,
    DT_F_LOG_ENABLED            = 1 << 9,
    DT_F_LOG_ACTIVE             = 1 << 10,
    DT_F_SETUP_ERROR            = 1 << 15
};

//----------------------------------------------------------------------------------------
// For creating the DCC track objects, the tracks are described by the data 
// structure below. In addition to the hardware resources, there are the limits
// for current consumption values, all specified in milliAmps. Mostly, the 
// hardware is described by this structure.
//
//----------------------------------------------------------------------------------------
struct LcsDccTrackDesc {

    DccTrackOptions     options;      
    DccTrackMode        initTrackMode;

    uint8_t             rNumTimer;
    uint8_t             rNumEnable;
    uint8_t             rNumControl;
    uint8_t             rNumSense;
    uint8_t             rNumUartRx;

    uint16_t            limitCurrentMilliAmp;
    uint16_t            maxCurrentMilliAmp;
    uint16_t            overloadEventThreshold;

    uint16_t            referenceMilliVolt;
    uint16_t            shuntMilliOhm;
    uint16_t            amplifierGain;
    uint16_t            adcBitResolution;
    uint16_t            adcZeroOffset;
};

//----------------------------------------------------------------------------------------
// DCC track definition. The DCC track object is responsible for managing track 
// power as well as building and sending the DCC packet bit stream. It is the 
// heart of the DCC signal generation. 
//
// A DCC packet consists of the preamble bits, the postamble bit, the data bytes
// separated with a ZERO bit, followed by the checksum byte. The DCC track object
// builds the packet bit stream from the DCC packet data structure and then sends
// it to the track. Sending the bit stream is done with a state machine that is
// invoked every 29 microseconds via a hardware timer interrupt. It creates the
// actual DCC signal on the track output pins.
//
// After the generating the track signals, the timer interrupt handler invokes
// follow up actions that fetch the next DCC bit or packet, measure the actual 
// power consumption, read in a railcom message and so on. Internal events are 
// recorded for testing and debugging in an internal log.
//

// ??? text ....

//
//----------------------------------------------------------------------------------------
const uint8_t   RAILCOM_BUF_SIZE = 8;
const uint16_t  LOG_BUF_SIZE     = 8192;

struct LcsDccTrack {

    public:

    LcsDccTrack( );

    bool                setupDccTrack( LcsDccTrackDesc* trackDesc );

    void                loadPacket( const uint8_t *packet, 
                                    uint8_t len, 
                                    uint8_t repeat = 0 );

    uint16_t            getFlags( );
    uint16_t            getOptions( );
    uint8_t             getErrCode( );

    void                powerEnable( bool enable );
    bool                isPowerOn( );
    bool                isPowerOverload( );

    void                setTrackMode( DccTrackMode mode );
    DccTrackMode        getTrackMode( );

    void                setLimitCurrent( uint16_t val );
    uint16_t            getLimitCurrent( );
    uint16_t            getActualCurrent( );
    uint16_t            getMaxCurrent( );
    uint16_t            getRMSCurrent( );

    uint16_t            decoderAckBaseline( uint8_t resetPacketsToSend );
    bool                decoderAckDetect( uint16_t baseValue, uint8_t retries );
   
    void                runDccTrackStateMachine( 
                            volatile uint8_t *timeToInterrupt, 
                            uint8_t *followUpAction 
                        );

    void                getNextBit( );
    void                getNextPacket( );
    void                powerManagement( );

    void                startRailComIO( );
    void                stopRailComIO( );
    uint8_t             handleRailComMsg( );
    uint8_t             getRailComMsg( uint8_t *buf, uint8_t bufLen );

    uint32_t            getDccPacketsSend( );
  
    void                printDccTrackConfig( );
    void                printDccTrackStatus( );

    void                enableLog( bool arg );
    void                beginLog( );
    void                endLog( );
    void                printLog( );
    uint8_t             printLogEntry( uint16_t ofs );
    void                printLogTimeStamp( uint16_t ofs );
    void                printLogVal( uint16_t ofs );
    void                printLogData( uint16_t ofs, uint8_t len );

    void                writeLogData( uint8_t id, uint8_t *buf, uint8_t len );
    void                writeLogId( uint8_t id );
    void                writeLogTs( );
    void                writeLogVal( uint8_t valId, uint16_t val );
   
    private:

    uint16_t            options                             = DT_OPT_NIL;
    DccTrackMode        trackMode                           = DT_M_NIL;

    volatile uint16_t   flags                               = DT_F_NIL;
    uint8_t             errCode                             = 0;

    volatile uint8_t    trackState                          = 0;

    uint8_t             rNumEnable                          = 0;
    uint8_t             rNumControl                         = 0;
    uint8_t             rNumSense                           = 0;
    uint8_t             rNumUartRx                          = 0;

    uint16_t            referenceMilliVolt                  = 0;
    uint16_t            shuntMilliOhm                       = 0;
    uint16_t            amplifierGain                       = 0;
    uint16_t            adcBitResolution                    = 0;
    uint16_t            adcZeroOffset                       = 0;

    uint16_t            limitCurrentMilliAmp                = 0;
    uint16_t            maxCurrentMilliAmp                  = 0;
   
    volatile uint16_t   actualCurrentDigitValue             = 0;
    volatile uint16_t   highWaterMarkDigitValue             = 0;
    volatile uint16_t   limitCurrentDigitValue              = 0;

    volatile uint16_t   overloadEventCount                  = 0;
    volatile uint16_t   overloadEventThreshold              = 0;
    uint16_t            ackThresholdDigitValue              = 0;

    volatile uint16_t   sampleIntervalSize                  = 0;
    volatile uint16_t   sampleIntervalCount                 = 0;
                      
    volatile uint64_t   sampleSum                           = 0; 
    volatile uint32_t   sampleCount                         = 0;
    volatile uint32_t   sampleSize                          = 0;
   
    uint8_t             preambleLen                         = 0;
    uint8_t             postambleLen                        = 0;
    volatile bool       currentBit                          = false;
    volatile uint8_t    bytesSent                           = 0;
    volatile uint8_t    bitsSent                            = 0;
    volatile uint8_t    preambleSent                        = 0;
    volatile uint8_t    postambleSent                       = 0;
    uint32_t            dccPacketsSend                      = 0;

    DccPacket           dccBuf1                             = { 0 }; 
    DccPacket           dccBuf2                             = { 0 };
    DccPacket           *activeBufPtr                       = nullptr;
    DccPacket           *pendingBufPtr                      = nullptr;

    uint16_t            logBufOfs                           = 0;
    uint8_t             logBuf[ LOG_BUF_SIZE ]              = { 0 };

    uint8_t             railComBufIndex                     = 0;
    uint8_t             railComMsgBuf[ RAILCOM_BUF_SIZE ]   = { 0 };

    // ??? add base station capabilities according to RCN200 - 4 16 bit words
    // sample values per second for samples and dcc packets
    // ??? add buffers for POM / XPOM data
    // ??? add queue for POM / XPOM commands

    public:

    // ??? setup both ? how to return errors ?
    static void         setup( LcsDccTrackDesc *desc );

    LcsDccTrack         *getTrackMain( );
    LcsDccTrack         *getTrackProg( );
    static void         startDccProcessing( );
};
