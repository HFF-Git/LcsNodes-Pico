///---------------------------------------------------------------------------------------
//
// LCS DCC Packet Formatter - Include file
//
///---------------------------------------------------------------------------------------
// The DCC packet formatter is a set of static routines that will analyze and print a DCC packet in human 
// readable terms. 
//
///---------------------------------------------------------------------------------------
//
// LCS - DCC Packet Formatter
// Copyright (C) 2021 - 2025  Helmut Fieres
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
#ifndef LcsDccPkttFmtLib_h
#define LcsDccPkttFmtLib_h

///---------------------------------------------------------------------------------------
// Include files.
//
///---------------------------------------------------------------------------------------
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

///---------------------------------------------------------------------------------------
// The LCS DCC packet formatter declarations. There are a set of options for setting up the formatting options
// and the actual formatting method. The options allow for formatting DCC packets for locomotives, accessories
// and CV programming packets. Additionally, each packet can be also shows in a  hex or binary format.
//
///---------------------------------------------------------------------------------------
namespace LcsDccPacketFormatter {

    int  formatDccPacketOpsMode( char *buf, uint16_t bufLen, uint8_t *dccPacket );
    int  formatDccPacketSvcMode( char *buf, uint16_t bufLen, uint8_t *dccPacket );
    int  formatDccPacketHex( char *buf, uint16_t bufLen, uint8_t *dccPacket );
    int  formatDccPacketBin( char *buf, uint16_t bufLen, uint8_t *dccPacket );

    bool isIdlePacket( uint8_t *dccPacket );
    bool isResetPacket( uint8_t *dccPacket );
    bool isOpsModeLocPkt( uint8_t *dccPacket );
    bool isOpsModeAccPkt( uint8_t *dccPacket );
    bool isSvcModePacket( uint8_t *dccPacket );
    bool isValidDccPacket( uint8_t *dccPacket );
};

#endif
