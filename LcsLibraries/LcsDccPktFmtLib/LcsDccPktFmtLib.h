//------------------------------------------------------------------------------------------------------------
//
// LCS DCC Packet Formatter - include file
//
//------------------------------------------------------------------------------------------------------------
// The DCC packet formatter is a set of static routines that will analyze and print a DCC packet in human 
// readable terms. 
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - DCC Packet Formatter
// Copyright (C) 2021 - 2022  Helmut Fieres
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
#ifndef LcsDccPkttFmtLib_h
#define LcsDccPkttFmtLib_h

//------------------------------------------------------------------------------------------------------------
// Include files.
//
//------------------------------------------------------------------------------------------------------------
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

#include <pico/stdio.h>
#include <pico/stdlib.h>

//------------------------------------------------------------------------------------------------------------
// The LCS DCC packet formatter declarations. There are a set of options for setting up the formatting options
// and the actual formatting method. The options allow for formatting DCC packets for locomotives, accessories
// and CV programming packets. Additionally, each packet can be also showns in a  hex or binary format.
//
//------------------------------------------------------------------------------------------------------------
struct LcsDccPacketFormatter {

  public:

    static int  formatDccPacketOpsMode( char *buf, uint16_t bufLen, uint8_t *dccPacket );
    static int  formatDccPacketSvcMode( char *buf, uint16_t bufLen, uint8_t *dccPacket );
    static int  formatDccPacketHex( char *buf, uint16_t bufLen, uint8_t *dccPacket );
    static int  formatDccPacketBin( char *buf, uint16_t bufLen, uint8_t *dccPacket );

    static bool isIdlePacket( uint8_t *dccPacket );
    static bool isResetPacket( uint8_t *dccPacket );
    static bool isOpsModeLocPkt( uint8_t *dccPacket );
    static bool isOpsModeAccPkt( uint8_t *dccPacket );
    static bool isSvcModePacket( uint8_t *dccPacket );
    static bool isValidDccPacket( uint8_t *dccPacket );
};

#endif
