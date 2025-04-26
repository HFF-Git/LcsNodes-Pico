//------------------------------------------------------------------------------------------------------------
//
// LCS Base Station - Locomotive Dictionary - implementation file
//
//------------------------------------------------------------------------------------------------------------
// The base station is the main node that manages among other things the active locomotive session. When a 
// session is established, it is necessary to get the default configuration for the particular locomotive.
// The locomotive dictionary is the part of the base station firmware that holds this kind of information.
// The locomotive dictionary is access by using the cabId as the key. The data for a locomotive is entered
// via dedicated node control items.
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Base Station
// Copyright (C) 2019 - 2025  Helmut Fieres
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
// External global variables.
//
//------------------------------------------------------------------------------------------------------------
extern uint16_t debugMask;

//------------------------------------------------------------------------------------------------------------
// Local declarations.
//
//------------------------------------------------------------------------------------------------------------
namespace {


}; // namespace


//------------------------------------------------------------------------------------------------------------
// Local declarations.
//
//------------------------------------------------------------------------------------------------------------
#if 0

// first ideas....

    struct LcsBaseStationLocoDictEntry {

    uint16_t  flags = 0;
    uint16_t  cabId = NIL_CAB_ID;
    uint8_t   functions[ MAX_DCC_FUNC_GROUP_ID ];

    // what else ? 
    // Mapping of functions for cab handheld ? 
    // Initial speed and direction ?

    };

    struct LcsBaseStationLocoDict {

        public:

        uint8_t setupLocoDict( );
        uint8_t lookupLocoDictEntry( uint16_t cabId, uint16_t *entryIndex );
        uint8_t addLocoDictEntry(  uint16_t cabId, uint16_t flags, ... );
        uint8_t removeLocoDictEntry(  uint16_t cabId );
        uint8_t updateLocoDictEntry(  uint16_t cabId, uin16_t flags, ...  );

        private:

        uint16_t                      numfEntries;
        LcsBaseStationLocoDictEntry   *locoDictMaxEntry  = nullptr;
        LcsBaseStationLocoDictEntry   *locoFDictHwm      = nullptr;
        LcsBaseStationLocoDictEntry   *locoDict          = nullPtr;
    };

#endif
