//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime Library Firmware Update.
//
//----------------------------------------------------------------------------------------
// The file contains the part of the LCS Runtime Library that implements the 
// node event handling. At the heart of LCS is the concept of events. Events 
// are broadcasted by a node and any other node interested in them registers 
// an event callback. LCS runtime functions provide the management of the event
// map and the search routines.
//
// The event map is a NVM structure that is accessed at node startup. We build 
// a hand table based memory version from it. Adding, updating and removing 
// entries in the event map are performed on the NVM and the hash table. All 
// lookup is however done using the hash table.
//
//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime Library Firmware Update. 
// Copyright (C) 2020 - 2026 Helmut Fieres
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
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------
#include "LcsRuntimeLib.h"
#include "LcsRtLibInt.h"

//----------------------------------------------------------------------------------------
// External declaration to global structures and functions.
//
//----------------------------------------------------------------------------------------
namespace LCS {

    extern uint16_t         debugMask;
    extern LcsEventHashMap  eventHashMap;

    extern uint8_t  rtNvmPutBytes(uint32_t ofs, uint8_t *buf, uint32_t len);
    extern uint8_t  rtNvmGetBytes(uint32_t ofs, uint8_t *buf, uint32_t len);
};

//----------------------------------------------------------------------------------------
// The LcsCoreLib implementation file local declarations and routines.
//
//----------------------------------------------------------------------------------------
namespace {

using namespace LCS;

//----------------------------------------------------------------------------------------
// "eventsDebugEnabled" and "retStat" are the debug support routines. We can 
// easily check whether debug is enabled at all. The return status routine will
// print out a return status message when debugging is enabled. The macro 
// "RET_STAT" is a nice helper that adds the function name to the message.
// 
//----------------------------------------------------------------------------------------
inline bool eventsDebugEnabled(  ) {

    return (( debugMask & LCS_DBG_ENABLE ) && ( debugMask & LCS_DBG_EVENTS )); 
}

inline uint8_t retStat( char *name, uint8_t errId ) {

    if ( eventsDebugEnabled( )) {

        if ( errId == LCS_OK )  printf( "%s: OK\n", name );
        else                    printf( "%s: %d\n", name, errId );
    }

    return ( errId );
}

#define ENTER_FUNC() enterFunc((char *) __func__)
#define RET_STAT(x) retStat((char *) __func__, ( x ))

//----------------------------------------------------------------------------------------
// Fundamental constants. 
//
//----------------------------------------------------------------------------------------
const uint32_t  FETCH_CHUNK_SIZE = 8;

//----------------------------------------------------------------------------------------
// Hash function.
//
//----------------------------------------------------------------------------------------
static inline uint32_t hash( uint16_t x ) {

    x ^= x >> 7;
    x *= 0x9E37;
    x ^= x >> 9;
    return x;
}

//----------------------------------------------------------------------------------------
// Little helper functions to access the event map high water mark and an event
// map entry.
//
//----------------------------------------------------------------------------------------
uint8_t getEventMapHwm( uint32_t *hwm ) {

    return ( rtNvmGetBytes( NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, mapHwm ),
                           (uint8_t *) hwm,
                           sizeof(uint32_t)));
}

uint8_t putEventMapHwm( uint32_t hwm ) {

    return ( rtNvmPutBytes( NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, mapHwm ),
                           (uint8_t *) &hwm,
                           sizeof(uint32_t)));
}

uint8_t getEventMapEntry( uint32_t index, LcsEventMapEntry *e ) {

    if ( index > MAX_EVENT_MAP_ENTRIES ) return ( ERR_INVALID_EVENT_MAP_INDEX );

    uint32_t ofs = NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, map ) +
                    index * sizeof(LcsEventMapEntry);

    return( rtNvmGetBytes( ofs, (uint8_t*)e, sizeof(LcsEventMapEntry)));
}

uint8_t putEventMapEntry( uint32_t index, LcsEventMapEntry *e ) {

    if ( index > MAX_EVENT_MAP_ENTRIES ) return ( ERR_INVALID_EVENT_MAP_INDEX );

    uint32_t ofs = NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, map ) +
                    index * sizeof(LcsEventMapEntry);

    return( rtNvmPutBytes( ofs, (uint8_t*)e, sizeof(LcsEventMapEntry)));
}

//----------------------------------------------------------------------------------------
// "addEventEntryNvm" adds an event entry to the NVM eventMap. We check if there
// is room and if so write the new entry at the HWM position and update the HWM.
//
//----------------------------------------------------------------------------------------
uint8_t addEventEntryNvm( uint16_t eventId, uint16_t eventMask ) {

    LcsEventMapEntry entry;
    entry.eventId   = eventId;
    entry.eventMask = eventMask;

    uint8_t  rStat;
    uint32_t hwm;
    uint32_t addr;

    rStat = getEventMapHwm( &hwm );
    if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

    if ( hwm > MAX_EVENT_MAP_ENTRIES ) return ( RET_STAT ( ERR_EVENT_MAP_FULL ));

    rStat = putEventMapEntry( hwm, &entry );
    if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

    hwm ++;

    rStat = putEventMapHwm( hwm );
    if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

    return RET_STAT( LCS_OK );
}

//----------------------------------------------------------------------------------------
// Remove an entry from the NVM event map. We will search for the entry in the
// NVM event map and if found, we will remove it by replacing the slot with the
// last entry in the map and decreasing the HWM. This way we maintain a compact
// event map without holes.
//
//----------------------------------------------------------------------------------------
uint8_t removeEventEntryNvm( uint16_t eventId ) {

    uint8_t rStat;
    uint32_t hwm;
    uint32_t mapAddr =  NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, map );

    rStat = getEventMapHwm( &hwm );
    if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

    uint32_t i = 0;
    int32_t foundIndex = -1;

    LcsEventMapEntry buffer[FETCH_CHUNK_SIZE];

    // --- 1. Search ---
    while ( i < hwm ) {

        uint32_t remaining = hwm - i;
        uint32_t count = (remaining >= FETCH_CHUNK_SIZE) ? FETCH_CHUNK_SIZE : remaining;

        uint32_t addr = mapAddr + i * sizeof(LcsEventMapEntry);

        rStat = rtNvmGetBytes( addr, (uint8_t*)buffer, count * sizeof(LcsEventMapEntry));
        if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

        for ( uint32_t j = 0; j < count; j++) {

            if ( buffer[j].eventId == eventId ) {

                foundIndex = i + j;
                break;
            }
        }

        if ( foundIndex >= 0 ) break;

        i += count;
    }

    if ( foundIndex < 0 ) return( RET_STAT( LCS_OK ));
        
    uint32_t lastIndex = hwm - 1;

    // --- 2. If last entry, just shrink ---
    if ((uint32_t)foundIndex == lastIndex) {

        hwm--;
        return ( RET_STAT( LCS_OK ));
    }

    // --- 3. Read last entry ---
    LcsEventMapEntry lastEntry;

    rStat = getEventMapEntry( lastIndex, &lastEntry );
    if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

    // --- 4. Write last entry into removed slot ---
    rStat = putEventMapEntry( foundIndex, &lastEntry );
    if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

    // --- 5. Decrease HWM ---
    hwm--;

    return ( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// "getEventEntryByIndex" gets an entry by its actual index in the event map.
//
//----------------------------------------------------------------------------------------
uint8_t getEventEntryByIndex( uint32_t index, 
                              uint16_t *eventId, 
                              uint16_t *eventMask ) {

    uint8_t          rStat;
    uint32_t         hwm;
    LcsEventMapEntry e;
   
    rStat = getEventMapHwm( &hwm );
    if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

    if ( index >= hwm ) return( RET_STAT( ERR_INVALID_EVENT_MAP_INDEX ));

    rStat = getEventMapEntry( index, &e );
    if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

    *eventId   = e.eventId;
    *eventMask = e.eventMask;
    return( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// Lookup an event in the hash table.
//
//----------------------------------------------------------------------------------------
LcsEventMapEntry *lookupEventMapEntry( uint16_t eventId ) {

    if ( eventId == NIL_EVENT_ID ) return ( nullptr );

    uint32_t i = hash( eventId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    uint32_t start = i;

    printf( "lookupEventMapEntry, id: %d, hash: %d\n", eventId, i );

    while ( eventHashMap.map[ i ].eventId != NIL_EVENT_ID ) {

        if ( eventHashMap.map[ i ].eventId == eventId ) {

            printf( "lookupEventMapEntry: FOUND\n" );
            return ( &eventHashMap.map[ i ] );
        }

        i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        if ( i == start ) return ( nullptr );
    }

    return ( nullptr );
}

//----------------------------------------------------------------------------------------
// Insert into event hash table. Straightforward. There is a sanity check that
// we do not loop forever when the table is full. The routine expects that a
// lookup for the event was done, so we can assume the entry is not on the 
// table so far.
//
//----------------------------------------------------------------------------------------
void insertEventMapEntry( uint16_t eventId, uint16_t eventMask ) {

    uint32_t i = hash( eventId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    uint32_t start = i;

    if ( eventsDebugEnabled( )) {

        printf( "insertEventMapEntry, id: 0x%04x, mask: 0x%04x, hash: %d\n", 
                eventId, eventMask, i  );
    }

    while ( eventHashMap.map[ i ].eventId != NIL_EVENT_ID ) {

        i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        if ( i == start ) return;
    }

    eventHashMap.map[ i ].eventId = eventId;
    eventHashMap.map[ i ].eventMask = eventMask;

    eventHashMap.numEntries ++;
}

//----------------------------------------------------------------------------------------
// Remove from event hash table. We first try to find the entry. If found, we
// invalidate the entry and then fix the hash chain. Each entry after the 
// removed entry needs to be removed too and rehashed, so that we have valid 
// chains again.
//
//----------------------------------------------------------------------------------------
void removeEventMapEntry( uint16_t eventId ) {

    uint32_t i      = hash( eventId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    uint32_t start  = i;

    while ( eventHashMap.map[ i ].eventId != NIL_EVENT_ID ) {

        if ( eventHashMap.map [ i ].eventId == eventId ) break;
        i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        if ( i == start ) return;
    }

    if ( eventHashMap.map[ i ].eventId == NIL_EVENT_ID ) return;

    eventHashMap.map [ i ].eventId = NIL_CAB_ID;

    uint32_t j = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );

    while ( eventHashMap.map[ j ].eventId != NIL_EVENT_ID ) {

        LcsEventMapEntry tmp = eventHashMap.map [ j ];
        eventHashMap.map[ j ].eventId = NIL_EVENT_ID;

        uint32_t k = hash( tmp.eventId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        uint32_t kstart = k;

        while ( eventHashMap.map[ k ].eventId != NIL_EVENT_ID ) {

            k = ( k + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
            if ( k == kstart ) break;
        }

        eventHashMap.map[ k ] = tmp;
        j = ( j + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    }

    eventHashMap.numEntries --;
}

//----------------------------------------------------------------------------------------
// "loadEventMapFromNvm" fills the hash table with entries from the NVM event map.
// The high water mark indicates how many entries are in the NVM event map. If
// however the HWM is not valid, we examine each entry, load valid ones and 
// set the HWM to a valid value. To speed up the load, we read the data entries
// in chunks.
//
// ??? could remember highest loaded and patch up hwm ....
// ??? what would we do about an eventMap with empty entries in between ?
//----------------------------------------------------------------------------------------
uint8_t loadEventMapFromNvm( uint32_t mapOfs, uint32_t hwm ) {

    if ( eventsDebugEnabled( )) {

        printf( "Load event map table, ofs: 0x%4x, hwm: %d\n", mapOfs, hwm  );
    }

    for ( int i = 0; i < MAX_CAB_HASH_TAB_ENTRIES; i++ ) {
        
        eventHashMap.map [ i ].eventId = NIL_EVENT_ID;
        eventHashMap.map [ i ].eventMask = 0;
    }

    if ( hwm > MAX_EVENT_MAP_ENTRIES ) hwm = MAX_EVENT_MAP_ENTRIES;

    uint8_t  rStat  = LCS_OK;
    uint32_t i      = 0;
    LcsEventMapEntry buffer[ FETCH_CHUNK_SIZE ];

    while ( i < hwm ) {

        uint32_t remaining = hwm - i;
        uint32_t addr  = mapOfs + i * sizeof(LcsEventMapEntry);
        uint32_t count = ( remaining >= FETCH_CHUNK_SIZE) ? 
                         FETCH_CHUNK_SIZE : remaining;

        rStat = rtNvmGetBytes( addr, 
                               (uint8_t*)buffer, 
                               count * sizeof(LcsEventMapEntry));

        if ( rStat != LCS_OK ) return( RET_STAT( rStat ));

        for ( int j = 0; j < count; j++ ) {

            uint16_t eventId = buffer[ j ].eventId;

            if ( eventId != NIL_EVENT_ID ) {

                insertEventMapEntry( eventId, buffer[ j ].eventMask );
            }
        }

        i += count;
    }

    return( RET_STAT( rStat ));
}

} // namespace

//----------------------------------------------------------------------------------------
// The LCS name space routines declared in this file. These routines are visible
// at the firmware level.
//
//----------------------------------------------------------------------------------------
namespace LCS {

//----------------------------------------------------------------------------------------
// Setup event the event map from the NVM data. We will read the event map 
// from NVM in chunks and insert all entries into the hash table. We read up 
// to the HWM mark, which points right after the last element in the sorted 
// NVM event map.
// 
//----------------------------------------------------------------------------------------
uint8_t loadEventMap( ) {

    uint32_t hwm   = 0;
    uint8_t  rStat = LCS_OK;

    rStat = getEventMapHwm( &hwm );
    if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

    rStat = loadEventMapFromNvm( NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, map ),
                                 hwm );
    
    return( rStat );
}

//----------------------------------------------------------------------------------------
// Lookup in hash table.
//
//----------------------------------------------------------------------------------------
LcsEventMapEntry *lookupEvent( uint16_t eventId ) {

    if ( eventId == NIL_EVENT_ID ) return ( nullptr );
    return( lookupEventMapEntry( eventId ));
}

//----------------------------------------------------------------------------------------
// Add an eventId / eventMask pair to the NVM event map. We also add it to the
// hash table. If the entry already exists, we just change the event mask and
// update both hash entry and NVM entry. Updating the NVM structure is simply
// an remove and add.
//
//----------------------------------------------------------------------------------------
uint8_t addEvent( uint16_t eventId, uint16_t eventMask ) {

    if ( eventsDebugEnabled( )) {

        printf( "Add event, eventId: 0x04x, mask: 0x4x\n", eventId, eventMask );
    }

    if ( eventId == NIL_EVENT_ID ) return ( RET_STAT( ERR_INVALID_EVENT_ID ));

    LcsEventMapEntry *e = lookupEvent( eventId );
    if ( e == nullptr ) {

        uint8_t rStat = addEventEntryNvm( eventId, eventMask );
        if ( rStat == LCS_OK ) insertEventMapEntry( eventId, eventMask );
        return ( RET_STAT( rStat ));
    }
    else {

        uint8_t rStat = removeEventEntryNvm( eventId );
        if ( rStat == LCS_OK ) rStat = addEventEntryNvm( eventId, eventMask );
        if ( rStat == LCS_OK ) e -> eventMask = eventMask; 
        return ( RET_STAT( LCS_OK ));  
    }
}

//----------------------------------------------------------------------------------------
// Remove an event from the NVM event map. We also remove it from the hash table.
//
//----------------------------------------------------------------------------------------
uint8_t removeEvent( uint16_t eventId ) {

    if ( eventsDebugEnabled( )) {

        printf( "Remove event, eventId: 0x04x\n", eventId );
    }

    if ( eventId == NIL_EVENT_ID ) return ( RET_STAT( ERR_INVALID_EVENT_ID ));

    LcsEventMapEntry *l = lookupEvent( eventId );
    if ( l == nullptr ) return ( RET_STAT( LCS_OK ));

    uint8_t rStat = removeEventEntryNvm( eventId );
    if ( rStat == LCS_OK ) removeEventMapEntry( eventId );
    return( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "getMemEmapEntry" returns the eventId and event mask pair from the MEM event map.
// It is used by the console command interface and the LCS message request handler 
// to obtain that data. The index starts at 0.
//
//----------------------------------------------------------------------------------------
uint8_t getMemEmapEntry( uint16_t index, 
                         uint16_t *eventId, 
                         uint16_t *eventMask ) {

    if ( eventsDebugEnabled( )) printf( "Get Event Entry: %d\n", index );

    if ( ! isInRangeU16( index, MIN_EVENT_ID, MAX_EVENT_ID )) 
        return( RET_STAT ( ERR_INVALID_EVENT_MAP_INDEX ));

    return( RET_STAT( getEventEntryByIndex( index, eventId, eventMask )));
}

} // namespace LCS
