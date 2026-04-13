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

#if 0
// ??? old version...
//----------------------------------------------------------------------------------------
// External declaration to global structures and functions.
//
//----------------------------------------------------------------------------------------
namespace LCS {

    extern uint16_t     debugMask;
    extern LcsEventMap  eventMap;  

    extern uint8_t      rtNvmPutWord( uint32_t ofs, uint16_t word );
    extern uint8_t      rtNvmPutBytes( uint32_t ofs, uint8_t *buf, uint32_t len );
    extern uint8_t      rtNvmPutBytes(uint32_t ofs, uint8_t *buf, uint32_t len);
    extern uint8_t      rtNvmGetBytes(uint32_t ofs, uint8_t *buf, uint32_t len);
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
// "compareEventEntry" is a little helper function to compare event and portId 
// to the data in an eventMap entry.
//
//----------------------------------------------------------------------------------------
int compareEventEntry( LcsEventMapEntry *e1, uint16_t eventId2 ) {

    if      ( e1 -> eventId < eventId2 )  return ( -1 );
    else if ( e1 -> eventId > eventId2 )  return ( 1 );
    else return ( 0 );
}

//----------------------------------------------------------------------------------------
// The event map search function performs a binary search of the event map. If
// the entry cannot be found, a -1 is returned.
//
//----------------------------------------------------------------------------------------
int searchEventMap( uint16_t eventId ) {

    int   res   = -1;
    int   low   = 0;
    int   high  = eventMap.mapHwm - 1;

    while ( low <= high ) {

        int mid = low + ( high - low + 1 ) / 2;

        if      ( eventMap.map[ mid ].eventId < eventId ) low  = mid + 1;
        else if ( eventMap.map[ mid ].eventId > eventId ) high = mid - 1;
        else if ( eventMap.map[ mid ].eventId == eventId ) {

            res   = mid;
            high  = mid - 1;
        }
    }
    
    if ( eventsDebugEnabled( )) {
        
        printf((char *) "searchEventMap: eventId: %d -> %d\n", eventId, res );
    }

    return ( res );
}

//----------------------------------------------------------------------------------------
// "addToMemEventMap" adds an event / port combination to the MEM event map if not
// already there. Given there is still room in the table, the entry is added in 
// sorted order.
//
//----------------------------------------------------------------------------------------
uint8_t addToMemEventMap( uint16_t eventId, uint16_t eventMask ) {

    int index = searchEventMap( eventId );

    if ( index >= 0 ) {

        eventMap.map[ index ].eventMask = eventMask; 
        return ( RET_STAT( LCS_OK ));
    }  

    if ( eventMap.mapHwm >= MAX_EVENT_MAP_ENTRIES ) {

         return ( RET_STAT( ERR_EVENT_MAP_FULL ));
    }
       
    index = eventMap.mapHwm;

    if ( eventMap.mapHwm > 0 ) {

        while (( index > 0 ) && 
               ( compareEventEntry( &eventMap.map[ index - 1 ], eventId ) > 0 )) {

            eventMap.map[ index ] = eventMap.map[ index - 1 ];
            index --;
        }
    }

    eventMap.map[ index ].eventId   = eventId;
    eventMap.map[ index ].eventMask = eventMask;
    eventMap.mapHwm++;

    return ( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// "removeFromMemEventMap" removes an entry from the memory event map. The sorted 
// order is maintained.
//
//----------------------------------------------------------------------------------------
uint8_t removeFromMemEventMap( uint16_t eventId ) {

    int index = searchEventMap( eventId );

    if ( index >= 0 ) {

        eventMap.mapHwm--;

        for ( uint16_t i = index; i < eventMap.mapHwm; i++ )
            eventMap.map[ i ] = eventMap.map[ i + 1 ];
    }

    return ( RET_STAT( LCS_OK ));
}

} // namespace

//----------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//----------------------------------------------------------------------------------------
namespace LCS {

//----------------------------------------------------------------------------------------
// The "addEventMask" routine will add or update an eventId/eventMask combination 
// in the event map.
//
//----------------------------------------------------------------------------------------
uint8_t setEventMask( uint16_t eventId, uint16_t eventMask ) {

    if ( ! isInRangeU( eventId, MIN_EVENT_ID, MAX_EVENT_ID )) { 
        
        return ( RET_STAT( ERR_INVALID_EVENT_ID ));
    }

    return ( addToMemEventMap( eventId, eventMask ));
}

//----------------------------------------------------------------------------------------
// The "removeEventMask" routine will remove an event Id / port Id from the MEM event 
// map.
//
//----------------------------------------------------------------------------------------
uint8_t removeEventMask( uint16_t eventId ) {

    if ( ! isInRangeU( eventId, MIN_EVENT_ID, MAX_EVENT_ID )) {
        
        return ( RET_STAT( ERR_INVALID_EVENT_ID ));
    }

    return ( removeFromMemEventMap( eventId ));
}

//----------------------------------------------------------------------------------------
// The event search function performs a binary search of the event map using the 
// event Id and the port Id. If the port Id is NIL, a matching entry with lowest 
// portId is returned. All eventMap entries with the same eventId follow. If the 
// entry cannot be found, a -1 is returned.
//
//----------------------------------------------------------------------------------------
int searchEvent( uint16_t eventId ) {

    if ( ! isInRangeU( eventId, MIN_EVENT_ID, MAX_EVENT_ID )) {
        
        return ( RET_STAT( ERR_INVALID_EVENT_ID ));
    }

    return ( searchEventMap( eventId ));
}

//----------------------------------------------------------------------------------------
// "syncEventMapToNvm" will write back the sorted MEM event map. We only write up
// to the HWM mark, which points right after the last element in the sorted MEM 
// event map. The idea is that all adds and removes are done on the MEM event map 
// and a SYNC control call will flush the sorted MEM event map to NVM.
//
//----------------------------------------------------------------------------------------
uint8_t syncEventMapToNvm( ) {

    uint8_t rStat = rtNvmPutBytes( NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, map ),
                                   (uint8_t *) eventMap.map, 
                                   eventMap.mapHwm * sizeof( LcsEventMapEntry ));

    if ( rStat == LCS_OK ) {

        uint32_t ofs = NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, mapHwm );
        rStat = rtNvmPutWord( ofs, eventMap.mapHwm );
    }

    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "syncEventMapToMem" will read back the sorted NVM event map. The event map stores
// all events this node is interested to process. The map is a sorted map of event 
// Id and port mask pairs. There is a high water mark, so that we only read up to 
// the last used entry in the map. We just read up to the HWM, if the HWM is valid.
// If not, we have to assume that there are issues with the event map. In this case
// we will read the entire  map entry by entry, add used entries, i.e. entries with
// a non-NIL event ID to the memory map. After reading all entries, the newly created 
// event map is written back to the NVM. We now have a valid map again and write 
// back to NVM so that there is also a valid map. This routine is also called during
// the node setup sequence. 
//
//----------------------------------------------------------------------------------------
uint8_t syncEventMapToMem( ) {

    uint32_t hwm   = 0;
    uint8_t  rStat = rtNvmGetBytes( NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, mapHwm ),
                                   (uint8_t *) &hwm,
                                   sizeof(uint32_t));
    if ( rStat == LCS_OK ) {

        if ( hwm < MAX_EVENT_MAP_ENTRIES ) {

            rStat = rtNvmGetBytes( NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, map ),
                                   (uint8_t *) &eventMap.map,
                                   hwm * sizeof( LcsEventMapEntry ));

            eventMap.mapHwm = hwm;
        }
        else {

            for ( int i = 0; i < MAX_EVENT_MAP_ENTRIES; i++ ) {

                LcsEventMapEntry e;
                eventMap.map[i] = e;
            }

            eventMap.mapHwm = 0;
            for ( int i = 0; i < MAX_EVENT_MAP_ENTRIES; i++ ) {

                LcsEventMapEntry e;

                rStat = rtNvmGetBytes( 
                            NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, map ) +
                                            (i * sizeof(LcsEventMapEntry)),
                            (uint8_t *) &e,
                            sizeof( LcsEventMapEntry ));

                if (( rStat == LCS_OK ) && ( e.eventId != NIL_EVENT_ID )) {

                    setEventMask( e.eventId, e.eventMask );
                }
            }

            rStat = syncEventMapToNvm( );
        }
    }

    return ( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "getMemEmapEntry" returns the eventId and event mask pair from the MEM event map.
// It is used by the console command interface and the LCS message request handler 
// to obtain that data. The index starts at 0.
//
//----------------------------------------------------------------------------------------
uint8_t getMemEmapEntry( uint16_t index, uint16_t *eventId, uint16_t *eventMask ) {

    if ( index <  eventMap.mapHwm ) {

        *eventId    = eventMap.map[ index ].eventId;
        *eventMask  = eventMap.map[ index ].eventMask;
        return ( RET_STAT( LCS_OK ));
    }
    else return ( RET_STAT( ERR_INVALID_EVENT_MAP_INDEX ));
}

} // namespace LCS

#else 

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

    return ( rtNvmGetBytes( NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, mapHwm ),
                           (uint8_t *) &hwm,
                           sizeof(uint32_t)));
}

uint8_t getEventMapEntry( uint32_t index, LcsEventMapEntry *e ) {

    if ( index > MAX_EVENT_MAP_ENTRIES ) return ( ERR_INVALID_EVENT_MAP_INDEX );

    uint32_t ofs = NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, map ) +
                    index * sizeof(LcsEventMapEntry);

    return( rtNvmGetBytes( ofs, (uint8_t*)&e, sizeof(LcsEventMapEntry)));
}

uint8_t putEventMapEntry( uint32_t index, LcsEventMapEntry *e ) {

    if ( index > MAX_EVENT_MAP_ENTRIES ) return ( ERR_INVALID_EVENT_MAP_INDEX );

    uint32_t ofs = NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, map ) +
                    index * sizeof(LcsEventMapEntry);

    return( rtNvmPutBytes( ofs, (uint8_t*)&e, sizeof(LcsEventMapEntry)));
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
//
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

    rStat = rtNvmGetBytes( NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, map ),
                           (uint8_t *) &e,
                           sizeof( e ));
    if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

    *eventId   = e.eventId;
    *eventMask = e.eventMask;
    return( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// Lookup in hash table.
//
//----------------------------------------------------------------------------------------
LcsEventMapEntry *lookupEventMapEntry( uint16_t eventId ) {

    if ( eventId == NIL_EVENT_ID ) return ( nullptr );

    uint32_t i = hash( eventId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    uint32_t start = i;

    while ( eventHashMap.map[ i ].eventId != NIL_EVENT_ID ) {

        if ( eventHashMap.map[ i ].eventId == eventId )
            return ( &eventHashMap.map[ i ] );

        i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        if ( i == start ) return ( nullptr );
    }

    return ( nullptr );
}

//----------------------------------------------------------------------------------------
// Insert into event hash table. Straightforward. There is a sanity check that
// we do not loop forever when the table is full.
//
//----------------------------------------------------------------------------------------
void insertEventMapEntry( uint16_t eventId, uint16_t eventMask ) {

    uint32_t i = hash( eventId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    uint32_t start = i;

    while ( eventHashMap.map[ i ].eventId != NIL_EVENT_ID ) {

        i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        if ( i == start ) return;
    }

    eventHashMap.map[ i ].eventId = eventId;
    eventHashMap.map[ i ].eventMask = eventMask;
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
}

//----------------------------------------------------------------------------------------
// "loadEventMapFromNvm" fills the hash table with entries from the NVM event map.
// The high water mark indicates how many entries are in the NVM event map. If
// however the HWM is not valid, we examine each entry, load valid ones and 
// set the HWM to a valid value. To speed up the load, we read the data entries
// in chunks.
//
// ??? could remember highest loaded and patch up hwm ....
//----------------------------------------------------------------------------------------
uint8_t loadEventMapFromNvm( uint32_t mapOfs, uint32_t hwm ) {

    if ( eventsDebugEnabled( )) {

        printf( "Load event map table, ofs: 0x4x, hwm: %d\n", mapOfs, hwm  );
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


#endif
