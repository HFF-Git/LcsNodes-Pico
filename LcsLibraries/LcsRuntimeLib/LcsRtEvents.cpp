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
// The event map is a NVM structure that is accessed at node startup. We will 
// load the actual entries from the map into memory and sort the table for 
// quick lookup.
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
    extern LcsEventMap      eventMap;

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

inline uint8_t retStat( const char *name, uint8_t errId ) {

    if ( eventsDebugEnabled( )) {

        if ( errId == LCS_OK )  printf( "%s: OK\n", name );
        else                    printf( "%s: %d\n", name, errId );
    }

    return ( errId );
}

inline void enterFunc( const char *name ) {

    if ( eventsDebugEnabled( )) printf( "--> %s\n", name );
}

#define ENTER_FUNC() enterFunc( __func__)
#define RET_STAT(x) retStat( __func__, ( x ))

//----------------------------------------------------------------------------------------
// Comparison function for the event map entries sort.
//
//----------------------------------------------------------------------------------------
int compareEventId( const void *a, const void *b ) {

    const LcsEventMapEntry *x = (const LcsEventMapEntry *) a;
    const LcsEventMapEntry *y = (const LcsEventMapEntry *) b;

    if ( x -> eventId < y -> eventId ) return -1;
    if ( x -> eventId > y -> eventId ) return  1;

    return 0;
}

//----------------------------------------------------------------------------------------
// Sort the active portion of the event map.  The map is sorted by eventId.
// "mapHwm" is a byte offset. The C library qsort() function requires the
// number of elements. This routine expects that the parameters have been 
// validated before calling it. 
//
//
//----------------------------------------------------------------------------------------
void sortEventMap( ) {

    qsort( eventMap.map,
           eventMap.mapHwm / sizeof( LcsEventMapEntry ),
           sizeof( LcsEventMapEntry ),
           compareEventId );
}

//----------------------------------------------------------------------------------------
// Look up an event in the sorted event map. A successful lookup returns a 
// pointer to he entry. The search uses a half-open interval [first,last), 
// where last points one entry beyond the current search range.
//
//----------------------------------------------------------------------------------------
LcsEventMapEntry *lookupEventEntry( uint16_t eventId ) {

    size_t first = 0;
    size_t last  = eventMap.mapHwm / sizeof( LcsEventMapEntry );

    while ( first < last ) {

        size_t middle = first + ( last - first ) / 2;

        if      ( eventMap.map[ middle ].eventId < eventId ) first = middle + 1;
        else if ( eventMap.map[ middle ].eventId > eventId ) last = middle;
        else return( &eventMap.map[ middle ]);
    }

    return( nullptr );
}

//----------------------------------------------------------------------------------------
// Add a new event map entry to the MEM event map. If the event entry already 
// exists, we just update update the port mask. Otherwise, the new entry is 
// appended and the map is sorted again.
//
//----------------------------------------------------------------------------------------
uint8_t addToEventMap( const LcsEventMapEntry *entry ) {

    if ( eventMap.mapHwm > sizeof( eventMap.map )) return( ERR_EVENT_MAP_FULL );

    LcsEventMapEntry *e = lookupEventEntry( entry -> eventId );

    if ( e == nullptr ) {

        size_t index = eventMap.mapHwm / sizeof( LcsEventMapEntry );
        eventMap.map[ index ] = *entry;

        eventMap.mapHwm += sizeof( LcsEventMapEntry );
        sortEventMap( );
    }
    else e -> eventMask = entry -> eventMask;

    return( LCS_OK  );
}

//----------------------------------------------------------------------------------------
// Remove an event map entry. Since the map is sorted, find the entry and 
// replace it with the last active entry. The map is then sorted again. We 
// could also just move all entries above the HWM down by one, but since we
// do add and remove only during configuration, we can keep it simple and
// just sort the map again.
// 
//----------------------------------------------------------------------------------------
uint8_t removeFromEventMap( uint16_t eventId ) {

    size_t count = eventMap.mapHwm / sizeof( LcsEventMapEntry );

    for ( size_t i = 0; i < count; i++ ) {

        if ( eventMap.map[ i ].eventId == eventId ) {

            if ( i < count - 1 ) eventMap.map[ i ] = eventMap.map[ count - 1 ];
            eventMap.mapHwm -= sizeof( LcsEventMapEntry );

            sortEventMap( );
            return( LCS_OK );
        }

        if ( eventMap.map[ i ].eventId > eventId ) break;
    }

    return( LCS_OK  );
}

} // namespace

//----------------------------------------------------------------------------------------
// The LCS name space routines declared in this file. These routines are visible
// at the firmware level.
//
//----------------------------------------------------------------------------------------
namespace LCS {

//----------------------------------------------------------------------------------------
// loadEventMap() loads the event map from NVM. The complete map is loaded 
// and the header is validated. If the header is valid, the map is sorted for
// quick lookup. 
//
//----------------------------------------------------------------------------------------
uint8_t loadEventMap( ) {

    ENTER_FUNC();

    uint8_t rStat = rtNvmGetBytes( NVM_EVENT_MAP_OFS,
                                   (uint8_t *) &eventMap,
                                   sizeof( LcsEventMap ));
    if ( rStat != LCS_OK ) return( RET_STAT( rStat ));

    if (( eventMap.nvmOfs != NVM_EVENT_MAP_OFS )      ||
        ( eventMap.nvmSize != sizeof( LcsEventMap )) ||
        ( eventMap.mapHwm > sizeof( eventMap.map ))  ||
        (( eventMap.mapHwm % sizeof( LcsEventMapEntry )) != 0 ))
        return( RET_STAT( ERR_EVENT_MAP_HEADER ));

    sortEventMap( );
    return( RET_STAT( LCS_OK ));
}

//----------------------------------------------------------------------------------------
// "lookupEvent" searches the event map.
//
//----------------------------------------------------------------------------------------
LcsEventMapEntry *lookupEvent( uint16_t eventId ) {

    if ( eventsDebugEnabled( )) printf( "lookupEvent, eventId: %d\n", eventId );

    if ( eventId != NIL_EVENT_ID ) {
        
        LcsEventMapEntry *entry = lookupEventEntry( eventId );

        if ( eventsDebugEnabled( )) {

            if ( entry != nullptr ) printf( "found\n" );
            else                    printf( "not found\n" );
        }
        
        return ( entry );
    }
    else return ( nullptr );
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

        printf( "Add event, eventId: %d, mask: 0x%04x\n", eventId, eventMask );
    }

    if ( eventId == NIL_EVENT_ID ) return ( RET_STAT( ERR_INVALID_EVENT_ID ));

    LcsEventMapEntry e;
    e.eventId   = eventId;
    e.eventMask = eventMask;

    return( RET_STAT( addToEventMap( &e )));
}

//----------------------------------------------------------------------------------------
// Remove an event from the NVM event map. We also remove it from the map.
//
//----------------------------------------------------------------------------------------
uint8_t removeEvent( uint16_t eventId ) {

    if ( eventsDebugEnabled( )) printf( "Remove event, eventId: %d\n", eventId );

    if ( eventId == NIL_EVENT_ID ) return ( RET_STAT( ERR_INVALID_EVENT_ID ));

    return ( RET_STAT ( removeFromEventMap( eventId )));
}

//----------------------------------------------------------------------------------------
// updateEventMap() writes the active portion of the event map back to NVM.
// The header and unused map entries do not need to be written. Since mapHwm 
// is a byte offset, it can be used directly as the number of bytes to write.
//
//---------------------------------------------------------------------------------------
uint8_t updateEventMap( ) {

   ENTER_FUNC();

    uint8_t rStat = rtNvmPutBytes( NVM_EVENT_MAP_OFS +
                                   offsetof( LcsEventMap, map ),
                                   (uint8_t *) eventMap.map,
                                   eventMap.mapHwm );

    return( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "getEventEntryByIndex" gets an entry by its actual index in the event map.
// The index is the logical entry number, not a byte offset. 
//
//----------------------------------------------------------------------------------------
uint8_t getEventEntryByIndex( uint16_t index,
                              uint16_t *eventId,
                              uint16_t *eventMask ) {

    if ( eventsDebugEnabled( )) {

        printf( "getEventEntryByIndex, index: %d\n", index );
    }

    size_t nEntries = eventMap.mapHwm / sizeof( LcsEventMapEntry );

    if ((size_t) index >= nEntries )
        return( RET_STAT( ERR_INVALID_EVENT_MAP_INDEX ));

    *eventId   = eventMap.map[index].eventId;
    *eventMask = eventMap.map[index].eventMask;

    return( RET_STAT( LCS_OK ));
}

} // namespace LCS
