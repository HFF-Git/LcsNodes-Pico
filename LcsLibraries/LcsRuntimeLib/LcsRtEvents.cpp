//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime Library Firmware Update.
//
//----------------------------------------------------------------------------------------
// The file contains the part of the LCS Runtime Library that implements the node 
// event handling. At the heart of LCS is the concept of events. Events are 
// broadcasted by a node and any other node interested in them registers an event
// callback. LCS runtime functions provide the management of the event map and the 
// search routines.
//
// The event map can be found as a MEM and an NVM structure. During operations, the 
// sorted MEM event map is the map to work with. Entries are sorted by eventId. New 
// events can be added, old removed and the map can be searched. There is a SYNC 
// function to write the contents of the MEM event map to the NVM event map. The idea 
// is that all changes are made to the MEM version and then written back in one swoop.
//
// On node start or reset, the NVM event map is read as part of the overall setup 
// process. Since we only write a sorted version to the NVM event map, we can always 
// assume a sorted NVM version, except when the eventMap high water mark is not valid. 
// In this case we read entry by entry from the NVM and add it sorted to the MEM twin.
// The high water mark specifies the number of entires actually used.
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
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details. You 
// should have received a copy of the GNU General Public License along with this 
// program. If not, see <http://www.gnu.org/licenses/>.
//
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------
#include "LcsRuntimeLib.h"
#include "LcsRtLibInt.h"
#include <stdlib.h>

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
// "eventsDebugEnabled" and "retStat" are the debug support routines. We can easily 
// check whether debug is enabled at all. The return status routine will print 
// out a return status message when debugging is enabled. The macro "RET_STAT" 
// is a nice helper that adds the function name to the message.
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

#define RET_STAT(x) retStat((char *) __func__, ( x ))

//----------------------------------------------------------------------------------------
// "compareEventEntry" is a little helper function to compare event and portId to 
// the data in an eventMap entry.
//
//----------------------------------------------------------------------------------------
int compareEventEntry( LcsEventMapEntry *e1, uint16_t eventId2 ) {

    if      ( e1 -> eventId < eventId2 )  return ( -1 );
    else if ( e1 -> eventId > eventId2 )  return ( 1 );
    else return ( 0 );
}

//----------------------------------------------------------------------------------------
// The event map search function performs a binary search of the event map. If the
// entry cannot be found, a -1 is returned.
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

};
