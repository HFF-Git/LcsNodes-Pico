//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime Library Firmware Update.
//
//----------------------------------------------------------------------------------------
// The file contains the part of the LCS Runtime Library that implements the node 
// event handling. At the heart of LCS is the concept of events. Events are broadcasted
// by a node and any other node interested in them registers an event callback. LCS 
// runtime functions provide the management of the event map and the search routines.
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
// Copyright (C) 2022 - 2025 Helmut Fieres
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

    extern int          searchEvent( uint16_t eventId );
    extern uint8_t      rtNvmPutWord( uint32_t ofs, uint16_t word );
    extern uint8_t      rtNvmPutBytes( uint32_t ofs, uint8_t *buf, uint32_t len );
};

//----------------------------------------------------------------------------------------
// The LcsCoreLib implementation file local declarations and routines.
//
//----------------------------------------------------------------------------------------
namespace {

using namespace LCS;

//----------------------------------------------------------------------------------------
// Utility routines.
//
//----------------------------------------------------------------------------------------
uint8_t errStat( uint8_t errId ) {

    if (( debugMask & LCS_DBG_ENABLE ) && ( debugMask & LCS_DBG_EVENTS )) {

        printf( "Ret: %d\n", errId );
    }

    return ( errId );
}

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

    if (( debugMask & LCS_DBG_ENABLE ) && ( debugMask & LCS_DBG_EVENTS )) {
        
        printf( "Search Event Map: %d ", eventId );
    }
    
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
    
    if (( debugMask & LCS_DBG_ENABLE ) && ( debugMask & LCS_DBG_EVENTS )) {
        
        printf( "-> %d\n", res );
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

    if (( debugMask & LCS_DBG_ENABLE ) && ( debugMask & LCS_DBG_EVENTS )) {

        printf( "addToMemEventMap: %d : 0x%x\n", eventId, eventMask );
    }

    int index = searchEvent( eventId );

    if ( index >= 0 ) {

        eventMap.map[ index ].eventMask; 
        return ( errStat( NO_ERR ));
    }  

    if ( eventMap.mapHwm >= MAX_EVENT_MAP_ENTRIES ) {

         return ( errStat( ERR_EVENT_MAP_FULL ));
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

    return ( errStat( NO_ERR ));
}

//----------------------------------------------------------------------------------------
// "removeFromMemEventMap" removes an entry from the memory event map. The sorted 
// order is maintained.
//
//----------------------------------------------------------------------------------------
uint8_t removeFromMemEventMap( uint16_t eventId ) {

     if (( debugMask & LCS_DBG_ENABLE ) && ( debugMask & LCS_DBG_EVENTS )) {
        
        printf( "removeFromMemEventMap: %d\n", eventId );
    }

    int index = searchEvent( eventId );

    if ( index >= 0 ) {

        eventMap.mapHwm--;

        for ( uint16_t i = index; i < eventMap.mapHwm; i++ )
            eventMap.map[ i ] = eventMap.map[ i + 1 ];
    }

    return ( errStat( NO_ERR ));
}

} // namespace

//----------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//----------------------------------------------------------------------------------------
namespace LCS {

//----------------------------------------------------------------------------------------
// The "addEvent" routine will add or update an eventId/eventMask combination in the
// event map.
//
//----------------------------------------------------------------------------------------
uint8_t addEvent( uint16_t eventId, uint16_t eventMask ) {

    if ( ! isInRangeU( eventId, MIN_EVENT_ID, MAX_EVENT_ID )) { 
        
        return ( errStat( ERR_INVALID_EVENT_ID ));
    }

    return ( errStat( addToMemEventMap( eventId, eventMask )));
}

//----------------------------------------------------------------------------------------
// The "removeEvent" routine will remove an event Id / port Id from the MEM event 
// map.
//
//----------------------------------------------------------------------------------------
uint8_t removeEvent( uint16_t eventId ) {

    if ( ! isInRangeU( eventId, MIN_EVENT_ID, MAX_EVENT_ID )) {
        
        return ( errStat( ERR_INVALID_EVENT_ID ));
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
        
        return ( errStat( ERR_INVALID_EVENT_ID ));
    }

    return ( searchEventMap( eventId ));
}

//----------------------------------------------------------------------------------------
// "syncEventMap" will write back the sorted MEM event map. We only write up to the
// HWM mark, which points right after the last element in the sorted MEM event map. 
// The idea is that all adds and removes are done on the MEM event map and a SYNC 
// control call will flush the sorted MEM event map to NVM.
//
//----------------------------------------------------------------------------------------
uint8_t syncEventMap( ) {

    if (( debugMask & LCS_DBG_ENABLE ) && ( debugMask & LCS_DBG_EVENTS )) {
        
        printf( "syncEventMap \n" );  
    }

    uint8_t rStat =  
        rtNvmPutBytes( NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, map ),
                       (uint8_t *) eventMap.map, 
                       eventMap.mapHwm * sizeof( LcsEventMapEntry ));

    if ( rStat == ALL_OK ) {

        uint32_t ofs = NVM_EVENT_MAP_OFS + offsetof( LcsEventMap, mapHwm );
        rStat = rtNvmPutWord( ofs, eventMap.mapHwm );
    }

    return ( errStat( rStat ));
}

//----------------------------------------------------------------------------------------
// "getMemEmapEntry" returns the eventId and event mask pair from the MEM event map.
// It is used by the console command interface and the LCS message request handler 
// to obtain that data. The index starts at 0.
//
//----------------------------------------------------------------------------------------
uint8_t getMemEmapEntry( uint16_t index, uint16_t *eventId, uint16_t *eventMask ) {

    if (( debugMask & LCS_DBG_ENABLE ) && ( debugMask & LCS_DBG_EVENTS )) {
        
        printf( "getMemEmapEntry: %d\n", index );
    }
  
    if ( index <  eventMap.mapHwm ) {

        *eventId    = eventMap.map[ index ].eventId;
        *eventMask  = eventMap.map[ index ].eventMask;
        return ( errStat( ALL_OK ));
    }
    else return ( errStat( ERR_INVALID_EVENT_MAP_INDEX ));
}

};
