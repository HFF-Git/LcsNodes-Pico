//------------------------------------------------------------------------------------------------------------
//
// Layout Control System - implementation file.
//
//------------------------------------------------------------------------------------------------------------
// The file contains the part of the LCS Runtime Library that implements the node event handling. At the
// heart of LCS is the concept of events. Events are broadcasted by a node and any other node that is
// interested in it registers a callback or this event. The runtime functions provide the management of the
// event map and the search routines.
//
// The event map can be found as a MEM and an NVM structure. During operations, the soreted MEM event map is
// the  map to work with. Entries are sorted by eventId and as a secondary sort key the portId. New events
// can be added, old removed and the map can be searched. Thefre is a SYNC function to write the contents
// of the MEM event map to the NV event map. The idea is that all changes are made to the MEM version and
// then written back in one swoop.
//
// On node start or reset, the NVM event map is read as part of the overall NVM read process. Since we only
// write a sorted version to the NVM event map, we can always assume a sorted NVM version. The high water
// mark specifies the number of entires actually used.
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Core Library
// Copyright (C) 2021 - 2024  Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under the terms of the GNU
// General Public License as published by the Free Software Foundation, either version 3 of the License,
// or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
// License for more details. You should have received a copy of the GNU General Public License along with
// this program.  If not, see <http://www.gnu.org/licenses/>.
//
//------------------------------------------------------------------------------------------------------------
#include "LcsRuntimeLib.h"
#include "LcsRtLibInt.h"
#include <stdlib.h>

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
extern LCS::LcsNodeMap  nodeMap;
extern LCS::LcsEventMap eventMap;

//------------------------------------------------------------------------------------------------------------
// The LcsCoreLib implementation file local declarations and routines.
//
//------------------------------------------------------------------------------------------------------------
namespace {

  using namespace LCS;

  //----------------------------------------------------------------------------------------------------------  
  // Debug and Trace support. Instead of conditional cimpilation, we will print debug messages based on the
  // settoin of the debiug level.
  //---------------------------------------------------------------------------------------------------------- 
  uint8_t debugLevel = 0;

  //----------------------------------------------------------------------------------------------------------
  // Utility routines for number range check.
  //
  //----------------------------------------------------------------------------------------------------------
  bool isInRangeU( uint16_t val, uint16_t lower, uint16_t upper ) {

    return (( val >= lower ) && ( val <= upper ));
  }

  //-----------------------------------------------------------------------------------------------------------
  // "compareEventEntry" is a litle helper function to compare event and portId to the data in an eventMap
  // entry.
  //
  //-----------------------------------------------------------------------------------------------------------
  int compareEventEntry( LcsEventMapEntry *e1, uint16_t eventId2, uint16_t portId2 ) {

    if      ( e1 -> eventId < eventId2 )  return ( -1 );
    else if ( e1 -> eventId > eventId2 )  return ( 1 );
    else if ( e1 -> portId < portId2 )    return ( -1 );
    else if ( e1 -> portId > portId2 )    return ( 1 );
    else return ( 0 );
  }

  int compareEventEntry( const void *arg1, const void *arg2 ) {

    LcsEventMapEntry *e1 = (LcsEventMapEntry *)  arg1;
    LcsEventMapEntry *e2 = (LcsEventMapEntry *)  arg2;

    if      ( e1 -> eventId < e2 -> eventId )  return ( -1 );
    else if ( e1 -> eventId > e2 -> eventId )  return ( 1 );
    else if ( e1 -> portId  < e2 -> portId )   return ( -1 );
    else if ( e1 -> portId  > e2 -> portId )   return ( 1 );
    else return ( 0 );
  }

  //----------------------------------------------------------------------------------------------------------
  // "addToMemEventMap" adds an event / port combination to the MEM event map if not already there. Given
  // there is still room in the table, the entry is added in sorted order.
  //
  //----------------------------------------------------------------------------------------------------------
  uint8_t addToMemEventMap( uint16_t eventId, uint16_t portId ) {

    #if DEBUG_EVENTS == 1
    printf( "Add to MEM Event Map: %d : %d \n", eventId, portId );
    #endif

    if ( searchEvent( eventId, portId ) >= 0 )    return ( ALL_OK );
    if ( nodeMap.eventMapHwm >= MAX_EVENT_MAP_ENTRIES )  return ( ERR_EVENT_MAP_FULL );

    uint16_t index = nodeMap.eventMapHwm;

    if ( nodeMap.eventMapHwm > 0 ) {

      while (( index > 0 ) && ( compareEventEntry( &eventMap.map[ index - 1 ], eventId, portId ) > 0 )) {

        eventMap.map[ index ] = eventMap.map[ index - 1 ];
        index --;
      }
    }

    eventMap.map[ index ].eventId = eventId;
    eventMap.map[ index ].portId  = portId;
    nodeMap.eventMapHwm++;

    return ( ALL_OK );
  }

  //----------------------------------------------------------------------------------------------------------
  // "removeFromMemEventMap" removes an entry from the memory event map. The sorted order is maintained.
  //
  //----------------------------------------------------------------------------------------------------------
  uint8_t removeFromMemEventMap( uint16_t eventId, uint16_t portId ) {

    #if DEBUG_EVENTS == 1
    printf( "Remove from MEM Event Map: %d : %d \n", eventId, portId );
    #endif

    int index = searchEvent( eventId, portId );

    if ( index >= 0 ) {

      nodeMap.eventMapHwm--;

      for ( uint16_t i = index; i < nodeMap.eventMapHwm; i++ )
        eventMap.map[ i ] = eventMap.map[ i + 1 ];
    }

    return ( ALL_OK );
  }

} // namespace

//------------------------------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

//------------------------------------------------------------------------------------------------------------
// The "addEvent" routine will add an eventId/portId combination to the event map if not already there. If
// the portId parameter is a NIL_PORT_ID, the event is added to all portMap entries.
//
//------------------------------------------------------------------------------------------------------------
uint8_t addEvent( uint16_t eventId, uint16_t portId ) {

  #if DEBUG_EVENTS == 1
  printf( "Add Event: %d : %d\n", eventId, portId );
  #endif

  int rStat = ALL_OK;

  if ( ! isInRangeU( eventId, MIN_EVENT_ID, MAX_EVENT_ID )) return ( ERR_INVALID_EVENT_ID );

  if ( portId == NIL_PORT_ID ) {

    for ( uint8_t p = 1; p <= MAX_PORT_MAP_ENTRIES; p++ ) {

      rStat = addToMemEventMap( eventId, p );
      if ( rStat != ALL_OK ) break;
    }
  }
  else if ( isInRangeU( portId, MIN_PORT_ID, MAX_PORT_ID )) {

    rStat = addToMemEventMap( eventId, portId );
  }
  else rStat = ERR_INVALID_PORT_ID;

  return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// The "removeEvent" routine will remove an event Id / port Id from the MEM event map. If the port ID is
// NIL_PORT_ID, all port map entries matching event Id are removed.
//
//------------------------------------------------------------------------------------------------------------
uint8_t removeEvent( uint16_t eventId, uint16_t portId ) {

  #if DEBUG_EVENTS == 1
  printf( "Remove Event: %d : %d\n", eventId, portId );
  #endif

  int rStat = ALL_OK;

  if ( ! isInRangeU( eventId, MIN_EVENT_ID, MAX_EVENT_ID )) return ( ERR_INVALID_EVENT_ID );

  if ( portId == NIL_PORT_ID ) {

    for ( uint16_t p = 1; p <= MAX_PORT_MAP_ENTRIES; p++ ) {

      rStat = removeFromMemEventMap( eventId, p );
      if ( rStat != ALL_OK ) break;
    }
  }
  else if ( isInRangeU( portId, MIN_PORT_ID, MAX_PORT_ID )) {

    rStat = removeFromMemEventMap( eventId, portId );
  }
  else rStat = ERR_INVALID_PORT_ID;

  return ( rStat );
}

//------------------------------------------------------------------------------------------------------------
// The event search function performs a binary search of the event map using the event Id and the port Id.
// If the port Id is NIL, a matching entry with lowest portId is returned. All eventMap entries with the
// same eventId follow. If the entry cannot be found, a -1 is returned.
//
//------------------------------------------------------------------------------------------------------------
int searchEvent( uint16_t eventId, uint16_t portId ) {

  #if DEBUG_EVENTS == 1
  printf( "Search Event: %d : %d", eventId, portId );
  #endif

  int   res   = -1;
  int   low   = 0;
  int   high  = nodeMap.eventMapHwm - 1;

  if ( portId == NIL_PORT_ID ) {

    while ( low <= high ) {

      int mid = low + ( high - low + 1 ) / 2;

      if      ( eventMap.map[ mid ].eventId < eventId ) low  = mid + 1;
      else if ( eventMap.map[ mid ].eventId > eventId ) high = mid - 1;
      else if ( eventMap.map[ mid ].eventId == eventId ) {

        res   = mid;
        high  = mid - 1;
      }
    }
  }
  else {

    while ( low <= high ) {

      int mid = low + ( high - low ) / 2;
      int tst = compareEventEntry( &eventMap.map[ mid ], eventId, portId );

      if      ( tst < 0 ) low   = mid + 1;
      else if ( tst > 0 ) high  = mid - 1;
      else {

        res = mid;
        break;
      }
    }
  }

  #if DEBUG_EVENTS == 1
  printf( "-> %d\n", res );
  #endif

  return ( res );
}

//------------------------------------------------------------------------------------------------------------
// "syncEventMap" will write back the sorted MEM event map. We only write up to the HWM mark, which points
// right after the last element in the sorted MEM event map. The idea is that all adds and removes are done
// on teh MEM event map and a SYNC control call will flush the sorted MEM event map to NVM.
//
//------------------------------------------------------------------------------------------------------------
uint8_t syncEventMap( ) {

  // ??? simply write back the event map... sort it first, just to be sure ?
  // ?? make sure that we only have used entries below HWM before using sort up to the HWM!!!!

  qsort( &eventMap.map, MAX_EVENT_MAP_ENTRIES, sizeof( LcsEventMapEntry ), compareEventEntry );

  // update HWM , flags 

  return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// "getMemEmapEntry" returns the eventId and portId pair from the MEM event map. It is used by the console
// command interface and also the LCS message handler to obtain that data. The index starts at 0.
//
//------------------------------------------------------------------------------------------------------------
uint8_t getMemEmapEntry( uint16_t index, uint16_t *evId, uint16_t *pId ) {

  #if DEBUG_EVENTS == 1
  printf( "Get Emap Entry: %d\n", index );
  #endif

  if ( index <  nodeMap.eventMapHwm ) {

    *evId = eventMap.map[ index ].eventId;
    *pId  = eventMap.map[ index ].portId;
    return ( ALL_OK );
  }
  else return ( ERR_INVALID_EVENT_MAP_INDEX );
}

};
