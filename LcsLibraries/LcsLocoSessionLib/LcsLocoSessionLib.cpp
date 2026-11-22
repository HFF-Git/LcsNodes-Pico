///---------------------------------------------------------------------------------------
//
// LCS - Loco Session Lib
//
///---------------------------------------------------------------------------------------
// The library contains the locomotive dictionary which is a simple array of 
// locomotive descriptors. The dictionary is stored in the global NVM attribute 
// range. An entry is therefore carefully mapped to item numbers and alignment.
// If the locomotive cabId is not in this roster, it does not exist.
//
// Commands for a locomotive trigger the ad hoc creation of a locomotive session.
// Although we can store hundreds of locomotives in the dictionary, we cannot
// keep track of so many active sessions. A session is defined as a locomotive
// receiving commands. These engines are refreshed, i.e. their last command is
// repeated according to the DCC standard. When no commands are received in a 
// given time period, the locomotive is removed from the active list. 
//
// At system startup, the dictionary entries are copied to a MEM structure. This
// is our cab data base. It is a sorted table. Adding and removing an entry will
// be done at NVM level and then we re-sort the MEM table. The NVM data is 
// not sorted.
//
// An auxiliary structure contains the entry index of the currently active
// cabs. Looking up a cab in the cabMap result in adding the index to the list.
// A refresh mechanism will remove indices of non-active cabs. 
//
// The majority of the library functions are to build the DCC packets that we 
// send to the cabs. Also, for DCC the CV programming mode packets are created.
// 
///---------------------------------------------------------------------------------------
//
// LCS - Controller Dependent Code - Raspberry PI Pico Implementation
// Copyright (C) 2020 - 2026  Helmut Fieres
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
#include "LcsLocoSessionLib.h"

///---------------------------------------------------------------------------------------
// Local name space.
//
///---------------------------------------------------------------------------------------
namespace {

using namespace LCS;

//----------------------------------------------------------------------------------------
// Debug support. The following routines control the printing of debug messages.
// 
//  ENTER_FUNC - a macro to just print that we are entering the function.
//  REET_STAT  - a macro that prints we are leaving along with status code.
// 
// ??? need a concept to share the RtLib debug mask...
//----------------------------------------------------------------------------------------
uint16_t debugMask;

inline bool sLibDebugEnabled( ) {

    return (( debugMask & LCS_DBG_ENABLE ) && ( debugMask & LCS_DBG_SETUP )); 
}

inline void enterFunc( char *name ) {

    if ( sLibDebugEnabled( )) usbIoPrintf( CDC_USB_CONSOLE, "--> %s\n", name );
}

inline uint8_t retStat( char *name, uint8_t errId ) {

    if ( sLibDebugEnabled( )) {

        if ( errId == LCS_OK )  
            usbIoPrintf( CDC_USB_CONSOLE, "<-- %s: OK\n", name );
        else        
            usbIoPrintf( CDC_USB_CONSOLE, "<-- %s: %d\n", name, errId );
    }

    return ( errId );
}

#define ENTER_FUNC() enterFunc((char *) __func__)
#define RET_STAT(x) retStat((char *) __func__, ( x ))

//----------------------------------------------------------------------------------------
// DCC packet definitions. A DCC packet payload is at most 10 bytes long, 
// excluding the checksum byte. This is true for XPOM support, otherwise it 
// is according to NMRA up to 6 bytes.
//
//----------------------------------------------------------------------------------------
const uint8_t   MIN_DCC_PACKET_SIZE         = 2;
const uint8_t   MAX_DCC_PACKET_SIZE         = 16;
const uint8_t   MIN_DCC_PACKET_REPEATS      = 0;
const uint8_t   MAX_DCC_PACKET_REPEATS      = 8;

//----------------------------------------------------------------------------------------
// Constant values definition. We need the RESET and IDLE packet as well as a 
// bit mask for a quick bit select in the data byte.
//
//----------------------------------------------------------------------------------------
const uint8_t   idleDccPacketData[ ]    = { 0xFF, 0x00 };
const uint8_t   resetDccPacketData[ ]   = { 0x00, 0x00 };
const uint8_t   eStopDccPacketData[ ]   = { 0x00, 0x01 };

//----------------------------------------------------------------------------------------
// Utility routines.
//
// ??? what of that should actually be in the runtime lib ?
//----------------------------------------------------------------------------------------
bool validCabId( uint16_t cabId ) {

    return ( isInRangeU( cabId, MIN_CAB_ID, MAX_CAB_ID ));
}

bool validCvId( uint16_t cvId ) {

    return ( isInRangeU( cvId, MIN_DCC_CV_ID, MAX_DCC_CV_ID ));
}

bool validFunctionId( uint8_t fId ) {

    return ( fId <= MAX_DCC_FUNC_ID );
}

bool validFunctionGroupId( uint8_t fGroup ) {

    return ( isInRangeU( fGroup, MIN_DCC_FUNC_GROUP_ID , MAX_DCC_FUNC_GROUP_ID ));
}

bool validDccPacketlen( uint8_t len ) {

    return ( len <= MAX_DCC_PACKET_SIZE );
}

bool validDccPacketRepeatCnt( uint8_t nRepeat ) {

    return ( nRepeat <= MAX_DCC_PACKET_REPEATS );
}

uint8_t bitRead( uint8_t arg, uint8_t pos ) {

    return ( arg >> ( pos % 8 )) & 1;
}

void bitWrite( uint8_t *arg, uint8_t pos, bool val ) {

    if ( val )  *arg |= ( 1 << pos );  
    else        *arg &= ~( 1 << pos ); 
}

//----------------------------------------------------------------------------------------
// DDC function flags. The DCC function flags F0 .. F68 are stored in ten 
// groups.  A DCC function group is labelled starting with index 1. Group 1 
// contains F0 .. F4, group 2 contains F5 .. F8, Group 3 contains F9 .. F12 
// in DCC command byte format. The remainder F13 .. F68 are stored in 8 bits
// groups also in DCC command byte format. The routines support the get/set 
// of an individual bit as well as setting an entire function group. 
//
//
//----------------------------------------------------------------------------------------
bool getDccFuncBit( uint8_t *funcFlags, uint8_t fNum ) {

    if ( fNum == 0 ) {

        return ( bitRead( funcFlags[ 0 ], 4 ));
    }
    else if ( isInRangeU( fNum, 1, 4 )) {

        return ( bitRead( funcFlags[ 0 ], fNum - 1 ));
    }
    else if ( isInRangeU( fNum, 5, 8 )) {

        return ( bitRead( funcFlags[ 1 ], fNum - 5 ));
    }
    else if ( isInRangeU( fNum, 9, 12 )) {

        return ( bitRead( funcFlags[ 2 ], fNum - 9 ));
    }
    else if ( isInRangeU( fNum, 13, 68 )) {

        return ( bitRead( funcFlags[ ( fNum - 13 ) / 8 + 3 ], ( fNum - 13 ) % 8 ));
    }   
    else return ( false );
}

void setDccFuncBit( uint8_t *funcFlags, uint8_t fNum, bool val ) {

    if ( fNum == 0 ) {
                        
        bitWrite( &funcFlags[ 0 ], 4, val );
    }
    else if ( isInRangeU( fNum, 1, 4 )) {

        bitWrite( &funcFlags[ 0 ], fNum - 1, val );
    }
    else if ( isInRangeU( fNum, 5, 8 )) {   
        
        bitWrite( &funcFlags[ 1 ], fNum - 5, val );
    }
    else if ( isInRangeU( fNum, 9, 12 )) {
        
        bitWrite( &funcFlags[ 2 ], fNum - 9, val );
    }
    else if ( isInRangeU( fNum, 13, 68 )) {

        bitWrite( &funcFlags[ ( fNum - 13 ) / 8 + 3 ], ( fNum - 13 ) % 8, val );
    }
}

void setDccFuncGroupByte( uint8_t *funcFlags, uint8_t fGroup, uint8_t dccByte ) {

    if       ( fGroup == 1 )                funcFlags[ 0 ] = dccByte & 0x1F;
    else if  ( fGroup == 2 )                funcFlags[ 1 ] = dccByte & 0x0F;
    else if  ( fGroup == 3 )                funcFlags[ 2 ] = dccByte & 0x0F;
    else if  ( isInRangeU( fGroup, 4, 10 )) funcFlags[ fGroup - 1 ] = dccByte;
}

uint8_t dccFunctionBitToGroup( uint8_t fNum ) {

    if      ( isInRangeU( fNum, 0, 4 ))     return ( 1 );
    else if ( isInRangeU( fNum, 5, 8 ))     return ( 2 );
    else if ( isInRangeU( fNum, 9, 12 ))    return ( 3 );
    else if ( isInRangeU( fNum, 13, 68 ))   return (( fNum - 13 ) / 8 + 4 );
    else                                    return ( 0 );
}

//----------------------------------------------------------------------------------------
// CabId sort support.
//
//----------------------------------------------------------------------------------------
int compareCabId( const void *a, const void *b ) {

    const LcsCabEntry *x = (const LcsCabEntry *) a;
    const LcsCabEntry *y = (const LcsCabEntry *) b;

    if ( x -> cabId < y -> cabId ) return -1;
    if ( x -> cabId > y -> cabId ) return 1;
    return 0;
}

void sortCabMap( LcsCabEntry *map, uint16_t len ) {

    qsort( map, len, sizeof( LcsCabEntry ), compareCabId );
}

//----------------------------------------------------------------------------------------
// Cab data structures initialization routines.
//
//----------------------------------------------------------------------------------------
void initCabEntry( LcsCabEntry *e ) {

    memset( e, 0, sizeof(LcsCabEntry));
}

void initCabMap( LcsCabMap *cMap ) {

    cMap -> magicWord           = CAB_MAP_MAGIC_WORD;
    cMap -> flags               = 0;
    cMap -> maxCabCount         = MAX_CAB_DICT_SESSIONS;           
    cMap -> currentCabCount     = 0;                           
    cMap -> cabEntryItemSize    = CAB_ENTRY_ITEM_SIZE;  
    cMap -> rsv1                = 0;
    cMap -> rsv2                = 0;
    
    for ( int i = 0; i < MAX_CAB_DICT_SESSIONS; i ++ ) {
        
        initCabEntry( & (cMap -> map[ i ] ));
    }
}

void initActiveCabMap( LcsActiveCabMap *aMap ) {

    aMap -> activeCabCount = 0;
    aMap -> refreshIndex   = 0;

    for ( int i = 0; i < MAX_CAB_ACTIVE_SESSIONS; i ++ ) {
        
        aMap -> tsMap[ i ] = 0;
        aMap -> map[ i ] = -1;
    }
}

}; // namespace

//========================================================================================
//========================================================================================

//----------------------------------------------------------------------------------------
// "LocoSession" constructor. Nothing to do here.
//
//----------------------------------------------------------------------------------------
LcsLocoSessions::LcsLocoSessions( ) { }

//----------------------------------------------------------------------------------------
// "setupLocoSessions" initializes the session data. We are passed the options
// as well as the two DCC tracks.
//
// ??? use options to also pass the debug flag ?
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::setupLocoSessions( uint16_t options, 
                                            LcsDccTrack *mainTrack,
                                            LcsDccTrack *progTrack ) {

    ENTER_FUNC( );

    if (( mainTrack == nullptr ) || ( progTrack == nullptr )) {

        return ( RET_STAT( ERR_SESSION_SETUP ));
    } 
        
    this -> options                 = options;
    this -> mainTrack               = mainTrack;
    this -> progTrack               = progTrack;
    this -> flags                   = SM_F_NIL;

    if ( options & SM_OPT_ENABLE_REFRESH ) {
             
        flags |= SM_F_ENABLE_REFRESH;
    }

    initCabMap( &cabMap );
    initActiveCabMap( &activeCabMap );

    return( RET_STAT( loadCabMap( )));
}

//----------------------------------------------------------------------------------------
// "formatCabMap" initializes the cab map data structure. It is called when the
// cab map is not valid or we have a new board. The data structure is build in 
// memory and then written in one swoop to the NVM.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::formatCabMap( ) {

    ENTER_FUNC( );

    initCabMap( &cabMap );

    uint8_t rStat = setItem( NIL_NODE_ID, 
                             CAB_MAP_ITEM_ID_START,
                             (uint16_t *) &cabMap, 
                             sizeof( LcsCabMap ) / sizeof( uint16_t ));

    return( RET_STAT( rStat ));    
}

//----------------------------------------------------------------------------------------
// "loadCabMap" retrieves the cab map from NVM. The cab map is a NVM data 
// structure, with an array of cab entries, unsorted. If the cabId is not in 
// this dictionary, it does not exist. We first read the header portion and if 
// valid, read the remaining cab array data. Otherwise, we format the cap map 
// and have a vanilla cabMap. In either case we have a valid cab map.
//
// Adding or removing cab entries is solely done on the NVM structure, which 
// is unsorted. After reading the NVM data, we sort the MEM structure.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::loadCabMap( ) {

    ENTER_FUNC( );

    initCabMap( &cabMap );

    if ( sLibDebugEnabled( )) {
            
            usbIoPrintf( CDC_USB_CONSOLE, "Read Cab map head ...\n" );
    }

    uint8_t rStat = LCS_OK;

    rStat = getItem( NIL_NODE_ID, 
                     CAB_MAP_ITEM_ID_START, 
                     (uint16_t *) &cabMap, 
                     CAB_DICT_HEADER_SIZE );
    if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

    if (( cabMap.magicWord != CAB_MAP_MAGIC_WORD ) ||
        ( cabMap.cabEntryItemSize != CAB_ENTRY_ITEM_SIZE ) ||
        ( cabMap.currentCabCount > MAX_CAB_DICT_SESSIONS )) {

        if ( sLibDebugEnabled( )) {
            
            usbIoPrintf( CDC_USB_CONSOLE, "Not valid, formatting ...\n" );
        }

        rStat = formatCabMap( );
        if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

        rStat = getItem( NIL_NODE_ID, 
                     CAB_MAP_ITEM_ID_START, 
                     (uint16_t *) &cabMap, 
                     CAB_DICT_HEADER_SIZE );
        if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));
    }

    if ( sLibDebugEnabled( )) {

        usbIoPrintf( CDC_USB_CONSOLE, "Load Cab map entries ...\n" );
    }

    rStat = getItem( NIL_NODE_ID, 
                     CAB_MAP_ITEM_ID_MAP_START, 
                     (uint16_t *) &cabMap.map, 
                     cabMap.currentCabCount * cabMap.cabEntryItemSize );
    if ( rStat != LCS_OK ) return ( RET_STAT( rStat ));

    if ( sLibDebugEnabled( )) {

        usbIoPrintf( CDC_USB_CONSOLE, "Sort Cab map ...\n" );
    }
        
    sortCabMap( cabMap.map, cabMap.currentCabCount );
    return( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// "updateCabMap" will copy the entire MEM cabMap to the NVM. We only write 
// up to the currently used entries. This method is typically used when we 
// added or removed entries from the cab dictionary.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::updateCabMap( ) {

    ENTER_FUNC( );

    uint8_t  rStat = LCS_OK;

    if ( cabMap.currentCabCount <= MAX_CAB_DICT_SESSIONS ) {
   
        rStat = setItem( NIL_NODE_ID, 
                         CAB_MAP_ITEM_ID_START, 
                         (uint16_t *) &cabMap, 
                         cabMap.currentCabCount * cabMap.cabEntryItemSize );
    }
    else rStat = ERR_CAB_MAP_INVALID_COUNT;

    return( RET_STAT( rStat ));
}

//----------------------------------------------------------------------------------------
// Getter methods for session related info. Straightforward.
//
//----------------------------------------------------------------------------------------
uint16_t LcsLocoSessions::getOptions( ) {

    return ( options );
}

uint16_t LcsLocoSessions::getFlags( ) {

    return ( flags );
}

uint16_t LcsLocoSessions::getCabCount( ) {

    return ( cabMap.currentCabCount );
}

uint16_t LcsLocoSessions::getMaxCabCount( ) {

    return ( cabMap.maxCabCount );
}

uint16_t LcsLocoSessions::getActiveCabCount( ) {

    return ( activeCabMap.activeCabCount );
}

//----------------------------------------------------------------------------------------
// "lookupCabEntry" looks for a cab entry in the sorted loco base and returns 
// a pointer to the entry if found, otherwise a nullptr.
//
//----------------------------------------------------------------------------------------
LcsCabEntry *LcsLocoSessions::lookupCabEntry( uint16_t cabId ) {

    uint16_t low  = 0;
    uint16_t high = cabMap.currentCabCount;

    while ( low < high ) {

        uint16_t mid = low + ( high - low ) / 2;

        if      ( cabMap.map[ mid ].cabId < cabId ) low  = mid + 1;
        else if ( cabMap.map[ mid ].cabId > cabId ) high = mid;
        else return( &cabMap.map[ mid ] );
    }

    return( nullptr );
}

//----------------------------------------------------------------------------------------
// "addCabEntry" is used to add a cab configuration to the cabMap. We first 
// check whether the cab entry already exists. If so, we just update the entry.
// Otherwise, we check for room in the cabMap and add the entry. 
//
// Note that we only update our memory structure for the cabMap. When we want
// this permanently saved, we need to update the NVM image. Splitting the task
// allows us however to do many updates to the memory image and then flush it
// after all changes are done.
//
// ??? validate the entry first ?
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::addCabEntry( LcsCabEntry *entry ) {

    LcsCabEntry *cPtr = lookupCabEntry( entry -> cabId );
    if ( cPtr != nullptr ) {
        
        *cPtr = *entry;
        return( LCS_OK );
    }

    if ( cabMap.currentCabCount >= MAX_CAB_DICT_SESSIONS ) {
        
        return ( ERR_CAB_MAP_FULL );
    }

    cabMap.map[ cabMap.currentCabCount ] = *entry;
    cabMap.currentCabCount ++;

    sortCabMap( cabMap.map, cabMap.currentCabCount );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "removeCabEntry" removes an entry from the cabMap and perhaps also from the
// active cab map. We first get the index in the cab map array. If the entry
// is active, we need to remove it also from the active cab map. cabMap is 
// sorted by cabId, therefore all entries following the removed entry are 
// shifted down by one position. 
//
// As a last step we just rebuild the active cab map. Since entries in the 
// canMap shifted around, it is the safest approach to just rebuild. Removing
// is a task most likely done during configuration, so performance of removal
// is traded for safety.
// 
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::removeCabEntry( uint16_t cabId ) {

    LcsCabEntry *cPtr = lookupCabEntry( cabId );
    if ( cPtr == nullptr ) return( LCS_OK );

    uint16_t cabMapIndex = cPtr - cabMap.map;

    if ( cabMapIndex < cabMap.currentCabCount - 1 ) {

        memmove( &cabMap.map[ cabMapIndex ],
                 &cabMap.map[ cabMapIndex + 1 ],
                 ( cabMap.currentCabCount - cabMapIndex - 1 ) * 
                                                    sizeof( LcsCabEntry ));
    }

    cabMap.currentCabCount --;

    rebuildActiveCabMap( );
    return( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "addActiveCab" adds a cab entry index to the active cab list. The index is
// only added if it is not already in the list.
//
// ??? what do we do when the table is full ?
//----------------------------------------------------------------------------------------
void LcsLocoSessions::addActiveCab( uint16_t cabMapIndex ) {

    if ( cabMapIndex >= MAX_CAB_DICT_SESSIONS ) return;

    for ( uint16_t i = 0; i < activeCabMap.activeCabCount; i++ ) {

        if ( activeCabMap.map[ i ] == cabMapIndex ) return;
    }

    if ( activeCabMap.activeCabCount < MAX_CAB_ACTIVE_SESSIONS ) {

        activeCabMap.map[ activeCabMap.activeCabCount ] = cabMapIndex;
        activeCabMap.activeCabCount ++;
    }
}

//----------------------------------------------------------------------------------------
// "removeActiveCab" removes a cab entry index from the active cab list. Order
// of the active list is not significant, so the last entry is moved into the 
// position of the removed entry, and the total count decremented.
//
//----------------------------------------------------------------------------------------
void LcsLocoSessions::removeActiveCab( uint16_t cabMapIndex ) {

    if ( cabMapIndex >= MAX_CAB_DICT_SESSIONS ) return;
    if ( activeCabMap.activeCabCount == 0 ) return;

    for ( uint16_t i = 0; i < activeCabMap.activeCabCount; i++ ) {

        if ( activeCabMap.map[ i ] == cabMapIndex ) {

            if ( i != activeCabMap.activeCabCount - 1 ) {

                activeCabMap.map[ i ] = 
                        activeCabMap.map[ activeCabMap.activeCabCount ];
            }

             activeCabMap.activeCabCount --;

            return;
        }
    }
}

//----------------------------------------------------------------------------------------
// "rebuildActiveCabMap" is used to scan the cab map and add all active cab 
// entries are. It is typically used when we removed an entry from the cab map 
// and entry indices shifted. This is a linear search and may take a while.
// However, it is only done when deleting entries, which we do not do during
// operations.
//
//----------------------------------------------------------------------------------------
void LcsLocoSessions::rebuildActiveCabMap( ) {

    initActiveCabMap( &activeCabMap );

    for ( uint16_t i = 0; i < cabMap.currentCabCount; i ++ ) {

        if ( cabMap.map[ i ].flags & CMAP_F_ACTIVE ) addActiveCab( i );
    }
}

//----------------------------------------------------------------------------------------
// "activateCabEntry" activates the cab entry. It is used by all routines that 
// need to manipulate a cab and does all the active list housekeeping. The entry
// must already exists in the loco dictionary. If it is found, it is added to 
// the active loco list if it isn't already active. Returns a pointer to the 
// entry, or nullptr if the cab does not exist.
//
//----------------------------------------------------------------------------------------
LcsCabEntry *LcsLocoSessions::activateCabEntry( uint16_t cabId ) {

    LcsCabEntry *cPtr = lookupCabEntry( cabId );
    if ( cPtr == nullptr ) return( nullptr );

    if ( ! ( cPtr -> flags & CMAP_F_ACTIVE )) { 
        
        addActiveCab( cPtr - cabMap.map );
        cPtr -> flags |= CMAP_F_ACTIVE;
    }

    return( cPtr );
}

//----------------------------------------------------------------------------------------
// "deactivateCabEntry" deactivates a cab entry and removes it from the active
// loco list. The entry remains of course in the dictionary base.
//
//----------------------------------------------------------------------------------------
void LcsLocoSessions::deactivateCabEntry( uint16_t cabId ) {

    LcsCabEntry *cPtr = lookupCabEntry( cabId );
    if ( cPtr == nullptr ) return;

    if ( cPtr -> flags & CMAP_F_ACTIVE ) { 
        
        removeActiveCab( cPtr - cabMap.map );
        cPtr -> flags &= ~ CMAP_F_ACTIVE;
    }
}

//----------------------------------------------------------------------------------------
// "emergencyStopAll" is called when one of the clients issued an emergency stop 
// all request. There is a DCC broadcast packet that causes all decoders to stop
// the locos. In addition, the base station is expected to discontinue sending
// non-zero speed packets until the situation is cleared. The standard does not 
// really say what exactly to do. In our base station, we will first issue the 
// ESTOP DCC broadcast packet and then set the speed value in each session to 
// one, which is the value for emergency stop. All else is unchanged.
//
// ??? still not clear what to really send...
//----------------------------------------------------------------------------------------
void LcsLocoSessions::emergencyStopAll( ) {

    mainTrack -> loadPacket( eStopDccPacketData, 2, 4 );

    for ( int i = 0; i < activeCabMap.activeCabCount; i++ ) {

        LcsCabEntry *cPtr = &cabMap.map[ activeCabMap.map[ i ] ];
        cPtr -> speedInfo  = 1;
    }
}

//----------------------------------------------------------------------------------------
// "refreshActiveCabs" manages the cab refresh task. We need to send to an
// active cab the DCC throttle commands to refresh the decoder settings. Since
// we have also other things to do in a base station, we will only refresh one
// cab at a time and go round robin though the active cab list. The refresh 
// interval is set by the base station. 
//
//----------------------------------------------------------------------------------------
void LcsLocoSessions::refreshActiveCabs( ) {

    if ( activeCabMap.activeCabCount == 0 ) return;

    uint16_t cabMapIndex = activeCabMap.map[ activeCabMap.refreshIndex ];
    if ( cabMapIndex == -1 ) return;

    LcsCabEntry *cab        = &cabMap.map[ cabMapIndex ];
    uint32_t    currentTs   = getMillis( );

    if (( currentTs -  activeCabMap.tsMap[ cabMapIndex ] <= 
                                        DCC_SESSION_TIMEOUT_MILLIS )) { 

        setThrottle( cab, 
                 (uint8_t) cab -> speedInfo & 0x7F, 
                 (uint8_t) (( cab -> speedInfo & 0x80 ) ? 1 : 0 ));

        activeCabMap.tsMap[ cabMapIndex ] = currentTs;
    } 
    else {
     
        setThrottle( cab, 1, 1 );
        removeActiveCab( cabMapIndex );
    }
    
    activeCabMap.refreshIndex ++;
    if ( activeCabMap.refreshIndex >= activeCabMap.activeCabCount ) {
        
        activeCabMap.refreshIndex = 0;
    }
}

//----------------------------------------------------------------------------------------
// "setThrottle" is perhaps the most used function. After all, we want to run 
// engines on the track. This signature will just locate the session map entry 
// and then invoke the internal signature with accepts a pointer to the entry.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::setThrottle( uint16_t cabId, 
                                      uint8_t  speed, 
                                      uint8_t  direction ) {

    LcsCabEntry *ptr = activateCabEntry( cabId );                                    
    if ( ptr == nullptr ) return( ERR_INVALID_CAB_ID );

    return ( setThrottle( ptr, speed, direction ));
}

//----------------------------------------------------------------------------------------
// "setThrottle" will send a DCC packet with speed and direction for a loco. If
// the combined speed and function refresh option is enabled, the DCC command 
// will specify speed, direction and functions to refresh in one packet.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::setThrottle( LcsCabEntry *cPtr, 
                                      uint8_t speed, 
                                      uint8_t direction ) {

    uint8_t pBuf[ MAX_DCC_PACKET_SIZE ];
    uint8_t pLen = 0;

    cPtr -> speedInfo  = (( speed & 0x7F ) | (( direction ) ? 0x80 : 0 ));
   
    if ( cPtr -> cabId > 127 ) pBuf[pLen++] = highByte( cPtr -> cabId ) | 0xC0;
    pBuf[pLen++] = lowByte( cPtr -> cabId );

    pBuf[pLen++] = (( cPtr -> flags & CMAP_F_COMBINED_REFRESH ) ? 0x3c : 0x3F );
    pBuf[pLen++] = (uint8_t) cPtr -> speedInfo & 0xFF;

    if ( cPtr -> flags & CMAP_F_COMBINED_REFRESH ) {

        pBuf[pLen++]  = ((( cPtr -> d.funcMap[0] & 0x10 ) >> 4 ) |
                        (( cPtr -> d.funcMap[0] & 0x0F ) << 1 ) |
                        (( cPtr -> d.funcMap[1] & 0x07 ) << 5 ));

        pBuf[pLen++]  = ((( cPtr -> d.funcMap[1] & 0x0F ) >> 3 ) |
                        (( cPtr -> d.funcMap[2] & 0x0F ) << 1 ) |
                        (( cPtr -> d.funcMap[3] & 0x07 ) << 5 ));

        pBuf[pLen++]  = ((( cPtr -> d.funcMap[3] & 0xf80 ) >> 3 ) |
                        (( cPtr -> d.funcMap[4] & 0x07 ) << 5 ));

        pBuf[pLen++]  = (( cPtr -> d.funcMap[4] & 0xf80 ) >> 3 );
    }

    mainTrack -> loadPacket( pBuf, pLen );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "setDccFunctionBit" controls the functions in a decoder. The DCC function 
// flags F0 .. F68 are stored in ten groups. The routines first updates the 
// function bit in the cab entry data structure, so we can keep track of the 
// values. This is important as the DCC commands send out entire groups only. 
// The actual work is then done by the "setDccFunctionGroup" method.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::setDccFunctionBit( uint16_t cabId, 
                                            uint8_t fNum, 
                                            uint8_t val ) {

    if ( ! validFunctionId( fNum )) return ( ERR_INVALID_FUNC_ID );
    
    LcsCabEntry *ptr = activateCabEntry( cabId );                                    
    if ( ptr == nullptr ) return( ERR_INVALID_CAB_ID );

    setDccFuncBit( ptr -> d.funcMap, fNum, val );

    uint8_t fGroup = dccFunctionBitToGroup( fNum );

    return ( setDccFunctionGroup( ptr, fGroup, ptr -> d.funcMap[ fGroup - 1 ] ));
}

//----------------------------------------------------------------------------------------
// "setDccFunctionGroup" sets an entire group of function flags. This signature 
// will first find the session entry, do the argument checks and the invoke the 
// internal signature.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::setDccFunctionGroup( uint16_t cabId, 
                                              uint8_t fGroup, 
                                              uint8_t dccByte ) {

    LcsCabEntry *ptr = activateCabEntry( cabId );                                    
    if ( ptr == nullptr ) return( ERR_INVALID_CAB_ID );

    return ( setDccFunctionGroup( ptr, fGroup, dccByte ));
}

//----------------------------------------------------------------------------------------
// "setDccFunctionGroup" sets an entire group of function flags.The DCC 
// function flags F0 .. F68 are stored in ten groups.
//
//      Group 1:  F0, F4, F3, F2, F1      DCC Command Format: 100DDDDD
//      Group 2:  F8, F7, F6, F5          DCC Command Format: 1011DDDD
//      Group 3:  F12, F11, F10, F9       DCC Command Format: 1010DDDD
//      Group 4:  F20 .. F13              DCC Command Format: 0xDE DDDDDDDD
//      Group 5:  F28 .. F21              DCC Command Format: 0xDF DDDDDDDD
//      Group 6:  F36 .. F29              DCC Command Format: 0xD8 DDDDDDDD
//      Group 7:  F44 .. F37              DCC Command Format: 0xD9 DDDDDDDD
//      Group 8:  F52 .. F45              DCC Command Format: 0xDA DDDDDDDD
//      Group 9:  F60 .. F53              DCC Command Format: 0xDB DDDDDDDD
//      Group 10: F68 .. F61              DCC Command Format: 0xDC DDDDDDDD
//
// The routines updates the entire function group byte in the loco session
// entry, so we can keep track of the values. The function command is repeated 
// 4 times to the track.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::setDccFunctionGroup( LcsCabEntry *cPtr, 
                                              uint8_t fGroup, 
                                              uint8_t dccByte ) {

    if ( ! validFunctionGroupId( fGroup )) return ( ERR_INVALID_FGROUP_ID );
    setDccFuncGroupByte( cPtr -> d.funcMap, fGroup, dccByte );

    uint8_t pBuf[ MAX_DCC_PACKET_SIZE];
    uint8_t pLen = 0;

    if ( cPtr -> cabId > 127 ) pBuf[pLen++] = highByte( cPtr -> cabId ) | 0xC0;
    pBuf[pLen++] = lowByte( cPtr -> cabId );

    switch ( fGroup - 1 ) {

        case 0: pBuf[pLen++] = ( cPtr -> d.funcMap[ 0 ] & 0x1F ) | 0x80; break;
        case 1: pBuf[pLen++] = ( cPtr -> d.funcMap[ 1 ] & 0x0F ) | 0xB0; break;
        case 2: pBuf[pLen++] = ( cPtr -> d.funcMap[ 2 ] & 0x0F ) | 0xA0; break;

        case 3: pBuf[pLen++] = 0xDE; pBuf[pLen++] = cPtr -> d.funcMap[ 3 ]; break;
        case 4: pBuf[pLen++] = 0xDF; pBuf[pLen++] = cPtr -> d.funcMap[ 4 ]; break;
        case 5: pBuf[pLen++] = 0xD8; pBuf[pLen++] = cPtr -> d.funcMap[ 5 ]; break;
        case 6: pBuf[pLen++] = 0xD9; pBuf[pLen++] = cPtr -> d.funcMap[ 6 ]; break;
        case 7: pBuf[pLen++] = 0xDA; pBuf[pLen++] = cPtr -> d.funcMap[ 7 ]; break;
        case 8: pBuf[pLen++] = 0xDB; pBuf[pLen++] = cPtr -> d.funcMap[ 8 ]; break;
        case 9: pBuf[pLen++] = 0xDC; pBuf[pLen++] = cPtr -> d.funcMap[ 9 ]; break;
    }

    mainTrack -> loadPacket( pBuf, pLen, 4 );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "writeCVMain" writes a CV value to the decoder on the main track. CV numbers 
// range from 1 to 1024, but are encoded from 0 to 1023. The DCC standard 
// defines various modes for retrieving CV values. This function implements CV
// write mode mode 0 and 1, by calling the respective method. The other modes 
// are not supported. For bit mode access, the bit position and bit value are 
// encoded in the "val" parameter with bit 3 containing the data and bit 0 ..2
// the bit offset.
//
//    0 Direct Byte
//    1 Direct Bit
//    2 Page Mode
//    3 Register Mode
//    4 Address Only Mode
//
// Note on the MAIN track, there is no way for the decoder to answer via a raise 
// in power consumption. The command shown here is just sent. If however RailCom 
// is available, the decoder can answer with the CV value in a following cutout. 
// This is currently not implemented.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::writeCVMain( uint16_t cabId, 
                                      uint16_t cvId, 
                                      uint8_t mode, 
                                      uint8_t val ) {

    if ( mode == 0 ) {

        return ( writeCVByteMain( cabId, cvId, val ));
    }
    else if ( mode == 1 ) {

        return ( writeCVBitMain( cabId, 
                                 cvId, 
                                 ( val & 0x07 ), 
                                 (( val & 0x08 ) >> 3 )));
    }
    else  return ( ERR_INVALID_CV_MODE );
}

//----------------------------------------------------------------------------------------
// "writeCVByteMain" writes a byte to the CV while the loco is on the main 
// track. The CV numbers range from 1 to 1024, but are encoded from 0 to 1023. 
// This function implements CV write mode mode 0, which is write a byte at a 
// time. There is no way to validate our operation, only writes are possible. 
// The packet is sent four times.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::writeCVByteMain( uint16_t cabId, 
                                          uint16_t cvId, 
                                          uint8_t val ) {

    uint8_t   pBuf[ MAX_DCC_PACKET_SIZE ];
    uint8_t   pLen = 0;

    LcsCabEntry *cPtr = activateCabEntry( cabId );                                    
    if ( cPtr == nullptr ) return( ERR_INVALID_CAB_ID );

    if ( ! validCvId( cvId )) return ( ERR_INVALID_CV_ID );
    cvId--;

    if ( cPtr -> cabId > 127 ) pBuf[pLen++] = highByte( cPtr -> cabId ) | 0xC0;
    pBuf[pLen++] = lowByte( cPtr -> cabId );
    pBuf[pLen++] = 0xEC + ( highByte( cvId ) & 0x03 );
    pBuf[pLen++] = lowByte( cvId );
    pBuf[pLen++] = val;

    mainTrack -> loadPacket( pBuf, pLen, 4 );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "writeCVBitMain" writes a bit to the CV while the loco is on the main track. 
// The CV numbers range from 1 to 1024, but are encoded from 0 to 1023. This 
// function implements CV write mode mode 1, which is write a bit at a time. On 
// input the "val" parameter encodes the bit position in bits 0 - 2 and the bit 
// value in bit 3. There is no way to validate our operation, only CV writes are 
// possible. The packet is sent four times.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::writeCVBitMain( uint16_t cabId, 
                                         uint16_t cvId, 
                                         uint8_t bitPos, 
                                         uint8_t val ) {

    LcsCabEntry *cPtr = activateCabEntry( cabId );                                    
    if ( cPtr == nullptr ) return( ERR_INVALID_CAB_ID );

    if ( ! validCvId( cvId )) return ( ERR_INVALID_CV_ID );
    cvId--;

    uint8_t pBuf[ MAX_DCC_PACKET_SIZE ];
    uint8_t pLen = 0;

    if ( cPtr -> cabId > 127 ) pBuf[pLen++] = highByte( cPtr -> cabId ) | 0xC0;
    pBuf[pLen++] = lowByte( cPtr -> cabId );
    pBuf[pLen++] = 0xE8 + (highByte( cvId ) & 0x03 );
    pBuf[pLen++] = lowByte( cvId );
    pBuf[pLen++] = 0xF0 + (( val % 2 ) << 3 ) + ( bitPos % 8 );

    mainTrack -> loadPacket( pBuf, pLen, 4 );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "readCV" retrieves a CV value from the decoder in service mode. CV numbers 
// range from 1 to 1024, but are encoded from 0 to 1023. This command is only 
// available in service mode, i.e. on a programming track. The DCC standard 
// defines various modes for retrieving CV values. We only support mode 0 and 1.
// The other modes are not supported. For bit mode access, the bit position and 
// bit value are encoded in the "val" parameter with bit 3 containing the data 
// and bit 0 ..2 the bit offset.
//
//    0 - Direct Byte
//    1 - Direct Bit
//    2 - Page Mode
//    3 - Register Mode
//    4 - Address Only Mode
//
// This function implements the CV read mode 0 and 1, which is reading a byte or 
// a bit at a time by calling the respective method.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::readCV( uint16_t cvId, uint8_t mode, uint8_t *val ) {

    if        ( mode == 0 )  return ( readCVByte( cvId, val ));
    else if   ( mode == 1 )  return ( readCVBit( cvId, *val % 8, val ));
    else                     return ( ERR_INVALID_CV_MODE );
}

//----------------------------------------------------------------------------------------
// "readCVByte" will retrieve a complete byte from the decoder. CV numbers 
// range from 1 to 1024, but are encoded from 0 to 1023. This command is only 
// available in service mode, i.e. on a programming track. Reading a CV value 
// where the decoder can only respond with a "yes" or "no" is a tedious matter. 
// We are actually reading the CV value bit by bit and then ask if the assembled
// byte read is the one just read. The general packet sequence is a according 
// to DCC standard standard 3 or more RESET packets, 5 or more identical READ
// packets and then RESET packages until acknowledge or timeout. The RESET 
// packet preamble and postamble series are sent during the decoder ack setup 
// and detect call to the DCC track object. During the preamble we figure out 
// the base current consumption of the decoder, during the postamble packets 
// we measure to get the decoder acknowledge, which is a short raise in power
// consumption to indicate an ACK.
//
//
// ??? This command may take a long time, a lot of packets are sent. While this 
// not an issue with the signal generation, which is done via interrupt handlers,
// it may be an issue with any other work of the base station. This code needs 
// to be redesigned to use a kind of state machine that sends a packet at a time
// so other work can interleave.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::readCVByte( uint16_t cvId, uint8_t *val ) {

    if ( progTrack -> getTrackMode( ) != DT_M_ACC_DETECT ) {
        
        return ( ERR_NO_SVC_MODE );
    }

    if ( ! validCvId( cvId )) return ( ERR_INVALID_CV_ID );
    cvId--;

    uint8_t   pBuf[ MAX_DCC_PACKET_SIZE ];
    uint8_t   bValue  = 0;
    uint16_t  base    = progTrack -> decoderAckBaseline( 5 );

    pBuf[0]  = 0x78 + ( highByte( cvId ) & 0x03 );
    pBuf[1]  = lowByte( cvId );

    for ( int i = 0; i < 8; i++ ) {

        pBuf[2] = 0xE8 + i;
        progTrack -> loadPacket( pBuf, 3, 5 );
        bitWrite( &bValue, i, progTrack -> decoderAckDetect( base, 9 ));
    }

    *val    = bValue;
    pBuf[0] = 0x74 + ( highByte( cvId ) & 0x03 );
    pBuf[1] = lowByte( cvId );
    pBuf[2] = bValue;
    progTrack -> loadPacket( pBuf, 3, 5 );

    return (( progTrack -> decoderAckDetect( base, 9 )) ? 
            LCS_OK : (LcsErrorCodes) ERR_CV_OP_FAILED );
}

//----------------------------------------------------------------------------------------
// "readCVBit" will retrieve one bit from a CV variable from the decoder. CV 
// numbers range from 1 to 1024, but are encoded from 0 to 1023. This command 
// is only available in service mode, i.e. on a programming track. The "val" 
// parameter encodes the bit position in bits 0 - 2. We are reading the CV 
// value bit and then ask if the bit read is the one just read. We first try 
// to validate a "zero" bit. If that succeeds, fine. Otherwise we try to 
// validate a "one" bit. If that succeeds, fine. Otherwise we have a CV read
// error. 
//
// The general packet sequence is according to DCC standard 3 or more RESET 
// packets, 5 or more identical READ packets and then RESET packages until 
// acknowledge or timeout. The RESET packet preamble and postamble are sent 
// during the decoder ack setup and detect call to the DCC track object. During 
// the preamble we figure out the base current consumption of the decoder, 
// during the postamble we measure to get the decoder acknowledge, which is a 
// short raise in power consumption to indicate an ACK.
//
// ??? This command may take a long time, a lot of packets are sent. While this 
// not an issue with the signal generation, which is done via interrupt handlers,
// it may be an issue with any other work of the base station. This code needs 
// to be redesigned to use a kind of state machine that sends a packet at a 
// time so other work can interleave.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::readCVBit( uint16_t cvId, 
                                    uint8_t bitPos, 
                                    uint8_t *val ) {

    if ( progTrack -> getTrackMode( ) != DT_M_ACC_DETECT ) {
        
        return ( ERR_NO_SVC_MODE );
    }

    if ( ! validCvId( cvId )) return ( ERR_INVALID_CV_ID );
    cvId--;

    uint8_t  pBuf[ MAX_DCC_PACKET_SIZE ];
    int   base = progTrack -> decoderAckBaseline( 5 );

    pBuf[0]  = 0x78 + ( highByte( cvId ) & 0x03 );
    pBuf[1]  = lowByte( cvId );
    pBuf[2]  = 0xE8 + ( bitPos % 8 );

    progTrack -> loadPacket( pBuf, 3, 5 );

    if ( ! ( progTrack -> decoderAckDetect( base, 9 ))) {
        
        pBuf[2] = 0xE8 + 8 + ( bitPos % 8 );
        progTrack -> loadPacket( pBuf, 3, 5 );
    
        if ( progTrack -> decoderAckDetect( base, 9 )) {

            *val = 1;
            return ( LCS_OK );
        }
        else return ( ERR_CV_OP_FAILED );
    }
    else return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "writeCV" writes a CV value to the decoder. CV numbers range from 1 to 1024,
// but are encoded from 0 to 1023. This command is only available in service 
// mode, i.e. on a programming track. The DCC standard defines various modes 
// for accessing CV values. For bit mode access, the bit position and bit value
// are encoded in the "val" parameter with bit 3 containing the data and bit 
// 0 .. 2 the bit offset.
//
//    0 Direct Byte
//    1 Direct Bit
//    2 Page Mode
//    3 Register Mode
//    4 Address Only Mode
//
// This function implements the CV write mode 0 and 1, which is writing a byte
// or a bit at a time by calling the respective method.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::writeCV( uint16_t cvId, uint8_t mode, uint8_t val ) {

    if ( mode == 0 ) 
        return ( writeCVByte( cvId, val ));
    else if ( mode == 1 )  
        return ( writeCVBit( cvId, ( val & 0x07 ), (( val & 0x08 ) >> 3 )));
    else                     
        return ( ERR_INVALID_CV_MODE );
}

//----------------------------------------------------------------------------------------
// "writeCVByte" puts a data byte into the CV on the decoder. This function is 
// only available in service mode. The CV numbers range from 1 to 1024, but are
// encoded from 0 to 1023. The data byte written will also be verified. The 
// packet sequence follows the DCC standard. We will send the CV byte write 
// packet four times, send out several RESET packets and the send the verify 
// packets to get the acknowledge from the decoder that the operation was 
// successful.
//
// ??? This command may take a long time, a lot of packets are sent. While this 
// not an issue with the signal generation, which is done via interrupt handlers,
// it may be an issue with any other work of the base station. If so, this code 
// needs to be redesigned to use a kind of state machine that sends a packet at
// a time so other work can interleave.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::writeCVByte( uint16_t cvId, uint8_t val ) {

    if ( progTrack -> getTrackMode( ) != DT_M_ACC_DETECT ) {
        
        return ( ERR_NO_SVC_MODE );
    }

    if ( ! validCvId( cvId )) return ( ERR_INVALID_CV_ID );
    cvId--;

    uint8_t pBuf[ MAX_DCC_PACKET_SIZE ];
    int     base = progTrack -> decoderAckBaseline( 5 );

    pBuf[0] = 0x7C + ( highByte( cvId ) & 0x03 );
    pBuf[1] = lowByte( cvId );
    pBuf[2] = val;

    progTrack -> loadPacket( pBuf, 3, 4 );
    progTrack -> loadPacket( resetDccPacketData, 2, 11 );

    pBuf[0] = 0x74 + ( highByte( cvId ) & 0x03 );
    progTrack -> loadPacket( pBuf, 3, 5 );

    return (( progTrack -> decoderAckDetect( base, 9 )) ?
                         LCS_OK : (LcsErrorCodes) ERR_CV_OP_FAILED );
}

//----------------------------------------------------------------------------------------
// "writeCVBit" puts a data bit into the CV on the decoder. This function is 
// only available in session mode. The CV numbers range from 1 to 1024, but are
// encoded from 0 to 1023. For the bit mode,  the "val" parameter encodes the
// bit position in bits 0 - 2 and the bit value in bit 3. The packet sequence 
// follows the DCC standard, similar to the byte write operation.
//
// ??? This command may take a long time, a lot of packets are sent. While this 
// not an issue with the signal generation, which is done via interrupt handlers,
// it may be an issue with any other work of the base station. If so, this code 
// needs to be redesigned to use a kind of state machine that sends a packet at
// a time so other work can interleave.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::writeCVBit( uint16_t cvId, 
                                     uint8_t bitPos, 
                                     uint8_t val ) {

    if ( progTrack -> getTrackMode( ) != DT_M_ACC_DETECT ) {
        
        return ( ERR_NO_SVC_MODE );
    }

    if ( ! validCvId( cvId )) return ( ERR_INVALID_CV_ID );
    cvId--;

    uint8_t pBuf[ MAX_DCC_PACKET_SIZE ];
    int     base = progTrack -> decoderAckBaseline( 5 );

    pBuf[0] = 0x78 + ( highByte( cvId ) & 0x03 );
    pBuf[1] = lowByte( cvId );
    pBuf[2] = 0xF0 + (( val % 2 ) * 8 ) + ( bitPos % 8 );

    progTrack -> loadPacket( pBuf, 3, 4 );
    progTrack -> loadPacket( resetDccPacketData, 2, 11 );

    bitWrite( &pBuf[2], 4, false );
    progTrack -> loadPacket( pBuf, 3, 5 );

    return (( progTrack -> decoderAckDetect( base, 9 )) ? 
                        LCS_OK : (LcsErrorCodes) ERR_CV_OP_FAILED );
}

//----------------------------------------------------------------------------------------
// The verify family of functions is used to verify that a CV value has been 
// written correctly. They are convenience functions. We just read the CV
// value and compare it.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::verifyCV( uint16_t cvId, uint8_t mode, uint8_t val ) {

    if ( mode == 0 ) {

        return ( verifyCVByte( cvId, val ));
    }
    else if ( mode == 1 ) {

        return ( verifyCVBit( cvId, ( val & 0x07 ), (( val & 0x08 ) >> 3 )));
    }
    else return ( ERR_INVALID_CV_MODE );
}

//----------------------------------------------------------------------------------------
// "verifyCVByte" reads the CV byte and compares it to the test value.
// 
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::verifyCVByte( uint16_t cvId, uint8_t val ) {

    uint8_t rStat   = LCS_OK;
    uint8_t cvVal   = 0;

    rStat = readCVByte( cvId, &cvVal );
    if ( rStat != LCS_OK ) return( rStat );

    return(( cvVal != val ) ? ERR_CV_VAL_VERIFY_MATCH : LCS_OK );
}

//----------------------------------------------------------------------------------------
// "verifyCVBit" reads the CV bit at bitPos and compares it to the test value.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::verifyCVBit( uint16_t cvId, 
                                      uint8_t  bitPos, 
                                      uint8_t  val ) {

    uint8_t rStat   = LCS_OK;
    uint8_t cvVal   = 0;         
    
    rStat = readCVBit( cvId, bitPos, &cvVal );
    if ( rStat != LCS_OK ) return( rStat );

   return(( cvVal != val ) ? ERR_CV_VAL_VERIFY_MATCH : LCS_OK );
}

//----------------------------------------------------------------------------------------
// "writeDccPacketMain" just emits a raw DCC packet for the MAIN track.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::writeDccPacketMain( uint8_t *packet, uint8_t len ) {

    mainTrack -> loadPacket( packet, len );
    return( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "writeDccPacketProg" just emits a raw DCC packet for the PROGRAMMING track.
//
//----------------------------------------------------------------------------------------
uint8_t LcsLocoSessions::writeDccPacketProg( uint8_t *packet, uint8_t len ) {

    progTrack -> loadPacket( packet, len );
    return( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "printCabEntry" lists a cab entry. We have two formats. The summary format
// lists the cabId, speed info and flags. The detailed format adds the 
// remainder of the cab entry data.
//
//----------------------------------------------------------------------------------------
void  LcsLocoSessions::printCabMapEntry( LcsCabEntry *cPtr, bool detail ) {

    if ( cPtr == nullptr ) return;

    usbIoPrintf( CDC_USB_CONSOLE, "Entry: %d, ", cPtr - cabMap.map );
    usbIoPrintf( CDC_USB_CONSOLE, "CabId: %d, ", cPtr -> cabId );
    usbIoPrintf( CDC_USB_CONSOLE, "Speed: %d, ", cPtr -> speedInfo & 0x7f );

    usbIoPrintf( CDC_USB_CONSOLE, "%s, ", 
                 (( cPtr -> speedInfo & 0x80 ) ? "Rev" : "Fwd" ));

    usbIoPrintf( CDC_USB_CONSOLE, " Flags: 0x%4x", ( cPtr -> flags ));

    usbIoPrintf( CDC_USB_CONSOLE, " %16s", cPtr -> locoName );
    usbIoPrintf( CDC_USB_CONSOLE , ":%16s", cPtr -> trainName );
    usbIoPrintf( CDC_USB_CONSOLE, "\n" );

    if ( detail ) {

        usbIoPrintf( CDC_USB_CONSOLE, "   SpeedMap: " );
        for ( int i = 0; i < 6; i++ ) { 
            
            usbIoPrintf( CDC_USB_CONSOLE, "%d ", cPtr -> speedMap[ i ]);
        }

        usbIoPrintf( CDC_USB_CONSOLE, "\n" );

        // ??? how do we do dcc and analog ?

        usbIoPrintf( CDC_USB_CONSOLE, "   FuncMap: " );
        for ( int i = 0; i < 10; i++ ) {
            
            usbIoPrintf( CDC_USB_CONSOLE, "0x%x ", cPtr -> d.funcMap[ i ]);
        }

        usbIoPrintf( CDC_USB_CONSOLE, "\n" );
    }
}

//----------------------------------------------------------------------------------------
// Print out the cab map.
//
//----------------------------------------------------------------------------------------
void LcsLocoSessions::printCabMap( ) {

    usbIoPrintf( CDC_USB_CONSOLE, "CabMap\n\n" );

    usbIoPrintf( CDC_USB_CONSOLE, "Magic Word: 0x%x\n", cabMap.magicWord );
    usbIoPrintf( CDC_USB_CONSOLE, "Flags: 0x%x\n", cabMap.flags );
    usbIoPrintf( CDC_USB_CONSOLE, "Max Cab Count: %d\n", cabMap.maxCabCount );  

    usbIoPrintf( CDC_USB_CONSOLE, "Current Cab Count: %d\n", 
                                  cabMap.currentCabCount );

    usbIoPrintf( CDC_USB_CONSOLE, "Cab Entry Item Size: %d\n", 
                                  cabMap.cabEntryItemSize );
                                  
    usbIoPrintf( CDC_USB_CONSOLE, "\n\n" );

    for ( int i = 0; i < cabMap.currentCabCount; i++ ) {

        printCabMapEntry( &cabMap.map[ i ] );
    }
}

//----------------------------------------------------------------------------------------
// Print out the active cab map.
// 
//----------------------------------------------------------------------------------------
void  LcsLocoSessions::printActiveCabMap( ) {

    usbIoPrintf( CDC_USB_CONSOLE, "Active Cab Map \n\n" );

    usbIoPrintf( CDC_USB_CONSOLE, "Cab count: %d\n", 
                 activeCabMap.activeCabCount );

    for ( int i = 0; i < activeCabMap.activeCabCount; i++ ) {

        uint16_t locoIndex = activeCabMap.map[ i ];
        LcsCabEntry *cPtr = &cabMap.map[ locoIndex ];

        printCabMapEntry( cPtr );
    }
}
