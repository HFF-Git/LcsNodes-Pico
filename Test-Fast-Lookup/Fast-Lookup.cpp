#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "LcsCdcLib.h" // ??? not found at link time ...

//----------------------------------------------------------------------------------------
//
// ??? goes into LcsRtLib.
//----------------------------------------------------------------------------------------
const uint16_t NIL_CAB_ID = 0;

//----------------------------------------------------------------------------------------
// The cab table entry. Each active locomotive is represented by a cab entry, 
// which contains the cabId and the last seen timestamp. The cabId is a 16 bit 
// value that uniquely identifies the locomotive, and the last seen timestamp is
// used to determine when a locomotive has been inactive for too long and should 
//be removed from the table.
//
//----------------------------------------------------------------------------------------
struct CabTableEntry {

    uint16_t cabId;
    uint32_t lastSeenTs;
};

//----------------------------------------------------------------------------------------
// The Cab table is a simple hash table with open addressing and linear probing. 
// It is used to allow quick lookup of cab entries by cabId, which is a 16 bit 
// value. The hash table size is 512 entries, which allows for a good load factor
// and fast lookups. The cab entries are stored in a separate array, which allows
// for up to 256 active locomotives. The has table should by a power of two in 
// size to allow for efficient hashing and probing.
//
//----------------------------------------------------------------------------------------
struct CabTableIndexEntry {

    uint16_t cabId;
    uint16_t index;
};

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
namespace {

const int           MAX_CAB_ENTRIES = 256;
const int           MAX_CAB_HASH_TAB_ENTRIES = 512;   // must be power of two

uint16_t            cabTableHwm;
CabTableEntry       locoTable [ MAX_CAB_ENTRIES ];
CabTableIndexEntry  locoIndexTable [ MAX_CAB_HASH_TAB_ENTRIES ];


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
// Insert into hash table. The routine will insert the cabId and index into the
//  hash table. The index is the index of the cab entry in the locoTable array. 
// The routine will use linear probing to resolve collisions. The routine assumes 
// that the cabId is not already in the table.
//
//----------------------------------------------------------------------------------------
void insertCabTableIndex( uint16_t cabId, uint16_t index ) {

    uint32_t i = hash( cabId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    uint32_t start = i;

    while ( locoIndexTable [ i ].cabId != NIL_CAB_ID ) {

        i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        if ( i == start ) return; 
    }

    locoIndexTable [ i ].cabId = cabId;
    locoIndexTable [ i ].index = index;
}

//----------------------------------------------------------------------------------------
// Remove from hash table. The routine will remove the cabId from the hash table.
// The routine will use linear probing to find the cabId and remove it. The 
// routine will also reinsert any entries that were in the same cluster as the 
// removed entry to ensure that they can still be found by their cabId.
//
//----------------------------------------------------------------------------------------
void removeCabTableIndex( uint16_t cabId ) {

    uint32_t i      = hash( cabId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    uint32_t start  = i;

    while ( locoIndexTable [ i ].cabId != NIL_CAB_ID ) {

        if ( locoIndexTable [ i ].cabId == cabId ) break;
        i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        if ( i == start ) return;
    }

    if ( locoIndexTable [ i ].cabId == NIL_CAB_ID ) return;

    locoIndexTable [ i ].cabId = NIL_CAB_ID;

    uint32_t j = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );

    while ( locoIndexTable [ j ].cabId != NIL_CAB_ID ) {

        CabTableIndexEntry tmp = locoIndexTable [ j ];
        locoIndexTable [ j ].cabId = NIL_CAB_ID;

        uint32_t k = hash( tmp.cabId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        uint32_t kstart = k;

        while ( locoIndexTable [ k ].cabId != NIL_CAB_ID ) {

            k = ( k + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
            if ( k == kstart ) break;
        }

        locoIndexTable [ k ] = tmp;
        j = ( j + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    }
}

//----------------------------------------------------------------------------------------
// Remove from cab table. The routine will remove the cab entry from the locoTable
// array and update the cabTableHwm. The routine will also move the last entry in
// the locoTable array to the removed entry's position to maintain a contiguous 
// array of active entries.
//
//----------------------------------------------------------------------------------------
void removeCabTableEntry( CabTableEntry *entry ) {

    uint16_t index = ( uint16_t )( entry - locoTable );
    if ( index >= cabTableHwm ) return;

    uint16_t last = cabTableHwm - 1;

    if ( index != last ) {
        
        locoTable [ index ] = locoTable [ last ];

        uint16_t movedId = locoTable [ index ].cabId;
        uint32_t i       = hash( movedId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );

        while ( locoIndexTable [ i ].cabId != NIL_CAB_ID ) {

            if ( locoIndexTable [ i ].cabId == movedId ) {

                locoIndexTable [ i ].index = index;
                break;
            }

            i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        }
    }

    cabTableHwm--;

} // namespace

//----------------------------------------------------------------------------------------
// Setup Cab Table. The routine will initialize the cab table and the hash table. 
//
//----------------------------------------------------------------------------------------
void cabTableInit( ) {

    for ( int i = 0; i < MAX_CAB_HASH_TAB_ENTRIES; i++ ) {
        
        locoIndexTable [ i ].cabId = NIL_CAB_ID;
    }

    for ( int i = 0; i < MAX_CAB_ENTRIES; i++ ) {

        locoTable [ i ].cabId = NIL_CAB_ID;
        locoTable [ i ].lastSeenTs = 0;
    }

    cabTableHwm = 0;
}

//----------------------------------------------------------------------------------------
// Lookup a cabId. The routine will lookup the cabId in the hash table and return
// a pointer to the cab entry in the locoTable array if found, or nullptr if not
// found.
//
//----------------------------------------------------------------------------------------
CabTableEntry *lookupCab( uint16_t cabId ) {

    uint32_t i = hash( cabId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    uint32_t start = i;

    while ( locoIndexTable [ i ].cabId != NIL_CAB_ID ) {

        if ( locoIndexTable [ i ].cabId == cabId )
            return &locoTable [ locoIndexTable [ i ].index ];

        i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        if ( i == start ) return NULL;
    }

    return ( nullptr );
}

//----------------------------------------------------------------------------------------
// Lookup cabId and add if not found. The routine will lookup the cabId in the 
// hash table and return a pointer to the cab entry in the locoTable array if +
// found. If not found, the routine will add a new entry to the locoTable array
// and insert the cabId and index into  the hash table. The routine will return
// a pointer to the new cab entry, or nullptr if the cab table is full.
//
//----------------------------------------------------------------------------------------
CabTableEntry *lookupCabAndAdd( uint16_t cabId ) {

    if ( cabId == NIL_CAB_ID ) return ( nullptr );

    CabTableEntry *l = lookupCab( cabId );
    if ( l ) return l;

    if ( cabTableHwm >= MAX_CAB_ENTRIES ) return ( nullptr );

    uint16_t index                  = cabTableHwm++;
    locoTable [ index ].cabId       = cabId;
    locoTable [ index ].lastSeenTs  = CDC::getMillis( );

    insertCabTableIndex( cabId, index );
    return ( &locoTable [ index ] );
}

//----------------------------------------------------------------------------------------
// Remove a cabId. The routine will remove the cabId from the hash table and the
// cab entry from the locoTable array.
//
//----------------------------------------------------------------------------------------
void removeCab( uint16_t cabId ) {

    CabTableEntry *l = lookupCab( cabId );
    if ( ! l ) return;

    removeCabTableIndex( cabId );
    removeCabTableEntry( l );
}

//----------------------------------------------------------------------------------------
// Scan CabTable for expired cabs. The timeout value is in milliseconds. The 
// routine will be called periodically to remove cabs that have not been seen 
// for a certain time.
//
//----------------------------------------------------------------------------------------
void cabTableAge( uint32_t timeout ) {

    uint32_t now = CDC::getMillis( );

    for ( uint16_t i = 0; i < cabTableHwm; ) {

        if ( ( now - locoTable [ i ].lastSeenTs ) > timeout ) {

            removeCab( locoTable [ i ].cabId );
        } 
        else i++;
    }
}

//----------------------------------------------------------------------------------------
// List cabTable table.
//
//----------------------------------------------------------------------------------------
void dumpCabTable( ) {

    printf( "---- LOCOS ( %u ) ----\n", cabTableHwm );

    for ( uint16_t i = 0; i < cabTableHwm; i++ ) {

        printf( " [ %3u ] id: %5u lastTs: %u\n",
               i,
               locoTable [ i ].cabId,
               locoTable [ i ].lastSeenTs );
    }
}

//----------------------------------------------------------------------------------------
/* ---------------- MAIN TEST ---------------- */
//
//----------------------------------------------------------------------------------------
int main( void ) {

    cabTableInit( );

    for ( int i = 0; i < 20; i++ ) {

        for ( int j = 0; j < 3; j++ ) {

            uint16_t id = rand(  ) % 100;

            CabTableEntry *l = lookupCabAndAdd( id );
            if ( l ) l -> lastSeenTS = CDC::getMillis( );
        }

        CDC::sleepMillis( 2000 );

        cabTableAge( 1000 );

        printf( "\nTICK\n" );
        dumpCabTable( );
    }

    return 0;
}
