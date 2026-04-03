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
//
//
//----------------------------------------------------------------------------------------
struct CabTableEntry {

    uint16_t cabId;
    uint32_t lastSeen;
};

//----------------------------------------------------------------------------------------
//
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
// Insert into hash table.
//
//----------------------------------------------------------------------------------------
void insertCabTableIndex( uint16_t cabId, uint16_t index ) {

    uint32_t i = hash( cabId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    uint32_t start = i;

    while ( locoIndexTable [ i ].cabId != NIL_CAB_ID ) {
        i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );

        if ( i == start ) return; // table full ( should not happen )
    }

    locoIndexTable [ i ].cabId = cabId;
    locoIndexTable [ i ].index = index;
}

//----------------------------------------------------------------------------------------
// Remove from hash table.
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

    // remove
    locoIndexTable [ i ].cabId = NIL_CAB_ID;

    // reinsert cluster
    uint32_t j = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );

    while ( locoIndexTable [ j ].cabId != NIL_CAB_ID ) {

        CabTableIndexEntry tmp = locoIndexTable [ j ];
        locoIndexTable [ j ].cabId = NIL_CAB_ID;

        uint32_t k = hash( tmp.cabId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        uint32_t kstart = k;

        while ( t -> locoIndexTable [ k ].cabId != NIL_CAB_ID ) {

            k = ( k + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
            if ( k == kstart ) break;
        }

        locoIndexTable [ k ] = tmp;
        j = ( j + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    }
}

} // namespace

//----------------------------------------------------------------------------------------
// Setup hash table.
//
//----------------------------------------------------------------------------------------
void cabTableInit( ) {

    for ( int i = 0; i < MAX_CAB_HASH_TAB_ENTRIES; i++ ) {
        
        locoIndexTable [ i ].cabId = NIL_CAB_ID;
    }
    cabTableHwm = 0;
}

//----------------------------------------------------------------------------------------
// Lookup in hash table.
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
// Lookup cabId and add if not found. 
//
//----------------------------------------------------------------------------------------
CabTableEntry *lookupCabAndAdd( uint16_t cabId ) {

    if ( cabId == NIL_CAB_ID ) return ( nullptr );

    CabTableEntry *l = lookupCab( cabId );
    if ( l ) return l;

    if ( cabTableHwm >= MAX_CAB_ENTRIES ) return ( nullptr );

    uint16_t index = cabTableHwm++;
    locoTable [ index ].cabId = cabId;
    locoTable [ index ].lastSeen = CDC::getMillis( );

    insertCabTableIndex( cabId, index );
    return ( &locoTable [ index ] );
}

//----------------------------------------------------------------------------------------
// Remove a cabId.
//
//----------------------------------------------------------------------------------------
void removeCab( uint16_t cabId ) {

    CabTableEntry *l = lookupCab( cabId );
    if ( !l ) return;

    uint16_t index = ( uint16_t )( l - locoTable );
    uint16_t last = cabTableHwm - 1;

    removeCabTableIndex( cabId );

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
}

//----------------------------------------------------------------------------------------
// Scan CabTable for expired cabs.
//
//----------------------------------------------------------------------------------------
void cabTableAge( uint32_t timeout ) {

    uint32_t now = CDC::getMillis( );

    for ( uint16_t i = 0; i < cabTableHwm; ) {

        if ( ( now - locoTable [ i ].lastSeen ) > timeout ) {

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

        printf( " [ %3u ] id=%5u last=%u\n",
               i,
               locoTable [ i ].cabId,
               locoTable [ i ].lastSeen );
    }
}

//----------------------------------------------------------------------------------------
// Remove from hash table.
//
//----------------------------------------------------------------------------------------
/* ---------------- MAIN TEST ---------------- */

int main( void ) {

    cabTableInit( );

    for ( int i = 0; i < 20; i++ ) {

        for ( int j = 0; j < 3; j++ ) {

            uint16_t id = rand(  ) % 100;

            CabTableEntry *l = lookupCabAndAdd( id );
            if ( l ) l -> lastSeen = CDC::getMillis( );
        }

        CDC::sleepMillis( 2000 );

        cabTableAge( 1000 );

        printf( "\nTICK\n" );
        dumpCabTable( );
    }

    return 0;
}
