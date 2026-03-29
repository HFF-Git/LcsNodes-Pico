#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

const int  MAX_CAB_ENTRIES = 256;
const int MAX_CAB_HASH_TAB_ENTRIES = 512;   // must be power of two
const uint16_t NIL_CAB_ID = 0;

struct CabTableEntry {

    uint16_t cabId;
    uint32_t last_seen;
};

struct CabTableIndexEntry {

    uint16_t cabId;
    uint16_t index;
};

struct CabTable {

    uint16_t            hwm;
    uint32_t            tick;
    CabTableEntry       locoTable [ MAX_CAB_ENTRIES ];
    CabTableIndexEntry  locoIndexTable [ MAX_CAB_HASH_TAB_ENTRIES ];
};

namespace {

/* ---------------- HASH ---------------- */

static inline uint32_t hash( uint16_t x ) {

    x ^= x >> 7;
    x *= 0x9E37;
    x ^= x >> 9;
    return x;
}

/* ---------------- INSERT INDEX ---------------- */

void insertCabTableIndex( CabTable *t, uint16_t id, uint16_t idx ) {

    uint32_t i = hash( id ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    uint32_t start = i;

    while ( t -> locoIndexTable [ i ].cabId != NIL_CAB_ID ) {
        i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );

        if ( i == start )
            return; // table full ( should not happen )
    }

    t -> locoIndexTable [ i ].cabId = id;
    t -> locoIndexTable [ i ].index = idx;
}

/* ---------------- REMOVE INDEX ---------------- */

void removeCabTableIndex( CabTable *t, uint16_t id ) {

    uint32_t i = hash( id ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    uint32_t start = i;

    while ( t -> locoIndexTable [ i ].cabId != NIL_CAB_ID ) {

        if ( t -> locoIndexTable [ i ].cabId == id ) break;

        i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );

        if ( i == start ) return;
    }

    if ( t -> locoIndexTable [ i ].cabId == NIL_CAB_ID ) return;

    // remove
    t -> locoIndexTable [ i ].cabId = NIL_CAB_ID;

    // reinsert cluster
    uint32_t j = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );

    while ( t -> locoIndexTable [ j ].cabId != NIL_CAB_ID ) {

        CabTableIndexEntry tmp = t -> locoIndexTable [ j ];
        t -> locoIndexTable [ j ].cabId = NIL_CAB_ID;

        uint32_t k = hash( tmp.cabId ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        uint32_t kstart = k;

        while ( t -> locoIndexTable [ k ].cabId != NIL_CAB_ID ) {

            k = ( k + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );

            if ( k == kstart ) break;
        }

        t -> locoIndexTable [ k ] = tmp;

        j = ( j + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    }
}


} // namespace



/* ---------------- INIT ---------------- */

void cabTable_init(  CabTable *t  ) {

    for ( int i = 0; i < MAX_CAB_HASH_TAB_ENTRIES; i++ ) {
        
        t -> locoIndexTable [ i ].cabId = NIL_CAB_ID;
    }
    t -> hwm = 0;
    t -> tick = 0;
}

/* ---------------- FIND ---------------- */

CabTableEntry *lookupCab( CabTable *t, uint16_t id ) {

    uint32_t i = hash( id ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
    uint32_t start = i;

    while ( t -> locoIndexTable [ i ].cabId != NIL_CAB_ID ) {

        if ( t -> locoIndexTable [ i ].cabId == id )
            return &t -> locoTable [ t -> locoIndexTable [ i ].index ];

        i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );

        if ( i == start ) return NULL;
    }

    return NULL;
}

/* ---------------- GET OR ADD ---------------- */

CabTableEntry *lookupCabAndAdd( CabTable *t, uint16_t id ) {

    if ( id == NIL_CAB_ID )
        return NULL;

    CabTableEntry *l = lookupCab( t, id );
    if ( l ) return l;

    if ( t -> hwm >= MAX_CAB_ENTRIES ) return NULL;

    uint16_t idx = t -> hwm++;
    t -> locoTable [ idx ].cabId = id;
    t -> locoTable [ idx ].last_seen = t -> tick;

    insertCabTableIndex( t, id, idx );
    return &t -> locoTable [ idx ];
}

/* ---------------- REMOVE LOCO ---------------- */

void removeCab( CabTable *t, uint16_t id ) {

    CabTableEntry *l = lookupCab( t, id );
    if ( !l ) return;

    uint16_t idx = ( uint16_t )( l - t -> locoTable );
    uint16_t last = t -> hwm - 1;

    removeCabTableIndex( t, id );

    if ( idx != last ) {
        t -> locoTable [ idx ] = t -> locoTable [ last ];

        uint16_t moved_id = t -> locoTable [ idx ].cabId;

        uint32_t i = hash( moved_id ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );

        while ( t -> locoIndexTable [ i ].cabId != NIL_CAB_ID ) {

            if ( t -> locoIndexTable [ i ].cabId == moved_id ) {

                t -> locoIndexTable [ i ].index = idx;
                break;
            }

            i = ( i + 1 ) & ( MAX_CAB_HASH_TAB_ENTRIES - 1 );
        }
    }

    t -> hwm--;
}

/* ---------------- AGING ---------------- */

void cabTable_age( CabTable *t, uint32_t timeout ) {

    for ( uint16_t i = 0; i < t -> hwm;  ) {

        if ( ( t -> tick - t -> locoTable [ i ].last_seen ) > timeout ) {

            removeCab( t, t -> locoTable [ i ].cabId );

        } else {
            i++;
        }
    }
}

/* ---------------- DEBUG ---------------- */

void dumpCabTable( CabTable *t ) {

    printf( "---- LOCOS ( %u ) ----\n", t -> hwm );

    for ( uint16_t i = 0; i < t -> hwm; i++ ) {

        printf( " [ %3u ] id=%5u last=%u\n",
               i,
               t -> locoTable [ i ].cabId,
               t -> locoTable [ i ].last_seen );
    }
}

/* ---------------- MAIN TEST ---------------- */

int main( void ) {

    CabTable table;
    cabTable_init( &table );

    for ( table.tick = 0; table.tick < 50; table.tick++ ) {

        for ( int i = 0; i < 3; i++ ) {

            uint16_t id = rand(  ) % 100;

            CabTableEntry *l = lookupCabAndAdd( &table, id );
            if ( l ) l -> last_seen = table.tick;
        }

        cabTable_age( &table, 10 );

        printf( "\nTICK %u\n", table.tick );
        dumpCabTable( &table );
    }

    return 0;
}
