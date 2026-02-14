//----------------------------------------------------------------------------------------
//
// Layout Control System - Utility routines include file
//
//----------------------------------------------------------------------------------------
// This library includes some basic utility routines used by various LCS project 
// parts. Most of the routines are inlined C++ routines defined in this include 
// file.
//
//----------------------------------------------------------------------------------------
//
// Layout Control System - Command Interpreter
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
#pragma once

//----------------------------------------------------------------------------------------
//  Basic include files.
//
//----------------------------------------------------------------------------------------
#include <stdint.h>
#include <inttypes.h>

//----------------------------------------------------------------------------------------
// Little common helper functions.
//
//----------------------------------------------------------------------------------------
inline uint16_t rounddownPow2( uint16_t value, uint16_t align ) {

    return ( value & ~( align - 1 ));
}

inline uint16_t roundupPow2( uint16_t value, uint16_t align ) {

    return (( value + align - 1 ) & ~( align - 1 ));
}

inline uint16_t nodeId( uint16_t npId ) {

    return ( npId >> 8 );
}

inline uint16_t portId( uint16_t npId ) {

    return (( npId >> 4 ) & 0xF );
}

inline uint16_t chanId( uint16_t npId ) {

    return ( npId & 0xF );
}

inline uint16_t buildNpId( uint16_t nodeId, uint16_t portId, uint16_t chanId ) {

    return((( nodeId & 0xFF ) << 8 ) | (( portId &0xF ) << 4 ) | ( chanId & 0xF ));
}

inline bool equalNodeId( uint16_t npId1, uint16_t npId2 ) {

    return ( nodeId( npId1 ) == nodeId( npId2 ));
}

inline uint8_t lowByte( uint16_t arg ) { 
    
    return ( arg & 0xFF ); 
}

inline uint8_t highByte( uint16_t arg ) { 
    
    return ( arg >> 8 ); 
}

inline uint8_t bitGet( uint16_t value, uint8_t bit) {

    return (( value >> bit ) & 1U );
}

inline void bitSet( uint16_t *value, uint8_t bit ) {
    
    *value |= ( 1U << bit );
}

inline void bitClear( uint16_t *value, uint8_t bit) {
    
    *value &= ~ ( 1U << bit );
}

inline bool isInRange( int val, int lower, int upper ) {

    return (( val >= lower ) && ( val <= upper ));
}

inline bool isInRangeU( unsigned int val, unsigned int lower, unsigned int upper ) {

    return (( val >= lower ) && ( val <= upper ));
}

static inline bool isInRangeU8( uint8_t val, uint8_t lower, uint8_t upper ) {

    return (( val >= lower ) && ( val <= upper ));
}

static inline bool isInRangeU16( uint16_t val, uint16_t lower, uint16_t upper ) {

    return (( val >= lower ) && ( val <= upper ));
}

static inline bool isInRangeU32( uint32_t val, uint32_t lower, uint32_t upper ) {

    return (( val >= lower ) && ( val <= upper ));
}

static inline unsigned int clamp( unsigned int val, unsigned int low, unsigned int high ) {

    if ( val < low ) return low;
    if ( val > high ) return high;
    return val;
}

static inline uint8_t  clampU8( uint8_t val, uint8_t low, uint8_t high ) {

    if ( val < low ) return low;
    if ( val > high ) return high;
    return val;
}

static inline uint16_t clampU16( uint16_t val, uint16_t low, uint16_t high ) {

    if ( val < low ) return low;
    if ( val > high ) return high;
    return val;
}

static inline uint32_t clampU32( uint32_t val, uint32_t low, uint32_t high ) {
    
    if ( val < low ) return low;
    if ( val > high ) return high;
    return val;
}

