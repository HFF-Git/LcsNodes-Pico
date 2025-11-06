//----------------------------------------------------------------------------------------
//
// Layout Control System - Utility routines include file
//
//----------------------------------------------------------------------------------------
// This library includes some basic utility routines used by various LCS project 
// parts. Most of the routines are inlined C++ routines defined in this include file.
//
//----------------------------------------------------------------------------------------
//
// Layout Control System - Command Interpreter
// Copyright (C) 2025 - 2025 Helmut Fieres
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
#ifndef LCS_UTIL_LIB_h
#define LCS_UTIL_LIB_h

//----------------------------------------------------------------------------------------
//  Basic include files.
//
//----------------------------------------------------------------------------------------
#include <stdint.h>
#include <inttypes.h>

//----------------------------------------------------------------------------------------
// Little helper functions.
//
//----------------------------------------------------------------------------------------
inline uint16_t roundup( uint16_t elements, uint16_t alignSize ) {

    return ((( elements + alignSize - 1 ) / alignSize ) * alignSize );
}

inline bool isInRangeU( uint16_t val, uint16_t lower, uint16_t upper ) {

    return (( val >= lower ) && ( val <= upper ));
}

inline uint16_t buildNpId( uint16_t nodeId, uint16_t portId ) {

    return (( nodeId << 4 ) | ( portId & 0xF ));
}

inline uint16_t nodeId( uint16_t npId ) {

    return ( npId >> 4 );
}

inline uint16_t portId( uint16_t npId ) {

    return ( npId & 0xF );
}

inline uint8_t lowByte( uint16_t arg ) { 
    
    return ( arg & 0xFF ); 
}

inline uint8_t highByte( uint16_t arg ) { 
    
    return ( arg >> 8 ); 
}

inline bool isInRange( unsigned int val, unsigned int lower, unsigned int upper ) {

    return (( val >= lower ) && ( val <= upper ));
}

#endif