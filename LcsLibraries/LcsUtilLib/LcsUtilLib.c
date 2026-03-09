//----------------------------------------------------------------------------------------
//
// Layout Control System - Utility routines
//
//----------------------------------------------------------------------------------------
// The LCS Utility library includes some basic utility routines used by various LCS 
// project parts. Most of the routines are inlined C++ routines defined in the 
// corresponding include file. However, some routines cannot be inlined and are 
// therefore implemented as regular C functions in this source file.
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
#include "LcsUtilLib.h"

//----------------------------------------------------------------------------------------
// A simple helper to return an LCS message in a string.
//
//----------------------------------------------------------------------------------------
int lcsMsgStr( uint8_t *msg, uint8_t *buf, int bufLen ) {

    int  len;
    char lBuf[ 64 ];

    len = snprintf( lBuf, sizeof( lBuf ), "LCS MSG: op: %d, data: ", msg[ 0 ] & 0x1F );
    for ( int i = 0; i < ( msg[ 0 ] >> 5 ) + 1; i ++ ) {
        
        len += snprintf( lBuf + len, 8, "0x%2x ", msg[ i ] ); 
    }
   
    return( len );
}

//----------------------------------------------------------------------------------------
// A simple helper to print an LCS message.
//
//----------------------------------------------------------------------------------------
void printLcsMsg( uint8_t *msg ) {

    uint8_t msgStr[ 128 ];
    printf( "%s\n", msgStr, sizeof( msgStr ));
}


