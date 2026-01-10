//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
//
//----------------------------------------------------------------------------------------
// The whole purpose of this include file is to record version and patch level.
//
//----------------------------------------------------------------------------------------
//
// LCS - Controller dependent code Layer - Raspberry PI Pico Implementation
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
#ifndef CDC_LIB_VERSION_h
#define CDC_LIB_VERSION_h

#include <inttypes.h>

//----------------------------------------------------------------------------------------
// Each LCS project piece has a version, subversion and patch level.
// 
//----------------------------------------------------------------------------------------
namespace CDC {

    const char      CDC_LIB_GIT_BRANCH[ ] = "git-branch";
    const uint16_t  CDC_LIB_VERSION       = ( 1U << 8 ) | 1U;
    const uint16_t  CDC_LIB_PATCH_LEVEL   = 0;
}

#endif