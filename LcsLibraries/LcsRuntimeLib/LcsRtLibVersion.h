//------------------------------------------------------------------------------------------------------------
//
// Layout Control System - Runtime Library Version Info
//
//------------------------------------------------------------------------------------------------------------
// The whole purpose of this include file is to record version and patch level.
//
//------------------------------------------------------------------------------------------------------------
//
// Layout Control System - Runtime Library Version Info
// Copyright (C) 2025 - 2025  Helmut Fieres
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
#ifndef LCS_RT_LIB_VERSION_h
#define LCS_RT_LIB_VERSION_h

//------------------------------------------------------------------------------------------------------------
// 
//
// ??? how do we encode a version ? family, major, minor ?
//
// ??? check out the pre-commit option. Perhaps generalize the python prgram to fix these constants on GIT 
// commit
//------------------------------------------------------------------------------------------------------------
namespace LCS {

    const char      LCS_RT_LIB_GIT_BRANCH[ ] = "git-branch";
    const uint16_t  LCS_RT_LIB_VERSION       = 100;  // ??? for now...
    const uint16_t  LCS_RT_LIB_PATCH_LEVEL   = 0;
    
}

#endif