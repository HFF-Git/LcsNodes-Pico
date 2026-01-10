//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime Library Version Info
//
//----------------------------------------------------------------------------------------
// The whole purpose of this include file is to record version and patch level.
//
//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime Library Version Info
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
#ifndef LCS_RT_LIB_VERSION_h
#define LCS_RT_LIB_VERSION_h

//----------------------------------------------------------------------------------------
// We have a GIT pre-commit script that modifies the following constants. On each
// commit the patch level increases. A new version / sub version needs to be set 
// by hand.
// 
//----------------------------------------------------------------------------------------
namespace LCS {

    const char      LCS_RT_LIB_GIT_BRANCH[ ] = "git-branch";
    const uint16_t  LCS_RT_LIB_VERSION       = 0x0100;
    const uint16_t  LCS_RT_LIB_PATCH_LEVEL   = 0;
}

#endif