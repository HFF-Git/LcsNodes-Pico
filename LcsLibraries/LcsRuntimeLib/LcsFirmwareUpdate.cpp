//------------------------------------------------------------------------------------------------------------
//
// Layout Control System - Runtime Library Firmware Update.
//
//------------------------------------------------------------------------------------------------------------
// This file contains the part of he runtime library that deals with the remote firmware update.
//
//
// ??? highly processor family dependent... what to abstract ?
//------------------------------------------------------------------------------------------------------------
//
// LCS - Runtime Library Firmware Update.
// Copyright (C) 2021 - 2025  Helmut Fieres
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
#include "LcsRuntimeLib.h"
#include "LcsRtLibInt.h"

//------------------------------------------------------------------------------------------------------------
// External declaration to global structures defined in "LcsRtSetup".
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {

    using namespace LCS;


    //--------------------------------------------------------------------------------------------------------
    // "crc16_ccitt" is used to build a CRC-16 checksum over the data block.
    //
    //--------------------------------------------------------------------------------------------------------
    uint16_t crc16_ccitt( const uint8_t *data, size_t length ) {
    
        uint16_t crc = 0xFFFF; // Initial value
    
        for (size_t i = 0; i < length; i++) {
            
            crc ^= (uint16_t)data[i] << 8;
            
            for ( uint8_t j = 0; j < 8; j++ ) {
            
                if (crc & 0x8000)   crc = (crc << 1) ^ 0x1021;
                else                crc <<= 1;
            }
        }
    
        return crc;
    }


    //--------------------------------------------------------------------------------------------------------
    // "crc32_update" is used to build a checksum over a series of blocks, since we can only handle one
    // block transfer at a time.
    //
    //--------------------------------------------------------------------------------------------------------
    uint32_t crc32_update( uint32_t crc, const uint8_t *data, size_t length ) {

        for (size_t i = 0; i < length; i++) {
            
            crc ^= data[i];

            for (uint8_t j = 0; j < 8; j++) {

                if (crc & 1)    crc = (crc >> 1) ^ 0x04C11DB7;
                else            crc >>= 1;
            }
        }
        
        return crc;
    }

    #if 0

        // sketch how these routines are used.

        // Initialize the CRC-32
        uint32_t crc32 = 0xFFFFFFFF;

        // Process each chunk
        for (size_t chunk = 0; chunk < total_chunks; chunk++) {
            // Assuming `chunk_data` contains the current chunk's data
            crc32 = crc32_update(crc32, chunk_data, chunk_size);

            // Validate the CRC-16 of the chunk before updating
            if (!validate_crc16(chunk_data, chunk_crc16)) {
                // Request retransmission
                request_retransm    ission(chunk);
                continue;
            }
        }

        // Finalize the CRC-32
        crc32 = ~crc32;

        // Compare with the sender's CRC-32
        if (crc32 == sender_crc32)  printf("File transfer successful.\n");
        else                        printf("File transfer failed: CRC mismatch.\n");

    #endif

    // now, all we need is to define LCS messages that send the data in 8-byte messages, 
    // accumulate the data in 1Kbyte data blocks, verify the block checksum, update the
    // CRC32 checksum and finally test the overall checksum.

} // namespace


//------------------------------------------------------------------------------------------------------------
// The LCS name space routines declared in this file.
//
//------------------------------------------------------------------------------------------------------------
namespace LCS {


} // LCS namespace

