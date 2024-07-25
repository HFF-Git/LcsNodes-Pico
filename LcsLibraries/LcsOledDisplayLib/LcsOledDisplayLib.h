//------------------------------------------------------------------------------------------------------------
//
// LCS - OLED Display Driver - Include file
//
//------------------------------------------------------------------------------------------------------------
// 
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - OLED Display Driver - Include file
// Copyright (C) 2024 - 2024  Helmut Fieres
//
// Bill Greiman wrote a version for the Arduino world. I took his files, and adapated them for my needs and 
// the PICO environment. Here is the original copyright info.
//
// SSD1306Ascii - Oled Library for the Arduino world.
// Copyright (c) 2011-2023 Bill Greiman
//
// This program is free software: you can redistribute it and/or modify it under the terms of the GNU General
// Public License as published by the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along with this program. If not, see
// http://www.gnu.org/licenses
//
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//------------------------------------------------------------------------------------------------------------
#ifndef LcsOledDisplayLib_h
#define LcsOledDisplayLib_h

#include "LcsCdcLib.h"
#include "SSD1306Ascii.h"
#include "fonts/allFonts.h"


enum OledDevType : uint8_t {

    ODT_OLED_DISPLAY_NIL            = 0,
    ODT_OLED_DISPLAY_128x32_SSD1306 = 1,
    ODT_OLED_DISPLAY_128x64_SSD1306 = 2,
    ODT_OLED_DISPLAY_128x64_SH1106  = 3

};


//------------------------------------------------------------------------------------------------------------
// 
//
//------------------------------------------------------------------------------------------------------------
struct LcsOledDisplay : public SSD1306Ascii {

    LcsOledDisplay( );

    uint8_t begin(  uint8_t     devType, 
                    uint8_t     sclPin, 
                    uint8_t     sdaPin, 
                    uint8_t     i2cAddr, 
                    uint8_t     rstPin = CDC::UNDEFINED_PIN );



    void            displayOn( );
    void            displayOff( );
   

    private:

    void            setupDevType( uint8_t dType );
    void            writeDisplay( uint8_t b, uint8_t mode );

    uint8_t         i2cAdr = CDC::UNDEFINED_PIN;
    uint8_t         sclPin = CDC::UNDEFINED_PIN;
    uint8_t         sdaPin = CDC::UNDEFINED_PIN;
    uint8_t         rstPin = CDC::UNDEFINED_PIN;

    uint8_t         m_col;                      // Cursor column.
    uint8_t         m_row;                      // Cursor RAM row.
    uint8_t         m_displayWidth;             // Display width.
    uint8_t         m_displayHeight;            // Display height.
    uint8_t         m_colOffset;                // Column offset RAM to SEG.
    uint8_t         m_letterSpacing;            // Letter-spacing in pixels.
                                                // INCLUDE_SCROLLING
    uint8_t         m_skip = 0;
    
    uint8_t         m_invertMask = 0;           // font invert mask
    uint8_t         m_magFactor = 1;            // Magnification factor.
    const uint8_t   *m_font = nullptr;          // Current font.
};



#endif