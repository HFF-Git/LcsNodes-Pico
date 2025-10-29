//----------------------------------------------------------------------------------------
//
// LCS - OLED Display Driver - Include file
//
//----------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------
//
// LCS - OLED Display Driver - Include file
// Copyright (C) 2024 - 2025  Helmut Fieres
//
// Bill Greiman wrote a version for the Arduino world. I took his files, and adapted
// them for my needs and  the PICO environment. Here is the original copyright info.
//
// SSD1306Ascii - Oled Library for the Arduino world.
// Copyright (c) 2011-2023 Bill Greiman
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
#ifndef LcsOledDisplayLib_h
#define LcsOledDisplayLib_h

#include "LcsUtilLib.h"
#include "LcsCdcLib.h"
#include "fonts/allFonts.h"

/**
 * If ENABLE_NONFONT_SPACE is nonzero, a space of width FONT_WIDTH will
 * be enabled in fonts which do not have an encoding for 0X20, space.
 */
#ifndef ENABLE_NONFONT_SPACE
#define ENABLE_NONFONT_SPACE 1
#endif  // ENABLE_NONFONT_SPACE

//----------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------
enum OledDevType : uint8_t {

    ODT_OLED_DISPLAY_NIL            = 0,
    ODT_OLED_DISPLAY_128x32_SSD1306 = 1,
    ODT_OLED_DISPLAY_128x64_SSD1306 = 2,
    ODT_OLED_DISPLAY_128x64_SH1106  = 3
};

//----------------------------------------------------------------------------------------
// 
// ??? have a font id enum ... so we can keep the font business to this library...
//----------------------------------------------------------------------------------------
enum FontId : uint8_t {

  FID_DEF   = 0,
  FID_5x7   = 1,
  FID_8x8   = 2,
  FID_8x16  = 3,
  FID_10x16 = 4,
};

//----------------------------------------------------------------------------------------
// 
//
//
//----------------------------------------------------------------------------------------
struct LcsOledDisplay  {

    LcsOledDisplay( );

    uint8_t begin(  uint8_t     devType, 
                    uint8_t     rNumI2C, 
                    uint8_t     i2cAddr, 
                    uint8_t     rNumRST = CDC::CDC_RN_UNDEFINED );

    void            displayOn( );
    void            displayOff( );
    uint8_t         charSpacingPixels( uint8_t ch );
    uint8_t         charWidthPixels( uint8_t ch ) const;
  
    void            clear( );
    void            clearRegion( uint8_t startCol, uint8_t endCol, 
                                 uint8_t startRow, uint8_t endRow ) ;
    void            clearField( uint8_t col, uint8_t row, uint8_t numChars );
    void            clearToEOL( );

    uint8_t         col( ) const;
    uint8_t         row() const { return m_row; }
  
    uint8_t         displayHeightPixels( );
    uint8_t         displayWidthPixels( );
    uint8_t         displayRows( );
    void            displayRemap180Degrees( bool mode );

    size_t          fieldWidthPixels(uint8_t numChars ) const;
    size_t          strWidthPixels( const char* str ) const;

    const uint8_t*  font( ) const;
    void            setFont( uint8_t fontId );
    void            setFont( const uint8_t* fontTablePtr );
    uint8_t         fontCharCount( ) const;
    char            fontFirstChar( ) const;
    uint8_t         fontHeightPixels( ) const;
    uint8_t         fontWidthPixels( ) const;
    uint8_t         fontRows( ) const;
    uint16_t        fontSize( ) const;

    void            invertDisplay( bool invert );
    bool            invertMode( ) const;
    void            setInvertMode( bool mode );

    uint8_t         letterSpacingPixels( ) const { 
                
                        return m_magFactor * m_letterSpacing; 
                    }

    uint8_t         magFactor( ) const { return m_magFactor; }
    void            set1X( ) { m_magFactor = 1; }
    void            set2X( ) { m_magFactor = 2; }

    void            setColPixels( uint8_t colInPixels );
    void            skipColumnsPixels( uint8_t n ) { m_skip = n; }

    void            setContrast( uint8_t value) ;
    void            setCursor( uint8_t colInPixels, uint8_t rowIn8Pixels );
    
    void            setLetterSpacingPixels( uint8_t pixels ) {
                             m_letterSpacing = pixels; 
                    }
    
    void            setRow(uint8_t rowIn8Pixels );

    size_t          writeChar( uint8_t ch );

    private:

    void            setupDevType( uint8_t dType );
    void            ssd1306WriteCmd( uint8_t cmdByte );
    void            ssd1306WriteRamImmediate( uint8_t val );
    void            ssd1306WriteRamBuffered( uint8_t val );
    void            writeDisplay( uint8_t b, uint8_t mode );

    uint8_t         rNumI2C = 0;
    uint8_t         rNumRST = 0;
    uint8_t         i2cAdr  = 0;

    uint8_t         m_col;                      // Cursor column in pixels.
    uint8_t         m_row;                      // Cursor RAM row.
    uint8_t         m_displayWidth;             // Display width in pixels.
    uint8_t         m_displayHeight;            // Display height in pixels.
    uint8_t         m_colOffset;                // Column offset RAM to SEG.
    uint8_t         m_letterSpacing;            // Letter-spacing in pixels.
    uint8_t         m_skip = 0;                 // Skip columns
    uint8_t         m_invertMask = 0;           // font invert mask
    uint8_t         m_magFactor = 1;            // Magnification factor.
    const uint8_t   *m_font = nullptr;          // Current font table.
};

#endif