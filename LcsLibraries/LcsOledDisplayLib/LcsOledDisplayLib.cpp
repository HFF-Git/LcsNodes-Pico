//------------------------------------------------------------------------------------------------------------
//
// LCS - OLED Display Driver
//
//------------------------------------------------------------------------------------------------------------
// This source file contains ...
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - OLED Display Driver - Raspberry Pi PIOCO implementation
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

#include "LcsOledDisplayLib.h"

//------------------------------------------------------------------------------------------------------------
// Local name space. This file has two sections. The first is this local name space with all internal
// variables and routines local to the file. The second part contains the exported routines to be called by
// the core library and the firmware designers.
//
//------------------------------------------------------------------------------------------------------------
namespace {

    //--------------------------------------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------------------------------------
    
    /** Set Lower Column Start Address for Page Addressing Mode. */
    #define SSD1306_SETLOWCOLUMN 0x00
    /** Set Higher Column Start Address for Page Addressing Mode. */
    #define SSD1306_SETHIGHCOLUMN 0x10
    /** Set Memory Addressing Mode. */
    #define SSD1306_MEMORYMODE 0x20
    /** Set display RAM display start line register from 0 - 63. */
    #define SSD1306_SETSTARTLINE 0x40
    /** Set Display Contrast to one of 256 steps. */
    #define SSD1306_SETCONTRAST 0x81
    /** Enable or disable charge pump.  Follow with 0X14 enable, 0X10 disable. */
    #define SSD1306_CHARGEPUMP 0x8D
    /** Set Segment Re-map between data column and the segment driver. */
    #define SSD1306_SEGREMAP 0xA0
    /** Resume display from GRAM content. */
    #define SSD1306_DISPLAYALLON_RESUME 0xA4
    /** Force display on regardless of GRAM content. */
    #define SSD1306_DISPLAYALLON 0xA5
    /** Set Normal Display. */
    #define SSD1306_NORMALDISPLAY 0xA6
    /** Set Inverse Display. */
    #define SSD1306_INVERTDISPLAY 0xA7
    /** Set Multiplex Ratio from 16 to 63. */
    #define SSD1306_SETMULTIPLEX 0xA8
    /** Set Display off. */
    #define SSD1306_DISPLAYOFF 0xAE
    /** Set Display on. */
    #define SSD1306_DISPLAYON 0xAF
    /**Set GDDRAM Page Start Address. */
    #define SSD1306_SETSTARTPAGE 0XB0
    /** Set COM output scan direction normal. */
    #define SSD1306_COMSCANINC 0xC0
    /** Set COM output scan direction reversed. */
    #define SSD1306_COMSCANDEC 0xC8
    /** Set Display Offset. */
    #define SSD1306_SETDISPLAYOFFSET 0xD3
    /** Sets COM signals pin configuration to match the OLED panel layout. */
    #define SSD1306_SETCOMPINS 0xDA
    /** This command adjusts the VCOMH regulator output. */
    #define SSD1306_SETVCOMDETECT 0xDB
    /** Set Display Clock Divide Ratio/ Oscillator Frequency. */
    #define SSD1306_SETDISPLAYCLOCKDIV 0xD5
    /** Set Pre-charge Period */
    #define SSD1306_SETPRECHARGE 0xD9
    /** Deactivate scroll */
    #define SSD1306_DEACTIVATE_SCROLL 0x2E
    /** No Operation Command. */
    #define SSD1306_NOP 0XE3

    //------------------------------------------------------------------------------
    /** Set Pump voltage value: (30H~33H) 6.4, 7.4, 8.0 (POR), 9.0. */
    #define SH1106_SET_PUMP_VOLTAGE 0X30
    /** First byte of set charge pump mode */
    #define SH1106_SET_PUMP_MODE 0XAD
    /** Second byte charge pump on. */
    #define SH1106_PUMP_ON 0X8B
    /** Second byte charge pump off. */
    #define SH1106_PUMP_OFF 0X8A
    //------------------------------------------------------------------------------

    //------------------------------------------------------------------------------
    // Values for writeDisplay() mode parameter.
    /** Write to Command register. */
    #define SSD1306_MODE_CMD 0
    /** Write one byte to display RAM. */
    #define SSD1306_MODE_RAM 1
    /** Write to display RAM with possible buffering. */
    #define SSD1306_MODE_RAM_BUF 2



    //-------------------------------------------------------------------------------------------------------
    //
    // If ENABLE_NONFONT_SPACE is nonzero, a space of width FONT_WIDTH will
    // be enabled in fonts which do not have an encoding for 0X20, space.
    // ??? what to do about it ?
    //-------------------------------------------------------------------------------------------------------
    #ifndef ENABLE_NONFONT_SPACE
    #define ENABLE_NONFONT_SPACE 1
    #endif  // ENABLE_NONFONT_SPACE


    //--------------------------------------------------------------------------------------------------------
    //
    //
    //
    //--------------------------------------------------------------------------------------------------------
    struct DevTypeNew {
 
        const uint8_t *initCmdList;
        uint8_t initSizeBytes;
        uint8_t lcdWidthPixels;
        uint8_t lcdHeightPixels;
        uint8_t colOffset;
    };

    //--------------------------------------------------------------------------------------------------------
    //
    // this section is based on https://github.com/adafruit/Adafruit_SSD1306
    // Initialization commands for a 128x32 SSD1306 oled display. 
    // Init sequence for Adafruit 128x32 OLED module
    //--------------------------------------------------------------------------------------------------------
    constexpr uint8_t Adafruit128x32initNEW[ ] = {

        SSD1306_DISPLAYOFF,
        SSD1306_SETDISPLAYCLOCKDIV, 0x80,  // the suggested ratio 0x80
        SSD1306_SETMULTIPLEX, 0x1F,        // ratio 32
        SSD1306_SETDISPLAYOFFSET, 0x0,     // no offset
        SSD1306_SETSTARTLINE,              // line #0
        SSD1306_CHARGEPUMP, 0x14,          // internal vcc
        SSD1306_MEMORYMODE, 0x02,          // page mode
        SSD1306_SEGREMAP | 0x1,            // column 127 mapped to SEG0
        SSD1306_COMSCANDEC,                // column scan direction reversed
        SSD1306_SETCOMPINS, 0x02,          // sequential COM pins, disable remap
        SSD1306_SETCONTRAST, 0x7F,         // contrast level 127
        SSD1306_SETPRECHARGE, 0xF1,        // pre-charge period (1, 15)
        SSD1306_SETVCOMDETECT, 0x40,       // vcomh regulator level
        SSD1306_DISPLAYALLON_RESUME,
        SSD1306_NORMALDISPLAY,
        SSD1306_DISPLAYON
    };

    constexpr DevTypeNew  Adafruit128x32NEW = {
        
        Adafruit128x32initNEW,
        sizeof(Adafruit128x32initNEW),
        128,
        32,
        0
    };

    //--------------------------------------------------------------------------------------------------------
    //
    // This section is based on https://github.com/adafruit/Adafruit_SSD1306
    // Initialization commands for a 128x64 SSD1306 oled display.
    // Init sequence for Adafruit 128x64 OLED module
    //--------------------------------------------------------------------------------------------------------
    const uint8_t Adafruit128x64initNEW[] = {
    
        SSD1306_DISPLAYOFF,
        SSD1306_SETDISPLAYCLOCKDIV, 0x80,  // the suggested ratio 0x80
        SSD1306_SETMULTIPLEX, 0x3F,        // ratio 64
        SSD1306_SETDISPLAYOFFSET, 0x0,     // no offset
        SSD1306_SETSTARTLINE,              // line #0
        SSD1306_CHARGEPUMP, 0x14,          // internal vcc
        SSD1306_MEMORYMODE, 0x02,          // page mode
        SSD1306_SEGREMAP | 0x1,            // column 127 mapped to SEG0
        SSD1306_COMSCANDEC,                // column scan direction reversed
        SSD1306_SETCOMPINS, 0x12,          // alt COM pins, disable remap
        SSD1306_SETCONTRAST, 0x7F,         // contrast level 127
        SSD1306_SETPRECHARGE, 0xF1,        // pre-charge period (1, 15)
        SSD1306_SETVCOMDETECT, 0x40,       // vcomh regulator level
        SSD1306_DISPLAYALLON_RESUME,
        SSD1306_NORMALDISPLAY,
        SSD1306_DISPLAYON
    };

    constexpr DevTypeNew Adafruit128x64NEW = {
  
        Adafruit128x64initNEW,
        sizeof(Adafruit128x64initNEW),
        128,
        64,
        0
    };

    //--------------------------------------------------------------------------------------------------------
    //
    // This section is based on https://github.com/stanleyhuangyc/MultiLCD
    // Initialization commands for a 128x64 SH1106 oled display. 
     // SH1106 is a 132x64 controller.  Use middle 128 columns. ( the 2 at the end ... )
    //--------------------------------------------------------------------------------------------------------
    const uint8_t SH1106_128x64initNEW[] = {
  
        SSD1306_DISPLAYOFF,
        SSD1306_SETSTARTPAGE,                  // set page zero
        SSD1306_SETCONTRAST, 0x80,             // 128
        SSD1306_SEGREMAP | 0X1,                // set segment remap
        SSD1306_NORMALDISPLAY,                 // normal / reverse
        SSD1306_SETMULTIPLEX, 0x3F,            // ratio 64
        SH1106_SET_PUMP_MODE, SH1106_PUMP_ON,  // set charge pump enable
        SH1106_SET_PUMP_VOLTAGE | 0X2,         // 8.0 volts
        SSD1306_COMSCANDEC,                    // Com scan direction
        SSD1306_SETDISPLAYOFFSET, 0X00,        // set display offset
        SSD1306_SETDISPLAYCLOCKDIV, 0X80,      // set osc division
        SSD1306_SETPRECHARGE, 0X1F,            // set pre-charge period
        SSD1306_SETCOMPINS, 0X12,              // set COM pins
        SSD1306_SETVCOMDETECT,  0x40,          // set vcomh
        SSD1306_DISPLAYON
    };

    constexpr DevTypeNew SH1106_128x64NEW =  {
  
        SH1106_128x64initNEW,
        sizeof(SH1106_128x64initNEW),
        128,
        64,
        2   
    };


    //--------------------------------------------------------------------------------------------------------
    //
    //
    //
    //--------------------------------------------------------------------------------------------------------
    uint8_t setupHw( uint8_t sclPin, uint8_t sdaPin, uint8_t rstPin ) {

        uint8_t rStat = CDC::ALL_OK;

        rStat = CDC::configureI2C( sclPin, sdaPin );
        if ( rStat != CDC::ALL_OK ) return( rStat );

        if ( rstPin != CDC::UNDEFINED_PIN ) {

            rStat = CDC::configureDio( rstPin, CDC::OUT );
            if ( rStat != CDC::ALL_OK ) return( rStat );

            CDC::writeDio( rstPin, false );
            CDC::sleepMillis( 10 );
            CDC::writeDio( rstPin, true );
            CDC::sleepMillis( 10 );
        }

        return( rStat );
    }

    //------------------------------------------------------------------------------
    GLCDFONTDECL(scaledNibble) = {0X00, 0X03, 0X0C, 0X0F, 0X30, 0X33, 0X3C, 0X3F,
                              0XC0, 0XC3, 0XCC, 0XCF, 0XF0, 0XF3, 0XFC, 0XFF};

}; // namespace


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
LcsOledDisplay::LcsOledDisplay( ) {  }


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t LcsOledDisplay::begin(  uint8_t devType, 
                                uint8_t sclPin, 
                                uint8_t sdaPin, 
                                uint8_t i2cAdr,
                                uint8_t rstPin  ) {

    this -> sclPin = sclPin;
    this -> sdaPin = sdaPin;
    this -> i2cAdr = i2cAdr;
    this -> rstPin = rstPin;

    // ??? into a debug bracket ?
    printf( "Oled Display begin: sclPin: %d, sdaPin: %d, i2cAdr: 0x%x, rstPin: %d\n ", 
            sclPin, sdaPin, i2cAdr, rstPin );

    uint8_t rStat;

    rStat = setupHw( sclPin, sdaPin, rstPin );
    if ( rStat != CDC::ALL_OK ) return( rStat );

    setupDevType( devType );
    clear( );
    
    return( CDC::ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
void LcsOledDisplay::setupDevType( uint8_t dType ) {

    const DevTypeNew *dev = nullptr;

    switch ( dType ) {

        case ODT_OLED_DISPLAY_128x32_SSD1306:   dev = &Adafruit128x32NEW;   break;
        case ODT_OLED_DISPLAY_128x64_SSD1306:   dev = &Adafruit128x64NEW;   break;
        case ODT_OLED_DISPLAY_128x64_SH1106:    dev = &SH1106_128x64NEW;    break;
        default: dev = &Adafruit128x64NEW;
    }

    m_col           = 0;
    m_row           = 0;
    m_displayWidth  = readFontByte( &dev -> lcdWidthPixels );
    m_displayHeight = readFontByte( &dev -> lcdHeightPixels );
    m_colOffset     = readFontByte( &dev -> colOffset );

    const uint8_t* table = dev->initCmdList;

    uint8_t size    = readFontByte( &dev -> initSizeBytes );
    
    for ( uint8_t i = 0; i < size; i++ ) {
            
        ssd1306WriteCmd( readFontByte( table + i ));
    } 
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
void LcsOledDisplay::displayOn( ) {

    ssd1306WriteCmd( SSD1306_DISPLAYON );
}

void LcsOledDisplay::displayOff( ) {

    ssd1306WriteCmd( SSD1306_DISPLAYOFF );
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
void SSD1306Ascii::clear() {

  clear(0, displayWidth() - 1, 0, displayRows() - 1);
}

void SSD1306Ascii::clear(uint8_t c0, uint8_t c1, uint8_t r0, uint8_t r1) {

  // Cancel skip character pixels.
  m_skip = 0;

  // Insure only rows on display will be cleared.
  if (r1 >= displayRows()) r1 = displayRows() - 1;

  for (uint8_t r = r0; r <= r1; r++) {
    setCursor(c0, r);
    for (uint8_t c = c0; c <= c1; c++) {
      // Insure clear() writes zero. result is (m_invertMask^m_invertMask).
      ssd1306WriteRamBuf(m_invertMask);
    }
  }
  setCursor(c0, r0);
}

void SSD1306Ascii::clearToEOL() {

  clear(m_col, displayWidth() - 1, m_row, m_row + fontRows() - 1);
}

void SSD1306Ascii::clearField(uint8_t col, uint8_t row, uint8_t n) {
  
  clear(col, col + fieldWidth(n) - 1, row, row + fontRows() - 1);
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t SSD1306Ascii::charWidth(uint8_t c) const {

  if (!m_font) {
    return 0;
  }
  uint8_t first = readFontByte(m_font + FONT_FIRST_CHAR);
  uint8_t count = readFontByte(m_font + FONT_CHAR_COUNT);
  if (c < first || c >= (first + count)) {
    return 0;
  }
  if (fontSize() > 1) {
    // Proportional font.
    return m_magFactor * readFontByte(m_font + FONT_WIDTH_TABLE + c - first);
  }
  // Fixed width font.
  return m_magFactor * readFontByte(m_font + FONT_WIDTH);
}

size_t SSD1306Ascii::fieldWidth(uint8_t n) {
  return n * (fontWidth() + letterSpacing());
}

size_t SSD1306Ascii::strWidth(const char* str) const {
  size_t sw = 0;
  while (*str) {
    uint8_t cw = charWidth(*str++);
    if (cw == 0) {
      return 0;
    }
    sw += cw + letterSpacing();
  }
  return sw;
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t SSD1306Ascii::fontCharCount() const {

  return m_font ? readFontByte(m_font + FONT_CHAR_COUNT) : 0;
}

char SSD1306Ascii::fontFirstChar() const {

  return m_font ? readFontByte(m_font + FONT_FIRST_CHAR) : 0;
}

uint8_t SSD1306Ascii::fontHeight() const {

  return m_font ? m_magFactor * readFontByte(m_font + FONT_HEIGHT) : 0;
}

uint8_t SSD1306Ascii::fontRows() const {

  return m_font ? m_magFactor * ((readFontByte(m_font + FONT_HEIGHT) + 7) / 8) : 0;
}

uint16_t SSD1306Ascii::fontSize() const {

  return ( readFontByte(m_font) << 8 ) | readFontByte( m_font + 1 );
}

uint8_t SSD1306Ascii::fontWidth() const {

  return m_font ? m_magFactor * readFontByte(m_font + FONT_WIDTH) : 0;
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
void SSD1306Ascii::invertDisplay(bool invert) {

  ssd1306WriteCmd(invert ? SSD1306_INVERTDISPLAY : SSD1306_NORMALDISPLAY);
}

void SSD1306Ascii::displayRemap(bool mode) {
  ssd1306WriteCmd(mode ? SSD1306_SEGREMAP : SSD1306_SEGREMAP | 1);
  ssd1306WriteCmd(mode ? SSD1306_COMSCANINC : SSD1306_COMSCANDEC);
}

void SSD1306Ascii::setContrast(uint8_t value) {

  ssd1306WriteCmd(SSD1306_SETCONTRAST);
  ssd1306WriteCmd(value);
}

void SSD1306Ascii::setCursor(uint8_t col, uint8_t row) {

  setCol(col);
  setRow(row);
}

void SSD1306Ascii::setFont(const uint8_t* font) {

  m_font = font;
  if (font && fontSize() == 1) {
    m_letterSpacing = 0;
  } else {
    m_letterSpacing = 1;
  }
}

void SSD1306Ascii::setRow(uint8_t row) {

  if ( row < displayRows( )) {
    
    m_row = row;
    ssd1306WriteCmd(SSD1306_SETSTARTPAGE | m_row);
  }
}

void SSD1306Ascii::setCol(uint8_t col) {

  if (col < m_displayWidth) {
    m_col = col;
    col += m_colOffset;
    ssd1306WriteCmd(SSD1306_SETLOWCOLUMN | (col & 0XF));
    ssd1306WriteCmd(SSD1306_SETHIGHCOLUMN | (col >> 4));
  }
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
size_t SSD1306Ascii::write(uint8_t ch) {

  if (!m_font) {
    return 0;
  }
  uint8_t w = readFontByte(m_font + FONT_WIDTH);
  uint8_t h = readFontByte(m_font + FONT_HEIGHT);
  uint8_t nr = (h + 7) / 8;
  uint8_t first = readFontByte(m_font + FONT_FIRST_CHAR);
  uint8_t count = readFontByte(m_font + FONT_CHAR_COUNT);
  const uint8_t* base = m_font + FONT_WIDTH_TABLE;

  if (ch == '\r') {
    setCol(0);
    return 1;
  }
  if (ch == '\n') {
    setCol(0);
    uint8_t fr = m_magFactor * nr;
    setRow(m_row + fr);
    return 1;
  }
  bool nfSpace = false;
  if (first <= ch && ch < (first + count)) {
    ch -= first;
  } else if (ENABLE_NONFONT_SPACE && ch == ' ') {
    nfSpace = true;
  } else {
    // Error if not in font.
    return 0;
  }
  uint8_t s = letterSpacing();
  uint8_t thieleShift = 0;
  if (nfSpace) {
    // non-font space.
  } else if (fontSize() < 2) {
    // Fixed width font.
    base += nr * w * ch;
  } else {
    if (h & 7) {
      thieleShift = 8 - (h & 7);
    }
    uint16_t index = 0;
    for (uint8_t i = 0; i < ch; i++) {
      index += readFontByte(base + i);
    }
    w = readFontByte(base + ch);
    base += nr * index + count;
  }
  uint8_t scol = m_col;
  uint8_t srow = m_row;
  uint8_t skip = m_skip;
  for (uint8_t r = 0; r < nr; r++) {
    for (uint8_t m = 0; m < m_magFactor; m++) {
      skipColumns(skip);
      if (r || m) {
        setCursor(scol, m_row + 1);
      }
      for (uint8_t c = 0; c < w; c++) {
        uint8_t b = nfSpace ? 0 : readFontByte(base + c + r * w);
        if (thieleShift && (r + 1) == nr) {
          b >>= thieleShift;
        }
        if (m_magFactor == 2) {
          b = m ? b >> 4 : b & 0XF;
          b = readFontByte(scaledNibble + b);
          ssd1306WriteRamBuf(b);
        }
        ssd1306WriteRamBuf(b);
      }
      for (uint8_t i = 0; i < s; i++) {
        ssd1306WriteRamBuf(0);
      }
    }
  }
  setRow(srow);
  return 1;
}

void SSD1306Ascii::ssd1306WriteRam(uint8_t c) {

  if (m_col < m_displayWidth) {
    writeDisplay(c ^ m_invertMask, SSD1306_MODE_RAM);
    m_col++;
  }
}

void SSD1306Ascii::ssd1306WriteRamBuf(uint8_t c) {

  if (m_skip) {
    m_skip--;
  } else if (m_col < m_displayWidth) {
    writeDisplay(c ^ m_invertMask, SSD1306_MODE_RAM_BUF);
    m_col++;
  }
}

void LcsOledDisplay::writeDisplay( uint8_t b, uint8_t mode ) {

    uint8_t buf[ 2 ];
    buf[ 0 ] = ( mode == SSD1306_MODE_CMD ) ? 0X00 : 0X40;
    buf[ 1 ] = b;

   uint8_t rStat = CDC::i2cWrite( sclPin, i2cAdr, buf, 2 );

    // ??? into a debug bracket ?
   if ( rStat != CDC::ALL_OK ) printf( "Error in writing to display: %d\n", rStat );
}
