//------------------------------------------------------------------------------------------------------------
//
// LCS - OLED Display Driver
//
//------------------------------------------------------------------------------------------------------------
// This source file contains the methods to support a set of OLED displays. A display is simply a matrix of
// rows and columns, measured in 8x8 fields. The display will provide a several ASCII fonts to display. 
// no graphics are supported. 
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - OLED Display Driver - Raspberry Pi PIOCO implementation
// Copyright (C) 2024 - 2024  Helmut Fieres
//
// Bill Greiman wrote a version for the Arduino world. I took his files, and adapted them for my needs and 
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
    // SSD1306 commands.
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

    //-------------------------------------------------------------------------------------------------------
    //
    //-------------------------------------------------------------------------------------------------------
    //
    /** Set Pump voltage value: (30H~33H) 6.4, 7.4, 8.0 (POR), 9.0. */
    #define SH1106_SET_PUMP_VOLTAGE 0X30
    /** First byte of set charge pump mode */
    #define SH1106_SET_PUMP_MODE 0XAD
    /** Second byte charge pump on. */
    #define SH1106_PUMP_ON 0X8B
    /** Second byte charge pump off. */
    #define SH1106_PUMP_OFF 0X8A
    //------------------------------------------------------------------------------

    //-------------------------------------------------------------------------------------------------------
    // The display modes when we write to the controller.
    //
    //-------------------------------------------------------------------------------------------------------
    enum WriteDisplayMode : uint8_t {

        SSD1306_MODE_CMD      = 0,
        SSD1306_MODE_RAM      = 1,
        SSD1306_MODE_RAM_BUF  = 2
    };

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
    // Each controller is described by a device type structure. The structure contains the width and height
    // as well as a command list of commands to issue when the display is initialized.
    //
    //--------------------------------------------------------------------------------------------------------
    struct DevType {
 
        const uint8_t *initCmdList;
        uint8_t       initSizeBytes;
        uint8_t       lcdWidthPixels;
        uint8_t       lcdHeightPixels;
        uint8_t       colOffset;
    };

    //--------------------------------------------------------------------------------------------------------
    // Initialization commands for a 128x32 SSD1306 oled display. This section is based on 
    // https://github.com/adafruit/Adafruit_SSD1306
    // 
    //--------------------------------------------------------------------------------------------------------
    constexpr uint8_t Adafruit128x32init[ ] = {

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

    constexpr DevType  Adafruit128x32 = {
        
        Adafruit128x32init,
        sizeof(Adafruit128x32init),
        128,
        32,
        0
    };

    //--------------------------------------------------------------------------------------------------------
    // Initialization commands for a 128x64 SSD1306 oled display. This section is based on 
    // https://github.com/adafruit/Adafruit_SSD1306
    // 
    //--------------------------------------------------------------------------------------------------------
    const uint8_t Adafruit128x64init[] = {
    
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

    constexpr DevType Adafruit128x64 = {
  
        Adafruit128x64init,
        sizeof(Adafruit128x64init),
        128,
        64,
        0
    };

    //--------------------------------------------------------------------------------------------------------
    // Initialization commands for a 128x64 SH1106 oled display. This section is based on 
    // https://github.com/stanleyhuangyc/MultiLCD. The SH1106 is a 132x64 controller. We use the middle 128
    // columns.
    // 
    //--------------------------------------------------------------------------------------------------------
    const uint8_t SH1106_128x64init[] = {
  
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

    constexpr DevType SH1106_128x64 =  {
  
        SH1106_128x64init,
        sizeof(SH1106_128x64init),
        128,
        64,
        2   
    };

    //--------------------------------------------------------------------------------------------------------
    // "setupHw" sets up our IO pins and the I2C channel. If the display has a reset input we also initialize
    // the reset IO and issue the reset sequence.
    //
    //--------------------------------------------------------------------------------------------------------
    uint8_t setupHw( uint8_t sclPin, uint8_t sdaPin, uint8_t rstPin ) {

        uint8_t rStat = CDC::NO_ERR;

        rStat = CDC::configureI2C( sclPin, sdaPin );
        if ( rStat != CDC::NO_ERR ) return( rStat );

        if ( rstPin != CDC::UNDEFINED_PIN ) {

            rStat = CDC::configureDio( rstPin, CDC::CDC_DIO_OUT );
            if ( rStat != CDC::NO_ERR ) return( rStat );

            CDC::writeDio( rstPin, false );
            CDC::sleepMillis( 10 );
            CDC::writeDio( rstPin, true );
            CDC::sleepMillis( 10 );
        }

        if ( rStat != CDC::NO_ERR ) printf( "setupHw Error: %d\n", rStat );

        return( rStat );
    }

    //--------------------------------------------------------------------------------------------------------
    //
    //
    //--------------------------------------------------------------------------------------------------------
    static const uint8_t scaledNibble[ ] = {  0X00, 0X03, 0X0C, 0X0F, 0X30, 0X33, 0X3C, 0X3F,
                                              0XC0, 0XC3, 0XCC, 0XCF, 0XF0, 0XF3, 0XFC, 0XFF };

}; // namespace


//------------------------------------------------------------------------------------------------------------
// Object constructor. Nothing to do here.
//
//------------------------------------------------------------------------------------------------------------
LcsOledDisplay::LcsOledDisplay( ) {  }


//------------------------------------------------------------------------------------------------------------
// The "begin" routine is the first method to call. It will configure the IO pins and setup the particular 
// OLED display.
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
    #if 1
    printf( "Oled Display begin: sclPin: %d, sdaPin: %d, i2cAdr: 0x%x, rstPin: %d\n ", 
            sclPin, sdaPin, i2cAdr, rstPin );
    #endif

    uint8_t rStat = setupHw( sclPin, sdaPin, rstPin );
    if ( rStat != CDC::NO_ERR ) return( rStat );

    setupDevType( devType );
    clear( );
    
    return( CDC::NO_ERR );
}

//------------------------------------------------------------------------------------------------------------
// "setupDevType" will initialize the OLED display. Currently, there are three different displays supported. 
// The Adafruit displays with a dimension for 128x64 and 128x32 use the SSD1306 controller. There is also a 
// 1.3" OLED display which uses the SH1106 controller type. The service type descriptor contains the init
// command sequence which is sent command by command.
//
//------------------------------------------------------------------------------------------------------------
void LcsOledDisplay::setupDevType( uint8_t dType ) {

    const DevType *dev = nullptr;

    switch ( dType ) {

        case ODT_OLED_DISPLAY_128x32_SSD1306:   dev = &Adafruit128x32;   break;
        case ODT_OLED_DISPLAY_128x64_SSD1306:   dev = &Adafruit128x64;   break;
        case ODT_OLED_DISPLAY_128x64_SH1106:    dev = &SH1106_128x64;    break;
        default: dev = &Adafruit128x64;
    }

    m_col           = 0;
    m_row           = 0;
    m_displayWidth  = dev -> lcdWidthPixels;
    m_displayHeight = dev -> lcdHeightPixels;
    m_colOffset     = dev -> colOffset;

    const uint8_t *table  = dev -> initCmdList;
    uint8_t       size    = dev -> initSizeBytes;
    
    for ( uint8_t i = 0; i < size; i++ ) ssd1306WriteCmd( table[ i ] );
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

void LcsOledDisplay::invertDisplay(bool invert) {

    ssd1306WriteCmd(invert ? SSD1306_INVERTDISPLAY : SSD1306_NORMALDISPLAY);
}

void LcsOledDisplay::clear() {

    clearRegion(0, displayWidthPixels() - 1, 0, displayRows() - 1);
}

void LcsOledDisplay::clearRegion( uint8_t c0, uint8_t c1, uint8_t r0, uint8_t r1 ) {

    // Cancel skip character pixels.
    m_skip = 0;

    // Insure only rows on display will be cleared.
    if (r1 >= displayRows()) r1 = displayRows() - 1;

    for (uint8_t r = r0; r <= r1; r++) {
        setCursor(c0, r);
        for (uint8_t c = c0; c <= c1; c++) {
        // Insure clear() writes zero. result is (m_invertMask^m_invertMask).
        ssd1306WriteRamBuffered( m_invertMask );
        }
    }
    setCursor(c0, r0);
}

void LcsOledDisplay::clearToEOL() {

    clearRegion(m_col, displayWidthPixels() - 1, m_row, m_row + fontRows() - 1);
}

void LcsOledDisplay::clearField(uint8_t col, uint8_t row, uint8_t n) {
  
    clearRegion(col, col + fieldWidthPixels(n) - 1, row, row + fontRows() - 1);
}

void LcsOledDisplay::displayRemap180Degrees( bool mode ) {

    ssd1306WriteCmd(mode ? SSD1306_SEGREMAP : SSD1306_SEGREMAP | 1);
    ssd1306WriteCmd(mode ? SSD1306_COMSCANINC : SSD1306_COMSCANDEC);
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t LcsOledDisplay::charWidthPixels(uint8_t c) const {

    if (!m_font) {
        return 0;
    }
    uint8_t first = m_font[ FONT_FIRST_CHAR ];
    uint8_t count = m_font[ FONT_CHAR_COUNT ];
    if (c < first || c >= (first + count)) {
        return 0;
    }
    if (fontSize() > 1) {
        // Proportional font.
        return m_magFactor * m_font[ FONT_WIDTH_TABLE + c - first ];
    }
    // Fixed width font.
    return m_magFactor * m_font[ FONT_WIDTH ];
}

uint8_t LcsOledDisplay::charSpacingPixels( uint8_t c ) { 
  
  return charWidthPixels( c ) + letterSpacingPixels( ); 
}

size_t LcsOledDisplay::fieldWidthPixels( uint8_t n ) const {

  return n * ( fontWidthPixels( ) + letterSpacingPixels( ));
}

size_t LcsOledDisplay::strWidthPixels(const char* str) const {

  size_t sw = 0;
  while (*str) {
    uint8_t cw = charWidthPixels(*str++);
    if (cw == 0) {
      return 0;
    }
    sw += cw + letterSpacingPixels( );
  }
  return sw;
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t LcsOledDisplay::fontCharCount( ) const {

  return m_font ? m_font[ FONT_CHAR_COUNT ] : 0;
}

char LcsOledDisplay::fontFirstChar( ) const {

  return m_font ? m_font[ FONT_FIRST_CHAR ] : 0;
}

uint8_t LcsOledDisplay::fontHeightPixels( ) const {

  return ( m_font ? m_magFactor * m_font[ FONT_HEIGHT ] : 0 );
}

uint8_t LcsOledDisplay::fontRows( ) const {

  return ( m_font ? m_magFactor * (( m_font[ FONT_HEIGHT ] + 7 ) / 8 ) : 0 );
}

uint16_t LcsOledDisplay::fontSize( ) const {

  return ( m_font[ 0 ] << 8 ) | m_font[ 1 ];
}

uint8_t LcsOledDisplay::fontWidthPixels( ) const {

  return m_font ? m_magFactor * m_font[ FONT_WIDTH ] : 0;
}

const uint8_t* LcsOledDisplay::font( ) const { 
  
  return m_font; 
}

void LcsOledDisplay::setFont( const uint8_t* font ) {

  m_font = font;

  if ( font && fontSize( ) == 1 ) m_letterSpacing = 0;
  else                            m_letterSpacing = 1;
}

void LcsOledDisplay::setFont( uint8_t fontId ) {

  switch( fontId ) {

    case FID_5x7:     setFont( Adafruit5x7 );     break;
    case FID_8x8:     setFont( font8x8 );         break;
    case FID_8x16:    setFont( ZevvPeep8x16 );    break;
    case FID_10x16:   setFont( TimesNewRoman16 ); break;
    default:          setFont( font8x8 );
  }
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t LcsOledDisplay::displayHeightPixels() { return m_displayHeight; }
 
uint8_t LcsOledDisplay::displayRows() { return m_displayHeight / 8; }
  
uint8_t LcsOledDisplay::displayWidthPixels() { return m_displayWidth; }

uint8_t LcsOledDisplay::col( ) const { return m_col; }

bool LcsOledDisplay::invertMode() const { return !!m_invertMask; }

void LcsOledDisplay::setInvertMode( bool mode ) { m_invertMask = mode ? 0XFF : 0; }

void LcsOledDisplay::setContrast(uint8_t value) {

  ssd1306WriteCmd(SSD1306_SETCONTRAST);
  ssd1306WriteCmd(value);
}

void LcsOledDisplay::setCursor( uint8_t col, uint8_t row ) {

  setColPixels( col );
  setRow(row);
}

void LcsOledDisplay::setRow( uint8_t row ) {

  if ( row < displayRows( )) {
    
    m_row = row;
    ssd1306WriteCmd( SSD1306_SETSTARTPAGE | m_row );
  }
}

void LcsOledDisplay::setColPixels( uint8_t col ) {

  if ( col < m_displayWidth ) {

    m_col =   col;
    col   +=  m_colOffset;

    ssd1306WriteCmd( SSD1306_SETLOWCOLUMN | (col & 0XF));
    ssd1306WriteCmd( SSD1306_SETHIGHCOLUMN | (col >> 4));
  }
}

//------------------------------------------------------------------------------------------------------------
//
//
// ??? try to truly understand all this ...
//------------------------------------------------------------------------------------------------------------
size_t LcsOledDisplay::writeChar( uint8_t ch ) {

    if ( !m_font ) {

        return 0;
    }

    uint8_t         w     = m_font[ FONT_WIDTH ];
    uint8_t         h     = m_font[ FONT_HEIGHT ];
    uint8_t         nr    = (h + 7) / 8;
    uint8_t         first = m_font[ FONT_FIRST_CHAR ];
    uint8_t         count = m_font[ FONT_CHAR_COUNT ];
    const uint8_t   *base = m_font + FONT_WIDTH_TABLE;

    if (ch == '\r') {

        setColPixels( 0 );
        return 1;
    }

    if (ch == '\n') {

        setColPixels( 0 );
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

    uint8_t s = letterSpacingPixels( );
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

        index += base[ i ];
        }

        w = base[ ch ];
        base += nr * index + count;
    }

    uint8_t scol = m_col;
    uint8_t srow = m_row;
    uint8_t skip = m_skip;
    for ( uint8_t r = 0; r < nr; r++ ) {
        for ( uint8_t m = 0; m < m_magFactor; m++ ) {

            skipColumnsPixels(skip);
            if (r || m) {
                setCursor(scol, m_row + 1);
            }
            for (uint8_t c = 0; c < w; c++) {

                uint8_t b = nfSpace ? 0 : base[ c + r * w ];
            
                if (thieleShift && (r + 1) == nr) {
                b >>= thieleShift;
                }
                if (m_magFactor == 2) {
                b = m ? b >> 4 : b & 0XF;
                b = scaledNibble[ b ];
                ssd1306WriteRamBuffered(b);
                }
                ssd1306WriteRamBuffered(b);
            }
            for (uint8_t i = 0; i < s; i++) {
                ssd1306WriteRamBuffered(0);
            }
        }
    } 

    setRow(srow);
    return 1;
}

void LcsOledDisplay::ssd1306WriteCmd( uint8_t cmdByte ) { 
  
  writeDisplay( cmdByte, SSD1306_MODE_CMD ); 
}

void LcsOledDisplay::ssd1306WriteRamImmediate(uint8_t c) {

  if (m_col < m_displayWidth) {
    writeDisplay(c ^ m_invertMask, SSD1306_MODE_RAM );
    m_col++;
  }
}

void LcsOledDisplay::ssd1306WriteRamBuffered( uint8_t c ) {

    if ( m_skip ) {

        m_skip--;
    } 
    else if (m_col < m_displayWidth) {
    
        writeDisplay( c ^ m_invertMask, SSD1306_MODE_RAM_BUF );
        m_col++;
    }
}

void LcsOledDisplay::writeDisplay( uint8_t b, uint8_t mode ) {

    uint8_t buf[ 2 ];
    buf[ 0 ] = ( mode == SSD1306_MODE_CMD ) ? 0X00 : 0X40;
    buf[ 1 ] = b;

   uint8_t rStat = CDC::i2cWrite( sclPin, i2cAdr, buf, 2 );

    // ??? into a debug bracket ?
   if ( rStat != CDC::NO_ERR ) printf( "Error in writing to display: %d\n", rStat );
}
