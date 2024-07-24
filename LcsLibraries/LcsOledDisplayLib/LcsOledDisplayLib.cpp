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
    
    //--------------------------------------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------------------------------------


}; // namespace


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
LcsOledDisplay::LcsOledDisplay( ) { }


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t LcsOledDisplay::begin(  const DevType*  dev, 
                                uint8_t         sclPin, 
                                uint8_t         sdaPin, 
                                uint8_t         i2cAdr,
                                uint8_t         rstPin  ) {

    this -> sclPin = sclPin;
    this -> sdaPin = sdaPin;
    this -> i2cAdr = i2cAdr;
    this -> rstPin = rstPin;

    uint8_t rStat;

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

    init( dev );
    
    return( CDC::ALL_OK );
}

/*
void LcsOledDisplay::display( ) {

}

void LcsOledDisplay::noDisplay( ) {

}

void LcsOledDisplay::setCursor( uint8_t col, uint8_t row ) {

}

void LcsOledDisplay::setFont( uint8_t fontId ) {

}

uint8_t LcsOledDisplay::print( const char *s ) {

}

uint8_t LcsOledDisplay::print( char ch ) {

}

void LcsOledDisplay::clear( ) {

}

void LcsOledDisplay::clearLine( uint8_t row ) {


}
*/


//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
void LcsOledDisplay::writeDisplay( uint8_t b, uint8_t mode ) {

    uint8_t buf[ 2 ];
    buf[ 0 ] = ( mode == SSD1306_MODE_CMD ) ? 0X00 : 0X40;
    buf[ 1 ] = b;

    uint8_t rStat = CDC::i2cWrite( sclPin, i2cAdr, &b, 2 );     
    
    /*
        m_oledWire.beginTransmission(m_i2cAddr);
        m_oledWire.write(mode == SSD1306_MODE_CMD ? 0X00 : 0X40);
        m_oledWire.write(b);
        m_oledWire.endTransmission();
    */

}

  
