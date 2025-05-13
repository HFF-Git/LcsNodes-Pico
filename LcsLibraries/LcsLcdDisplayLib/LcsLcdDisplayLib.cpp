//------------------------------------------------------------------------------------------------------------
//
// LCS - LCD Display Driver - PICO implementation
//
//------------------------------------------------------------------------------------------------------------
// This source file contains ...
//
//------------------------------------------------------------------------------------------------------------
//
// ... mention whee it came from ...
//
//
// LCS - LCD Display Driver
// Copyright (C) 2024- 2025  Helmut Fieres
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
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

#include "LcsLcdDisplayLib.h"

//------------------------------------------------------------------------------------------------------------
// Local name space. This file has two sections. The first is this local name space with all internal
// variables and routines local to the file. The second part contains the exported routines to be called by
// the core library and the firmware designers.
//
//------------------------------------------------------------------------------------------------------------
namespace {

//------------------------------------------------------------------------------------------------------------
// Commands.
//
//------------------------------------------------------------------------------------------------------------
constexpr uint8_t CLEAR_DISPLAY = 0x01;
constexpr uint8_t RETURN_HOME = 0x02;
constexpr uint8_t ENTRY_MODE_SET = 0x04;
constexpr uint8_t DISPLAY_CONTROL = 0x08;
constexpr uint8_t CURSOR_SHIFT = 0x10;
constexpr uint8_t FUNCTION_SET = 0x20;
constexpr uint8_t SET_CGRAM_ADDR = 0x40;
constexpr uint8_t SET_DDRAM_ADDR = 0x80;

//------------------------------------------------------------------------------------------------------------
// Flags for display entry mode set.
//
//------------------------------------------------------------------------------------------------------------
constexpr uint8_t ENTRY_RIGHT = 0x00;
constexpr uint8_t ENTRY_LEFT = 0x02;
constexpr uint8_t ENTRY_SHIFT_INCREMENT = 0x01;
constexpr uint8_t ENTRY_SHIFT_DECREMENT = 0x00;

//------------------------------------------------------------------------------------------------------------
// Flags for display on/off control.
//
//------------------------------------------------------------------------------------------------------------
constexpr uint8_t DISPLAY_ON = 0x04;
constexpr uint8_t DISPLAY_OFF = 0x00;
constexpr uint8_t CURSOR_ON = 0x02;
constexpr uint8_t CURSOR_OFF = 0x00;
constexpr uint8_t BLINK_ON = 0x01;
constexpr uint8_t BLINK_OFF = 0x00;

//------------------------------------------------------------------------------------------------------------
// Flags for cursor or display shift.
//
//------------------------------------------------------------------------------------------------------------
constexpr uint8_t DISPLAY_MOVE = 0x08;
constexpr uint8_t CURSOR_MOVE = 0x00;
constexpr uint8_t MOVE_RIGHT = 0x04;
constexpr uint8_t MOVE_LEFT = 0x00;

//------------------------------------------------------------------------------------------------------------
// Flags for function set.
//
//------------------------------------------------------------------------------------------------------------
constexpr uint8_t MODE_8_BIT = 0x10;
constexpr uint8_t MODE_4_BIT = 0x00;
constexpr uint8_t LINE_2 = 0x08;
constexpr uint8_t LINE_1 = 0x00;
constexpr uint8_t DOTS_5x10 = 0x04;
constexpr uint8_t DOTS_5x8 = 0x00;

//------------------------------------------------------------------------------------------------------------
// Flags for backlight control.
//
//------------------------------------------------------------------------------------------------------------
constexpr uint8_t BACKLIGHT = 0x08;
constexpr uint8_t NO_BACKLIGHT = 0x00;

//------------------------------------------------------------------------------------------------------------
// Special flags.
//
//------------------------------------------------------------------------------------------------------------
constexpr uint8_t ENABLE = 0x04;
constexpr uint8_t READ_WRITE = 0x02;
constexpr uint8_t REGISTER_SELECT = 0x01;
constexpr uint8_t COMMAND = 0x00;
constexpr uint8_t CHAR = 0x01;

constexpr uint8_t MAX_CUSTOM_CHARS = 8;

}; // namespace

//------------------------------------------------------------------------------------------------------------
//
// ??? we need to keep the data in private variables...
// ??? use resource number scheme for CDC
//------------------------------------------------------------------------------------------------------------
LcsLcdDisplay::LcsLcdDisplay (  uint8_t columns, 
                                uint8_t rows,
                                uint8_t sclPin,
                                uint8_t sdaPin,
                                uint8_t i2cAdr ) noexcept {


                    }

void  LcsLcdDisplay::i2cWriteByte( uint8_t val ) noexcept {
    
  static uint8_t data;

  data = val | backLight;

  // ??? fix how to get there ...
 //  i2c_write_blocking( I2C_instance, address, &data, 1, false);

}

void  LcsLcdDisplay::pulseEnable( uint8_t val ) noexcept {

    static constexpr uint16_t DELAY = 600;

    CDC::sleepMicros(DELAY);
    i2cWriteByte(val | ENABLE);
    CDC::sleepMicros(DELAY);
    i2cWriteByte(val & ~ENABLE);
    CDC::sleepMicros(DELAY);
}

void  LcsLcdDisplay::sendNibble(uint8_t val) noexcept {

    i2cWriteByte(val);
    pulseEnable(val);
}

void  LcsLcdDisplay::sendByte(uint8_t val, uint8_t mode) noexcept {

    static constexpr uint8_t UPPER_NIBBLE = 0B1111'0000;

    static uint8_t high;
    static uint8_t low;

    high = val & UPPER_NIBBLE;
    low = (val << 4) & UPPER_NIBBLE;

    sendNibble(high | mode);
    sendNibble(low | mode);
}

void  LcsLcdDisplay::sendCommand( uint8_t val ) noexcept {

    sendByte(val, COMMAND);
}

void  LcsLcdDisplay::sendChar(uint8_t val) noexcept {
    
    sendByte(val, CHAR);
}

void  LcsLcdDisplay::sendRegisterSelect( uint8_t val ) noexcept {

    sendByte(val, REGISTER_SELECT);
}

void LcsLcdDisplay::displayOn( ) noexcept {

    displayControl |= DISPLAY_ON;
    sendCommand( DISPLAY_CONTROL | displayControl );
}

void LcsLcdDisplay::displayOff() noexcept {

    displayControl &= ~DISPLAY_ON;
    sendCommand(DISPLAY_CONTROL | displayControl);
}

void LcsLcdDisplay::backlightOn() noexcept {

    backLight = BACKLIGHT;
    i2cWriteByte( backLight );
}

void LcsLcdDisplay::backlightOff() noexcept {

    backLight = NO_BACKLIGHT;
    i2cWriteByte( backLight );
}

void LcsLcdDisplay::cursorOn() noexcept {

    displayControl |= CURSOR_ON;
    sendCommand( DISPLAY_CONTROL | displayControl);
}

void LcsLcdDisplay::cursorOff() noexcept {

    displayControl &= ~CURSOR_ON;
    sendCommand(DISPLAY_CONTROL | displayControl);
}

void LcsLcdDisplay::cursorBlinkOn() noexcept {

    displayControl |= BLINK_ON;
    sendCommand( DISPLAY_CONTROL | displayControl );
}

void LcsLcdDisplay::cursorBlinkOff() noexcept {
    displayControl &= ~BLINK_ON;
    sendCommand(DISPLAY_CONTROL | displayControl);
}

void LcsLcdDisplay::setTextLeftToRight() noexcept {

    displayMode |= ENTRY_LEFT;
    sendCommand(ENTRY_MODE_SET | displayMode);
}

void LcsLcdDisplay::setTextRightToLeft() noexcept {

    displayMode &= ~ENTRY_LEFT;
    sendCommand(ENTRY_MODE_SET | displayMode);
}

void LcsLcdDisplay::clear( ) noexcept {

    sendCommand(CLEAR_DISPLAY);
}

void LcsLcdDisplay::home( ) noexcept {

    sendCommand(RETURN_HOME);
}

void LcsLcdDisplay::setCursor( uint8_t row, uint8_t column) noexcept {

  // ??? check this out ... the famous 4 column issue ?
  /*
    static const std::array ROW_OFFSETS = {0x00, 0x40, 0x00 + columns, 0x40 + columns };

    row = std::min( rows, row );
    column = std::min( columns, column );
    sendCommand( SET_DDRAM_ADDR | ( ROW_OFFSETS.at(row) + column ));
  */

}

void LcsLcdDisplay::printChar( char ch ) noexcept {
    
    sendChar( ch );
}

void LcsLcdDisplay::printString( char *str ) noexcept {

  //  ??? for (char const CHARACTER: str) printChar(CHARACTER);
}

void LcsLcdDisplay::printCustomChar( uint8_t location ) noexcept {

    sendRegisterSelect( location );
}

void LcsLcdDisplay::createCustomChar( uint8_t location, uint8_t *char_map ) noexcept
{
    
  /* 
    location = std::min(MAX_CUSTOM_CHARS, location);
    sendCommand(SET_CGRAM_ADDR | (location << 3));
    for (size_t i = 0; i < CUSTOM_SYMBOL_SIZE; ++i)
    {
        Send_Register_Select(char_map.at(i));
    }
  */
}
  
