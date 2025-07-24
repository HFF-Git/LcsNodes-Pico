///---------------------------------------------------------------------------------------
//
// LCS - LCD Display Driver - Include file
//
///---------------------------------------------------------------------------------------
// 
//
///---------------------------------------------------------------------------------------
//
//  ... mention where it came from ...
//
//
// LCS - LCD Display Driver
// Copyright (C) 2024 - 2024  Helmut Fieres
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
#ifndef LcsLcdDisplayLib_h
#define LcsLcdDisplayLib_h

#include "LcsCdcLib.h"

class LcsLcdDisplay final {

    public:

    LcsLcdDisplay ( uint8_t columns, 
                    uint8_t rows,
                    uint8_t sclPin,
                    uint8_t sdaPin,
                    uint8_t i2cAdr ) noexcept;

    void displayOn( ) noexcept;
    void displayOff( ) noexcept;

    void backlightOn( ) noexcept;
    void backlightOff( ) noexcept;

    void cursorOn( ) noexcept;
    void cursorOff( ) noexcept;

    void cursorBlinkOn( ) noexcept;
    void cursorBlinkOff( ) noexcept;

    void setTextLeftToRight( ) noexcept;
    void setTextRightToLeft( ) noexcept;

    void clear( ) noexcept;
    void home( ) noexcept;

    void setCursor( uint8_t row, uint8_t column ) noexcept;

    void printChar( char ch ) noexcept;
    void printString( char *str ) noexcept;
    
    void createCustomChar( uint8_t location, uint8_t *char_map ) noexcept;
    void printCustomChar( uint8_t location ) noexcept;

 private:

    void i2cWriteByte( uint8_t val ) noexcept;
    void pulseEnable( uint8_t val ) noexcept;
    void sendNibble(uint8_t val) noexcept;
    void sendByte(uint8_t val, uint8_t mode) noexcept;
    void sendCommand( uint8_t val ) noexcept;
    void sendChar(uint8_t val) noexcept;
    void sendRegisterSelect( uint8_t val ) noexcept;


    uint8_t address;
    uint8_t columns;
    uint8_t rows;
    uint8_t backLight;
    uint8_t displayFunction;
    uint8_t displayControl;
    uint8_t displayMode;

};

#endif