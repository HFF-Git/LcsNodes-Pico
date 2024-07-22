//------------------------------------------------------------------------------------------------------------
//
// UIDisplayElements - implementation file.
//
//------------------------------------------------------------------------------------------------------------
//
//
//
//------------------------------------------------------------------------------------------------------------
//
// UIDisplayElements
// Copyright (C) 2019 - 2023  Helmut Fieres
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
#include "LcsUIElements.h"
#include "LcsCdcLib.h"

#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"


//------------------------------------------------------------------------------------------------------------
// Local declarations.
//
//------------------------------------------------------------------------------------------------------------
namespace {

  struct {

    uint8_t       fontId;
    const uint8_t *font;

  } FontTab[ ] = {

    { FT_DEF,   font8x8         },
    { FT_5x7,   font5x7         },
    { FT_8x8,   font8x8,        },
    { FT_8x16,  ZevvPeep8x16    },
    { FT_10x16, TimesNewRoman16   }

  };

  // perhaps add some more....
  /*
    const uint8_t* fontList[] = {
      Arial14,
      Arial_bold_14,
      Callibri11,
      Callibri11_bold,
      Callibri11_italic,
      Callibri15,
      Corsiva_12,
      fixed_bold10x15,
      Iain5x7,
      lcd5x7,
      Stang5x7,
      System5x7,
      TimesNewRoman16,
      TimesNewRoman16_bold,
      TimesNewRoman16_italic,
      utf8font10x16,
      Verdana12,
      Verdana12_bold,
      Verdana12_italic,
      X11fixed7x14,
      X11fixed7x14B,
    };
  */

} // namespace


//============================================================================================================
// UIDisplay Section.
//============================================================================================================


//------------------------------------------------------------------------------------------------------------
// The base class constructor. A display features a row x column matrix for ASCII display. The maximum matrix
// size is set from the display type passed.
//
//------------------------------------------------------------------------------------------------------------
UIDisplay::UIDisplay( uint8_t dType ) {

  switch ( dType ) {

    case DT_LCD_DISPLAY_16_2: {

        maxColumns  = 16;
        maxRows     = 2;

      } break;

    case DT_LCD_DISPLAY_20_4: {

        maxColumns  = 20;
        maxRows     = 4;

      } break;

    case DT_OLED_DISPLAY_128x32_16_4: {

        maxColumns  = 16;
        maxRows     = 4;

      } break;

    case DT_OLED_DISPLAY_128x32_8_2: {

        maxColumns  = 8;
        maxRows     = 2;

      } break;

    case DT_OLED_DISPLAY_128x64_16_8: {

        maxColumns  = 16;
        maxRows     = 8;

      } break;

    case DT_OLED_DISPLAY_128x64_16_4: {

        maxColumns  = 8;
        maxRows     = 4;

      } break;

    case DT_OLED_DISPLAY_128x64_2F_4: {

        maxColumns  = 16;
        maxRows     = 8;

      } break;

    default: {

        maxColumns  = 16;
        maxRows     = 2;
      }
  }
}

//------------------------------------------------------------------------------------------------------------
// Each UIElement has a function to process period work. So far, for displays, there is nothing to do. But
// perhaps one day for example, we implement blinking characters on an Oled or so. Until then ...
//
//------------------------------------------------------------------------------------------------------------
void  UIDisplay::processTick( ) { }

#if 0

//============================================================================================================
// UIDisplayI2C Section.
//============================================================================================================


// ??? currently disabled. The Arduino used to have a library already integrated... to do ...

// ??? we will just call outr functions in the LcsLcdDisplay object .... 


//------------------------------------------------------------------------------------------------------------
// LCD with an I2C interface.  Most of the methods to control the LCS display are just inherited from the
// LiquidCrystal_I2C class. The "setCursor" method is overriden and checks the columns and row offsets first.
//
//------------------------------------------------------------------------------------------------------------
UIDisplayLcdI2C::UIDisplayLcdI2C( uint8_t dType, uint8_t sclPin, uint8_t sdaPin, uint8_t I2CAddress ) :

  UIDisplay( dType ), LiquidCrystal_I2C( I2CAddress, maxColumns, maxRows ) {

  // ??? check pins ? do we need pins when we have an I2C interface on other pins ?
  // ??? use CDC routines...
}

void UIDisplayLcdI2C::setCursor( uint8_t col, uint8_t row ) {

  LiquidCrystal_I2C::setCursor((( col > maxColumns ) ? maxColumns : col ),
                               (( row > maxColumns ) ? maxRows : row ));
}

uint8_t UIDisplayLcdI2C::print( const char *buf ) {

  return ( LiquidCrystal_I2C::print( buf ));
}

uint8_t UIDisplayLcdI2C::print( char ch ) {

  return ( UIDisplayLcdI2C::print( ch ));
}

void UIDisplayLcdI2C::clear( ) {

  LiquidCrystal_I2C::setCursor( 0, 0 );
  LiquidCrystal_I2C::clear( );
}
 #endif

//============================================================================================================
// UIDisplayOledSSD1306 Section.
//============================================================================================================

// ??? this should become more generic ?
// ??? UIDisplayOled ???

//------------------------------------------------------------------------------------------------------------
// Oled Version using the SSD1306 controller chip. There is no nice mapping of display function via base
// class inheritance, as used in the LiquidCrystal displays. We need to create the OLed display object and
// implement each generic display function if possible. The Oled Display Class implements three methods.
// "setCursor" sets the cursor to the desifed row and columns. These values are however depending on the
// current font. The colum, measured in pixels, is computed to be the column parameter times the font width
// of the current font. The display row and column parameter need to be multiplied with the dimensions
// needed for the current font measured in multiple of 8 pixels. The "print" and "clear" methods just pass
// through to their specific Oled Display Class counterparts.
//
// ??? watch out what display HW you really have ... it may otherwise not work...
//------------------------------------------------------------------------------------------------------------
UIDisplayOledSSD1306::UIDisplayOledSSD1306(
  uint8_t dType, uint8_t sclPin, uint8_t sdaPin, uint8_t I2cAddress ) : UIDisplay( dType ) {

    uint8_t rStat = CDC::configureI2C( sclPin, sdaPin );

 // Wire.begin();
 // Wire.setClock(400000L);

  oled = new SSD1306AsciiWire( );
  (( SSD1306AsciiWire * ) oled ) -> begin( &Adafruit128x64, I2cAddress );

  switch ( dType ) {

    case DT_OLED_DISPLAY_128x32_16_4: oled -> setFont( FontTab[ FT_8x8 ].font );    break;
    case DT_OLED_DISPLAY_128x32_8_2:  oled -> setFont( FontTab[ FT_8x16 ].font );   break;
    case DT_OLED_DISPLAY_128x64_16_8: oled -> setFont( FontTab[ FT_8x8 ].font );    break;
    case DT_OLED_DISPLAY_128x64_16_4: oled -> setFont( FontTab[ FT_8x16 ].font );   break;
    case DT_OLED_DISPLAY_128x64_2F_4: oled -> setFont( FontTab[ FT_8x8 ].font );    break;
    default:                          oled -> setFont( FontTab[ FT_DEF ].font );
  }
}

void UIDisplayOledSSD1306::setCursor( uint8_t col, uint8_t row ) {

  uint8_t lCol = (( col > maxColumns ) ? maxColumns : col ) * oled -> fontWidth( );
  uint8_t lRow = row;
  uint8_t fRow = oled -> fontRows( );

  if ( fRow - 1 > row ) lRow = fRow - 1;
  if ( row > maxRows ) lRow = maxRows;

  oled -> setCursor( lCol, lRow );
}

void UIDisplayOledSSD1306::setFont( uint8_t fontId ) {

  switch ( fontId ) {

    case FT_5x7:  oled -> setFont( FontTab[ FT_5x7 ].font );  break;
    case FT_8x8:  oled -> setFont( FontTab[ FT_8x8 ].font );  break;
    case FT_8x16: oled -> setFont( FontTab[ FT_8x16 ].font ); break;
    default: oled -> setFont( FontTab[ FT_DEF ].font );
  }
}

uint8_t UIDisplayOledSSD1306::print( const char *buf ) {

  return ( oled -> print( buf ));
}

uint8_t UIDisplayOledSSD1306::print( char ch ) {

  return ( oled -> print( ch ));
}

void UIDisplayOledSSD1306::clear( ) {

  oled -> setCursor( 0, 0 );
  oled -> clear( );
}

void UIDisplayOledSSD1306::clearLine( uint8_t row ) {

  setCursor( 0, row );
  for ( int i = 0; i < maxColumns; i++ ) oled -> print((char *) " " );
}
