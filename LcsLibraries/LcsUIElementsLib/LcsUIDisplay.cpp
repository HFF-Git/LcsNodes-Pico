//----------------------------------------------------------------------------------------
//
// UIDisplayElements - implementation file.
//
//----------------------------------------------------------------------------------------
// The UI element library features a simple display object. It is a basic ASCII 
// matrix of rows and columns.The display classes are LCD and OLED. While the LCD
// is rather fixed with respect to columns and rows, the OLED class allows for 
// different fonts. For OLEDs the basic raster is 8x8 pixels.
//
//----------------------------------------------------------------------------------------
//
// UIDisplayElements
// Copyright (C) 2019 - 2025  Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details. You 
// should have received a copy of the GNU General Public License along with this 
// program. If not, see <http://www.gnu.org/licenses/>.
//
//----------------------------------------------------------------------------------------
#include "LcsUIElements.h"
#include "LcsCdcLib.h"
#include "LcsOledDisplayLib.h"

//----------------------------------------------------------------------------------------
// Local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

//----------------------------------------------------------------------------------------
// The font table for the OLED display.
//
//----------------------------------------------------------------------------------------
struct {

    uint8_t         fontId;
    const uint8_t   *font;

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


//========================================================================================
//
// UIDisplay Section.
//
//========================================================================================


//----------------------------------------------------------------------------------------
// The base class constructor. A display features a row x column matrix for ASCII display. 
// The maximum matrix size is set from the display type passed.
//
// ???? we think in 8x8 !!!!!!
//----------------------------------------------------------------------------------------
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

    case DT_OLED_DISPLAY_128x32: {

        maxColumns  = 16;
        maxRows     = 4;

      } break;

    case DT_OLED_DISPLAY_128x64: {

        maxColumns  = 16;
        maxRows     = 8;

      } break;

    default: {

        maxColumns  = 16;
        maxRows     = 2;
      }
  }
}

//----------------------------------------------------------------------------------------
// Each UIElement has a function to process period work. So far, for displays, there is
// nothing to do. But perhaps one day for example, we implement blinking characters on 
// an OLed or so. Until then ...
//
//----------------------------------------------------------------------------------------
void  UIDisplay::processTick( ) { }

//========================================================================================
//
// UIDisplayLcdI2C Section.
//
//========================================================================================

//----------------------------------------------------------------------------------------
// LCD with an I2C interface.
//
//----------------------------------------------------------------------------------------
UIDisplayLcdI2C::UIDisplayLcdI2C(   uint8_t dType, 
                                    uint8_t sclPin, 
                                    uint8_t sdaPin, 
                                    uint8_t I2CAddress ) : UIDisplay( dType ) {

  lcd = new LcsLcdDisplay ( maxColumns, maxRows, sclPin, sdaPin, I2CAddress );
}

void UIDisplayLcdI2C::displayOn( ) {

  lcd -> displayOn( );
}

void UIDisplayLcdI2C::displayOff( ) {

  lcd -> displayOff( );
}

void UIDisplayLcdI2C::setCursor( uint8_t col, uint8_t row ) {

  lcd -> setCursor((( col > maxColumns ) ? maxColumns : col ),
                    (( row > maxColumns ) ? maxRows : row ));
}

uint8_t UIDisplayLcdI2C::print( const char *buf ) {

    while ( *buf != 0 ) {

        lcd -> printChar( *buf );
        buf ++;
    }

  return ( 0 );
}

uint8_t UIDisplayLcdI2C::print( char ch ) {

  lcd -> printChar( ch );
  return( 0 );
}

void UIDisplayLcdI2C::clear( ) {

  lcd -> setCursor( 0, 0 );
  lcd -> clear( );
}

//========================================================================================
// UIDisplayOled Section.
//========================================================================================

//----------------------------------------------------------------------------------------
// Oled Version using the SSD1306 controller chip. There is no nice mapping of display 
// function via base class inheritance, as used in the LiquidCrystal displays. We need 
// to create the OLed display object and implement each generic display function if 
// possible. The Oled Display Class implements three methods. "setCursor" sets the cursor
// to the desired row and columns. These values are however depending on the current font.
// The column, measured in pixels, is computed to be the column parameter times the font 
// width of the current font. The display row and column parameter need to be multiplied
// with the dimensions needed for the current font measured in multiple of 8 pixels. The
// "print" and "clear" methods just pass through to their specific Oled Display Class 
// counterparts.
//
// ??? watch out what display HW you really have ... it may otherwise not work...
//----------------------------------------------------------------------------------------
UIDisplayOled::UIDisplayOled(   uint8_t dType, 
                                uint8_t rNumI2C, 
                                uint8_t i2cAdr, 
                                uint8_t rNumRST ) : UIDisplay( dType ) {

    oled = new LcsOledDisplay( );
    oled -> begin( ODT_OLED_DISPLAY_128x64_SSD1306, rNumI2C, i2cAdr, rNumRST );
   

    switch ( dType ) {

        case DT_OLED_DISPLAY_128x32: oled -> setFont( FontTab[ FT_8x8 ].font );    break;
        case DT_OLED_DISPLAY_128x64: oled -> setFont( FontTab[ FT_8x8 ].font );    break;
        default:                     oled -> setFont( FontTab[ FT_DEF ].font );
    }
}

void UIDisplayOled::displayOn( ) {

  oled -> displayOn( );
}

void UIDisplayOled::displayOff( ) {

  oled -> displayOff( );
}

void UIDisplayOled::setCursor( uint8_t col, uint8_t row ) {

    uint8_t lCol = (( col > maxColumns ) ? maxColumns : col ) * oled -> fontWidthPixels( );
    uint8_t lRow = row;
    uint8_t fRow = oled -> fontRows( );

    if ( fRow - 1 > row ) lRow = fRow - 1;
    if ( row > maxRows ) lRow = maxRows;

    oled -> setCursor( lCol, lRow );
}

void UIDisplayOled::setFont( uint8_t fontId ) {

  switch ( fontId ) {

    case FT_5x7:  oled -> setFont( FontTab[ FT_5x7 ].font );  break;
    case FT_8x8:  oled -> setFont( FontTab[ FT_8x8 ].font );  break;
    case FT_8x16: oled -> setFont( FontTab[ FT_8x16 ].font ); break;
    default: oled -> setFont( FontTab[ FT_DEF ].font );
  }
}

uint8_t UIDisplayOled::print( const char *buf ) {

    while ( *buf != 0 ) {

        oled -> writeChar( *buf );
        buf ++;
    }

  return ( 1 );
}

uint8_t UIDisplayOled::print( char ch ) {

  return ( oled -> writeChar( ch ));
}

void UIDisplayOled::clear( ) {

  oled -> setCursor( 0, 0 );
  oled -> clear( );
}

void UIDisplayOled::clearLine( uint8_t row ) {

  setCursor( 0, row );
  for ( int i = 0; i < maxColumns; i++ ) oled -> writeChar( ' ' );
}
