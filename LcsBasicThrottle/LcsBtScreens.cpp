//----------------------------------------------------------------------------------------
//
// LCS - Cab Handheld Cab Screens implementation file
//
//----------------------------------------------------------------------------------------
// This file contains the screen methods for the cab handheld screens. All cab handheld screens have a
// common screen layout. The screen is divided into a top line, which has room for a title and the labelling
// of the adjacent buttons MENU and UP. Likewise, there is a bottom line with room for some status flags
// and the label fields for SELECT and DOWN. In between are two lines in a larger font which are the screen
// content.
//
//
//                         0    2 3              12 13  15
//                        :------:-----------------:------:
//      Top Line    ->    : Menu :  Title          : Up   :
//                        :------:-----------------:------:
//      Line 1      ->    : Text                          :
//                        :-------------------------------:
//      Line 1      ->    : Text                          :
//                        :------:-----------------:------:
//      Top Line    ->    : Sel  :  Status Flags   : Down :
//                        :------:-----------------:------:
//
// There are methods to set these fields easy and straightforward. The top and bottom line are printed in an
// 8x8 font, the two main lines in a 8x16 font. Note that the display object expects rows and columns based
// on an 8-pixel raster. So, a 128x64 screen has 16 columns and 8 rows. A font that takes two rows starts at
// the lower row of the two rows it occupies.
//
//----------------------------------------------------------------------------------------
//
// LCS - Cab Handheld Cab Screens implementation file
// Copyright (C) 2019 - 2024  Helmut Fieres
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
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------
#include "LcsBasicThrottle.h"

using namespace LCS;

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
extern  UIDisplay   *oled;
extern  UIEncoder   *encoder;
extern  CabStack    *cabStack;
extern  CabMsgBus   *msgBus;

//----------------------------------------------------------------------------------------
// File local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

//----------------------------------------------------------------------------------------
// The screens use text fields with pre-assigned text content. These constant string tokens are grouped here
// and returned when needed. Perhaps one day, this allows also support for several languages. The table is
// not sorted right now, the lookup routine just does a linear search.
//
//----------------------------------------------------------------------------------------
enum ScreenTextTokens : uint8_t {

    SCR_TX_NIL                  = 0,
    SCR_TX_NEXT                 = 1,
    SCR_TX_HOME                 = 2,
    SCR_TX_UP                   = 3,
    SCR_TX_DOWN                 = 4,
    SCR_TX_ON                   = 5,
    SCR_TX_OFF                  = 6,
    SCR_TX_SEL                  = 7,
    SCR_TX_OK                   = 8,
    SCR_TX_YES                  = 9,
    SCR_TX_NO                   = 10,
    SCR_TX_ABORT                = 11,
    SCR_TX_SET                  = 12,
    SCR_TX_FUNC_MOM             = 13,
    SCR_TX_FUNC_TOGGLE          = 14,
    SCR_TX_FUNC_DISABLE         = 15,
    SCR_TX_FUNC_GROUP           = 16,
    SCR_TX_FRONT                = 18,
    SCR_TX_REAR                 = 19,
    SCR_TX_BRIGHT               = 20,
    SCR_TX_DIM                  = 21,

    SCR_TX_OPERATE              = 30,
    SCR_TX_ENGINE               = 31,
    SCR_TX_LIGHTS               = 32,
    SCR_TX_NEW_CAB              = 33,
    SCR_TX_SEL_CAB              = 34,
    SCR_TX_SEL_CAB_L            = 35,
    SCR_TX_SAVE_CAB             = 36,
    SCR_TX_LOAD_CAB             = 37,
    SCR_TX_SET_FUNC             = 38,
    SCR_TX_CONFIG_FUNC          = 39,
    SCR_TX_CONFIG_FUNC_L        = 40,
    SCR_TX_OPTIONS              = 41,

    SCR_TX_ERROR                = 50,
    SCR_TX_CONFIRM              = 51,
    SCR_TX_UI_TEST              = 52,
    SCR_TX_DIAG                 = 53,
};

//----------------------------------------------------------------------------------------
// Screen token text table and lookup function.
//
//----------------------------------------------------------------------------------------
struct {

    uint8_t   textId;
    char      *textStr; 

} const screenTextTokensTab[ ] = {

    { .textId = SCR_TX_NIL,           .textStr = (char *) ""              },
    { .textId = SCR_TX_HOME,          .textStr = (char *) "Home"          },
    { .textId = SCR_TX_NEXT,          .textStr = (char *) "Nxt"           },
    { .textId = SCR_TX_UP,            .textStr = (char *) "Up"            },
    { .textId = SCR_TX_DOWN,          .textStr = (char *) "Dn"            },
    { .textId = SCR_TX_ON,            .textStr = (char *) "On"            },
    { .textId = SCR_TX_OFF,           .textStr = (char *) "Off"           },
    { .textId = SCR_TX_SEL,           .textStr = (char *) "Sel"           },
    { .textId = SCR_TX_OK,            .textStr = (char *) "Ok"            },

    { .textId = SCR_TX_SET,           .textStr = (char *) "Set"           },
    { .textId = SCR_TX_FUNC_MOM,      .textStr = (char *) "MOM"           },
    { .textId = SCR_TX_FUNC_TOGGLE,   .textStr = (char *) "TOG"           },
    { .textId = SCR_TX_FUNC_DISABLE,  .textStr = (char *) "---"           },
    { .textId = SCR_TX_FUNC_GROUP,    .textStr = (char *) "Grp"           },
    { .textId = SCR_TX_LIGHTS,        .textStr = (char *) "Lights"        },
    { .textId = SCR_TX_FRONT,         .textStr = (char *) "Head"          },
    { .textId = SCR_TX_REAR,          .textStr = (char *) "Tail"          },

    { .textId = SCR_TX_OPERATE,       .textStr = (char *) "Operate"       },
    { .textId = SCR_TX_ENGINE,        .textStr = (char *) "Engine"        },
    { .textId = SCR_TX_NEW_CAB,       .textStr = (char *) "New Cab"       },
    { .textId = SCR_TX_SEL_CAB,       .textStr = (char *) "Sel Cab"       },
    { .textId = SCR_TX_SEL_CAB_L,     .textStr = (char *) "Select Cab"    },
    { .textId = SCR_TX_SAVE_CAB,      .textStr = (char *) "Save Cab"      },
    { .textId = SCR_TX_LOAD_CAB,      .textStr = (char *) "Load Cab"      },
    { .textId = SCR_TX_SET_FUNC,      .textStr = (char *) "Set Func"      },
    { .textId = SCR_TX_CONFIG_FUNC,   .textStr = (char *) "Cfg Func"      },
    { .textId = SCR_TX_CONFIG_FUNC_L, .textStr = (char *) "Config Func"   },
    { .textId = SCR_TX_OPTIONS,       .textStr = (char *) "Options"       },
    { .textId = SCR_TX_DIAG,          .textStr = (char *) "Diag"          },
    { .textId = SCR_TX_UI_TEST,       .textStr = (char *) "UI Test"       },

};

char *lookupTextStr( uint16_t textId ) {

    for ( unsigned int i = 0; i < sizeof( screenTextTokensTab ) / sizeof( *screenTextTokensTab ); i++ ) {

        if ( screenTextTokensTab[ i ].textId == textId ) return ( screenTextTokensTab[ i ].textStr );
    }

    return ((char *) "" );
}

//----------------------------------------------------------------------------------------
// The logical functions names for the cab handled UI elements and lookup function.
//
//----------------------------------------------------------------------------------------
struct  {

    uint8_t mapId;
    char    *mapStr;

} const dccMapFunctionTab[ ] = {

    { .mapId = DCC_F_M_HORN,      .mapStr = (char *) "Horn"         },
    { .mapId = DCC_F_M_BELL,      .mapStr = (char *) "Bell"         },
    { .mapId = DCC_F_M_ENG_ON,    .mapStr = (char *) "Eng On"       },
    { .mapId = DCC_F_M_ENG_OFF,   .mapStr = (char *) "Eng Off"      },
    { .mapId = DCC_F_M_F1,        .mapStr = (char *) "F1"           },
    { .mapId = DCC_F_M_F2,        .mapStr = (char *) "F2"           },
    { .mapId = DCC_F_M_F3,        .mapStr = (char *) "F3"           },
    { .mapId = DCC_F_M_F4,        .mapStr = (char *) "F4"           },
    { .mapId = DCC_F_M_F5,        .mapStr = (char *) "F5"           },
    { .mapId = DCC_F_M_F6,        .mapStr = (char *) "F6"           },
    { .mapId = DCC_F_M_F7,        .mapStr = (char *) "F7"           },
    { .mapId = DCC_F_M_F8,        .mapStr = (char *) "F8"           },

};

char *lookupMapStr( uint16_t mapId ) {

    for ( unsigned int i = 0; i < sizeof( dccMapFunctionTab ) / sizeof( *dccMapFunctionTab ); i++ ) {

      if ( dccMapFunctionTab[ i ].mapId == mapId ) return ( dccMapFunctionTab[ i ].mapStr );
    }

    return ((char *) "" );
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
char *lookupDccFuncTypeStr( uint8_t dccFuncOptId ) {

    switch ( dccFuncOptId ) {

      case DCC_F_M_T_DIS: return ( lookupTextStr( SCR_TX_FUNC_DISABLE ));
      case DCC_F_M_T_MOM: return ( lookupTextStr( SCR_TX_FUNC_MOM ));
      case DCC_F_M_T_TOG: return ( lookupTextStr( SCR_TX_FUNC_TOGGLE ));
      default:            return ( lookupTextStr( SCR_TX_FUNC_DISABLE ));
    }
}

//----------------------------------------------------------------------------------------
// Data to the screen are printed using this central routine. We expect the row, column and font
// information. The rest is just like you are used from the "printf" family.
//
//----------------------------------------------------------------------------------------
template<typename... Args>
void printFieldStr( uint8_t col, uint8_t row, uint8_t fontId, const char* fmt, Args... args ) {

    char buf[ 18 ];

    oled -> setFont( fontId );
    oled -> setCursor( col, row );

    snprintf( buf, sizeof( buf ), fmt, args... );
    oled -> print( buf );
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
// ??? a bit sloppy, what to test before doing ?

void clearLine( int line ) {

    oled -> clearLine( line );
}

void clearLines( int start, int len ) {

    for ( int i = start; i < start + len; i++ ) clearLine( i );
}

}; // namespace


//----------------------------------------------------------------------------------------
// Global variables.
//
//----------------------------------------------------------------------------------------
OperateScreen               *operateScreen              = nullptr;
EngineOnOffScreen           *engineOnOffScreen          = nullptr;
EngineLightsScreen          *engineLightsScreen         = nullptr;

TopMenuItemScreen           *newCabMenuScreen           = nullptr;
TopMenuItemScreen           *selectCabMenuScreen        = nullptr;
TopMenuItemScreen           *saveCabMenuScreen          = nullptr;
TopMenuItemScreen           *storeCabMenuScreen         = nullptr;
TopMenuItemScreen           *setFunctionMenuScreen      = nullptr;
TopMenuItemScreen           *configFunctionMenuScreen   = nullptr;
TopMenuItemScreen           *optionMenuScreen           = nullptr;
TopMenuItemScreen           *diagMenuScreen             = nullptr;

NewCabScreen                *newCabScreen               = nullptr;
SelectCabScreen             *selectCabScreen            = nullptr;
SaveCabScreen               *saveCabScreen              = nullptr;
SetFunctionSelectScreen     *setFunctionSelectScreen    = nullptr;
SetFunctionOperateScreen    *setFunctionOperateScreen   = nullptr;
ConfigFunctionSelectScreen  *configFunctionSelectScreen = nullptr;
ConfigFunctionEditScreen    *configFunctionEditScreen   = nullptr;
TestUIScreen                *testUIScreen               = nullptr;

//----------------------------------------------------------------------------------------
// "createScreens" will create the screen objects and build the screen hierarchy. There are the XXX menu
// screens, which become children to the root screen. This list forms the top line of screens. A menu screen
// will itself have one or more child screens. The MENU button will toggle through a screen list. The
// SELECT button will enter a child list, if there is any. Note that both MENU and SELECT button can be
// overwritten and will then not handle navigation. This is typically the case when the leaf screen is
// using the buttons for screen specific purposes.
//
//----------------------------------------------------------------------------------------
uint8_t setupScreens( ) {

    operateScreen               = new OperateScreen( );
    engineOnOffScreen           = new EngineOnOffScreen( );
    engineLightsScreen          = new EngineLightsScreen( );

    selectCabMenuScreen         = new TopMenuItemScreen( SCR_TX_SEL_CAB_L );
    saveCabMenuScreen           = new TopMenuItemScreen( SCR_TX_SAVE_CAB );
    newCabMenuScreen            = new TopMenuItemScreen( SCR_TX_NEW_CAB );
    setFunctionMenuScreen       = new TopMenuItemScreen( SCR_TX_SET_FUNC );
    configFunctionMenuScreen    = new TopMenuItemScreen( SCR_TX_CONFIG_FUNC_L );
    optionMenuScreen            = new TopMenuItemScreen( SCR_TX_OPTIONS );
    diagMenuScreen              = new TopMenuItemScreen( SCR_TX_DIAG );

    selectCabScreen             = new SelectCabScreen( encoder );
    saveCabScreen               = new SaveCabScreen( encoder );
    newCabScreen                = new NewCabScreen( encoder );
    setFunctionSelectScreen     = new SetFunctionSelectScreen( encoder );
    setFunctionOperateScreen    = new SetFunctionOperateScreen( );
    configFunctionSelectScreen  = new ConfigFunctionSelectScreen( encoder );
    configFunctionEditScreen    = new ConfigFunctionEditScreen( encoder );
    testUIScreen                = new TestUIScreen( );

    UIScreen::getRootScreen( )  -> append( operateScreen );
    UIScreen::getRootScreen( )  -> append( engineOnOffScreen );
    UIScreen::getRootScreen( )  -> append( engineLightsScreen );
    UIScreen::getRootScreen( )  -> append( selectCabMenuScreen );
    UIScreen::getRootScreen( )  -> append( saveCabMenuScreen );
    UIScreen::getRootScreen( )  -> append( newCabMenuScreen );
    UIScreen::getRootScreen( )  -> append( setFunctionMenuScreen );
    UIScreen::getRootScreen( )  -> append( configFunctionMenuScreen );
    UIScreen::getRootScreen( )  -> append( optionMenuScreen );
    UIScreen::getRootScreen( )  -> append( diagMenuScreen );

    configFunctionMenuScreen    -> append( configFunctionSelectScreen );
    setFunctionMenuScreen       -> append( setFunctionSelectScreen );
    newCabMenuScreen            -> append( newCabScreen );
    selectCabMenuScreen         -> append( selectCabScreen );
    saveCabMenuScreen           -> append( saveCabScreen );
    diagMenuScreen              -> append( testUIScreen );

    setFunctionSelectScreen     -> append( setFunctionOperateScreen );
    configFunctionSelectScreen  -> append( configFunctionEditScreen );

    return ( LCS::ALL_OK );
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//
//  Cab Handheld Screen common Methods.
//
//----------------------------------------------------------------------------------------
// All cab handheld screens have a common screen layout. The screen is divided into a top line, which has
// room for a title and the labelling of the adjacent buttons MENU and UP. Likewise, there is a bottom line
// with room for some status flags and the label fields for SELECT and DOWN. In between are two lines in a
// larger font which are the screen content.
//
//
//                         0    2 3              12 13  15
//                        :------:-----------------:------:
//      Top Line    ->    : Menu :  Title          : Up   :
//                        :------:-----------------:------:
//      Line 1      ->    : Text                          :
//                        :-------------------------------:
//      Line 1      ->    : Text                          :
//                        :------:-----------------:------:
//      Top Line    ->    : Sel  :  Status Flags   : Down :
//                        :------:-----------------:------:
//
// There are methods to set these fields easy and straightforward. The top and bottom line are printed in an
// 8x8 font, the two main lines in a 8x16 font. Note that the display object expects rows and columns based
// on an 8-pixel raster. So, a 128x64 screen has 16 columns and 8 rows. A font that takes two rows starts at
// the lower row of the two rows it occupies.
//
// For all printing functions, there is one template method, which uses the "printf" family style. All that
// is added is the screen location and font data.
//
//----------------------------------------------------------------------------------------
void CabHandheldScreen::printMenuLabel( char *str )          { printFieldStr( 0, 0, FT_8x8, "%3s", str ); }
void CabHandheldScreen::printMenuLabel( uint16_t textId )    { printMenuLabel( lookupTextStr( textId )); }

void CabHandheldScreen::printTitle( char *str )              { printFieldStr( 4, 0, FT_8x8, "%8s", str ); }
void CabHandheldScreen::printTitle( uint16_t textId  )       { printTitle( lookupTextStr( textId )); }

void CabHandheldScreen::printUpLabel( char *str )            { printFieldStr( 13, 0, FT_8x8, "%3s", str ); }
void CabHandheldScreen::printUpLabel( uint16_t textId )      { printUpLabel( lookupTextStr( textId )); }

void CabHandheldScreen::printSelectLabel( char *str )        { printFieldStr( 0, 7, FT_8x8, "%3s", str ); }
void CabHandheldScreen::printSelectLabel( uint16_t textId )  { printSelectLabel( lookupTextStr( textId )); }

void CabHandheldScreen::printDownLabel( char *str )          { printFieldStr( 13, 7, FT_8x8, "%3s", str ); }
void CabHandheldScreen::printDownLabel( uint16_t textId )    { printDownLabel( lookupTextStr( textId )); }

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//
//  Cab Handheld Top Menu Screen Methods.
//
//----------------------------------------------------------------------------------------
// The menu screens. This is the top level screen list. They are straightforward. All we do is to show the
// menu and select labels and a text that says what this menu is.
//
//----------------------------------------------------------------------------------------
TopMenuItemScreen::TopMenuItemScreen( uint8_t item ) {

    this -> item = item;
}

void TopMenuItemScreen::enterScreen( bool init ) {

    oled -> clear( );
    printMenuLabel( SCR_TX_NEXT );
    printSelectLabel( SCR_TX_SEL );

    printFieldStr( 3, 3, FT_8x16, "%s", lookupTextStr( item ));
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//
//  Scrollable Screen Methods.
//
//----------------------------------------------------------------------------------------
// Some screens display a list of items that you can scroll through. The UP, DOWN buttons and also the encoder
// knob allow for a scrolling function. This class allows for a convenient constructions of such screens. The
// "showScreenData" method can be overridden to display the actual content. Before the object can be used,
// the "setScreenData" method sets up the screen title and the encoder limits.
//
//----------------------------------------------------------------------------------------
ScrollableScreen::ScrollableScreen( UIEncoder *encoder ) {

    this -> encoder = encoder;
    this -> low     = INT_MIN;
    this -> high    = INT_MAX;
}

void ScrollableScreen::enterScreen( bool init ) {

    oled -> clear( );
    printMenuLabel( SCR_TX_NEXT );
    printUpLabel( SCR_TX_UP );
    printSelectLabel( SCR_TX_SEL );
    printDownLabel( SCR_TX_DOWN );

    if ( encoder != nullptr ) {

        encoder -> setLimits( low, high );
        encoder -> setPosition( low );
    }

    index = low;
    showScreenData( index );
}

void ScrollableScreen::upButtonClick( UIButton * buttonObj ) {

    if ( index < high ) index ++;
    else                index = low;

    if ( encoder != nullptr ) encoder -> setPosition( index );
    showScreenData( index );
}

void ScrollableScreen::downButtonClick( UIButton *buttonObj ) {

    if ( index > low ) index --;
    else               index = high;

    if ( encoder != nullptr ) encoder -> setPosition( index );
    showScreenData( index );
}

void ScrollableScreen::encoderPosChange( UIEncoder *encoderObj ) {

    if ( encoder != nullptr ) {

        int pos = encoderObj -> getPosition( );

        index = (( pos > high ) ? high : pos );
        showScreenData( index );
    }
}

int ScrollableScreen::getIndex( ) {

    return ( index );
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//
//  Operate Screen Methods.
//
//----------------------------------------------------------------------------------------
// "OperateScreen" is our main screen. It has all the relevant control elements and information items that
// are necessary to run the current loco.
//
//----------------------------------------------------------------------------------------
void OperateScreen::enterScreen( bool init ) {

    oled -> clear( );
    printTitle( SCR_TX_OPERATE );
    printMenuLabel( SCR_TX_NEXT );
    printUpLabel( SCR_TX_FUNC_GROUP );
    printSelectLabel( SCR_TX_SEL );
    printDownLabel( SCR_TX_NIL );

    showCabData( );

    encoder -> setLimits( MIN_LOCO_SPEED, MAX_LOCO_SPEED );
    encoder -> setPosition( MIN_LOCO_SPEED );
}

void OperateScreen::buttonClick( UIButton *buttonObj ) {

    uint8_t   cNum          = buttonObj -> getResId( );
    CabEntry  *currentCab   = &cabStack -> currentCab;

    if ( functionSet == 2 ) {

        if      ( cNum == DCC_F_M_F1 )  cNum = DCC_F_M_F5;
        else if ( cNum == DCC_F_M_F2 )  cNum = DCC_F_M_F6;
        else if ( cNum == DCC_F_M_F3 )  cNum = DCC_F_M_F7;
        else if ( cNum == DCC_F_M_F4 )  cNum = DCC_F_M_F8;
    }

    if ( currentCab -> getDccFuncTypeForChFuncId( cNum ) == DCC_F_M_T_TOG ) {

        currentCab -> toggleDccFuncState( currentCab -> getDccFuncIdForChFuncId( cNum ));
        msgBus -> sendDccFuncVal( currentCab, currentCab -> getDccFuncIdForChFuncId( cNum ));
    }

    showCabData( );
}

void OperateScreen::buttonLongPressStart( UIButton *buttonObj ) {

    uint8_t   cNum          = buttonObj -> getResId( );
    uint8_t   hwId          = buttonObj -> getHwId( );
    CabEntry  *currentCab   = &cabStack -> currentCab;

    if      ( hwId == RNUM_FWD_BUTTON ) cabStack -> currentCab.setDirection( 1 );
    else if ( hwId == RNUM_REV_BUTTON ) cabStack -> currentCab.setDirection( 2 );
    else {

        if ( functionSet == 2 ) {

            if      ( cNum == DCC_F_M_F1 )  cNum = DCC_F_M_F5;
            else if ( cNum == DCC_F_M_F2 )  cNum = DCC_F_M_F6;
            else if ( cNum == DCC_F_M_F3 )  cNum = DCC_F_M_F7;
            else if ( cNum == DCC_F_M_F4 )  cNum = DCC_F_M_F8;
        }

        if ( currentCab -> getDccFuncTypeForChFuncId( cNum ) == DCC_F_M_T_MOM ) {

            currentCab -> setDccFuncState( currentCab -> getDccFuncIdForChFuncId( cNum ), true );
            msgBus -> sendDccFuncVal( currentCab, currentCab -> getDccFuncIdForChFuncId( cNum ));
        }
    }

    showCabData( );
}

void OperateScreen::buttonLongPressStop( UIButton *buttonObj ) {

    uint8_t   cNum          = buttonObj -> getResId( );
    CabEntry  *currentCab   = &cabStack -> currentCab;

    if ( functionSet == 2 ) {

        if      ( cNum == DCC_F_M_F1 )  cNum = DCC_F_M_F5;
        else if ( cNum == DCC_F_M_F2 )  cNum = DCC_F_M_F6;
        else if ( cNum == DCC_F_M_F3 )  cNum = DCC_F_M_F7;
        else if ( cNum == DCC_F_M_F4 )  cNum = DCC_F_M_F8;
    }

    if ( currentCab -> getDccFuncTypeForChFuncId( cNum ) == DCC_F_M_T_MOM ) {

        currentCab -> setDccFuncState( currentCab -> getDccFuncIdForChFuncId( cNum ), false );
        msgBus -> sendDccFuncVal( currentCab, currentCab -> getDccFuncIdForChFuncId( cNum ));
    }

    showCabData( );
}

void OperateScreen::upButtonClick( UIButton *buttonObj ) {

    if      ( functionSet == 1 ) functionSet = 2;
    else if ( functionSet == 2 ) functionSet = 1;
    showCabData( );
}

void OperateScreen::encoderPosChange( UIEncoder *encoderObj ) {

    CabEntry  *currentCab   = &cabStack -> currentCab;

    cabStack -> currentCab.setSpeed( encoderObj -> getPosition( ));
    msgBus -> sendSpeedAndDir( currentCab );
    showCabData( );
}

void OperateScreen::showCabData( ) {

    if ( cabStack -> currentCab.getCabId( ) != LCS::NIL_CAB_ID ) {

        printFieldStr( 0, 2, FT_8x16, "Cab: %04d %c",
                        cabStack -> currentCab.getCabId( ),
                        cabStack -> currentCab.getEngineTypeChar( ));
    }
    else printFieldStr( 0, 2, FT_8x16, "Cab: --- %c", cabStack -> currentCab.getEngineTypeChar( ));

    printFieldStr( 0, 4, FT_8x16, "Dir: %03d %s",
                    cabStack -> currentCab.getSpeed( ),
                    (( cabStack -> currentCab.getDirection( ) == 1 ) ? ((char *) "fwd" ) : ((char *) "rev" )));

    if      ( functionSet == 1 ) printFieldStr( 0, 7, FT_8x8, "F1  F2  F3  F4  " );
    else if ( functionSet == 2 ) printFieldStr( 0, 7, FT_8x8, "F5  F6  F7  F8  " );
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//
//  Engine On/Off Screen Methods.
//
//----------------------------------------------------------------------------------------
// Engine on and off screen. Especially a diesel engine needs to first turn its prime mover on first. And you
// cannot turn it off when the engine is not stopped.
//
//----------------------------------------------------------------------------------------
void EngineOnOffScreen::enterScreen( bool init ) {

    oled -> clear( );
    printMenuLabel( SCR_TX_NEXT );
    printSelectLabel( SCR_TX_SEL );
    printUpLabel( SCR_TX_ON );
    printDownLabel( SCR_TX_OFF );

    // ??? get engine status for display from current cab...
    // ??? get text from our table ?

    printFieldStr( 3, 3, FT_8x16, "Engine ---" );
}

void EngineOnOffScreen::upButtonClick( UIButton *buttonObj ) {

    // ??? set engine status
    // ??? get text from our table ?

    CabEntry  *currentCab   = &cabStack -> currentCab;

    msgBus -> sendEngineOnOff( currentCab );
    printFieldStr( 3, 3, FT_8x16, "Engine On  " );
}

void EngineOnOffScreen::downButtonClick( UIButton *buttonObj ) {

    // ??? check engine speed... off only when speed = 0!
    // ??? set engine status
    // ??? get text from our table ?

    CabEntry  *currentCab   = &cabStack -> currentCab;

    msgBus -> sendEngineOnOff( currentCab );
    printFieldStr( 3, 3, FT_8x16, "Engine Off " );
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//
//  Engine Lights Screen Methods.
//
//----------------------------------------------------------------------------------------
// Engine lights screen. An engine has front and rear lights.
//
// OFF, DIM and BRIGHT, optional BRIGHT+DITCH LIGHTS ( Diesels only ? )
//
//----------------------------------------------------------------------------------------
void EngineLightsScreen::enterScreen( bool init ) {

    oled -> clear( );
    printMenuLabel( SCR_TX_NEXT );
    printSelectLabel( SCR_TX_SEL );
    printUpLabel( SCR_TX_FRONT );
    printDownLabel( SCR_TX_REAR );

    // ??? get engine status for display from current cab...
    // ??? get text from our table ?

    // ??? how exactly would we present the lights data ...

    printFieldStr( 3, 3, FT_8x16, lookupTextStr( SCR_TX_LIGHTS ));
}

void EngineLightsScreen::upButtonClick( UIButton *buttonObj ) {

    CabEntry  *currentCab   = &cabStack -> currentCab;

    // ??? get text from our table ?
    // ??? show setting...

}

void EngineLightsScreen::downButtonClick( UIButton *buttonObj ) {

    CabEntry  *currentCab   = &cabStack -> currentCab;

    // ??? get text from our table ?
    // ??? show setting...

}

void EngineLightsScreen::buttonClick( UIButton *buttonObj ) {

    uint8_t   cNum          = buttonObj -> getResId( );
    CabEntry  *currentCab   = &cabStack -> currentCab;

    // ??? how exactly would we present the lights data ...
  
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//
//  Select Cab Screen Methods.
//
//----------------------------------------------------------------------------------------
// Select Cab Screen. There is a stack of known cabs.We can scroll through the list with UP/DOWN and the
// encoder knob. The SELECT button will make the entry shown the current loco. The MENU button click is
// overwritten, so that we do not enter this screen over and over.
//
//----------------------------------------------------------------------------------------
SelectCabScreen::SelectCabScreen( UIEncoder *encoder ) : ScrollableScreen( encoder ) { }

void SelectCabScreen::enterScreen( bool init ) {

    low   = 1;
    high  = cabStack -> getMaxEntries( );

    ScrollableScreen::enterScreen( init );
    printTitle( SCR_TX_SEL_CAB );
}

void SelectCabScreen::menuButtonClick( UIButton *buttonObj ) { }

void SelectCabScreen::selectButtonClick( UIButton *buttonObj ) {

    cabStack -> loadCurrentCabFromSlot( getIndex( ));
    UIScreen::setCurrentScreen( operateScreen );

    // ??? send a "dispatched" o "close" message ?
}

void SelectCabScreen::showScreenData( int index ) {

    if ( cabStack -> cabSlots[ index - 1 ].getCabId( ) != LCS::NIL_CAB_ID ) {

        printFieldStr( 0, 2, FT_8x16, "Cab: %04d", cabStack -> cabSlots[ index - 1 ].getCabId( ));
    }
    else printFieldStr( 0, 2, FT_8x16, "Cab: ---");

    printFieldStr( 5, 7, FT_8x8, "%d", index );
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//
//  Save Cab Screen Methods.
//
//----------------------------------------------------------------------------------------
// Save cab Screen. The current cab can be saved to the cab stack. We can scroll through the list with UP/DOWN
// and the encoder knob. The SELECT button will store the current cab to the selected slot. The MENU button
// click is overwritten, so that we do not enter this screen over and over.
//
//----------------------------------------------------------------------------------------
SaveCabScreen::SaveCabScreen( UIEncoder *encoder ) : ScrollableScreen( encoder ) { }

void SaveCabScreen::enterScreen( bool init ) {

    low   = 1;
    high  = cabStack -> getMaxEntries( );

    ScrollableScreen::enterScreen( init );
    printTitle( SCR_TX_SAVE_CAB );
}

void SaveCabScreen::menuButtonClick( UIButton *buttonObj ) { }

void SaveCabScreen::selectButtonClick( UIButton *buttonObj ) {

    cabStack -> storeCurrentCabToSlot( getIndex( ));
    cabStack -> updateCabSlotInNVM( getIndex( ));
    UIScreen::setCurrentScreen( operateScreen );
}

void SaveCabScreen::showScreenData( int index ) {

    if ( cabStack -> cabSlots[ index - 1 ].getCabId( ) != LCS::NIL_CAB_ID ) {

        printFieldStr( 0, 2, FT_8x16, "Cab: %04d", cabStack -> cabSlots[ index - 1 ].getCabId( ));
    }
    else printFieldStr( 0, 2, FT_8x16, "Cab: ---");

    printFieldStr( 5, 7, FT_8x8, "%d", index );
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//
// New Cab Screen Methods.
//
//----------------------------------------------------------------------------------------
// New Cab Screen. There needs to be a way to set an engine cab number. We will display 4 digits among we
// can toggle with the MENU button. The UP/DOWN buttons advance the current digit position. The optional
// encoder knob offers a fast way to scroll a digit. The number of valid arguments in a digit is determined 
// by the digit position. The first digit, encodes the engine type, the second a number between 0 and 9 as 
// well as the "S" for a short DCC address and the other three position are just numbers from 0 to 9. The 
// SELECT button completes the data entering. We make the current cab this new cab. Note, that it would need
// to be explicitly saved.
//
// ??? we could also get initial data from the base station if the cab is known there... tbd.
// ??? what about the short address ?
//----------------------------------------------------------------------------------------
NewCabScreen::NewCabScreen( UIEncoder *encoder ) {

    this -> encoder = encoder;
}

void NewCabScreen::enterScreen( bool init ) {

    oled -> clear( );
    printTitle( SCR_TX_NEW_CAB );
    printMenuLabel( SCR_TX_NEXT );
    printSelectLabel( SCR_TX_SEL );
    printUpLabel( SCR_TX_UP );
    printDownLabel( SCR_TX_DOWN );

    printFieldStr( 6, 2, FT_8x16, "- 0000" );
    printFieldStr( 6, 4, FT_8x16, "     ^" );

    itemIndex   = 4;
    items[ 0 ]  = 0;
    items[ 1 ]  = 0;
    items[ 2 ]  = 0;
    items[ 3 ]  = 0;
    items[ 4 ]  = 0;

    encoder -> setLimits( 0, itemLimit( 4 ));
    encoder -> setPosition( 0 );
}

void NewCabScreen::menuButtonClick( UIButton * buttonObj ) {

    itemIndex = (( itemIndex == 0 ) ? 4 : itemIndex - 1 );

    switch ( itemIndex ) {

        case 0: printFieldStr( 6, 4, FT_8x16, "^     " ); break;
        case 1: printFieldStr( 6, 4, FT_8x16, "  ^   " ); break;
        case 2: printFieldStr( 6, 4, FT_8x16, "   ^  " ); break;
        case 3: printFieldStr( 6, 4, FT_8x16, "    ^ " ); break;
        case 4: printFieldStr( 6, 4, FT_8x16, "     ^" ); break;
    }

    encoder -> setLimits( 0, itemLimit( itemIndex ));
    encoder -> setPosition( items[ itemIndex ] );
}

void NewCabScreen::upButtonClick( UIButton * buttonId ) {

    if ( items[ itemIndex ] < itemLimit( itemIndex ))  items[ itemIndex ] ++;
    else                                               items[ itemIndex ] = 0;

    if ( encoder != nullptr ) encoder -> setPosition( items[ itemIndex ] );

    showCabId( );
}

void NewCabScreen::downButtonClick( UIButton * buttonId ) {

    if ( items[ itemIndex ] > 0 )  items[ itemIndex ] --;
    else                           items[ itemIndex ] = itemLimit( itemIndex );

    if ( encoder != nullptr ) encoder -> setPosition( items[ itemIndex ] );

    showCabId( );
}

void NewCabScreen::encoderPosChange( UIEncoder * encoderObj ) {

    if ( encoder != nullptr ) {

        uint8_t limit = itemLimit( itemIndex );
        int     pos   = encoderObj -> getPosition( );
        items[ itemIndex ] = (( pos > limit ) ? limit : pos );
    }

    showCabId( );
}

void NewCabScreen::selectButtonClick( UIButton * buttonId ) {

    // ??? set short DCC address right here...

    CabEntry  *currentCab = &cabStack -> currentCab;

    currentCab -> reset( );
    currentCab -> setCabId( buildCabId( ));
    currentCab -> setEngineType( items[ 0 ] );

    uint8_t rStat = msgBus -> requestLocoSession( currentCab );

    if ( rStat == LCS::ALL_OK ) {

        UIScreen::setCurrentScreen( operateScreen );
    }
    else {

        // ??? error .... how to display ?
    }
}

uint16_t NewCabScreen::buildCabId( ) {

    bool shortDccAdr = ( items[ 1 ] == 10 );

    if ( items[ 1 ] == 10 ) items[ 1 ] = 0;

    uint8_t   engineType  = items[ 0 ];
    uint16_t  cabNum      = (((((( items[ 1 ] * 10 ) + items[ 2 ] ) * 10 ) + items[ 3 ] ) * 10 ) + items[ 4 ] );

    return (((uint16_t ) engineType << 14 ) | ( cabNum & 0x3FFF ));
}

void NewCabScreen::showCabId( ) {

    printFieldStr( 6, 2, FT_8x16, "%c %c%c%c%c",
                    engTypeToChar[ items[ 0 ]],
                    digitsToChar[ items[ 1 ]],
                    digitsToChar[ items[ 2 ]],
                    digitsToChar[ items[ 3 ]],
                    digitsToChar[ items[ 4 ]] );
}

int NewCabScreen::itemLimit( int index ) {

    if      ( index == 0 )  return ( 3 );
    else if ( index == 1 )  return ( 10 );
    else                    return ( 9 );
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//
// Set DCC Function Select Screen Methods.
//
//----------------------------------------------------------------------------------------
// Set DCC function select screen. DCC has a set of 69 functions, F0 to F68. This screen selects the function
// we want to set. UP/DOWN and the encoder knob allow for scrolling through the list. Upon SELECT, the set
// function "operate" screen is entered.
//
//----------------------------------------------------------------------------------------
SetFunctionSelectScreen::SetFunctionSelectScreen( UIEncoder *encoder ) : ScrollableScreen( encoder ) { }

void SetFunctionSelectScreen::enterScreen( bool init ) {

    low   = MIN_DCC_F_M;
    high  = MAX_DCC_F_M;

    ScrollableScreen::enterScreen( init );
    printTitle( SCR_TX_SET_FUNC );
}

void SetFunctionSelectScreen::menuButtonClick( UIButton *buttonObj ) { }

void SetFunctionSelectScreen::showScreenData( int index ) {

    bool functionState = cabStack -> currentCab.getDccFuncState( index );

    printFieldStr( 0, 2, FT_8x16, "DccF: %02d -> %s", index,
                    (( functionState ) ? ((char *) "ON " ) : ((char *) "OFF" )));
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//
//  Set DCC Function Operate Screen Methods.
//
//----------------------------------------------------------------------------------------
// Set DCC function operate screen. The set function operate screen allows to set and reset the DCC function.
// The UP/DOWN buttons are used for ON and OFF setting. The MENU button gets us back to the cab handheld
// operate screen.
//
//----------------------------------------------------------------------------------------
void SetFunctionOperateScreen::enterScreen( bool init ) {

    functionId = (( ScrollableScreen *) getParentScreen( )) -> getIndex( );

    printSelectLabel( SCR_TX_NIL );
    printUpLabel( SCR_TX_ON );
    printDownLabel( SCR_TX_OFF );
    showScreenData( );
}

void SetFunctionOperateScreen::menuButtonClick( UIButton *buttonObj ) {

    functionId = -1;
    UIScreen::setCurrentScreen( operateScreen );
}

void SetFunctionOperateScreen::upButtonClick( UIButton *buttonObj ) {

    CabEntry  *currentCab = &cabStack -> currentCab;

    currentCab -> setDccFuncState( functionId, true );
    msgBus -> sendDccFuncVal( currentCab, functionId );
    showScreenData( );
}

void SetFunctionOperateScreen::downButtonClick( UIButton *buttonObj ) {

    CabEntry  *currentCab = &cabStack -> currentCab;

    currentCab -> setDccFuncState( functionId, false );
    msgBus -> sendDccFuncVal( currentCab, functionId );
    showScreenData( );
}

void SetFunctionOperateScreen::showScreenData( ) {

    bool functionState = cabStack -> currentCab.getDccFuncState( functionId );

    printFieldStr( 0, 2, FT_8x16, "DccF: %02d -> %s", functionId,
                    (( functionState ) ? ((char *) "ON " ) : ((char *) "OFF" )));
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//
//  Configure Cab Function Select Screen Methods.
//
//----------------------------------------------------------------------------------------
// Config function select screen. The cab handheld UI elements such as the buttons HORN, BELL and functions 
// need to be mapped to their DCC function code for the particular engine. This screen will select the logical
// function to map. On SELECT, we will enter the child screen, which will actually configure the item. The
// MENU button click is disabled, so that we do not enter this screen over and over.
//
//----------------------------------------------------------------------------------------
ConfigFunctionSelectScreen::ConfigFunctionSelectScreen( UIEncoder *encoder ) : ScrollableScreen( encoder ) { }

void ConfigFunctionSelectScreen::enterScreen( bool init ) {

    low   = MIN_DCC_F_M;
    high  = MAX_DCC_F_M;

    ScrollableScreen::enterScreen( init );
    printTitle( SCR_TX_CONFIG_FUNC );
}

void ConfigFunctionSelectScreen::menuButtonClick( UIButton *buttonObj ) { }

void ConfigFunctionSelectScreen::showScreenData( int index ) {

    clearLines( 2, 2 );
    printFieldStr( 0, 2, FT_8x16, "CabF: %s", lookupMapStr( index ));
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//
//  Configure Cab Function Edit Screen Methods.
//
//----------------------------------------------------------------------------------------
// Config function edit screen. We are the child of the config function select screen, which has the index
// of the selected item. The edit screen uses the UP and DOWN buttons for the selection of the DCC function
// Id. The MENU button toggles through the function options to set and the SELECT button will confirm the
// setting and we return to the Config Function Menu. Note, we also override the enterScreen to display the
// rather static data so that the screen is not flickering.
//
//----------------------------------------------------------------------------------------
ConfigFunctionEditScreen::ConfigFunctionEditScreen( UIEncoder *encoder ) : ScrollableScreen( encoder ) { }

void ConfigFunctionEditScreen::enterScreen( bool init ) {

    low           = LCS::MIN_DCC_FUNC_ID;
    high          = LCS::MAX_DCC_FUNC_ID;

    ScrollableScreen::enterScreen( init );
    printTitle( SCR_TX_CONFIG_FUNC );

    cabFuncId     = (uint8_t ) (( ScrollableScreen *) getParentScreen( )) -> getIndex( );
    dccFuncId     = cabStack -> currentCab.getDccFuncIdForChFuncId( cabFuncId );
    dccFuncOptId  = cabStack -> currentCab.getDccFuncTypeForChFuncId( cabFuncId );

    clearLines( 1, 4 );
    printFieldStr( 0, 2, FT_8x16, "CabF: %s", lookupMapStr( cabFuncId ));
    printFieldStr( 0, 4, FT_8x16, "DccF: %02d %s", dccFuncId, lookupDccFuncTypeStr( dccFuncOptId ));
}

void ConfigFunctionEditScreen::menuButtonClick( UIButton *buttonObj ) {

    dccFuncOptId ++;
    if ( dccFuncOptId >= 3 ) dccFuncOptId = 0;

    printFieldStr( 0, 4, FT_8x16, "DccF: %02d %s", getIndex( ), lookupDccFuncTypeStr( dccFuncOptId ));
}

void ConfigFunctionEditScreen::selectButtonClick( UIButton *buttonObj ) {

    CabEntry *currentCab  = &cabStack -> currentCab;

    currentCab -> setDccFuncIdForChFuncId( cabFuncId, dccFuncId );
    currentCab -> setDccFuncTypeForChFuncId( cabFuncId, dccFuncOptId );

    UIScreen::setCurrentScreen( getParentScreen( ) -> getParentScreen( ));
}

void ConfigFunctionEditScreen::showScreenData( int index ) {

    dccFuncId = index;
    printFieldStr( 0, 4, FT_8x16, "DccF: %02d %s", index, lookupDccFuncTypeStr( dccFuncOptId ));
}

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//
//  Test UI Elements Screen Methods.
//
//----------------------------------------------------------------------------------------
// Test UI Elements. This menu is a very handy menu to test the individual UI elements for basic function.
// It is invoked from the DIAG menu. The UI tests use all buttons so, we can only return via the MENU button
// long press function to the main menu. This screen is a child screen of the test UI elements screen found
// in the DIAG menu.
//
//----------------------------------------------------------------------------------------
void TestUIScreen::enterScreen( bool init ) {

    oled -> clear( );
    printTitle( SCR_TX_UI_TEST );
}

void TestUIScreen::menuButtonClick( UIButton *buttonId ) {

    oled -> clearLine( 2 );
    printFieldStr( 0, 2, FT_8x16, "MENU CLICK" );
}

void TestUIScreen::selectButtonClick( UIButton *buttonId ) {

    oled -> clearLine( 2 );
    printFieldStr( 0, 2, FT_8x16, "SELECT CLICK" );
}

void TestUIScreen::upButtonClick( UIButton *buttonId ) {

    oled -> clearLine( 2 );
    printFieldStr( 0, 2, FT_8x16, "UP CLICK" );
}

void TestUIScreen::downButtonClick( UIButton *buttonId ) {

    oled -> clearLine( 2 );
    printFieldStr( 0, 2, FT_8x16, "DOWN CLICK" );
}

void TestUIScreen::buttonClick( UIButton *buttonId ) {

    oled -> clearLine( 2 );

    switch ( buttonId -> getHwId( )) {

        case RNUM_HORN_BUTTON:      printFieldStr( 0, 2, FT_8x16, "HORN CLICK" );     break;
        case RNUM_BELL_BUTTON:      printFieldStr( 0, 2, FT_8x16, "BELL CLICK" );     break;
        case RNUM_FWD_BUTTON:       printFieldStr( 0, 2, FT_8x16, "FWD CLICK" );      break;
        case RNUM_REV_BUTTON:       printFieldStr( 0, 2, FT_8x16, "REV CLICK" );      break;
        case RNUM_F1_BUTTON:        printFieldStr( 0, 2, FT_8x16, "F1 CLICK" );       break;
        case RNUM_F2_BUTTON:        printFieldStr( 0, 2, FT_8x16, "F2 CLICK" );       break;
        case RNUM_F3_BUTTON:        printFieldStr( 0, 2, FT_8x16, "F3 CLICK" );       break;
        case RNUM_F4_BUTTON:        printFieldStr( 0, 2, FT_8x16, "F4 CLICK" );       break;
        case RNUM_ENCODER_BUTTON:   printFieldStr( 0, 2, FT_8x16, "ENCODER CLICK" );  break;
    }
}

void TestUIScreen::buttonLongPressStart( UIButton *buttonObj ) {

    oled -> clearLine( 2 );

    switch ( buttonObj -> getHwId( )) {

        case RNUM_HORN_BUTTON:      printFieldStr( 0, 2, FT_8x16, "HORN LP START" );     break;
        case RNUM_BELL_BUTTON:      printFieldStr( 0, 2, FT_8x16, "BELL LP START" );     break;
        case RNUM_FWD_BUTTON:       printFieldStr( 0, 2, FT_8x16, "FWD LP START" );      break;
        case RNUM_REV_BUTTON:       printFieldStr( 0, 2, FT_8x16, "REV LP START" );      break;
        case RNUM_F1_BUTTON:        printFieldStr( 0, 2, FT_8x16, "F1 LP START" );       break;
        case RNUM_F2_BUTTON:        printFieldStr( 0, 2, FT_8x16, "F2 LP START" );       break;
        case RNUM_F3_BUTTON:        printFieldStr( 0, 2, FT_8x16, "F3 LP START" );       break;
        case RNUM_F4_BUTTON:        printFieldStr( 0, 2, FT_8x16, "F4 LP START" );       break;
        case RNUM_ENCODER_BUTTON:   printFieldStr( 0, 2, FT_8x16, "ENCODER LP START" );  break;
    }
}

void TestUIScreen::buttonLongPressStop( UIButton *buttonObj ) {

    oled -> clearLine( 2 );

    switch ( buttonObj -> getHwId( )) {

        case RNUM_HORN_BUTTON:      printFieldStr( 0, 2, FT_8x16, "HORN LP STOP" );     break;
        case RNUM_BELL_BUTTON:      printFieldStr( 0, 2, FT_8x16, "BELL LP STOP" );     break;
        case RNUM_FWD_BUTTON:       printFieldStr( 0, 2, FT_8x16, "FWD LP STOP" );      break;
        case RNUM_REV_BUTTON:       printFieldStr( 0, 2, FT_8x16, "REV LP STOP" );      break;
        case RNUM_F1_BUTTON:        printFieldStr( 0, 2, FT_8x16, "F1 LP STOP" );       break;
        case RNUM_F2_BUTTON:        printFieldStr( 0, 2, FT_8x16, "F2 LP STOP" );       break;
        case RNUM_F3_BUTTON:        printFieldStr( 0, 2, FT_8x16, "F3 LP STOP" );       break;
        case RNUM_F4_BUTTON:        printFieldStr( 0, 2, FT_8x16, "F4 LP STOP" );       break;
        case RNUM_ENCODER_BUTTON:   printFieldStr( 0, 2, FT_8x16, "ENCODER LP STOP" );  break;
    }
}

void TestUIScreen::encoderPosChange( UIEncoder *encoderObj ) {

    oled -> clearLine( 2 );
    printFieldStr( 0, 2, FT_8x16, "Pos: %04d", encoderObj -> getPosition( ));
}
