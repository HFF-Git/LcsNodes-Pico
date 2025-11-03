//----------------------------------------------------------------------------------------
//
// LCS - Cab Handheld Include file
//
//----------------------------------------------------------------------------------------
// Welcome to the LCS Cab Handheld.
//
//
//  Key modules...
//
//  - CabLcsBus
//  - CabScreens
//  - CabStack
//  - CabUIElements
//
//
//----------------------------------------------------------------------------------------
//
// LCS - Cab Handheld Include file
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
#ifndef CabHandheld_h
#define Cabhandheld_h

#include "LcsBasicThrottleBoardDesc.h"
#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"
#include "LcsUIElements.h"

using namespace LCS;

//----------------------------------------------------------------------------------------
// There are plenty of defines... some will go away after the design stabilizes....
//
//----------------------------------------------------------------------------------------
#define DEBUG                           1
#define DEBUG_LCS_MSG_INTERFACE         1

//----------------------------------------------------------------------------------------
// Default CanBus Id. The CBUS standard defines devices that have a fixed node and 
// also a fixed can bus id. CanID numbers above 100 are used for this purpose. This
// is true for the base station. For handhelds, we use a default CAN ID and enumerate
// if there is a can id conflict.
//
//----------------------------------------------------------------------------------------
const int CAN_BUS_DEFAULT_ID   = 110;

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
enum DccMapFunctionId : uint8_t {

    DCC_F_M_HORN         = 1,
    DCC_F_M_BELL         = 2,
    DCC_F_M_ENG_ON       = 3,
    DCC_F_M_ENG_OFF      = 4,
    DCC_F_M_F1           = 5,
    DCC_F_M_F2           = 6,
    DCC_F_M_F3           = 7,
    DCC_F_M_F4           = 8,
    DCC_F_M_F5           = 9,
    DCC_F_M_F6           = 10,
    DCC_F_M_F7           = 11,
    DCC_F_M_F8           = 12,
    DCC_F_M_ENC_BTN      = 13,

    // ??? more to come ...

    NIL_DCC_F_M          = 0,
    MIN_DCC_F_M          = 1,
    MAX_DCC_F_M          = 14
};

enum DccMapFunctionType : uint8_t {

    DCC_F_M_T_DIS = 0,
    DCC_F_M_T_TOG = 1,
    DCC_F_M_T_MOM = 2,

    MIN_DCC_F_M_T = 0,
    MAX_DCC_F_M_T = 2
};

//----------------------------------------------------------------------------------------
// The cab entry is the central structure to describe a cab. It contains the engine
// address, its speed and direction, all the function settings and some other flags
// and data.
//
// ??? common declarations perhaps into LCS core lib include file...
// ??? this structure should rather be convenient for the local operation.
// ??? when we import/export the loco data, it will be packed into the 16-bit aligned
// format and bit fields.
//----------------------------------------------------------------------------------------
static const int     MAX_LOCO_NAME_SIZE   = 16;
static const int     MAX_CAB_FUNC_ENTRIES = MAX_DCC_F_M;
static const int     MAX_FUNC_STATE_SIZE  = 10;

struct CabEntry {

    public:

    void        reset( uint16_t cabId = NIL_CAB_ID );

    uint8_t     getSessionId( );
    void        setSessionId( uint8_t id );

    uint8_t     getSessionState( );
    void        setSessionState( uint8_t state );

    uint8_t     getCabId( );
    void        setCabId( uint16_t arg );

    uint8_t     getEngineType( );
    char        getEngineTypeChar( );
    void        setEngineType( uint8_t arg );

    bool        getDccFuncState( uint8_t fNum );
    void        setDccFuncState( uint8_t fNum, bool val );
    void        toggleDccFuncState( uint8_t fNum );

    uint8_t     getDccFuncIdForChFuncId( uint8_t cNum );
    void        setDccFuncIdForChFuncId( uint8_t cNum, uint8_t fNum );

    uint8_t     getDccFuncTypeForChFuncId( uint8_t cNum );
    void        setDccFuncTypeForChFuncId( uint8_t cNum, uint8_t typ );

    uint8_t     mapRnumFuncSetToDccFuncMapId( uint8_t rNum, uint8_t functionSet );

    uint8_t     getSpeed( );
    void        setSpeed( int speed );

    uint8_t     getDirection( );
    void        setDirection( uint8_t direction );

    uint8_t     getDataByItem( uint8_t item, uint16_t *arg );
    uint8_t     setDataByItem( uint8_t item, uint16_t arg );

    uint8_t     dccSpeedAndDirectionByte( );
    void        printCabEntry( );

    private:

    uint16_t    flags                                 = 0;

    uint8_t     sessionId                             = NIL_LOCO_SESSION_ID;
    uint8_t     sessionState                          = 0;

    uint16_t    cabId                                 = NIL_CAB_ID;
    uint8_t     engineType                            = LOC_T_NIL;
    uint8_t     speed                                 = 0;
    uint8_t     direction                             = 0;

    char        engineName[ MAX_LOCO_NAME_SIZE ]      = { 0 };
    uint8_t     dccFuncState[ MAX_FUNC_STATE_SIZE ]   = { 0 };
    uint8_t     cabFuncIdMap[ MAX_CAB_FUNC_ENTRIES ]  = { 0 };

    uint32_t    keepAliveTimer                        = 0;
};

//----------------------------------------------------------------------------------------
// A cab handheld can manage a set of cab entries. One entry is the active cab, also
// called the current cab. All operations are applied to the current cab. In addition
// there is stack of inactive cab entries. The cab stack slots are numbered from one
// to MAX. The cab entry stack is restored from the NVM at handheld start. Changes
// to the configuration of a cab entry are stored to the NVM.
//
//----------------------------------------------------------------------------------------
struct CabStack {

    public:

    CabStack( );

    void    loadCurrentCabFromSlot( int index );
    void    storeCurrentCabToSlot( int index );
    uint8_t loadCabSlotsFromNVM ( );
    uint8_t updateCabSlotInNVM ( int index );
    uint8_t getMaxEntries( );
    void    printCabSlots( );

    public:

    CabEntry currentCab;
    CabEntry *cabSlots = nullptr;
};

//----------------------------------------------------------------------------------------
// All cab handheld screens have a common screen layout. The screen is divided into
// a top line, which has room for a title and the labelling of the adjacent buttons
// MENU and UP. Likewise, there is a bottom line with room for some status flags and
// the label fields for SELECT and DOWN. In between are two lines in a larger font
// which are the screen content.
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
// There are methods to set these fields easy and straightforward. The top and bottom
// line are printed in an 8x8 font, the two main lines in a 8x16 font. Note that the
// display object expects rows and columns based on an 8-pixel raster. So, a 128x64
// screen has 16 columns and 8 rows. A font that takes two rows starts at the lower
// row of the two rows it occupies.
//
//----------------------------------------------------------------------------------------
struct CabHandheldScreen : UIScreen {

    protected:

    void printMenuLabel( char *str );
    void printMenuLabel( uint16_t textId );

    void printTitle( char *str );
    void printTitle( uint16_t textId  );

    void printUpLabel( char *str );
    void printUpLabel( uint16_t textId );

    void printSelectLabel( char *str );
    void printSelectLabel( uint16_t textId );

    void printDownLabel( char *str );
    void printDownLabel( uint16_t textId );
};


//----------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------
struct CabMsgBus {

    public:

    uint8_t sendSpeedAndDir( CabEntry *cab );
    uint8_t sendDccFuncVal( CabEntry *cab, uint8_t dccFundId );

    uint8_t sendEngineOnOff( CabEntry *cab );

    uint8_t requestLocoSession( CabEntry *cab );
    uint8_t closeLocoSession( CabEntry *cab );

    uint8_t loadCabData( CabEntry *cab );
};

//----------------------------------------------------------------------------------------
// The top menu screens. This is the top level screen list.  We show the menu text
// and select labels. The SELECT button gets us to the child list of the menu.
//
//----------------------------------------------------------------------------------------
struct TopMenuItemScreen : CabHandheldScreen {

    public:

    TopMenuItemScreen( uint8_t item );
    void enterScreen( bool init );

    private:

    uint16_t item = 0;
};

//----------------------------------------------------------------------------------------
// Some screens display a list of scrollable items. This class allows for a convenient
// constructions of such screens. The UP, DOWN buttons and also an optional encoder
// knob allow for faster scrolling functions. The "showScreenData" method must be 
// implemented to display the actual content. It is passed the actual value of the
// scrolling index.
//
//----------------------------------------------------------------------------------------
struct ScrollableScreen : CabHandheldScreen {

    public:

    ScrollableScreen( UIEncoder *encoder = nullptr );

    void          enterScreen( bool init );
    void          upButtonClick( UIButton *buttonObj );
    void          downButtonClick( UIButton *buttonObj );
    void          encoderPosChange( UIEncoder *encoderObj );
    int           getIndex( );
    virtual void  showScreenData( int index ) = 0;

    protected:

    UIEncoder *encoder    = nullptr;
    int       low         = INT_MIN;
    int       high        = INT_MAX;
    int       index       = 0;
};

//----------------------------------------------------------------------------------------
// "OperateScreen" is our main screen. It has all the relevant control elements and
// information items that are necessary to run the current loco. Most of the other 
// menus will return to this screen after their work.
//
//----------------------------------------------------------------------------------------
struct OperateScreen : CabHandheldScreen {

    public:

    void enterScreen( bool init );

    void buttonClick( UIButton *buttonObj );
    void upButtonClick( UIButton *buttonObj );
    void buttonLongPressStart( UIButton *buttonObj );
    void buttonLongPressStop( UIButton *buttonObj );
    void encoderPosChange( UIEncoder *encoderObj );
    void showCabData( );

    private:

    int  functionSet = 1;
};

//----------------------------------------------------------------------------------------
// Engine on and off screen. Especially a diesel engine needs to first turn its 
// prime mover on. And you cannot turn it off when the engine is not stopped.
//
//----------------------------------------------------------------------------------------
struct EngineOnOffScreen : CabHandheldScreen {

    public:

    void enterScreen( bool init );
    void upButtonClick( UIButton *buttonObj );
    void downButtonClick( UIButton *buttonObj );
};

//----------------------------------------------------------------------------------------
// Engine lights screen. An engine has a front and back section with lights.
//
//----------------------------------------------------------------------------------------
struct EngineLightsScreen : CabHandheldScreen {

    public:

    void enterScreen( bool init );
    void upButtonClick( UIButton *buttonObj );
    void downButtonClick( UIButton *buttonObj );
    void buttonClick( UIButton *buttonObj );
};

//----------------------------------------------------------------------------------------
// Select Cab Screen. There is a stack of cabs used before. We can scroll through 
// the list with UP/DOWN and the encoder knob. The SELECT button will make the entry
// shown the current loco. The MENU button click is disabled, so that we do not 
// enter this screen over and over. After selection, the selected loco will be 
// become the current loco.
//
//----------------------------------------------------------------------------------------
struct SelectCabScreen : ScrollableScreen {

    public:

    SelectCabScreen( UIEncoder *encoder = nullptr );

    void enterScreen( bool init );
    void menuButtonClick( UIButton *buttonObj );
    void selectButtonClick( UIButton *buttonObj );
    void showScreenData( int index );
};

//----------------------------------------------------------------------------------------
// Save cab Screen. The current cab can be saved to the cab stack. We can scroll 
// through the list with UP/DOWN and the encoder knob. The SELECT button will store 
// the current cab to the selected slot. If there was already a slot that contained
// this cab ID, it will be cleared. After operation, we return to the unchanged 
// current loco.
//
//----------------------------------------------------------------------------------------
struct SaveCabScreen : ScrollableScreen {

    public:

    SaveCabScreen( UIEncoder *encoder = nullptr );

    void enterScreen( bool init );
    void menuButtonClick( UIButton *buttonObj );
    void selectButtonClick( UIButton *buttonObj );
    void showScreenData( int index );
};

//----------------------------------------------------------------------------------------
// New Cab Screen. There needs to be a way to enter a new engine. We will display 4 
// digits among we can toggle with the MENU button. The UP/DOWN buttons advance the
// current digit position. The encoder knob offers a fast way to scroll a digit. 
// The high value digit allows to set an "S" instead of the number to indicate a 
// short loco DCC address. The SELECT button completes the number entering. We make
// the current cab this new loco. Note, that for keeping it in the stack it would
// need to be explicitly saved.
//
// ??? we could also get initial data from the base station if the cab is known
// there... tbd.
//----------------------------------------------------------------------------------------
struct NewCabScreen : CabHandheldScreen {

    public:

    NewCabScreen( UIEncoder *encoder = nullptr );

    void enterScreen( bool init );

    void menuButtonClick( UIButton *buttonObj );
    void upButtonClick( UIButton *buttonId );
    void downButtonClick( UIButton *buttonId );
    void encoderPosChange( UIEncoder *encoderObj );
    void selectButtonClick( UIButton *buttonId );

    private:

    uint16_t    buildCabId( );
    void        showCabId( );
    int         itemLimit( int index );

    UIEncoder   *encoder          = nullptr;
    uint8_t     itemIndex         = 3;
    uint8_t     items[ 5 ]        = { 0 };

    const char  *digitsToChar     = "0123456789S";
    const char  *engTypeToChar    = "-SDE";
};

//----------------------------------------------------------------------------------------
// Set function select screen. This menu will select a DCC function based on its 
// number F0 to F68. The UP/DOWN button advances the current digit. The Encoder 
// offers a fast way to scroll a function number. The SELECT button enters the
// setting phase, handled by the set function operate screen.
//
//----------------------------------------------------------------------------------------
struct SetFunctionSelectScreen : ScrollableScreen {

    public:

    SetFunctionSelectScreen( UIEncoder *encoder = nullptr );

    void  enterScreen( bool init );
    void  menuButtonClick(  UIButton *buttonObj );
    void  showScreenData( int index );
};


//----------------------------------------------------------------------------------------
// Set function operate screen. This screen uses the UP and DOWN buttons for setting
// the previously selected DCC function to ON or OFF. The MENU button gets us back
// to the operate screen.
//
//----------------------------------------------------------------------------------------
struct SetFunctionOperateScreen : CabHandheldScreen {

    public:

    void  enterScreen( bool init );
    void  menuButtonClick(  UIButton *buttonObj );
    void  upButtonClick( UIButton *buttonObj );
    void  downButtonClick( UIButton *buttonObj );

    private:

    int   functionId = -1;
    void  showScreenData( );
};

//----------------------------------------------------------------------------------------
// Config function select screen. The cab handheld UI elements such as the buttons
// HORN, BELL and Functions need to be mapped to their DCC function code for the
// particular engine. This screen will select the logical function to configure.
// On SELECT, we will enter the child screen, which will actually configure the 
// item.
//
//----------------------------------------------------------------------------------------
struct ConfigFunctionSelectScreen : ScrollableScreen {

    public:

    ConfigFunctionSelectScreen( UIEncoder *encoder = nullptr );

    void enterScreen( bool init );
    void menuButtonClick( UIButton *buttonObj );
    void showScreenData( int index );
};

//----------------------------------------------------------------------------------------
// Config function edit screen. This screen is the child of the config function 
// select screen, which has the index of the selected item. The edit screen uses
// the UP and DOWN buttons for the selection of the DCC function Id. The MENU 
// button toggles through the options that can be set and the SELECT button will 
// confirm the setting and we return to the Config Function Menu.
//
//----------------------------------------------------------------------------------------
struct ConfigFunctionEditScreen : ScrollableScreen {

    public:

    ConfigFunctionEditScreen( UIEncoder *encoder = nullptr );

    void enterScreen( bool init );
    void menuButtonClick( UIButton *buttonObj );
    void selectButtonClick( UIButton *buttonObj );
    void showScreenData( int index );

    private:

    uint8_t cabFuncId     = 0;
    uint8_t dccFuncId     = 0;
    uint8_t dccFuncOptId  = 0;
};

//----------------------------------------------------------------------------------------
// Test UI Elements. This menu is a very handy menu to test the individual UI
// elements for basic function. It is invoked from the DIAG menu. The UI tests use
// all buttons so, we can only return via the MENU button long press function to 
// the main menu. This screen is a child screen of the test UI elements screen 
// found in the DIAG menu.
//
//----------------------------------------------------------------------------------------
struct TestUIScreen : CabHandheldScreen {

    public:

    void enterScreen( bool init );
    void menuButtonClick( UIButton *buttonId );
    void selectButtonClick( UIButton *buttonId );
    void upButtonClick( UIButton *buttonId );
    void downButtonClick( UIButton *buttonId );
    void buttonClick( UIButton *buttonId );
    void buttonLongPressStart( UIButton *buttonObj );
    void buttonLongPressStop( UIButton *buttonObj );
    void encoderPosChange( UIEncoder *encoderObj );
};

#endif
