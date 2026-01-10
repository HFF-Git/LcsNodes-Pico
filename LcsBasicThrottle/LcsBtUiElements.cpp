//----------------------------------------------------------------------------------------
//
// LCS - Cab Handheld UI elements implementation file
//
//----------------------------------------------------------------------------------------
// This file implements the linkage between UIU elements and the screen subsystem.
// Each UI elements is created. Next, the callback function that the UI elements 
// call are connected to the static routines in the UIScreen class. This class 
// will dispatch a UI event to the current window. The other linkage to set up 
// is the callback for getting and setting data from a hardware resource.
//
//----------------------------------------------------------------------------------------
//
// LCS - Cab Handheld UI elements implementation file
// Copyright (C) 2020 - 2026  Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under 
// the terms of the GNU General Public License as published by the Free Software 
// Foundation, either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details. You 
// should have received a copy of the GNU General Public License along with this 
// program. If not, see <http://www.gnu.org/licenses/>.
//
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------
#include "LcsBasicThrottle.h"

using namespace CDC;

//----------------------------------------------------------------------------------------
// File local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

};

//----------------------------------------------------------------------------------------
// Global variables.
//
//----------------------------------------------------------------------------------------
UIDisplay       *oled                       = nullptr;

UIButton        *upButton                   = nullptr;
UIButton        *downButton                 = nullptr;
UIButton        *selectButton               = nullptr;
UIButton        *menuButton                 = nullptr;
UIButton        *f1Button                   = nullptr;
UIButton        *f2Button                   = nullptr;
UIButton        *f3Button                   = nullptr;
UIButton        *f4Button                   = nullptr;
UIButton        *bellButton                 = nullptr;
UIButton        *hornButton                 = nullptr;
UIButton        *fwdButton                  = nullptr;
UIButton        *revButton                  = nullptr;

UIEncoder       *encoder                    = nullptr;
UIButton        *encoderButton              = nullptr;

//----------------------------------------------------------------------------------------
// Configure the UI Resource Elements. The actual data used to configure is stored
// the CDC descriptor map.
//
//----------------------------------------------------------------------------------------
uint8_t configureUiElements( ) {

    configureDio( RNUM_MENU_BUTTON );
    configureDio( RNUM_SELECT_BUTTON );
    configureDio( RNUM_UP_BUTTON );
    configureDio( RNUM_DOWN_BUTTON );
    configureDio( RNUM_HORN_BUTTON );
    configureDio( RNUM_BELL_BUTTON );
    configureDio( RNUM_FWD_BUTTON );
    configureDio( RNUM_REV_BUTTON );
    configureDio( RNUM_F1_BUTTON );
    configureDio( RNUM_F2_BUTTON );
    configureDio( RNUM_F3_BUTTON );
    configureDio( RNUM_F4_BUTTON );
    configureDio( RNUM_ENCODER_BUTTON );
    configureDio( RNUM_ENCODER_KNOB );
    return ( LCS_OK );
}

//----------------------------------------------------------------------------------------
// "getData" and getDataPair are the interfaces for the UI elements to read in the 
// state of buttons and encoders. The routines are passed the resource number of 
// the UI element.  
//
//----------------------------------------------------------------------------------------
void getData( uint8_t rNum, bool *val ) {

    readDio( rNum, val );
}

void getDataPair( uint8_t rNum, bool *valA, bool *valB ) {

    // ??? what value do we actually return ? active low ?
    // ??? remove comment after test ...

    readDio( rNum, valA, valB );
}

//----------------------------------------------------------------------------------------
// Create the Buttons and the Encoder objects. We also attach to each UI element
// the data retrieval function.
//
//----------------------------------------------------------------------------------------
uint8_t createUIElements( ) {

    upButton      = new UIButton( RNUM_UP_BUTTON );
    downButton    = new UIButton( RNUM_DOWN_BUTTON );
    selectButton  = new UIButton( RNUM_SELECT_BUTTON );
    menuButton    = new UIButton( RNUM_MENU_BUTTON );
    f1Button      = new UIButton( RNUM_F1_BUTTON );
    f2Button      = new UIButton( RNUM_F2_BUTTON );
    f3Button      = new UIButton( RNUM_F3_BUTTON );
    f4Button      = new UIButton( RNUM_F4_BUTTON );
    bellButton    = new UIButton( RNUM_BELL_BUTTON );
    hornButton    = new UIButton( RNUM_HORN_BUTTON );
    fwdButton     = new UIButton( RNUM_FWD_BUTTON );
    revButton     = new UIButton( RNUM_REV_BUTTON );

    encoderButton = new UIButton( RNUM_ENCODER_BUTTON );

    // ??? active low not taken from desc map ?
    encoder       = new UIEncoder( RNUM_ENCODER_KNOB, -10, 10, false  );

    menuButton ->     attachGetDataFunction( getData );
    selectButton ->   attachGetDataFunction( getData );
    upButton ->       attachGetDataFunction( getData );
    downButton ->     attachGetDataFunction( getData );

    hornButton ->     attachGetDataFunction( getData );
    bellButton ->     attachGetDataFunction( getData );
    fwdButton ->      attachGetDataFunction( getData );
    revButton ->      attachGetDataFunction( getData );

    encoder ->        attachGetDataFunction( getDataPair );
    encoderButton ->  attachGetDataFunction( getData );

    f1Button ->       attachGetDataFunction( getData );
    f2Button ->       attachGetDataFunction( getData );
    f3Button ->       attachGetDataFunction( getData );
    f4Button ->       attachGetDataFunction( getData );

    oled = new UIDisplayOledI2C( DT_OLED_DISPLAY_128x64, CDC_RN_EXT_NVM, 0x3C );

    return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// "LinkScreens" will link the UI elements to the screen class static functions 
// that will pass the respective UI element event to the current screen. So, for
// example, a button click will be passed to the static function in the screen 
// class, which in turn forwards it to the current screen, or handle it directly.
// When developing screen objects, all UI elements that you want to react to need
// to implement the handlers for the incoming events.
//
//----------------------------------------------------------------------------------------
uint8_t linkScreens( ) {

  menuButton -> attachLongPressStart( UIScreen::menuButtonLongPressHandler );
  menuButton -> attachClick( UIScreen::menuButtonClickHandler );
  selectButton -> attachClick( UIScreen::selectButtonClickHandler );
  upButton -> attachClick( UIScreen::upButtonClickHandler );
  downButton -> attachClick( UIScreen::downButtonClickHandler );

  hornButton -> attachClick( UIScreen::buttonClickHandler );
  hornButton -> attachLongPressStart( UIScreen::buttonLongPressStartHandler );
  hornButton -> attachLongPressStop( UIScreen::buttonLongPressStopHandler );

  bellButton -> attachClick( UIScreen::buttonClickHandler );
  bellButton -> attachLongPressStart( UIScreen::buttonLongPressStartHandler );
  bellButton -> attachLongPressStop( UIScreen::buttonLongPressStopHandler );

  fwdButton -> attachLongPressStart( UIScreen::buttonLongPressStartHandler );
  revButton -> attachLongPressStart( UIScreen::buttonLongPressStartHandler );

  f1Button -> attachClick( UIScreen::buttonClickHandler );
  f1Button -> attachLongPressStart( UIScreen::buttonLongPressStartHandler );
  f1Button -> attachLongPressStop( UIScreen::buttonLongPressStopHandler );

  f2Button -> attachClick( UIScreen::buttonClickHandler );
  f2Button -> attachLongPressStart( UIScreen::buttonLongPressStartHandler );
  f2Button -> attachLongPressStop( UIScreen::buttonLongPressStopHandler );

  f3Button -> attachClick( UIScreen::buttonClickHandler );
  f3Button -> attachLongPressStart( UIScreen::buttonLongPressStartHandler );
  f3Button -> attachLongPressStop( UIScreen::buttonLongPressStopHandler );

  f4Button -> attachClick( UIScreen::buttonClickHandler );
  f4Button -> attachLongPressStart( UIScreen::buttonLongPressStartHandler );
  f4Button -> attachLongPressStop( UIScreen::buttonLongPressStopHandler );

  encoderButton -> attachClick( UIScreen::buttonClickHandler );
  encoder -> attachPositionChanged( UIScreen::encoderPosChangeHandler );

  return ( NO_ERR );
}

//----------------------------------------------------------------------------------------
// Set up the UI Elements.
//
//----------------------------------------------------------------------------------------
uint8_t setupUIElements( ) {

  uint8_t rStat = createUIElements( );
  if ( rStat == NO_ERR ) rStat = configureUiElements( );
  if ( rStat == NO_ERR ) rStat = linkScreens( );

  return ( rStat );
}
