//------------------------------------------------------------------------------------------------------------
//
// LCS - Cab Handheld UI elements implementation file
//
//------------------------------------------------------------------------------------------------------------
// ???


// Note that the ATmega version uses an I2C expander. The RPico version has all the UI elements directly
// connected. We will for now just have "defines" ( sigh ) to separate them throughout the code.
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Cab Handheld UI elements implementation file
// Copyright (C) 2019 - 2024  Helmut Fieres
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
#include "LcsBasicThrottle.h"

using namespace LCS;


//------------------------------------------------------------------------------------------------------------
// File local declarations.
//
//------------------------------------------------------------------------------------------------------------
namespace {

bool isInRange( unsigned int val, unsigned int lower, unsigned int upper ) {

    return (( val >= lower ) && ( val <= upper ));
}

};

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
#define DISPLAY_I2C_ADR 0x3C

//------------------------------------------------------------------------------------------------------------
// Global variables.
//
//------------------------------------------------------------------------------------------------------------
UIDisplay     *oled                       = nullptr;

UIButton      *upButton                   = nullptr;
UIButton      *downButton                 = nullptr;
UIButton      *selectButton               = nullptr;
UIButton      *menuButton                 = nullptr;
UIButton      *f1Button                   = nullptr;
UIButton      *f2Button                   = nullptr;
UIButton      *f3Button                   = nullptr;
UIButton      *f4Button                   = nullptr;
UIButton      *bellButton                 = nullptr;
UIButton      *hornButton                 = nullptr;
UIButton      *fwdButton                  = nullptr;
UIButton      *revButton                  = nullptr;

UIEncoder     *encoder                    = nullptr;
UIButton      *encoderButton              = nullptr;

//------------------------------------------------------------------------------------------------------------
// Configure the IO pins for the UI Elements. Note that there are two version for now. The Atmega version uses
// a main controller board and an extension board. All UI Elements are connected via the I2C channel zero. The
// extension board has an I2C Expander and all UI Elements are connected to the ports of this chip. The RPICO
// version is a monolithic board and all UI Elements are connected to the PICO itself. The PICO also has an
// LcsNode NVM which is connected via I2C channel 0. The UI Display elements is connected to the I2C channel 1.
//
// ?? over time, the ATmega version will go away. But for now ...
//------------------------------------------------------------------------------------------------------------
uint8_t setupIOPins( ) {

    CDC::configureDio( UP_BUTTON_ID, CDC::IN_PULLUP );

    CDC::configureDio( MENU_BUTTON_ID, CDC::IN_PULLUP );
    CDC::configureDio( SELECT_BUTTON_ID, CDC::IN_PULLUP );
    CDC::configureDio( UP_BUTTON_ID, CDC::IN_PULLUP );
    CDC::configureDio( DOWN_BUTTON_ID, CDC::IN_PULLUP );
    CDC::configureDio( HORN_BUTTON_ID, CDC::IN_PULLUP );
    CDC::configureDio( BELL_BUTTON_ID, CDC::IN_PULLUP );
    CDC::configureDio( FWD_BUTTON_ID, CDC::IN_PULLUP );
    CDC::configureDio( REV_BUTTON_ID, CDC::IN_PULLUP );
    CDC::configureDio( F1_BUTTON_ID, CDC::IN_PULLUP );
    CDC::configureDio( F2_BUTTON_ID, CDC::IN_PULLUP );
    CDC::configureDio( F3_BUTTON_ID, CDC::IN_PULLUP );
    CDC::configureDio( F4_BUTTON_ID, CDC::IN_PULLUP );
    CDC::configureDio( ENCODER_BUTTON_ID, CDC::IN_PULLUP );

    CDC::configureDio( ENCODER_ID_A, CDC::IN_PULLUP );
    CDC::configureDio( ENCODER_ID_B, CDC::IN_PULLUP );

    return ( LCS::ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// "getData" is the interface for the UI elements to read in the state of buttons and encoders. It uses the
// hardware resource ID stored with an UI Element. The interpretation is up to the function. For example,
// when we have direct pins, then it is the pin number on the controller chip, when the UI element is
// connected via an I2C expander or a shift register, it is the position on the chip.
//
//------------------------------------------------------------------------------------------------------------
bool getData( uint8_t hwId ) {

    #if 0
    printf( "Get data: %d -> %d\n", hwId, CDC::readDio( hwId ) == HIGH );
    #endif

    if (( hwId == ENCODER_ID_A ) || ( hwId == ENCODER_ID_B ))
        return ( CDC::readDio( hwId ) == true );
    else
        return ( CDC::readDio( hwId ) == false );

}

//------------------------------------------------------------------------------------------------------------
// Create the Buttons and the Encoder objects. We also attached to each UI element the data retrieval
// function.
//
//------------------------------------------------------------------------------------------------------------
uint8_t createUIElements( ) {

  upButton      = new UIButton( UP_BUTTON_ID );
  downButton    = new UIButton( DOWN_BUTTON_ID );
  selectButton  = new UIButton( SELECT_BUTTON_ID );
  menuButton    = new UIButton( MENU_BUTTON_ID );
  f1Button      = new UIButton( F1_BUTTON_ID );
  f2Button      = new UIButton( F2_BUTTON_ID );
  f3Button      = new UIButton( F3_BUTTON_ID );
  f4Button      = new UIButton( F4_BUTTON_ID );
  bellButton    = new UIButton( BELL_BUTTON_ID );
  hornButton    = new UIButton( HORN_BUTTON_ID );
  fwdButton     = new UIButton( FWD_BUTTON_ID );
  revButton     = new UIButton( REV_BUTTON_ID );

  encoderButton = new UIButton( ENCODER_BUTTON_ID );
  encoder       = new UIEncoder( ENCODER_ID_A, ENCODER_ID_B, -10, 10, false  );

  menuButton ->     attachGetDataFunction( getData );
  selectButton ->   attachGetDataFunction( getData );
  upButton ->       attachGetDataFunction( getData );
  downButton ->     attachGetDataFunction( getData );

  hornButton ->     attachGetDataFunction( getData );
  bellButton ->     attachGetDataFunction( getData );
  fwdButton ->      attachGetDataFunction( getData );
  revButton ->      attachGetDataFunction( getData );

  encoder ->        attachGetDataFunction( getData );
  encoderButton ->  attachGetDataFunction( getData );

  f1Button ->       attachGetDataFunction( getData );
  f2Button ->       attachGetDataFunction( getData );
  f3Button ->       attachGetDataFunction( getData );
  f4Button ->       attachGetDataFunction( getData );

  hornButton ->     setResId( DCC_F_M_HORN );
  bellButton ->     setResId( DCC_F_M_BELL );
  f1Button ->       setResId( DCC_F_M_F1 );
  f2Button ->       setResId( DCC_F_M_F2 );
  f3Button ->       setResId( DCC_F_M_F3 );
  f4Button ->       setResId( DCC_F_M_F4 );
  encoderButton ->  setResId( DCC_F_M_ENC_BTN );


/*
  oled = new UIDisplayOledSSD1306 ( DT_OLED_DISPLAY_128x64_2F_4,
                                    cfg.I2C_SCL_PIN_0,
                                    cfg.I2C_SDA_PIN_0,
                                    cfg.I2C_ADR_0 );
*/

  return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// "LinkScreens" will link the button and encoder UI elements to the screen class static functions that will
// pass the respective UI element event to the current screen. So, for example, a button click will be passed
// to the static function in the screen class, which in turn forwards it to the current screen, or handle it
// directly. When writing a screen object, all UI elements that you want to react to need to implement the
// handlers for the incoming events.
//
//------------------------------------------------------------------------------------------------------------
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

  return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
// Create the Buttons and the Encoder objects. We also attach to each UI element the data retrieval function.
// This function will differ for a set of UI Elements directly connected to controller GPIO pins versus UI
// Elements connected to an I2C Expander.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupUIElements( ) {

  uint8_t rStat = createUIElements( );
  if ( rStat == ALL_OK ) rStat = setupIOPins( );
  if ( rStat == ALL_OK ) rStat = linkScreens( );

  return ( rStat );
}
