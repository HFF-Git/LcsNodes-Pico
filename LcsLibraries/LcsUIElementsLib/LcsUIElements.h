//------------------------------------------------------------------------------------------------------------
//
// UI Elements - include file.
//
//------------------------------------------------------------------------------------------------------------
// UI Elements. You start an Arduino project and the first Led is blinking, a button is pushed. Before you
// know it, buttons need to be debounced, short and long pressed, active high or active low. You would like
// to toggle a Led and remember its state. There are displays with different interfaces and capabilties. On
// some displays you want to control the brigthness and contrast. Finally, in some projects you run out of
// physical pins and want to connect an array of buttons or Leds through somethig like a parallel IO extender
// or a simple shift registers. The list is long. In all Arduino projects you implement it somehow directly
// in project just to take part of the functions to the next project and so on.
//
// UI Elements is the library the implements the most common UI elements. This file includes all the class
// definitions. UIElements is the base class and also maintains a linked list of all created objects. This
// list is used when the "tick" function is called to advance the state machines in the relevant objects.
//
// UITimer
// UILed
// UIButton
// UIEncoder
//
// UIDisplay
// UIDisplayLcdI2C
// UIDisplayLcdP
// UIDisplayOledSSD1306
//
// UIScreen
//
//------------------------------------------------------------------------------------------------------------
// Some of the UI Element classes were inspired by the work of "Matthias Hertel", here is his copyright notice.
// I like his approach for handling events with a finite state machine very much. The button, button array,
// encoders and Leds are all managed by a state machine that advances with a call to the function "tick".
//
// ( Original state machine->  Copyright (c) by Matthias Hertel, https://www.mathertel.de. )
//------------------------------------------------------------------------------------------------------------
//
// UI Elements
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
#ifndef UI_ELEMENTS_h
#define UI_ELEMENTS_h

/*
//------------------------------------------------------------------------------------------------------------
// Arduino and common include files.
//------------------------------------------------------------------------------------------------------------
#include <limits.h>
#include <arduino.h>
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>
#include "SSD1306AsciiWire.h"
*/

//------------------------------------------------------------------------------------------------------------
// Include files.
//
//------------------------------------------------------------------------------------------------------------
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <limits.h>

#include "LcsCdcLib.h"
#include "LcsLcdDisplayLib.h"
#include "LcsOledDisplayLib.h"

//------------------------------------------------------------------------------------------------------------
// Resources and Pins use the value of 255 to indicate an invalid number.
//
//------------------------------------------------------------------------------------------------------------
const uint8_t INVALID_ID       = 255;
const uint8_t INVALID_PIN      = 255;

//------------------------------------------------------------------------------------------------------------
// There are quite a few displays to support. While the LCD displays just feature a fixed column and row size,
// the Oled displays support a column and row size that depends on the font used. 

// For simplicity, we will only support a few fonts. There is an 8x8 pixel font and a 8x16 font. 
// The configured size is encoded in column / row numbers at the end of the type of display.
//
// ??? perhaps rethink this one. We could always think in units of 8 pixels and do the math what to say for
// row and column at caller level.
//------------------------------------------------------------------------------------------------------------
enum DisplayType : uint8_t {

  // ??? simplify, default for 128x64 is 8Px font, a parameter create can set the font... ?

  DT_LCD_DISPLAY_16_2           = 1,
  DT_LCD_DISPLAY_20_4           = 2,
  DT_LCD_DISPLAY_128_32         = 3,
  DT_LCD_DISPLAY_128_64         = 4,


  DT_OLED_DISPLAY_128x32_16_4   = 5,
  DT_OLED_DISPLAY_128x32_8_2    = 6,
  DT_OLED_DISPLAY_128x64_16_8   = 7,
  DT_OLED_DISPLAY_128x64_16_4   = 8,
  DT_OLED_DISPLAY_128x64_2F_4   = 9   // ??? why do we need this one ?
};


//------------------------------------------------------------------------------------------------------------
// OLED displays feature a set of fonts. A small set of all possible fonts is available for the OLED display.
// The font type is meaningless for thw LCD displays, they have only one character set.
//
//------------------------------------------------------------------------------------------------------------
enum FontType : uint8_t {

  FT_DEF   = 0,
  FT_5x7   = 1,
  FT_8x8   = 2,
  FT_8x16  = 3,
  FT_10x16 = 4,
};

//------------------------------------------------------------------------------------------------------------
// Callback function definition. UI Elements implement two kinds of callback functions. The first group is the
// data setting and retrieval function, which is used by buttons, LEDs and encoders to work with the hardware
// elements that reprsent these objects. UI elements that process events additionally implement the second
// type callback function mechanism to inform the client on the event that occured. For example, when a button
// is pushed and has registered a callback function, this is the function signature invoked.
//------------------------------------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

typedef void (*UITimerCallBackFunction) ( struct UITimer *timerObj );
typedef void (*UIButtonCallBackFunction) ( struct UIButton *buttonObj );
typedef void (*UIEncoderCallBackFunction) ( struct UIEncoder *encoderObj );
typedef void (*UISetDataFunction) ( uint8_t hwId, bool val );
typedef bool (*UIGetDataFunction) ( uint8_t hwId );

#ifdef __cplusplus
}
#endif

//------------------------------------------------------------------------------------------------------------
// The UIElements class. This is the base class for all UI elements. There are two static functions, "setup"
// and "tick", which typically are called in the Arduino setup and loop phase. Especially the tick function
// should be called very often, as it advances the state machine in each UI element via "processTick". The UI
// elements themselves are added to a linked list so that we can process all elements created. This class
// cannot be instantiated, only the subclasses can. Each UI Element features also a resource ID to keep a 
// use case specific ID.
//
//------------------------------------------------------------------------------------------------------------
struct UIElements {

  public:

    int               getResId( );
    void              setResId( int arg );
    
  public:

    static uint8_t    setup( );
    static void       tick( );

  protected:

    UIElements( bool  atHead = false );

    virtual void      processTick( )  = 0;

  private:

    UIElements*       next  = nullptr;
    int               resId = 0;

    static void       append( UIElements* res );
    static void       insert( UIElements* res );
};

//------------------------------------------------------------------------------------------------------------
// "UITimer" definitions.
//
//------------------------------------------------------------------------------------------------------------
struct UITimer : UIElements {

  public:

    UITimer( );

    void setTimer( uint32_t val );
    void attachTimer( UITimerCallBackFunction functionId );

  private:

    void                      processTick( );

    bool                      timerEnabled    = true;
    uint32_t                  timerInterval   = 0L;
    uint32_t                  startTimerVal   = 0L;
    UITimerCallBackFunction   timerCallback   = nullptr;
};

//------------------------------------------------------------------------------------------------------------
// "UIButton" class definition.
//
//------------------------------------------------------------------------------------------------------------
struct UIButton : UIElements {

  public:

    UIButton( uint8_t hwId, bool activeLow = false );

    void                      attachClick( UIButtonCallBackFunction functionId );
    void                      attachDoubleClick( UIButtonCallBackFunction functionId );
    void                      attachLongPressStart( UIButtonCallBackFunction functionId ) ;
    void                      attachLongPressStop( UIButtonCallBackFunction functionId );
    void                      attachDuringLongPress( UIButtonCallBackFunction functionId );
    void                      attachGetDataFunction( UIGetDataFunction functionId );

    void                      setActiveLow( bool val );
    bool                      isLongPressed( );
    uint32_t                  getDurationPressedMillis( );
    uint32_t                  getPressedMillisSinceStart( );
    uint8_t                   getHwId( );

    void                      reset( );
    void                      processTick( );

  private:

    uint8_t                   hwId                  = INVALID_PIN;
    uint8_t                   buttonState           = 0;
    uint32_t                  startTime             = 0;
    uint32_t                  stopTime              = 0;

    UIButtonCallBackFunction  clickFunc             = nullptr;
    UIButtonCallBackFunction  doubleClickFunc       = nullptr;
    UIButtonCallBackFunction  longPressStartFunc    = nullptr;
    UIButtonCallBackFunction  longPressStopFunc     = nullptr;
    UIButtonCallBackFunction  duringLongPressFunc   = nullptr;
    UIGetDataFunction         getDataFunc           = nullptr;

  public:

    static void               setDebounceMillis( uint32_t ticks );
    static void               setClickMillis( uint32_t ticks );
    static void               setPressMillis( uint32_t ticks );
};

//------------------------------------------------------------------------------------------------------------
// "UIEncoder" class definition.
//
//------------------------------------------------------------------------------------------------------------
struct UIEncoder : UIElements {

  public:

    UIEncoder( uint8_t hwIdA, uint8_t hwIdB, int lower = INT_MIN, int upper = INT_MAX, bool activeLow = false );

    void                        reset( );
    void                        processTick( void );

    void                        setLimits( int lower, int upper );
    int                         getLowerLimit( );
    int                         getUpperLimit( );
    int                         getPosition( );
    void                        setPosition( int newPosition, bool supressCallback = false );
    uint32_t                    getMillisBetweenRotations( );
    void                        attachPositionChanged( UIEncoderCallBackFunction functionId );
    void                        attachGetDataFunction( UIGetDataFunction functionId );
    uint8_t                     getHwIdA( );
    uint8_t                     getHwIdB( );

  private:

    uint8_t                     hwIdA               = INVALID_PIN;
    uint8_t                     hwIdB               = INVALID_PIN;

    bool                        idAVal              = false;
    bool                        idBVal              = false;

    bool                        activeLow           = false;
    bool                        oldState            = false;
    int                         position            = 0;
    int                         positionPrev        = 0;
    int                         upperLimit          = INT_MAX;
    int                         lowerLimit          = INT_MIN;
    uint32_t                    positionTime        = 0;
    uint32_t                    positionTimePrev    = 0;

    UIEncoderCallBackFunction   positionChangedFunc = nullptr;
    UIGetDataFunction           getDataFunc         = nullptr;
};

//------------------------------------------------------------------------------------------------------------
// "UILed" class definition.
//
//------------------------------------------------------------------------------------------------------------
struct UILed : UIElements {

  public:

    UILed( uint8_t hwId );

    void processTick( );
    void attachSetDataFunction( UISetDataFunction functionId );

    bool isOn( );
    bool isOff( );
    void setOn( );
    void setOff( );
    void setVal( bool val );
    void toggle( );
    void blink( );

  private:

    uint8_t             hwId         = INVALID_PIN;
    bool                ledOn         = false;
    bool                ledBlink      = false;
    uint32_t            lastChange    = 0;
    UISetDataFunction   setDataFunc   = nullptr;

  public:

    static void setBlinkIntervalMillis( uint32_t val );
};

//------------------------------------------------------------------------------------------------------------
// The "UIDisplay" object is the common subset object for all displays. It implements a simple row x column
// ASCII display. Although some display are far more capable, this simple display type will often do. All
// further capabilities of an actual display are not masked and can be used. However, the code is then display
// specific.
//
//------------------------------------------------------------------------------------------------------------
struct UIDisplay : UIElements {

    public:

    UIDisplay( uint8_t dType );

    virtual void    setCursor( uint8_t col, uint8_t row )   = 0;
    virtual void    setFont( uint8_t fontId )               = 0;
    virtual uint8_t print( const char *s )                  = 0;
    virtual uint8_t print( char ch )                        = 0;
    virtual void    clear(  )                               = 0;
    virtual void    clearLine( uint8_t row )                = 0;

    protected:

    uint8_t         maxColumns = 0;
    uint8_t         maxRows    = 0;
    void            processTick( );
};

//------------------------------------------------------------------------------------------------------------
// The LCD display and an I2C interface are handled by this object. Like all displays defined, this object
// implements a simple matrix of ASCII characters. The display has a set of function to manage backlight,
// as well as cursor and blinking options. We simply inherit these functions. They are however only an
// option for the LCD kind of display.
//
//------------------------------------------------------------------------------------------------------------
struct UIDisplayLcdI2C : public UIDisplay {

  public:

    UIDisplayLcdI2C( uint8_t dType, uint8_t sclPin, uint8_t sdaPin, uint8_t I2CAddress = 0x27 );

    void    setCursor( uint8_t col, uint8_t row );
    uint8_t print( const char *s );
    uint8_t print( char ch );
    void    clear( );

  private: 

  LcsLcdDisplay *lcd = nullptr;

};

//------------------------------------------------------------------------------------------------------------
// The "UIDisplayOled" manages an OLED display as a matrix of row * columns Adcii characters. Although an
// OLED display is fully graphical, we just use them for now as an ASCII display with a small set of fonts
// and a row by column matrix size.
//
//------------------------------------------------------------------------------------------------------------
struct UIDisplayOled : public UIDisplay {

    public:

    UIDisplayOled( uint8_t dType, uint8_t sclPin, uint8_t sdaPin, uint8_t I2cAddress = 0x3C );

    void    setCursor( uint8_t col, uint8_t row );
    void    setFont( uint8_t fontId );
    uint8_t print( const char *s );
    uint8_t print( char ch );
    void    clear( );
    void    clearLine( uint8_t row );

    private:

    LcsOledDisplay *oled = nullptr;

};

//------------------------------------------------------------------------------------------------------------
// The "UIScreen" is the central object for screens. A screen is just an array of rows and columns of ASCII
// characters. The object contains pointers to the parent screen, the next screen at that level and a pointer
// to an optional child list. Screen hierarchies are built by appending a screen to another screens child
// list. The UIScreen class provides callbacks for for handling UIElement events, which can be overriden to
// implement screen specific actions. The "menu" and "select" will as default handler have the menu
// navigation function, all other UIElements just end in a dummy function to be overriden if needed. A menu
// button toggles through the child list, the select button selects the first child of the current menu as
// next screen, if available. The "enterScreen", "exitScreen" methods are cab be overriden to provide entry
// and exit processing, such as showing the initial screen content or cleaning up data when leaving a sccreen.
//
// UIElement actions from buttons and encoders are passed to the active screen via the the respective element
// callback. There are static class functions that are registered at the UIElements and when invoked pass the
// data to the current screen. If UIScreen class contains dummy functions to avoid implmenting dummy functions
// in the derived screen classes. Also, when overriding the menu and select button in a derived class, the
// enter and exit screen invocation must be handled by the overiding procedure.
//
// ??? we may have to add the process tick mechanism so that we can implement timestamp based processing...
//------------------------------------------------------------------------------------------------------------
struct UIScreen {

  public:

    UIScreen( );

    void            append( UIScreen *screen );

    virtual void    enterScreen( bool init );
    virtual void    exitScreen( );

    virtual void    menuButtonClick( UIButton *buttonId );
    virtual void    menuButtonLongPress( UIButton *buttonId );
    virtual void    selectButtonClick( UIButton *buttonId );
    virtual void    upButtonClick( UIButton *buttonId );
    virtual void    downButtonClick( UIButton *buttonId );

    virtual void    buttonClick( UIButton *buttonId );
    virtual void    buttonLongPressStart( UIButton *buttonId );
    virtual void    buttonLongPressStop( UIButton *buttonId );
    virtual void    encoderPosChange( UIEncoder *encoderObj );

    UIScreen        *getParentScreen( );
    UIScreen        *getChildScreen( );
    void            enableScreen( );
    void            disableScreen( );
    bool            isEnabled( );

  private:

    UIScreen        *parent   = nullptr;
    UIScreen        *next     = nullptr;
    UIScreen        *child    = nullptr;
    bool            enabled   = true;

  public:

    static void     menuButtonClickHandler( UIButton *buttonObj );
    static void     menuButtonLongPressHandler( UIButton *buttonObj );
    static void     selectButtonClickHandler( UIButton *buttonObj );
    static void     upButtonClickHandler( UIButton *buttonObj );
    static void     downButtonClickHandler( UIButton *buttonObj );

    static void     buttonClickHandler( UIButton *buttonObj );
    static void     buttonLongPressStartHandler( UIButton *buttonObj );
    static void     buttonLongPressStopHandler( UIButton *buttonObj );
    static void     encoderPosChangeHandler( UIEncoder *encoderObj );

    static bool     setup( );
    static UIScreen *getRootScreen( );
    static UIScreen *getCurrentScreen( );
    static UIScreen *setCurrentScreen( UIScreen *screen, bool init = true );
};

#endif
