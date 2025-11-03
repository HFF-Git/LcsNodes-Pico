//----------------------------------------------------------------------------------------
//
// UI Screen - implementation file
//
//----------------------------------------------------------------------------------------
// UI Elements of Buttons, Knobs, LEDs and displays are the atoms of a User Interface. 
// The next level is to organize these atoms into screens. A screen is a display 
// UI element that is manipulated with the help of architected UI button elements.
//
// The UIScreen object is the class for implementing a screen. Screens are a display
// of whatever you want to put on that display. There are virtual methods that will
// be called when for example a button event such a click is encountered or any other
// data needs to be written to the display. Two buttons are needed for navigation. 
// They are the MENU and SELECT button. With the exception of the root screen, a 
// screen has a parent and a potential child list. The hierarchy is formed by 
// appending a screen to another screen child list. The MENU button is typically
// used to scroll through the child list. A long press of the menu button always 
// gets back to the root screen's first child, no matter where you are. Child lists
// are toggled through with the menu button in a circular fashion. From the last
// screen on the child list, the next toggle gets us to the first child. The SELCET 
// button selects the current screen and triggers an action.
//
// To keep the screen hierarchy flexible, the meaning of the buttons within a given
// screen is not fixed. Inside a screen the menu and select buttons an change their
// meaning, with the exception of the long press of the MENU button.  As a convention,
// the menu button should be used to toggle through screens, the select button is 
// used to enter a screen child list and as a kind of commit button. Quite common are
// also the up / down buttons, which manipulate screen content, such as incrementing
// a number on the screen. But they are not mandatory compared to the menu and select 
// button.
//
// The connection from button events to the screens is handled by registering the 
// static screen callback functions with the respective UI elements. For example, 
// the screen MENU button handler is registered with the MENU button object. Pressing
// that button will invoke the MENU button handler in the screen. The screen object
// will pass all UI Elements events for which the callback to the current screen. 
// For example, the UIScreen base class has the MENU and SELECT button event handler
// to navigate through the screens. A screen that inherits from the UIScreen could 
// overwrite these handlers and assign a new meaning to the MENU and SELECT button. 
// That's it.
//
//----------------------------------------------------------------------------------------
//
// UI Screen
// Copyright (C) 2019 - 2025  Helmut Fieres
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
//----------------------------------------------------------------------------------------
#include "LcsUIElements.h"

//----------------------------------------------------------------------------------------
// DEBUG_SCREEN_CALLBACK is used to show the callbacks invoked from the UI Elements 
// for which object.
//
//----------------------------------------------------------------------------------------
#define DEBUG_SCREEN_CALLBACK 0
#define DEBUG_SKIP_SCREENS    0

//----------------------------------------------------------------------------------------
// Local declarations.
//
//----------------------------------------------------------------------------------------
namespace {

  //--------------------------------------------------------------------------------------
  // The root screen is the top of the menu hierarchy. It is the screen to which a
  // long press of the menu button always will return to. The currentScreen is the
  // actual screen and all button events with the exception of the menu button long 
  // press will refer to this screen.
  //
  //--------------------------------------------------------------------------------------
  static UIScreen   *rootScreen       = new UIScreen( );
  static UIScreen   *currentScreen    = nullptr;

} // namespace

//----------------------------------------------------------------------------------------
// The following static routines are the connection points from the UI Elements to
// the screen. For example, a button event such as a click, will be passed to the
// screen methods of the current screen. When the UI Element, e.g. a button, is 
// configured, the callback function to register for the button event is the 
// corresponding static handler routine. For each UI element type used on a screen 
// there is a static function that will  route the event to the current screen 
// handler methods. So far, there are the button and encoder callbacks.
//
//----------------------------------------------------------------------------------------
void UIScreen::menuButtonClickHandler( UIButton *buttonObj ) {

    #if DEBUG_SCREEN_CALLBACK == 1
    Serial.print( "CurrentScreen: " );
    Serial.println((uint32_t ) currentScreen );
    Serial.print( "menuButtonCLickHandler: " );
    Serial.println( buttonObj -> getHwId( ));
    #endif

    if ( currentScreen != nullptr ) 
        currentScreen -> menuButtonClick( buttonObj );
}

void UIScreen::selectButtonClickHandler( UIButton *buttonObj ) {

    #if DEBUG_SCREEN_CALLBACK == 1
    Serial.print( "CurrentScreen: " );
    Serial.println((uint32_t ) currentScreen );
    Serial.print( "selectButtonCLickHandler: " );
    Serial.println( buttonObj -> getHwId( ));
    #endif

    if ( currentScreen != nullptr ) 
        currentScreen -> selectButtonClick( buttonObj );
}

void UIScreen::upButtonClickHandler( UIButton *buttonObj ) {

  #if DEBUG_SCREEN_CALLBACK == 1
  Serial.print( "CurrentScreen: " );
  Serial.println((uint32_t ) currentScreen );
  Serial.print( "upButtonCLickHandler: " );
  Serial.println( buttonObj -> getHwId( ));
  #endif

  if ( currentScreen != nullptr ) currentScreen -> upButtonClick( buttonObj );
}

void UIScreen::downButtonClickHandler( UIButton *buttonObj ) {

  #if DEBUG_SCREEN_CALLBACK == 1
  Serial.print( "CurrentScreen: " );
  Serial.println((uint32_t ) currentScreen );
  Serial.print( "downButtonCLickHandler: " );
  Serial.println( buttonObj -> getHwId( ));
  #endif

  if ( currentScreen != nullptr ) currentScreen -> downButtonClick( buttonObj );
}

void UIScreen::menuButtonLongPressHandler( UIButton *buttonObj ) {

    #if DEBUG_SCREEN_CALLBACK == 1
    Serial.print( "CurrentScreen: " );
    Serial.println((uint32_t ) currentScreen );
    Serial.print( "menuButtonLongPressHandler: " );
    Serial.println( buttonObj -> getHwId( ));
    #endif

    if ( currentScreen != nullptr ) 
        currentScreen -> menuButtonLongPress( buttonObj );
}

void UIScreen::buttonClickHandler( UIButton *buttonObj ) {

    #if DEBUG_SCREEN_CALLBACK == 1
    Serial.print( "CurrentScreen: " );
    Serial.println((uint32_t ) currentScreen );
    Serial.print( "buttonClickHandler: " );
    Serial.println( buttonObj -> getHwId( ));
    #endif

    if ( currentScreen != nullptr ) currentScreen -> buttonClick( buttonObj );
}

void UIScreen::buttonLongPressStartHandler( UIButton *buttonObj ) {

    #if DEBUG_SCREEN_CALLBACK == 1
    Serial.print( "CurrentScreen: " );
    Serial.println((uint32_t ) currentScreen );
    Serial.print( "buttonLongPressStartHandler: " );
    Serial.println( buttonObj -> getHwId( ));
    #endif

    if ( currentScreen != nullptr ) 
        currentScreen -> buttonLongPressStart( buttonObj );
}

void UIScreen::buttonLongPressStopHandler( UIButton *buttonObj ) {

    #if DEBUG_SCREEN_CALLBACK == 1
    Serial.print( "CurrentScreen: " );
    Serial.println((uint32_t ) currentScreen );
    Serial.print( "buttonLongPressStopHandler: " );
    Serial.println( buttonObj -> getHwId( ));
    #endif

    if ( currentScreen != nullptr ) 
        currentScreen -> buttonLongPressStop( buttonObj );
}

void UIScreen::encoderPosChangeHandler( UIEncoder *encoderObj ) {

    #if DEBUG_SCREEN_CALLBACK == 1
    Serial.print( "CurrentScreen: " );
    Serial.println((uint32_t ) currentScreen );
    Serial.print( "encoderPosHandler - Pos: " );
    Serial.println( encoderObj -> getPosition( ));
    #endif

    if ( currentScreen != nullptr ) 
        currentScreen -> encoderPosChange( encoderObj );
}

//----------------------------------------------------------------------------------------
// The UI screen constructor.
//
//----------------------------------------------------------------------------------------
UIScreen::UIScreen( ) { }

//----------------------------------------------------------------------------------------
// "menuButtonClick" and "selectButtonCLick" are the handler method that will manage
// the menu navigation. The menu button click  advances in a round robin fashion 
// through the menu child list of the actual menu screen. The select button selects
// the first child of the current menu screen. We do not make any changes to the 
// actual screen display content. Instead the enter and exit methods are invoked, 
// which are expected to handle the screen content.
//
// Sometimes it is useful to disable a screen in a screen list. Rather than taking
// it out of the list, there is an enable flag. When the menuButton click event is 
// handled it will skip disabled screens. If all screens are disabled, no action
// will be taken.
//
//----------------------------------------------------------------------------------------
void UIScreen::menuButtonClick( UIButton *buttonObj ) {

    if ( currentScreen != nullptr ) {

        UIScreen *tmp = (( currentScreen -> next == nullptr ) ?
                        currentScreen -> parent -> child : currentScreen -> next );

        while ( tmp != currentScreen ) {

        #if DEBUG_SKIP_SCREENS
        Serial.print( "MENU: current: " );
        Serial.print(( uint32_t ) currentScreen );
        Serial.print( "MENU: tmp: " );
        Serial.print(( uint32_t ) tmp );
        Serial.print( ", enabled: " );
        Serial.println(( uint32_t ) tmp -> enabled );
        #endif
      
        if ( tmp -> enabled ) {

            setCurrentScreen( tmp, true );
            break;
        }
        else 
            tmp = (( tmp -> next == nullptr ) ? 
                    currentScreen -> parent -> child : tmp -> next );
    }
  }
}

void UIScreen::selectButtonClick( UIButton * buttonObj ) {

    if ( currentScreen != nullptr ) {

        if ( currentScreen -> child != nullptr ) {

            setCurrentScreen( currentScreen -> child, true );
        }
    }
}

//----------------------------------------------------------------------------------------
// "menuButtonLongPress" is handler method will manage the unconditional return to
// the top level screen. This handler can also be overwritten to implement for 
// example a return to the first child of the current menu list. However, it is 
// convenient to always go to the first child of the root screen and have a known
// screen where to start.
//
//----------------------------------------------------------------------------------------
void UIScreen::menuButtonLongPress( UIButton * buttonObj) {

    if ( rootScreen -> child != nullptr ) 
        setCurrentScreen( rootScreen -> child, true );
}

//----------------------------------------------------------------------------------------
// The UIScreen object provides dummy functions that can be overridden by a subclass.
//
//----------------------------------------------------------------------------------------
void UIScreen::enterScreen( bool init ) { }
void UIScreen::exitScreen( ) { }
void UIScreen::buttonClick( UIButton * buttonObj ) { }
void UIScreen::buttonLongPressStart( UIButton * buttonObj ) { }
void UIScreen::buttonLongPressStop( UIButton * buttonObj ) { }
void UIScreen::upButtonClick( UIButton * buttonObj ) { }
void UIScreen::downButtonClick( UIButton * buttonObj ) { }
void UIScreen::encoderPosChange( UIEncoder * encoderObj ) { }

//----------------------------------------------------------------------------------------
// "append" is the building block for constructing menus. For a given screen, the 
// passed screen is appended to the end of the children list. The parent field becomes
// the object that the new screen object will be appended to.
//
//----------------------------------------------------------------------------------------
void UIScreen::append( UIScreen * screen ) {

    UIScreen *temp = child;

    if ( temp != nullptr ) {

        while ( temp -> next != nullptr ) temp = temp -> next;
        temp -> next = screen;
    }
    else child = screen;

    screen -> next    = nullptr;
    screen -> child   = nullptr;
    screen -> parent  = this;
}

//----------------------------------------------------------------------------------------
// The "setup" method gets the whole show going. The first child of the root screen
// is the first screen to show if it is already there.
//
//----------------------------------------------------------------------------------------
bool UIScreen::setup( ) {

    if (( rootScreen != nullptr ) && ( rootScreen -> child != nullptr )) {

        currentScreen = rootScreen -> child;
        currentScreen -> enterScreen( true );
        return ( true );
    }
    else return ( false );
}

//----------------------------------------------------------------------------------------
// Screen can be conditionally shown. If a screen is disabled, it will stay in the
// list but skipped when toggling through the windows.
//
//----------------------------------------------------------------------------------------
void UIScreen::enableScreen( ) {

    enabled = true;
}

void UIScreen::disableScreen( ) {

    enabled = false;
}

bool UIScreen::isEnabled( ) {

    return ( enabled );
}

//----------------------------------------------------------------------------------------
// Static Getter/Setter methods.
//
//----------------------------------------------------------------------------------------
UIScreen *UIScreen::getRootScreen( ) {

    return ( rootScreen );
}

UIScreen *UIScreen::getParentScreen( ) {

    return ( parent );
}

UIScreen *UIScreen::getChildScreen( ) {

    return ( child );
}

UIScreen *UIScreen::getCurrentScreen( ) {

    return ( currentScreen );
}

UIScreen *UIScreen::setCurrentScreen( UIScreen * screen, bool init ) {

    if ( screen != nullptr ) {

        if ( currentScreen != nullptr ) currentScreen -> exitScreen( );

        UIScreen *previousScreen = currentScreen;

        currentScreen = screen;
        currentScreen -> enterScreen( init );

        return ( previousScreen );
    
    } else return ( nullptr );
}
