//------------------------------------------------------------------------------------------------------------
//
// LCS Block Controller - Include file
//
//------------------------------------------------------------------------------------------------------------
//
// LCS Block Controller
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
#ifndef LcsBlockController_h
#define LcsBlockController_h

#include "LcsCdcLib.h"
#include "LcsRuntimeLib.h"

//------------------------------------------------------------------------------------------------------------
//
// Ideas how to use the node data:
//
// There is a static data portion, which describes the block. This is data is entered when the block is configured.
//
//  - block ID
//  - block length
//  - block name
//  - previous block(s)
//  - next block(s)
//
//  - number of sections
//  - section lengths
//  - speed level - slow, middle, high ... 
//  - support DCC and analog flag
//  - max current limit
//  - periodic time to send data
//  - timeout values of all kinds ?
//
//
// There is a dynamic data portion, which contains the data about the block current state
//
//  - mode ( DCC or analog or off )
//  - actual current
//  - section occupancy
//  - section enter / leave timestamps
//  - 
//
// ??? what is retrieved from the dynamic data on a restart ?
//
// The node attributes contains data about how many blocks this node contains ( nodeId + portId -> blockId )
// 
// Most of the data is stored in attributes for the port.
// 
//
// Finally, there are items that represent commands to the block. 
// 
//  - emergency stop
//  - switch to DCC or analog mode
//  - block on or off
//  - signals setting
//  - turnout setting
//  - ...

// There are predefined events that the controller node will send.
// 
//  - block state change
//  - section occupied
//  - section entered
//  - section left
//  - 
//  
//
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
// The block controller can contain up to four blocks. Each block track is described by the LcsBlockDesc
// descriptor. There are the hardware pins sel1Pin1, selPin2, sensePin and uartRxPin. In addition there are
// the limits for current consumption values, all specified in milliAmps. The initial current sets the current
// consumption limit after the track is turned on. The limit current consumption specifies the actual 
// configured value that is checked for a track current overload situation. The maximum current defines what 
// current the power module should never exceed. For the measurements to work, the power module needs to 
// deliver a voltage that corresponds to the current drawn on the track. The value is measured in milliVolt 
// per Ampere drawn. Finally, there are threshold times for managing the track overload and restart 
// capability.
//
//------------------------------------------------------------------------------------------------------------
struct LcsBlockTrackDesc {

    uint16_t    options;
    uint8_t     selPin1     = CDC::UNDEFINED_PIN;
    uint8_t     selPin2     = CDC::UNDEFINED_PIN;
    uint8_t     sensePin    = CDC::UNDEFINED_PIN;
    uint8_t     uartRxPin   = CDC::UNDEFINED_PIN;

    uint16_t  initCurrentMilliAmp           = 0;
    uint16_t  limitCurrentMilliAmp          = 0;
    uint16_t  maxCurrentMilliAmp            = 0;
    uint16_t  milliVoltPerAmp               = 0;

    uint16_t  startTimeThresholdMillis      = 0;
    uint16_t  stopTimeThresholdMillis       = 0;
    uint16_t  overloadTimeThresholdMillis   = 0;
    uint16_t  overloadEventThreshold        = 0;
    uint16_t  overloadRestartThreshold      = 0;
};


//------------------------------------------------------------------------------------------------------------
//
//
//
//------------------------------------------------------------------------------------------------------------
struct LcsBlockDesc {



};




//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
struct LcsBlockControllerLogic {


    LcsBlockControllerLogic( LcsBlockDesc *blockDesc );

    uint8_t setupBlocks( );

    private:

    LcsBlockDesc *blockDesc;

};


#endif
