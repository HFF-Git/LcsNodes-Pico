//------------------------------------------------------------------------------------------------------------
//
// LCS - Cab Handheld LCS Bus interface implementation file
//
//------------------------------------------------------------------------------------------------------------
// ???
//  - has LCS lb
//  - has Msg Bus
//
//
//------------------------------------------------------------------------------------------------------------
//
// LCS - Cab Handheld LCS Bus interface implementation file
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
#include "CabHandheld.h"

//------------------------------------------------------------------------------------------------------------
// File local declarations.
//
//------------------------------------------------------------------------------------------------------------
namespace {

  //----------------------------------------------------------------------------------------------------------
  //
  //----------------------------------------------------------------------------------------------------------
  void printLcsMsg( uint8_t *msg ) {

    int msgLen = (( msg[0] >> 5 ) + 1 ) % 8;

    for ( int i = 0; i < msgLen; i++ ) {

      INTERFACE.print( msg[i], HEX );
      INTERFACE.print( F(" "));
    }

    INTERFACE.println( );
  }

}; // nameSpace


//------------------------------------------------------------------------------------------------------------
//
//
//
//------------------------------------------------------------------------------------------------------------
LcsCoreLib          *lcsLib   = nullptr;
CabMsgBus           *msgBus = nullptr;


//----------------------------------------------------------------------------------------------------------
// LCS Core library callback functions, place holders for testing.
//
//----------------------------------------------------------------------------------------------------------
uint8_t initCallback (uint16_t nodeId, uint8_t portId, uint16_t flags ) {

  // ??? do we actually have anything wa want to do here ? If not, take out ...

  INTERFACE.print( F( "initCallback -> node: " ));
  INTERFACE.print( nodeId );
  INTERFACE.print( F( ", port: " ));
  INTERFACE.print( portId );
  INTERFACE.print( F( ", flags: " ));
  INTERFACE.print( flags );
  INTERFACE.println( );

  return ( ALL_OK );
}


//----------------------------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------------------------
uint8_t infoItemCallback( uint8_t portId, uint8_t item, uint16_t *arg1, uint16_t *arg2 ) {

  INTERFACE.print( F( "infoItemCallback -> port: " ));
  INTERFACE.print( portId );
  INTERFACE.print( F( ", item: " ));
  INTERFACE.print( item );
  INTERFACE.print( F( ":" ));
  if ( arg1 != nullptr ) INTERFACE.print( *arg1 ); else INTERFACE.print( F( "null" ));
  INTERFACE.print( F( ":" ));
  if ( arg2 != nullptr ) INTERFACE.print( *arg2 ); else INTERFACE.print( F( "null" ));
  INTERFACE.println( );
  return ( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------------------------
uint8_t ctrlItemCallback( uint8_t portId, uint8_t item, uint16_t arg1, uint16_t arg2 ) {

  INTERFACE.print (F( "nodeCtrlItemCallback -> port: " ));
  INTERFACE.print( portId );
  INTERFACE.print( F( ", item: " ));
  INTERFACE.print( item );
  INTERFACE.print( F( ":" ));
  INTERFACE.print( arg1 );
  INTERFACE.print(F(":"));
  INTERFACE.print( arg2 );
  INTERFACE.println( );
  return ( ALL_OK );
}

//----------------------------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------------------------
void itemReqCallback( uint16_t nodeId, uint8_t portId, uint8_t item, uint16_t arg1, uint16_t arg2 ) {

  INTERFACE.print(F( "nodeCtrlItemCallback -> node: " ));
  INTERFACE.print( nodeId );
  INTERFACE.print(F(", port: "));
  INTERFACE.print( portId );
  INTERFACE.print(F(", item: "));
  INTERFACE.print(item);
  INTERFACE.print(F( ":" ));
  INTERFACE.print( arg1 );
  INTERFACE.print(F( ":" ));
  INTERFACE.print( arg2 );
  INTERFACE.println( );
}

//----------------------------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------------------------
void portEventCallback( uint16_t nodeId, uint8_t portId, uint8_t eAction, uint16_t eId, uint16_t eData ) {

  // ?? what events would a can be interested in ...

  INTERFACE.print( F( "portEventCallback -> " ));
  INTERFACE.print( nodeId );
  INTERFACE.print( ":" );
  INTERFACE.print( portId );
  INTERFACE.print( ":" );
  INTERFACE.print( eId );
  INTERFACE.print( ":" );
  INTERFACE.print( eAction );
  INTERFACE.print( ":" );
  INTERFACE.print( eData );
  INTERFACE.println( );
}

//----------------------------------------------------------------------------------------------------------
// General LCS Bus messages we might want to handle....
//
//----------------------------------------------------------------------------------------------------------
void busMgtCallback( uint8_t *msg ) {

  INTERFACE.println( F( "busMgtCallback -> " ));
  printLcsMsg( msg );

  switch ( msg[ 0 ] ) {

    case LCS_OP_OPS:
    case LCS_OP_CFG:
    case LCS_OP_BON:
    case LCS_OP_BOF:
    case LCS_OP_NCOL:
    case LCS_OP_RESET: break;

    default: ;
  }
}

//----------------------------------------------------------------------------------------------------------
// DO WE EVEN HANDLE DCC MESSAGES ?????? this is the base station's job ...
//
//----------------------------------------------------------------------------------------------------------
void dccMsgCallback( uint8_t *msg ) {

  INTERFACE.println( F( "DCC Msg Callback -> " ));
  printLcsMsg( msg );

  switch ( msg[ 0 ] ) {

    case LCS_OP_REQ_LOC:
    case LCS_OP_REL_LOC:
    case LCS_OP_REP_LOC:
    case LCS_OP_SET_LCON:
    case LCS_OP_KEEP_LOC:
    case LCS_OP_SET_LSPD:
    case LCS_OP_SET_LMOD:
    case LCS_OP_LOC_FON:
    case LCS_OP_LOC_FOF:

    case LCS_OP_SET_CVM:
    case LCS_OP_REQ_CVS:
    case LCS_OP_REP_CVS:
    case LCS_OP_SET_CVS:

    case LCS_OP_REQ_TON:
    case LCS_OP_REQ_TOF:
    case LCS_OP_TON:
    case LCS_OP_TOF:
    case LCS_OP_REQ_ESTP:
    case LCS_OP_ESTP:

    case LCS_OP_SEND_DCC3:
    case LCS_OP_SEND_DCC4:
    case LCS_OP_SEND_DCC5:
    case LCS_OP_SEND_DCC6:

    case LCS_OP_DCC_ACK:
    case LCS_OP_DCC_ERR:  break;

    default: ;

  }
}

//------------------------------------------------------------------------------------------------------------
// The very first thing after CDC setup is to do is to set up the LCS core library. This is done by building
// the configuration descriptor and pass it to the "init" routine of the library. Note, there will only be
// one object of this class.
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupLcsLib( ) {

  LcsCoreLibConfigDesc  lcsDesc;

  lcsDesc.nodeId              = 1;
  lcsDesc.options             = NOPT_SKIP_NODE_ID_CONFIG | NOPT_USE_EXT_NVM;
  lcsDesc.numOfPorts          = 4;
  lcsDesc.numOfEvents         = 32;
  lcsDesc.numOfAttrs          = 128;
  lcsDesc.numOfPeriodicTasks  = 16;

  uint8_t rStat = LcsCoreLib::init( &cfg, &lcsDesc, &lcsLib );

  if ( rStat == ALL_OK ) {

    lcsLib ->registerPortEventCallback( portEventCallback );
    lcsLib-> registerInitCallback( 0, initCallback );
    lcsLib-> registerInfoCallback( 0, infoItemCallback);
    lcsLib-> registerCtrlCallback( 0, ctrlItemCallback);
    lcsLib-> registerReqRepCallback( itemReqCallback );
    lcsLib -> registerPeriodicTask( UIElements::tick );
  }

  return ( rStat );
}


//------------------------------------------------------------------------------------------------------------
//
// LCS Message Bus object methods...
//
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t setupMsgBus( ) {

  msgBus = new CabMsgBus( );

  lcsLib -> registerLcsMsgCallback( busMgtCallback );
  lcsLib -> registerDccMsgCallback( dccMsgCallback );

  return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t CabMsgBus::sendSpeedAndDir( CabEntry *cab ) {

  // ??? build the LCS message from the cab entry ...
  return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t CabMsgBus::sendDccFuncVal( CabEntry *cab, uint8_t dccFundId ) {

  // ??? get val from bitmap...

  return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t CabMsgBus::sendEngineOnOff( CabEntry *cab ) {

  return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t CabMsgBus::requestLocoSession( CabEntry *cab ) {

  return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t CabMsgBus::closeLocoSession( CabEntry *cab ) {

  // ??? not clear yet, a dispatch ? a true close ? what else ...

  return ( ALL_OK );
}

//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
uint8_t CabMsgBus::loadCabData( CabEntry *cab ) {

  // ??? we will load all data from the Basestation Dictionary for the engine... can be done one word at a time ...

  return ( ALL_OK );
}

// ??? is there a need for an "update CAB data ". E.g. when we change a config itenm ... why not update too ?
