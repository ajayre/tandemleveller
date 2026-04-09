// canopen

#include "CANopen.h"

// node IDs
#define CONTROLLER_NODE_ID       0x01
#define TRACTOR_IMU_NODE_ID      0x02
#define FRONTSCRAPER_IMU_NODE_ID 0x03
#define REARSCRAPER_IMU_NODE_ID  0x04
#define PENDANT_NODE_ID          0x05
#define FRONT_ANGLE_NODE_ID      0x10
#define REAR_ANGLE_NODE_ID       0x11

// how often to transmit TPDOs
#define TPDO_OUTPUT_PERIOD_MS 50

#define NMT_RESET_CMD 0x81
#define NMT_RESET_ALL 0x00

// NMT states
#define NMT_STATE_BOOTUP      0x00
#define NMT_STATE_OPERATIONAL 0x05

// maximum time to wait for a heartbeat before declaring a node as missing
#define MAX_HEARTBEAT_TIME 300

// time between heartbeats in millseconds
#define HB_PRODUCER_TIME_MS 100

// CANopen error code for estop
#define ESTOP_ERROR_CODE 0x1000


///////////////////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTIONS


///////////////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTIONS

// constructor
CANopen::CANopen
  (
  void
  )
{
}

// transmits a CAN message
void CANopen::TxCANMessage
  (
  uint16_t Id,
  uint8_t Length,
  uint8_t Data[]
  )
{
  CAN_message_t txmsg;
  txmsg.id = Id;
  txmsg.len = Length;
  for (uint8_t i = 0; i < Length; i++ ) txmsg.buf[i] = Data[i];
  CANBus.write(txmsg);
}

// transmits a bootup message
void CANopen::TxBootup
  (
  void
  )
{
  uint8_t Data[1];

  Data[0] = NMT_STATE_BOOTUP;
  TxCANMessage(0x700 + CONTROLLER_NODE_ID, 1, Data);
}

// transmits a heartbeat message
void CANopen::TxHeartbeat
  (
  void
  )
{
  uint8_t Data[1];

  Data[0] = NMT_STATE_OPERATIONAL;
  TxCANMessage(0x700 + CONTROLLER_NODE_ID, 1, Data);
}

// transmit TPDO1
void CANopen::TxTPDO1
  (
  blade_status_t *pFrontBladeStatus,
  blade_status_t *pRearBladeStatus
  )
{
  uint8_t Data[8];

  Data[0] = pFrontBladeStatus->BladePWM & 0xFF;
  Data[1] = (pFrontBladeStatus->BladePWM >> 8) & 0xFF;
  Data[2] = pFrontBladeStatus->BladeCommand;
  Data[3] = (pFrontBladeStatus->BladeDirection & 0x01) | ((pFrontBladeStatus->BladeAuto & 0x01) << 1);

  Data[4] = pRearBladeStatus->BladePWM & 0xFF;
  Data[5] = (pRearBladeStatus->BladePWM >> 8) & 0xFF;
  Data[6] = pRearBladeStatus->BladeCommand;
  Data[7] = (pRearBladeStatus->BladeDirection & 0x01) | ((pRearBladeStatus->BladeAuto & 0x01) << 1);

  TxCANMessage(0x180 + CONTROLLER_NODE_ID, 8, Data);
}

// transmit TPDO2
void CANopen::TxTPDO2
  (
  blade_status_t *pFrontBladeStatus,
  blade_status_t *pRearBladeStatus
  )
{
  uint8_t Data[2];

  Data[0] = (uint8_t)pFrontBladeStatus->SlaveOffset;
  Data[1] = (uint8_t)pRearBladeStatus->SlaveOffset;

  TxCANMessage(0x280 + CONTROLLER_NODE_ID, 2, Data);
}

// transmits an emergency message
void CANopen::EmergencyStop
  (
  uint32_t LineNumber     // line number where the emergency happened
  )
{
    // send emergency message so all CAN nodes are aware of the stop
    CAN_message_t txmsg;
    txmsg.id = 0x080 + CONTROLLER_NODE_ID;
    txmsg.len = 8;
    txmsg.buf[0] =  ESTOP_ERROR_CODE       & 0xFF;
    txmsg.buf[1] = (ESTOP_ERROR_CODE >> 8) & 0xFF;
    txmsg.buf[2] = 0x80;  // manufacturer-specific error
    txmsg.buf[3] =  LineNumber        & 0xFF;
    txmsg.buf[4] = (LineNumber >> 8)  & 0xFF;
    txmsg.buf[5] = (LineNumber >> 16) & 0xFF;
    txmsg.buf[6] = (LineNumber >> 24) & 0xFF;
    txmsg.buf[7] = 0x00;
    CANBus.write(txmsg);
}

// resets all nodes on the CAN bus
void CANopen::ResetAllNodes
  (
  void
  )
{
  CAN_message_t txmsg;
  
  txmsg.id = 0x000;
  txmsg.len = 2;
  txmsg.buf[0] = NMT_RESET_CMD;
  txmsg.buf[1] = NMT_RESET_ALL;

  CANBus.write(txmsg);
}
