// canopen

#include "CANopen.h"

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

// CAN bus speed
#define CAN_BITRATE_BPS 125000

// hold pointer to singleton
static CANopen *s_rxTarget = nullptr;


///////////////////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTIONS

// called when a CAN message is received
// trampoline
static void CANReceiveHandler
  (
  const CAN_message_t &msg
  )
{
  // forward to singleton
  if (s_rxTarget) s_rxTarget->_CANReceiveHandler(msg);
}


///////////////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTIONS

// constructor
CANopen::CANopen
  (
  void
  )
{
  // set up singleton pointer
  s_rxTarget = this;
}

// performs initialization
void CANopen::Init
  (
  void
  )
{
  // configure CAN bus
  CANBus.begin();
  CANBus.setBaudRate(CAN_BITRATE_BPS);
  CANBus.setMaxMB(64);
  CANBus.enableFIFO();
  CANBus.onReceive(FIFO, CANReceiveHandler);
  CANBus.enableFIFOInterrupt();
  
  // reset heartbeat timers and flags
  for (int n = 0; n < NUM_NODES; n++)
  {
    HBTime[n] = 0;
    NodeFound[n] = false;
  }

  // TPDO1s
  CANBus.setFIFOFilterRange(0, 0x181, 0x1FF, STD);
  // TPDO2s
  CANBus.setFIFOFilterRange(1, 0x281, 0x2FF, STD);
  // Heartbeats
  CANBus.setFIFOFilterRange(2, 0x701, 0x77F, STD);

  CANBus.setMB(MB63, TX); // Set mailbox as transmit

  HBTimestamp = 0;

  ProcessPDOCallback = NULL;
  NodeLostCallback = NULL;
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

// performs processing, call in the main loop
void CANopen::Process
  (
  void
  )
{
  // process can module
  CANBus.events();

  CheckForMissingNodes();

  // transmit heartbeats
  if (HBTimestamp >= HB_PRODUCER_TIME_MS)
  {
    HBTimestamp = 0;

    TxHeartbeat();
  }
}

// sets the callback function
void CANopen::SetCallbacks
  (
  canopen_process_pdo_callback_t _ProcessPDOCallback,
  canopen_node_lost_callback_t _NodeLostCallback,
  canopen_request_reset_callback_t _ResetRequestCallback
  )
{
  ProcessPDOCallback   = _ProcessPDOCallback;
  NodeLostCallback     = _NodeLostCallback;
  ResetRequestCallback = _ResetRequestCallback;
}

// called when a CAN message is received
void CANopen::_CANReceiveHandler
  (
  const CAN_message_t &msg
  )
{
  // process TPDOs
  uint8_t NodeId = msg.id & 0x7F;
  uint8_t PDONumber;
       if ((msg.id >= 0x181) && (msg.id <= 0x1FF)) PDONumber = 1;
  else if ((msg.id >= 0x281) && (msg.id <= 0x2FF)) PDONumber = 2;
  else if ((msg.id >= 0x381) && (msg.id <= 0x3FF)) PDONumber = 3;
  else if ((msg.id >= 0x481) && (msg.id <= 0x4FF)) PDONumber = 4;
  if (ProcessPDOCallback != NULL) ProcessPDOCallback(NodeId, PDONumber, msg.len, msg.buf);

  // process heartbeats
  if ((msg.id >= 0x701) && (msg.id <= 0x77F))
  {
    ProcessHeartbeat(NodeId, msg.len, msg.buf);
  }

  // process NMT message
  if ((msg.id == 0x000) && !msg.flags.extended && (msg.len == 2))
  {
    // reset
    if (msg.buf[0] == NMT_RESET_CMD)
    {
      // this node
      if ((msg.buf[1] == CONTROLLER_NODE_ID) || (msg.buf[1] == NMT_RESET_ALL))
      {
        if (ResetRequestCallback != NULL) ResetRequestCallback();
      }
    }
  }
}

// checks to see if any nodes are missing
void CANopen::CheckForMissingNodes
  (
  void
  )
{
  for (int n = 0; n < NUM_NODES; n++)
  {
    if (NodeFound[n])
    {
      if (HBTime[n] >= MAX_HEARTBEAT_TIME)
      {
        NodeFound[n] = false;

        if (NodeLostCallback != NULL) NodeLostCallback(n + 1);
      }
    }
  }
}

// process heartbeat from a node
void CANopen::ProcessHeartbeat
  (
  uint8_t NodeId,            // node that transmitted the heartbeat
  uint8_t Length,            // length of heartbeat message
  const uint8_t *pData       // heartbeat message data
  )
{
  if (Length == 1)
  {
    uint8_t NMTState = pData[0];

    // if we see a bootup message then we have found a node
    if (NMTState == NMT_STATE_BOOTUP)
    {
      NodeFound[NodeId - 1] = true;
      HBTime[NodeId - 1] = 0;
    }

    // on every heartbeat reset the timer
    if (NMTState == NMT_STATE_OPERATIONAL)
    {
      // must have missed the bootup message
      if (!NodeFound[NodeId -1])
      {
        NodeFound[NodeId - 1] = true;
      }

      HBTime[NodeId - 1] = 0;
    }
  }
}

// checks if a node has been found
// returns true if node found, false if not found
bool CANopen::IsNodeFound
  (
  uint8_t NodeId             // id of node to check
  )
{
  if (NodeId == 0) return false;

  return NodeFound[NodeId - 1];
}
