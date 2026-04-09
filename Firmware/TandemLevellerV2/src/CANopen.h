// canopen

#ifndef _CANOPENH_
#define _CANOPENH_

#include "Global.h"
#include "Arduino.h"
#include "FlexCAN_T4-master/FlexCAN_T4.h"
#include "Blades.h"

// number of nodes to monitor
#define NUM_NODES 8

class CANopen
{
  public:
    // constructor
    CANopen
      (
      void
      );

    // transmits a CAN message
    void TxCANMessage
      (
      uint16_t Id,
      uint8_t Length,
      uint8_t Data[]
      );

    // transmits a bootup message
    void TxBootup
      (
      void
      );

    // transmits a heartbeat message
    void TxHeartbeat
      (
      void
      );

    // transmit TPDO1
    void TxTPDO1
      (
      blade_status_t *pFrontBladeStatus,
      blade_status_t *pRearBladeStatus
      );

    // transmit TPDO2
    void TxTPDO2
      (
      blade_status_t *pFrontBladeStatus,
      blade_status_t *pRearBladeStatus
      );

    // transmits an emergency message
    void EmergencyStop
      (
      uint32_t LineNumber     // line number where the emergency happened
      );

    // resets all nodes on the CAN bus
    void ResetAllNodes
      (
      void
      );

  private:
    elapsedMillis HBTime[NUM_NODES];
    bool NodeFound[NUM_NODES];
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANBus;
};

#endif // _CANOPENH_
