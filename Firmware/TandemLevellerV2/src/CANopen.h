// canopen

#ifndef _CANOPENH_
#define _CANOPENH_

#include "Global.h"
#include "Arduino.h"
#include "FlexCAN_T4-master/FlexCAN_T4.h"
#include "Blades.h"
#include "GNSS.h"

// number of nodes to monitor
#define NUM_NODES 0x7F

// node IDs
#define CONTROLLER_NODE_ID       0x01
#define TRACTOR_IMU_NODE_ID      0x02
#define FRONTSCRAPER_IMU_NODE_ID 0x03
#define REARSCRAPER_IMU_NODE_ID  0x04
#define PENDANT_NODE_ID          0x05
#define FRONT_ANGLE_NODE_ID      0x10
#define REAR_ANGLE_NODE_ID       0x11
#define FRONT_BUCKET_IMU_NODE_ID 0x20
#define REAR_BUCKET_IMU_NODE_ID  0x21

// received PDO callback type
typedef void (*canopen_process_pdo_callback_t)(uint8_t NodeId, uint16_t PDONumber, size_t DataLength, const uint8_t *pData);
// node lost callback type
typedef void (*canopen_node_lost_callback_t)(uint8_t NodeId);
// reset request callback type
typedef void (*canopen_request_reset_callback_t)(void);

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

    // transmit TPDO3
    void TxTPDO3
      (
      double Latitude,                   // current tractor latitude
      double Longitude                   // current tractor longitude
      );

    // transmit TPDO4
    void TxTPDO4
      (
      double Altitude,                   // current tractor altitude
      gnss_rtk_status_t RtkStatus        // current tractor RTK status
      );

    // transmit TPDO5
    void TxTPDO5
      (
      int EastingMm,                     // fusor correction in easting in mm
      int NorthingMm,                    // fusor correction in northing in mm
      int AltitudeMm                     // fusor correction in altitude in mm
      );

    // transmit TPDO6
    void TxTPDO6
      (
      double Latitude,                   // current tractor latitude (raw)
      double Longitude                   // current tractor longitude (raw)
      );

    // transmits an emergency message
    void EmergencyStop
      (
      uint32_t LineNumber                // line number where the emergency happened
      );

    // resets all nodes on the CAN bus
    void ResetAllNodes
      (
      void
      );

    // performs processing, call in the main loop
    void Process
      (
      void
      );

    // performs initialization
    void Init
      (
      void
      );

    // checks if a node has been found
    // returns true if node found, false if not found
    bool IsNodeFound
      (
      uint8_t NodeId                     // id of node to check
      );

    // sets the callback function
    void SetCallbacks
      (
      canopen_process_pdo_callback_t ProcessPDOCallback,
      canopen_node_lost_callback_t NodeLostCallback,
      canopen_request_reset_callback_t _ResetRequestCallback
      );

    // called when a CAN message is received
    void _CANReceiveHandler
      (
      const CAN_message_t &msg
      );
      
  private:
    elapsedMillis HBTime[NUM_NODES];
    elapsedMillis HBTimestamp;
    bool NodeFound[NUM_NODES];
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANBus;
    canopen_process_pdo_callback_t ProcessPDOCallback;
    canopen_node_lost_callback_t NodeLostCallback;
    canopen_request_reset_callback_t ResetRequestCallback;

    // checks to see if any nodes are missing
    void CheckForMissingNodes
      (
      void
      );

    // process heartbeat from a node
    void ProcessHeartbeat
      (
      uint8_t NodeId,                    // node that transmitted the heartbeat
      uint8_t Length,                    // length of heartbeat message
      const uint8_t *pData               // heartbeat message data
      );
};

#endif // _CANOPENH_
