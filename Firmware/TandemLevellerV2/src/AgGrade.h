// interface to AgGrade

#ifndef _AGGRADEH_
#define _AGGRADEH_

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include "Global.h"
#include "UDPTransfer.h"
#include "Blades.h"
#include "IMU.h"

// maximum number of bytes in a PGN packet payload
#define MAX_PGN_LEN (MAX_NMEA_LENGTH + 2)

// information that is transmitted to/from AgGrade
typedef struct _pgnpacket_t
{
  pgn_t PGN;
  uint8_t Data[MAX_PGN_LEN] = { 0 };
} pgnpacket_t;

class AgGrade
{
  public:
    AgGrade
      (
      void
      );

    // connects to AgGrade
    void Connect
      (
      byte MACAddress[],            // our MAC address
      IPAddress LocalIPAddress,     // our IP address
      unsigned int LocalPort,       // port that we listen on
      IPAddress RemoteIPAddress,    // IP address of AgGrade
      unsigned int RemotePort       // port that AgGrade is listening on
      );

    // checks if a command is waiting to be read
    // returns true if command waiting
    bool IsCommandAvailable
      (
      void  
      );

    // gets a command from AgGrade
    pgnpacket_t GetCommand
      (
      void
      );

    // call when a blade changes height or direction
    // sends the new state to AgGrade.
    void SendBladeState
      (
      int BladeIndex,                // index of blade that changed xx_BLADE_IDX
      int PWM,                       // current PWM output to blade
      uint32_t Height,               // current blade height
      blade_direction_t Direction    // direction of blade movement
      );

    // sends the front blade height to AgGrade
    void SendFrontBladeHeight
      (
      uint32_t Height                // blade height (0 -> BLADE_HEIGHT_GROUND_LEVEL * 2)
      );

    // sends the rear blade height to AgGrade
    void SendRearBladeHeight
      (
      uint32_t Height                // blade height (0 -> BLADE_HEIGHT_GROUND_LEVEL * 2)
      );

    // sends an IMU state to AgGrade
    void SendIMUState
      (
      int IMUIndex,                  // index of IMU xxx_IDX
      imu_t *pIMUValue               // new IMU values
      );

    // sends front blade slave offset to AgGrade
    void TxFrontBladeSlaveOffset
      (
      int16_t Offset
      );

    // sends rear blade slave offset to AgGrade
    void TxRearBladeSlaveOffset
      (
      int16_t Offset
      );

    // sends front blade auto state to AgGrade
    void SendFrontBladeAuto
      (
      bool Auto
      );

    // sends rear blade auto state to AgGrade
    void SendRearBladeAuto
      (
      bool Auto
      );

    // sends an emergency stop notification to AgGrade
    void EmergencyStop
      (
      void  
      );

    // sends a clear emergency stop notification to AgGrade
    void ClearEmergencyStop
      (
      void  
      );

    // sends a ping to AgGrade
    void SendPing
      (
      void
      );

    // sends an NMEA sentence to AgGrade
    void SendNMEASentence
      (
      pgn_t PGN,                     // PGN for sentence
      char *sentence,                // sentence
      uint8_t length                 // sentence length
      );

    // gets a 32-bit value from a pgn packet
    uint32_t GetPGNPacketUInt32
      (
      pgnpacket_t *Packet
      );

  private:
    UDPTransfer UdpTransfer;
    // An EthernetUDP instance to let us send and receive packets over UDP
    EthernetUDP Udp;

    // sends status value over UDP using packet framing
    void Send
      (
      pgnpacket_t *pStatus
      );

    // starts ethernet
    void StartEthernet
      (
      byte MACAddress[],             // our MAC address
      IPAddress LocalIPAddress       // our IP address
      );

    // Stores a 16-bit value into a pgn packet
    void SetPGNPacketUInt16
      (
      pgnpacket_t *Packet,
      uint16_t Value
      );

    // Stores a 32-bit value into a pgn packet
    void SetPGNPacketUInt32
      (
      pgnpacket_t *Packet,
      uint32_t Value
      );

    // Stores a 32-bit value into a pgn packet at a specific byte offset
    void SetPGNPacketUInt32AtOffset
      (
      pgnpacket_t *Packet,
      uint8_t Offset,
      uint32_t Value
      );
};

#endif // _AGGRADEH_
