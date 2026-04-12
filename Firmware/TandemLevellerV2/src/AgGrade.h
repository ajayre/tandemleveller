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

// supported PGNs
typedef enum _pgn_t : uint16_t
{
  // misc
  PGN_ESTOP                      = 0x0000,
  PGN_RESET                      = 0x0001,
  PGN_AGGRADE_STARTED            = 0x0002,
  PGN_PING                       = 0x0003,
  PGN_CLEAR_ESTOP                = 0x0004,
  PGN_TRACTOR_IMU_FOUND          = 0x0005, // fixme - to do
  PGN_TRACTOR_IMU_LOST           = 0x0006, // fixme - to do
  PGN_FRONT_IMU_FOUND            = 0x0007, // fixme - to do
  PGN_FRONT_IMU_LOST             = 0x0008, // fixme - to do
  PGN_REAR_IMU_FOUND             = 0x0009, // fixme - to do
  PGN_REAR_IMU_LOST              = 0x000A, // fixme - to do
  PGN_FRONT_HEIGHT_FOUND         = 0x000B, // fixme - to do
  PGN_FRONT_HEIGHT_LOST          = 0x000C, // fixme - to do
  PGN_REAR_HEIGHT_FOUND          = 0x000D, // fixme - to do
  PGN_REAR_HEIGHT_LOST           = 0x000E, // fixme - to do

  // blade control
  PGN_FRONT_CUT_VALVE            = 0x1000,   // CUTVALVE_MIN -> CUTVALVE_MAX
  PGN_REAR_CUT_VALVE             = 0x1001,   // CUTVALVE_MIN -> CUTVALVE_MAX
  PGN_FRONT_ZERO_BLADE_HEIGHT    = 0x1002,
  PGN_REAR_ZERO_BLADE_HEIGHT     = 0x1003,
  PGN_FRONT_BLADE_JOG_UP         = 0x1004, // fixme - to do
  PGN_FRONT_BLADE_JOG_DOWN       = 0x1005, // fixme - to do
  PGN_REAR_BLADE_JOG_UP          = 0x1006, // fixme - to do
  PGN_REAR_BLADE_JOG_DOWN        = 0x1007, // fixme - to do

  // blade configuration
  PGN_FRONT_PWM_GAIN_UP          = 0x2002,
  PGN_FRONT_PWM_GAIN_DOWN        = 0x2003,
  PGN_FRONT_PWM_MIN_UP           = 0x2004,
  PGN_FRONT_PWM_MIN_DOWN         = 0x2005,
  PGN_FRONT_PWM_MAX_UP           = 0x2006,
  PGN_FRONT_PWM_MAX_DOWN         = 0x2007,
  PGN_FRONT_INTEGRAL_MULTPLIER   = 0x2008,
  PGN_FRONT_DEADBAND             = 0x2009,
  PGN_REAR_PWM_GAIN_UP           = 0x200A,
  PGN_REAR_PWM_GAIN_DOWN         = 0x200B,
  PGN_REAR_PWM_MIN_UP            = 0x200C,
  PGN_REAR_PWM_MIN_DOWN          = 0x200D,
  PGN_REAR_PWM_MAX_UP            = 0x200E,
  PGN_REAR_PWM_MAX_DOWN          = 0x200F,
  PGN_REAR_INTEGRAL_MULTPLIER    = 0x2010,
  PGN_REAR_DEADBAND              = 0x2011,

  // blade status
  PGN_FRONT_BLADE_OFFSET_SLAVE   = 0x5000,
  PGN_FRONT_BLADE_PWMVALUE       = 0x5001,
  PGN_FRONT_BLADE_DIRECTION      = 0x5002,
  PGN_FRONT_AUTO                 = 0x5003,
  PGN_REAR_BLADE_OFFSET_SLAVE    = 0x5004,
  PGN_REAR_BLADE_PWMVALUE        = 0x5005,
  PGN_REAR_BLADE_DIRECTION       = 0x5006,
  PGN_REAR_AUTO                  = 0x5007,
  PGN_FRONT_BLADE_HEIGHT         = 0x5008,
  PGN_REAR_BLADE_HEIGHT          = 0x5009,
  PGN_FRONT_DUMPING              = 0x500A, // fixme - to do
  PGN_REAR_DUMPING               = 0x500B, // fixme - to do

  // IMU
  PGN_TRACTOR_IMU                = 0x6000,
  PGN_FRONT_IMU                  = 0x6001,
  PGN_REAR_IMU                   = 0x6002,

  // GNSS
  PGN_TRACTOR_NMEA               = 0x7000,
  PGN_FRONT_NMEA                 = 0x7001,
  PGN_REAR_NMEA                  = 0x7002,

  // equipment
  PGN_TRACTOR_ANTENNA_HEIGHT     = 0x8000,
  PGN_TRACTOR_ANTENNA_LEFTOFF    = 0x8001,
  PGN_TRACTOR_ANTENNA_FORWARDOFF = 0x8002,
  PGN_FRONT_ANTENNA_HEIGHT       = 0x8003,
  PGN_FRONT_ANTENNA_LEFTOFF      = 0x8004,
  PGN_FRONT_ANTENNA_FORWARDOFF   = 0x8005,
  PGN_REAR_ANTENNA_HEIGHT        = 0x8006,
  PGN_REAR_ANTENNA_LEFTOFF       = 0x8007,
  PGN_REAR_ANTENNA_FORWARDOFF    = 0x8008,
} pgn_t;

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
