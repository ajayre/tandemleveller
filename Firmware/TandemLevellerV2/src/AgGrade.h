// interface to AgGrade

#ifndef _AGGRADEH_
#define _AGGRADEH_

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Global.h"
#include "UDPTransfer.h"

// maximum number of bytes in a PGN packet payload
#define MAX_PGN_LEN (MAX_NMEA_LENGTH + 2)

// supported PGNs
typedef enum _pgn_t : uint16_t
{
  // misc
  PGN_ESTOP                    = 0x0000,
  PGN_RESET                    = 0x0001,
  PGN_AGGRADE_STARTED          = 0x0002,
  PGN_PING                     = 0x0003,
  PGN_CLEAR_ESTOP              = 0x0004,
  PGN_TRACTOR_IMU_FOUND        = 0x0005, // fixme - to do
  PGN_TRACTOR_IMU_LOST         = 0x0006, // fixme - to do
  PGN_FRONT_IMU_FOUND          = 0x0007, // fixme - to do
  PGN_FRONT_IMU_LOST           = 0x0008, // fixme - to do
  PGN_REAR_IMU_FOUND           = 0x0009, // fixme - to do
  PGN_REAR_IMU_LOST            = 0x000A, // fixme - to do
  PGN_FRONT_HEIGHT_FOUND       = 0x000B, // fixme - to do
  PGN_FRONT_HEIGHT_LOST        = 0x000C, // fixme - to do
  PGN_REAR_HEIGHT_FOUND        = 0x000D, // fixme - to do
  PGN_REAR_HEIGHT_LOST         = 0x000E, // fixme - to do

  // blade control
  PGN_FRONT_CUT_VALVE          = 0x1000,   // CUTVALVE_MIN -> CUTVALVE_MAX
  PGN_REAR_CUT_VALVE           = 0x1001,   // CUTVALVE_MIN -> CUTVALVE_MAX
  PGN_FRONT_ZERO_BLADE_HEIGHT  = 0x1002,
  PGN_REAR_ZERO_BLADE_HEIGHT   = 0x1003,

  // blade configuration
  PGN_FRONT_PWM_GAIN_UP        = 0x2002,
  PGN_FRONT_PWM_GAIN_DOWN      = 0x2003,
  PGN_FRONT_PWM_MIN_UP         = 0x2004,
  PGN_FRONT_PWM_MIN_DOWN       = 0x2005,
  PGN_FRONT_PWM_MAX_UP         = 0x2006,
  PGN_FRONT_PWM_MAX_DOWN       = 0x2007,
  PGN_FRONT_INTEGRAL_MULTPLIER = 0x2008,
  PGN_FRONT_DEADBAND           = 0x2009,
  PGN_REAR_PWM_GAIN_UP         = 0x200A,
  PGN_REAR_PWM_GAIN_DOWN       = 0x200B,
  PGN_REAR_PWM_MIN_UP          = 0x200C,
  PGN_REAR_PWM_MIN_DOWN        = 0x200D,
  PGN_REAR_PWM_MAX_UP          = 0x200E,
  PGN_REAR_PWM_MAX_DOWN        = 0x200F,
  PGN_REAR_INTEGRAL_MULTPLIER  = 0x2010,
  PGN_REAR_DEADBAND            = 0x2011,

  // blade status
  PGN_FRONT_BLADE_OFFSET_SLAVE = 0x5000,
  PGN_FRONT_BLADE_PWMVALUE     = 0x5001,
  PGN_FRONT_BLADE_DIRECTION    = 0x5002,
  PGN_FRONT_CUTTING            = 0x5003,
  PGN_REAR_BLADE_OFFSET_SLAVE  = 0x5004,
  PGN_REAR_BLADE_PWMVALUE      = 0x5005,
  PGN_REAR_BLADE_DIRECTION     = 0x5006,
  PGN_REAR_CUTTING             = 0x5007,
  PGN_FRONT_BLADE_HEIGHT       = 0x5008,
  PGN_REAR_BLADE_HEIGHT        = 0x5009,
  PGN_FRONT_DUMPING            = 0x500A, // fixme - to do
  PGN_REAR_DUMPING             = 0x500B, // fixme - to do

  // IMU
  PGN_TRACTOR_IMU              = 0x6000,
  PGN_FRONT_IMU                = 0x6001,
  PGN_REAR_IMU                 = 0x6002,

  // GNSS
  PGN_TRACTOR_NMEA             = 0x7000,
  PGN_FRONT_NMEA               = 0x7001,
  PGN_REAR_NMEA                = 0x7002
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
        UDPTransfer *pUdpTransfer
      );

    // sends status value over UDP using packet framing
    void SendStatus
    (
      pgnpacket_t *pStatus
    );

  private:
    UDPTransfer *pUdpTransfer;
};

#endif // _AGGRADEH_
