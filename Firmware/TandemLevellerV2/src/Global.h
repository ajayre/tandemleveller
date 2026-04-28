// global definitions

#ifndef _GLOBALH_
#define _GLOBALH_

#include <Arduino.h>

// maximum length of an NMEA 0183 sentence
#define MAX_NMEA_LENGTH 128

// indices into arrays
#define FRONT_BLADE_IDX 0
#define REAR_BLADE_IDX  1
#define TRACTOR_IDX     2

// number of blades we support
#define NUM_BLADES 2

// the height of the blade that represents ground level
// when communicating with AgGrade
#define BLADE_HEIGHT_GROUND_LEVEL 200

// Antenna lever arm for IMU fusion: height mm; left mm (+ = port); forward mm (+ = ahead)
typedef struct _antenna_location_t
{
  uint32_t HeightMm;
  int32_t LeftMm;
  int32_t ForwardMm;
} antenna_location_t;

// supported PGNs
typedef enum _pgn_t : uint16_t
{
  // misc
  PGN_ESTOP                      = 0x0000,
  PGN_RESET                      = 0x0001,
  PGN_AGGRADE_STARTED            = 0x0002,
  PGN_PING                       = 0x0003,
  PGN_CLEAR_ESTOP                = 0x0004,
  PGN_YOU_ARE_SECONDARY          = 0x0005,  // fixme - to do

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

  // sensors
  PGN_TRACTOR_IMU                = 0x6000,
  PGN_FRONT_IMU                  = 0x6001,
  PGN_REAR_IMU                   = 0x6002,
  PGN_FRONT_APRON_IMU            = 0x6003, // fixme - to do
  PGN_FRONT_BUCKET_IMU           = 0x6004, // fixme - to do
  PGN_REAR_BUCKET_IMU            = 0x6005, // fixme - to do
  PGN_TRACTOR_IMU_FOUND          = 0x6006, // fixme - to do
  PGN_TRACTOR_IMU_LOST           = 0x6007, // fixme - to do
  PGN_FRONT_IMU_FOUND            = 0x6008, // fixme - to do
  PGN_FRONT_IMU_LOST             = 0x6009, // fixme - to do
  PGN_REAR_IMU_FOUND             = 0x600A, // fixme - to do
  PGN_REAR_IMU_LOST              = 0x600B, // fixme - to do
  PGN_FRONT_APRON_IMU_FOUND      = 0x600C, // fixme - to do
  PGN_FRONT_APRON_IMU_LOST       = 0x600D, // fixme - to do
  PGN_FRONT_BUCKET_IMU_FOUND     = 0x600E, // fixme - to do
  PGN_FRONT_BUCKET_IMU_LOST      = 0x600F, // fixme - to do
  PGN_REAR_BUCKET_IMU_FOUND      = 0x6010, // fixme - to do
  PGN_REAR_BUCKET_IMU_LOST       = 0x6011, // fixme - to do
  PGN_FRONT_HEIGHT_FOUND         = 0x6012, // fixme - to do
  PGN_FRONT_HEIGHT_LOST          = 0x6013, // fixme - to do
  PGN_REAR_HEIGHT_FOUND          = 0x6014, // fixme - to do
  PGN_REAR_HEIGHT_LOST           = 0x6015, // fixme - to do

  PGN_TRACTOR_IMU_LEVEL          = 0x6016,
  PGN_FRONT_IMU_LEVEL            = 0x6017,
  PGN_REAR_IMU_LEVEL             = 0x6018,
  PGN_FRONT_APRON_IMU_LEVEL      = 0x6019, // fixme - to do
  PGN_FRONT_BUCKET_IMU_LEVEL     = 0x601A,
  PGN_REAR_BUCKET_IMU_LEVEL      = 0x601B,
  PGN_TRACTOR_IMU_ORIENT         = 0x601C,
  PGN_FRONT_IMU_ORIENT           = 0x601D,
  PGN_REAR_IMU_ORIENT            = 0x601E,
  PGN_FRONT_APRON_IMU_ORIENT     = 0x601F, // fixme - to do
  PGN_FRONT_BUCKET_IMU_ORIENT    = 0x6020,
  PGN_REAR_BUCKET_IMU_ORIENT     = 0x6021,

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
  PGN_MAGNETIC_DECLINATION       = 0x8009,
} pgn_t;

#endif // _GLOBALH_
