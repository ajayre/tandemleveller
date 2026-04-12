// global definitions

#ifndef _GLOBALH_
#define _GLOBALH_

#include <Arduino.h>

// maximum length of an NMEA 0183 sentence
#define MAX_NMEA_LENGTH 90

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

#endif // _GLOBALH_
