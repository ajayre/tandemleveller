// IMU calibration and orientation in EEPROM

#ifndef _EEPROMCONFIGH_
#define _EEPROMCONFIGH_

#include <Arduino.h>

// Byte written last when saving; must match for stored data to be used.
#define EEPROMCONFIG_MAGIC_VALID ((uint8_t)0xA7)

// Defaults when EEPROM is blank, invalid, or after clear.
#define EEPROMCONFIG_DEFAULT_ZERO_PITCH   (0.0)
#define EEPROMCONFIG_DEFAULT_ZERO_ROLL    (0.0)
#define EEPROMCONFIG_DEFAULT_ORIENTATION  ((uint8_t)0)

typedef struct __attribute__((packed)) _eeprom_imu_store_t
{
  double ZeroPitch;
  double ZeroRoll;
  uint8_t Orientation;
  uint8_t Magic;
} eeprom_imu_store_t;

// Fills outputs from EEPROM when Magic is valid; otherwise uses defaults.
void EepromConfigLoad
  (
  double *ZeroPitch,                // out: subtracted pitch offset in degrees before CAN
  double *ZeroRoll,                 // out: subtracted roll offset in degrees before CAN
  uint8_t *Orientation              // out: stored mount mode (see orientation_t)
  );

// Persists pitch/roll trim and orientation (single EEPROM.put record, Magic byte included).
void EepromConfigSave
  (
  double ZeroPitch,                 // pitch trim (degrees)
  double ZeroRoll,                  // roll trim (degrees)
  uint8_t Orientation               // mount mode enum as uint8
  );

// Invalidates stored configuration (next Load returns defaults).
void EepromConfigClear
  (
  void
  );

#endif // _EEPROMCONFIGH_
