// IMU calibration and orientation in EEPROM

#include "EepromConfig.h"
#include <EEPROM.h>

#define EEPROM_IMU_ADDR ((int)0)

///////////////////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTIONS

// Zero entire store region so no stale magic remains at other offsets.
static void ClearStoreBytes
  (
  void
  )
{
  for (unsigned i = 0; i < sizeof(eeprom_imu_store_t); i++)
  {
    EEPROM.write(EEPROM_IMU_ADDR + (int)i, 0);
  }
}

///////////////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTIONS

// Fills outputs from EEPROM when Magic is valid; otherwise uses defaults.
void EepromConfigLoad
  (
  double *ZeroPitch,                // out: subtracted pitch offset in degrees before CAN
  double *ZeroRoll,                 // out: subtracted roll offset in degrees before CAN
  uint8_t *Orientation              // out: stored mount mode (see orientation_t)
  )
{
  eeprom_imu_store_t store;
  EEPROM.get(EEPROM_IMU_ADDR, store);

  if (store.Magic != EEPROMCONFIG_MAGIC_VALID)
  {
    *ZeroPitch   = EEPROMCONFIG_DEFAULT_ZERO_PITCH;
    *ZeroRoll    = EEPROMCONFIG_DEFAULT_ZERO_ROLL;
    *Orientation = EEPROMCONFIG_DEFAULT_ORIENTATION;
    return;
  }

  *ZeroPitch   = store.ZeroPitch;
  *ZeroRoll    = store.ZeroRoll;
  *Orientation = store.Orientation;
}

// Persists pitch/roll trim and orientation; Magic written with full record.
void EepromConfigSave
  (
  double ZeroPitch,                 // pitch trim (degrees)
  double ZeroRoll,                  // roll trim (degrees)
  uint8_t Orientation               // mount mode enum as uint8
  )
{
  eeprom_imu_store_t store;
  store.ZeroPitch   = ZeroPitch;
  store.ZeroRoll    = ZeroRoll;
  store.Orientation = Orientation;
  store.Magic       = EEPROMCONFIG_MAGIC_VALID;
  EEPROM.put(EEPROM_IMU_ADDR, store);
}

// Invalidates stored configuration (next Load returns defaults).
void EepromConfigClear
  (
  void
  )
{
  ClearStoreBytes();
}
