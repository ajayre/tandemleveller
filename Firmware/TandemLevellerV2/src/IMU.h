// IMU handling

#ifndef _IMUH_
#define _IMUH_

#include <Arduino.h>
#include "Global.h"

typedef struct _imu_t
{
  float Roll;
  float Pitch;
  float Heading;
  float YawRate;
  uint8_t CalibrationStatus;
} imu_t;

typedef void (*imu_changed_callback_t)(uint8_t Index, imu_t *pimu);

class IMU
{
  public:
    imu_t IMUValues[NUM_BLADES + 1];

    // constructor
    IMU
      (
      void
      );

    // initializes the module
    void Init
      (
      void  
      );

    // process TPDO1 from IMU
    void ProcessIMUTPDO1
      (
      uint8_t NodeId,            // node that transmitted the PDO
      uint8_t Length,            // length of the PDO
      const uint8_t *pData       // PDO data
      );

    // process TPDO2 from IMU
    void ProcessIMUTPDO2
      (
      uint8_t NodeId,            // node that transmitted the PDO
      uint8_t Length,            // length of the PDO
      const uint8_t *pData       // PDO data
      );

    // Sets the callback functions
    void SetCallbacks
      (
      imu_changed_callback_t _IMUChanged     // called when an IMU has changed
      );

  private:
    imu_changed_callback_t IMUChanged;
};

#endif // _IMUH_
