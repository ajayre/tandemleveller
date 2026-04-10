// IMU handling

#include "IMU.h"
#include "CANopen.h"


///////////////////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTIONS


///////////////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTIONS

// constructor
IMU::IMU
  (
  void
  )
{
  IMUChanged = NULL;
}

// initializes the module
void IMU::Init
  (
  void  
  )
{
  // clear all IMU values
  for (int i = 0; i < NUM_BLADES + 1; i++)
  {
    memset(&IMUValues[i], 0, sizeof(imu_t));
  }
}

// Sets the callback functions
void IMU::SetCallbacks
  (
  imu_changed_callback_t _IMUChanged     // called when an IMU has changed
  )
{
  IMUChanged = _IMUChanged;
}

// process TPDO1 from IMU
void IMU::ProcessIMUTPDO1
  (
  uint8_t NodeId,            // node that transmitted the PDO
  uint8_t Length,            // length of the PDO
  const uint8_t *pData       // PDO data
  )
{
  if (Length == 8)
  {
    float Heading = ((uint16_t)(pData[0] | ((uint16_t)pData[1] << 8))) / 100.0;
    float Pitch   = ((int16_t)(pData[2] | ((uint16_t)pData[3] << 8))) / 100.0;
    float Roll    = ((int16_t)(pData[4] | ((uint16_t)pData[5] << 8))) / 100.0;
    float YawRate = ((int16_t)(pData[6] | ((uint16_t)pData[7] << 8))) / 100.0;

    switch (NodeId)
    {
      case TRACTOR_IMU_NODE_ID:
        IMUValues[TRACTOR_IDX].Heading = Heading;
        IMUValues[TRACTOR_IDX].Pitch   = Pitch;
        IMUValues[TRACTOR_IDX].Roll    = Roll;
        IMUValues[TRACTOR_IDX].YawRate = YawRate;
        if (IMUChanged != NULL) IMUChanged(TRACTOR_IDX, &IMUValues[TRACTOR_IDX]);
        break;

      case FRONTSCRAPER_IMU_NODE_ID:
        IMUValues[FRONT_BLADE_IDX].Heading = Heading;
        IMUValues[FRONT_BLADE_IDX].Pitch   = Pitch;
        IMUValues[FRONT_BLADE_IDX].Roll    = Roll;
        IMUValues[FRONT_BLADE_IDX].YawRate = YawRate;
        if (IMUChanged != NULL) IMUChanged(FRONT_BLADE_IDX, &IMUValues[FRONT_BLADE_IDX]);
        break;

      case REARSCRAPER_IMU_NODE_ID:
        IMUValues[REAR_BLADE_IDX].Heading = Heading;
        IMUValues[REAR_BLADE_IDX].Pitch   = Pitch;
        IMUValues[REAR_BLADE_IDX].Roll    = Roll;
        IMUValues[REAR_BLADE_IDX].YawRate = YawRate;
        if (IMUChanged != NULL) IMUChanged(REAR_BLADE_IDX, &IMUValues[REAR_BLADE_IDX]);
        break;
    }
  }
}

// process TPDO2 from IMU
void IMU::ProcessIMUTPDO2
  (
  uint8_t NodeId,            // node that transmitted the PDO
  uint8_t Length,            // length of the PDO
  const uint8_t *pData       // PDO data
  )
{
  if (Length == 1)
  {
    switch (NodeId)
    {
      case TRACTOR_IMU_NODE_ID:
        IMUValues[TRACTOR_IDX].CalibrationStatus = pData[0];
        break;
      case FRONTSCRAPER_IMU_NODE_ID:
        IMUValues[FRONT_BLADE_IDX].CalibrationStatus = pData[0];
        break;
      case REARSCRAPER_IMU_NODE_ID:
        IMUValues[REAR_BLADE_IDX].CalibrationStatus = pData[0];
        break;
    }
  }
}
