// IMU handling

#include <math.h>
#include "IMU.h"
#include "CANopen.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// EMA on IMU samples (0..1, higher = trust new sample more). Prototype — tune for field.
static constexpr float kImuLpAlphaRollPitch = 0.30f;
static constexpr float kImuLpAlphaHeading  = 0.22f;
static constexpr float kImuLpAlphaYawRate   = 0.28f;

///////////////////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTIONS

void IMU::ApplyInputLowPass
  (
  uint8_t Index,
  float Heading,
  float Pitch,
  float Roll,
  float YawRate
  )
{
  ImuLpState *st = &imu_lp_[Index];

  if (!st->init)
  {
    IMUValues[Index].Heading = Heading;
    IMUValues[Index].Pitch   = Pitch;
    IMUValues[Index].Roll    = Roll;
    IMUValues[Index].YawRate = YawRate;
    const float hr = Heading * (float)(M_PI / 180.0);
    st->h_cos = cosf(hr);
    st->h_sin = sinf(hr);
    st->roll = Roll;
    st->pitch = Pitch;
    st->yaw_rate = YawRate;
    st->init = true;
    return;
  }

  st->roll = kImuLpAlphaRollPitch * Roll
    + (1.0f - kImuLpAlphaRollPitch) * st->roll;
  st->pitch = kImuLpAlphaRollPitch * Pitch
    + (1.0f - kImuLpAlphaRollPitch) * st->pitch;
  st->yaw_rate = kImuLpAlphaYawRate * YawRate
    + (1.0f - kImuLpAlphaYawRate) * st->yaw_rate;

  const float hr = Heading * (float)(M_PI / 180.0);
  const float cn = cosf(hr);
  const float sn = sinf(hr);
  st->h_cos = kImuLpAlphaHeading * cn
    + (1.0f - kImuLpAlphaHeading) * st->h_cos;
  st->h_sin = kImuLpAlphaHeading * sn
    + (1.0f - kImuLpAlphaHeading) * st->h_sin;

  float h_deg = atan2f(st->h_sin, st->h_cos) * (float)(180.0 / M_PI);
  while (h_deg < 0.0f)
  {
    h_deg += 360.0f;
  }
  while (h_deg >= 360.0f)
  {
    h_deg -= 360.0f;
  }

  IMUValues[Index].Heading = h_deg;
  IMUValues[Index].Pitch   = st->pitch;
  IMUValues[Index].Roll    = st->roll;
  IMUValues[Index].YawRate = st->yaw_rate;
}


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
    imu_lp_[i].init = false;
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
        ApplyInputLowPass(TRACTOR_IDX, Heading, Pitch, Roll, YawRate);
        if (IMUChanged != NULL) IMUChanged(TRACTOR_IDX, &IMUValues[TRACTOR_IDX]);
        break;

      case FRONTSCRAPER_IMU_NODE_ID:
        ApplyInputLowPass(FRONT_BLADE_IDX, Heading, Pitch, Roll, YawRate);
        if (IMUChanged != NULL) IMUChanged(FRONT_BLADE_IDX, &IMUValues[FRONT_BLADE_IDX]);
        break;

      case REARSCRAPER_IMU_NODE_ID:
        ApplyInputLowPass(REAR_BLADE_IDX, Heading, Pitch, Roll, YawRate);
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
