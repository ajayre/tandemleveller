// IMU handling

#include <math.h>
#include "IMU.h"
#include "CANopen.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// password to set the zero point, must be present in the RPDO
#define SET_ZERO_PASSWORD 0x23D4EE20

// password to set the orientation, must be present in the RPDO
#define SET_ORIENTATION_PASSWORD 0x739EAC22

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
  TxCANMessage = NULL;
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
    memset(ImuBuffer[i], 0, sizeof(ImuBuffer[i]));
    ImuBufferHead[i] = 0;
    ImuBufferCount[i] = 0;
  }
}

// retrieves the buffered sample closest to target_ms
bool IMU::GetSampleAtTime
  (
  uint8_t Index,                  // blade/tractor index
  uint32_t TargetMs,              // millis() timestamp to match
  imu_t *pResult                  // output: closest sample
  ) const
{
  if (Index >= NUM_BLADES + 1 || ImuBufferCount[Index] == 0)
  {
    return false;
  }

  int count = ImuBufferCount[Index];
  if (count > IMU_BUFFER_SIZE)
  {
    count = IMU_BUFFER_SIZE;
  }

  uint32_t best_diff = UINT32_MAX;
  int best_idx = -1;

  for (int i = 0; i < count; i++)
  {
    int slot = (ImuBufferHead[Index] - 1 - i + IMU_BUFFER_SIZE) % IMU_BUFFER_SIZE;
    uint32_t ts = ImuBuffer[Index][slot].timestamp_ms;
    uint32_t diff = (TargetMs >= ts) ? (TargetMs - ts) : (ts - TargetMs);
    if (diff < best_diff)
    {
      best_diff = diff;
      best_idx = slot;
    }
  }

  if (best_idx >= 0)
  {
    *pResult = ImuBuffer[Index][best_idx].values;
    return true;
  }
  return false;
}

// Sets the callback functions
void IMU::SetCallbacks
  (
  imu_changed_callback_t _IMUChanged,        // called when an IMU has changed
  imu_txcanmessage_callback_t _TxCANMessage  // called when IMU module wants to send a CAN message
  )
{
  IMUChanged   = _IMUChanged;
  TxCANMessage = _TxCANMessage;
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

    uint8_t idx = 0xFF;
    switch (NodeId)
    {
      case TRACTOR_IMU_NODE_ID:
        idx = TRACTOR_IDX;
        break;
      case FRONTSCRAPER_IMU_NODE_ID:
        idx = FRONT_BLADE_IDX;
        break;
      case REARSCRAPER_IMU_NODE_ID:
        idx = REAR_BLADE_IDX;
        break;
    }

    if (idx != 0xFF)
    {
      ApplyInputLowPass(idx, Heading, Pitch, Roll, YawRate);

      ImuBuffer[idx][ImuBufferHead[idx]].values = IMUValues[idx];
      ImuBuffer[idx][ImuBufferHead[idx]].timestamp_ms = millis();
      ImuBufferHead[idx] = (ImuBufferHead[idx] + 1) % IMU_BUFFER_SIZE;
      if (ImuBufferCount[idx] < IMU_BUFFER_SIZE)
      {
        ImuBufferCount[idx]++;
      }

      if (IMUChanged != NULL)
      {
        IMUChanged(idx, &IMUValues[idx]);
      }
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

// sets the pitch and roll to zero
void IMU::SetZero
  (
  uint8_t NodeId             // node to zero
  )
{
  uint8_t Data[4];

  Data[0] =  SET_ZERO_PASSWORD        & 0xFF;
  Data[1] = (SET_ZERO_PASSWORD >> 8)  & 0xFF;
  Data[2] = (SET_ZERO_PASSWORD >> 16) & 0xFF;
  Data[3] = (SET_ZERO_PASSWORD >> 24) & 0xFF;

  if (TxCANMessage != NULL)
  {
    TxCANMessage(0x200 + NodeId, 4, Data);
  }
}

// sets the orientation
void IMU::SetOrientation
  (
  uint8_t NodeId,                 // node to set orientation
  imu_orientation_t Orientation   // orientation to use
  )
{
  uint8_t Data[5];

  Data[0] =  SET_ORIENTATION_PASSWORD        & 0xFF;
  Data[1] = (SET_ORIENTATION_PASSWORD >> 8)  & 0xFF;
  Data[2] = (SET_ORIENTATION_PASSWORD >> 16) & 0xFF;
  Data[3] = (SET_ORIENTATION_PASSWORD >> 24) & 0xFF;
  Data[4] = (uint8_t)Orientation;

  if (TxCANMessage != NULL)
  {
    TxCANMessage(0x300 + NodeId, 5, Data);
  }
}
