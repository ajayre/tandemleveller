// Bosch BNO08x (BNO085-class) SPI driver: fusion output and calibration for IMU.cpp

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <SPI.h>
#include "EepromConfig.h"
#include "BNO085.h"

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (57.295779513082320876798154814105f)
#endif


///////////////////////////////////////////////////////////////////////////////////
// PUBLIC

// Constructor — wires the Adafruit instance to the board reset pin; does not touch SPI.
BNO085::BNO085
  (
  void
  )
  : bno08x_(kPinRst)
{
}

// Powers nominal SPI0 (Teensy: MOSI 11, MISO 12, SCK 13), drives /CS, /INT, /RST,
// opens the SH-2 stack, loads EEPROM trim and stored orientation, applies mount mode,
// and enables rotation + gyro + geomag reports at 50 ms.
void BNO085::Init
  (
  void
  )
{
  sensor_found_ = false;
  sensor_enabled_ = false;
  calibration_status_ = 0U;
  memset(&ypr_, 0, sizeof(ypr_));
  pending_set_zero_ = false;

  zero_pitch_ = EEPROMCONFIG_DEFAULT_ZERO_PITCH;
  zero_roll_ = EEPROMCONFIG_DEFAULT_ZERO_ROLL;

  SPI.begin();

  if (bno08x_.begin_SPI(kPinCs, kPinInt, &SPI))
  {
    sensor_found_ = true;

    uint8_t ori_stored = EEPROMCONFIG_DEFAULT_ORIENTATION;
    EepromConfigLoad(&zero_pitch_, &zero_roll_, &ori_stored);
    if (ori_stored > (uint8_t)BNO085_ORIENTATION_VERTICAL_A)
    {
      ori_stored = (uint8_t)BNO085_ORIENTATION_HORIZONTAL_A;
    }
    current_orientation_ = (bno085_orientation_t)ori_stored;

    ApplyOrientationMode(current_orientation_);

    SetReports(kReportTypeRotation, kReportIntervalUs);
    SetReports(SH2_GYROSCOPE_CALIBRATED, kReportIntervalUs);
    SetReports(SH2_GEOMAGNETIC_ROTATION_VECTOR, kReportIntervalUs);
  }
}

// Polls sh2 (via getSensorEvent), restores reports after hub reset, runs the
// quaternion/trim pipeline, then fills outputs. All pointer parameters must be non-NULL.
void BNO085::Read
  (
  float *pHeading,
  float *pPitch,
  float *pRoll,
  float *pYawRate,
  uint8_t *pCalibrationStatus
  )
{
  if (!pHeading || !pPitch || !pRoll || !pYawRate || !pCalibrationStatus)
  {
    return;
  }

  if (sensor_found_ && sensor_enabled_)
  {
    if (bno08x_.wasReset())
    {
      ApplyOrientationMode(current_orientation_);
      SetReports(kReportTypeRotation, kReportIntervalUs);
      SetReports(SH2_GYROSCOPE_CALIBRATED, kReportIntervalUs);
      SetReports(SH2_GEOMAGNETIC_ROTATION_VECTOR, kReportIntervalUs);
    }

    while (bno08x_.getSensorEvent(&sensor_value_))
    {
      ProcessSensorValue(&sensor_value_);
    }
  }

  *pHeading = ypr_.yaw;
  *pPitch = ypr_.pitch;
  *pRoll = ypr_.roll;
  *pYawRate = ypr_.yawrate;
  *pCalibrationStatus = calibration_status_;
}

// Clamps invalid orientation values, clears trim, reapplies mount mapping, saves EEPROM.
void BNO085::SetOrientation
  (
  bno085_orientation_t new_orientation
  )
{
  if (new_orientation > BNO085_ORIENTATION_VERTICAL_A)
  {
    new_orientation = BNO085_ORIENTATION_HORIZONTAL_A;
  }

  zero_pitch_ = 0.0;
  zero_roll_ = 0.0;

  ApplyOrientationMode(new_orientation);

  EepromConfigSave(zero_pitch_, zero_roll_, (uint8_t)new_orientation);
}

// Queues zero calibration — applied inside ProcessSensorValue on next fusion update.
void BNO085::SetZero
  (
  void
  )
{
  pending_set_zero_ = true;
}


///////////////////////////////////////////////////////////////////////////////////
// PRIVATE

// Requests one SH-2 sensor report at the given interval. sensor_enabled_ reflects the
// last enableReport result.
void BNO085::SetReports
  (
  sh2_SensorId_t report_type,
  uint32_t       report_interval_us
  )
{
  sensor_enabled_ = false;
  if (bno08x_.enableReport(report_type, report_interval_us))
  {
    sensor_enabled_ = true;
  }
}

// Converts a unit quaternion (w,x,y,z as qr,qi,qj,qk) to intrinsic yaw/pitch/roll.
void BNO085::QuaternionToEuler
  (
  float    qr,
  float    qi,
  float    qj,
  float    qk,
  euler_t *ypr,
  bool     degrees
  )
{
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2f(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asinf(-2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2f(2.0f * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees)
  {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

// Maps fused game rotation quaternion to application yaw/pitch/roll
void BNO085::FusedQuaternionToYpr
  (
  float    qi,
  float    qj,
  float    qk,
  float    qr,
  euler_t *out_ypr,
  bool     degrees
  )
{
  QuaternionToEuler(qr, qi, qj, qk, out_ypr, degrees);
  if (degrees)
  {
    out_ypr->yaw -= 90.0f;
    if (current_orientation_ == BNO085_ORIENTATION_VERTICAL_A)
    {
      out_ypr->roll -= 90.0f;
    }
    else
    {
      out_ypr->roll += 180.0f;
    }
  }
  else
  {
    out_ypr->yaw -= (float)(PI / 2.0);
    if (current_orientation_ == BNO085_ORIENTATION_VERTICAL_A)
    {
      out_ypr->roll -= (float)(PI / 2.0);
    }
    else
    {
      out_ypr->roll += (float)PI;
    }
  }
}

// Wraps a degree value to [-180, +180] (e.g. 190 -> -170; -190 -> 170).
void BNO085::WrapDegrees180
  (
  float *a_deg
  )
{
  float x = *a_deg;
  x = fmodf(x + 180.0f, 360.0f);
  if (x < 0.0f)
  {
    x += 360.0f;
  }
  *a_deg = x - 180.0f;
}

// Updates mount mode, yaw-rate axis, and sh2 reorientation identity. Does not change
// pitch/roll trim in zero_pitch_ / zero_roll_.
void BNO085::ApplyOrientationMode
  (
  bno085_orientation_t orientation
  )
{
  current_orientation_ = orientation;

  gyr_rot_[0][0] = 1.0f;
  gyr_rot_[0][1] = 0.0f;
  gyr_rot_[0][2] = 0.0f;
  gyr_rot_[1][0] = 0.0f;
  gyr_rot_[1][1] = 1.0f;
  gyr_rot_[1][2] = 0.0f;
  gyr_rot_[2][0] = 0.0f;
  gyr_rot_[2][1] = 0.0f;
  gyr_rot_[2][2] = 1.0f;

  if (orientation == BNO085_ORIENTATION_VERTICAL_A)
  {
    yaw_rate_axis_[0] = 0.0f;
    yaw_rate_axis_[1] = 1.0f;
    yaw_rate_axis_[2] = 0.0f;
  }
  else
  {
    yaw_rate_axis_[0] = 0.0f;
    yaw_rate_axis_[1] = 0.0f;
    yaw_rate_axis_[2] = 1.0f;
  }

  sh2_Quaternion_t q_id;
  q_id.x = 0.0f;
  q_id.y = 0.0f;
  q_id.z = 0.0f;
  q_id.w = 1.0f;
  (void)sh2_setReorientation(&q_id);
}

// Handles one decoded sh2_SensorValue_t; applies conventions and EEPROM trim.
void BNO085::ProcessSensorValue
  (
  sh2_SensorValue_t *p_val
  )
{
  bool orientation_updated = false;

  switch (p_val->sensorId)
  {
    case SH2_ROTATION_VECTOR:
      FusedQuaternionToYpr(
        p_val->un.rotationVector.i,
        p_val->un.rotationVector.j,
        p_val->un.rotationVector.k,
        p_val->un.rotationVector.real,
        &ypr_,
        true);
      calibration_status_ = p_val->status & 0x03U;
      orientation_updated = true;
      break;

    case SH2_GYROSCOPE_CALIBRATED:
      {
        float wx = p_val->un.gyroscope.x;
        float wy = p_val->un.gyroscope.y;
        float wz = p_val->un.gyroscope.z;
        float ox = gyr_rot_[0][0] * wx + gyr_rot_[0][1] * wy + gyr_rot_[0][2] * wz;
        float oy = gyr_rot_[1][0] * wx + gyr_rot_[1][1] * wy + gyr_rot_[1][2] * wz;
        float oz = gyr_rot_[2][0] * wx + gyr_rot_[2][1] * wy + gyr_rot_[2][2] * wz;
        ypr_.yawrate = (ox * yaw_rate_axis_[0] + oy * yaw_rate_axis_[1]
            + oz * yaw_rate_axis_[2]) * RAD_TO_DEG;
      }
      break;

    case SH2_GEOMAGNETIC_ROTATION_VECTOR:
      // Heading from game rotation vector; geomag report kept enabled for fusion.
      break;

    default:
      break;
  }

  if (orientation_updated)
  {
    if (ypr_.yaw < 0.0f)
    {
      ypr_.yaw += 360.0f;
    }
    ypr_.yaw = 360.0f - ypr_.yaw;

    ypr_.pitch = -ypr_.pitch;

    WrapDegrees180(&ypr_.pitch);
    WrapDegrees180(&ypr_.roll);

    if (pending_set_zero_)
    {
      pending_set_zero_ = false;
      zero_pitch_ = (double)ypr_.pitch;
      zero_roll_ = (double)ypr_.roll;
      EepromConfigSave(zero_pitch_, zero_roll_, (uint8_t)current_orientation_);
      ypr_.pitch = 0.0f;
      ypr_.roll = 0.0f;
    }
    else
    {
      ypr_.pitch -= (float)zero_pitch_;
      ypr_.roll -= (float)zero_roll_;
      WrapDegrees180(&ypr_.pitch);
      WrapDegrees180(&ypr_.roll);
    }
  }
}
