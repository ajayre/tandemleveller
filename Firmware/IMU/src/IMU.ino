#include <Arduino.h>
#include "FlexCAN_T4-master/FlexCAN_T4.h"
#include "WDT_T4-master/Watchdog_t4.h"
#include <Adafruit_BNO08x.h>
#include "EepromConfig.h"

// roll left = negative, roll right = positive
// pitch forwards = positive, pitch backwards = negative
// yaw clockwise (viewed from above) = increases angle, anti-clockwise = decreases angle

// "CANopen" node id
#if TRACTOR == 1
  #define NODE_ID 0x02
  #define TPDO_TX_PERIOD_MS 2
#elif FRONTSCRAPER == 1
  #define NODE_ID 0x03
  #define TPDO_TX_PERIOD_MS 10
#else
  #define NODE_ID 0x04
#define TPDO_TX_PERIOD_MS 10
#endif

// BNO08x GPIO pins
#define BNO08X_CS    10
#define BNO08X_INT   16
#define BNO08X_RESET 15

#define NMT_STATE_BOOTUP      0x00
#define NMT_STATE_OPERATIONAL 0x05

// NMT commands
#define NMT_CMD_RESET 0x81

// special value to reset all nodes
#define NMT_RESET_ALL 0x00

// time between heartbeats in millseconds
#define HB_PRODUCER_TIME_MS 100

// password to set the zero point, must be present in the RPDO
#define SET_ZERO_PASSWORD 0x23D4EE20

// password to set the orientation, must be present in the RPDO
#define SET_ORIENTATION_PASSWORD 0x739EAC22

// Mount / output frame selection. Reference fix is applied in firmware on the fused quaternion
// (not via sh2_setReorientation — that uses a different convention and scrambles YPR).
// Both modes use application +X = magnetic north; they differ in which chip axis is "up".
typedef enum orientation_t
{
  // PCB flat: application +Z is up (out of the board). Matches the legacy firmware convention
  // that used a -90 deg yaw correction so +X points north instead of fusion default +Y north.
  ORIENTATION_HORIZONTAL_A = 0,
  // Board edge-on (chip +Y toward world up, +X along north). Fusion quaternion already includes
  // this physical attitude — do not apply an extra +90 deg X quaternion in software.
  ORIENTATION_VERTICAL_A   = 1
} orientation_t;

struct euler_t
{
  float yaw;
  float pitch;
  float roll;
  float yawrate;  // deg/s
} ypr;

static Adafruit_BNO08x bno08x(BNO08X_RESET);
static sh2_SensorValue_t sensorValue;
static sh2_SensorId_t reportType = SH2_ROTATION_VECTOR;
static long reportIntervalUs = 50000;
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANBus;
static bool SensorFound = false;
static bool SensorEnabled = false;
elapsedMillis HBTime;
elapsedMillis TPDOTxTime;
static WDT_T4<WDT1> wdt;
static uint8_t CalibrationStatus;

// Stored in degrees: total subtracted from processed pitch/roll before CAN (see SetCalibration).
static double ZeroPitch;
static double ZeroRoll;

// Set by CAN RPDO; applied in loop after full pitch/roll pipeline (avoids reading partial ypr).
static volatile bool PendingCalibrateRequest;

// Last orientation applied via SetOrientation (re-applied after hub reset).
static orientation_t CurrentOrientation;

// Rotation matrix R: gyro in calibrated sensor frame -> application frame (rad/s).
// Both mounts use identity here; vertical vs horizontal only changes which gyro axis is "yaw rate".
static float GyrRot[3][3];

// Unit vector in application frame: axis for "yaw rate" = dot(GyrRot * ω, YawRateAxis).
static float YawRateAxis[3];

static void ApplyOrientationMode(orientation_t Orientation);
static void SetOrientation(orientation_t Orientation);

// reboot the device
void Reboot
  (
  void
  )
{
  SCB_AIRCR = 0x05FA0004;
}

static void setReports
  (
  sh2_SensorId_t reportType,
  long report_interval
  )
{
  SensorEnabled = false;
  if (bno08x.enableReport(reportType, report_interval))
  {
    SensorEnabled = true;
  }
}

static void quaternionToEuler
  (
  float qr, 
  float qi,
  float qj,
  float qk,
  euler_t *ypr,
  bool degrees = false
  )
{
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees)
  {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

static void quaternionToEulerRV
  (
  sh2_RotationVectorWAcc_t *rotational_vector,
  euler_t *ypr,
  bool degrees = false
  )
{
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

static void quaternionToEulerGI
  (
  sh2_GyroIntegratedRV_t *rotational_vector,
  euler_t *ypr,
  bool degrees = false
  )
{
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

static void quarternionToEulerGRV
  (
  sh2_RotationVectorWAcc_t *rotational_vector,
  euler_t *ypr,
  bool degrees = false
  )
{
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

static float FindHeading
  (
  sh2_RotationVectorWAcc_t *rotational_vector
  )
{
  float dqw = rotational_vector->real;
  float dqx = rotational_vector->i;
  float dqy = rotational_vector->j;
  float dqz = rotational_vector->k;

  float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
  dqw = dqw / norm;
  dqx = dqx / norm;
  dqy = dqy / norm;
  dqz = dqz / norm;

  float ysqr = dqy * dqy;

  float t3 = +2.0 * (dqw * dqz + dqx * dqy);
  float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
  float yaw_raw = atan2(t3, t4);
  float yaw = yaw_raw * 180.0 / PI;
  if (yaw > 0)
  {
    yaw = 360 - yaw;
  }
  else
  {
    yaw = abs(yaw);
  }
    
  return yaw;
}

// Wrap angle in degrees to [-180, +180] (e.g. 190 -> -170; -190 -> 170).
static void wrapDegrees180
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

// Fused quaternion -> ypr. Fusion already encodes physical mount; yaw -= 90 matches +X north.
// HORIZONTAL_A: nominal level pose can report roll = -180 deg; add 180 so level reads 0.
// VERTICAL_A: nominal pose reports ~+90 deg roll; subtract 90 so level reads 0.
// Pitch/roll are wrapped to [-180,+180] after sign/yaw fixes in loop (all orientations).
static void fusedQuaternionToYpr
  (
  float qi,
  float qj,
  float qk,
  float qr,
  euler_t *outYpr,
  bool degrees
  )
{
  quaternionToEuler(qr, qi, qj, qk, outYpr, degrees);
  if (degrees)
  {
    outYpr->yaw -= 90.0f;
    if (CurrentOrientation == ORIENTATION_VERTICAL_A)
    {
      outYpr->roll -= 90.0f;
    }
    else
    {
      outYpr->roll += 180.0f;
    }
  }
  else
  {
    outYpr->yaw -= (float)(PI / 2.0);
    if (CurrentOrientation == ORIENTATION_VERTICAL_A)
    {
      outYpr->roll -= (float)(PI / 2.0);
    }
    else
    {
      outYpr->roll += (float)PI;
    }
  }
}

// transmits a CAN message
static void TxCANMessage
  (
  uint16_t Id,
  uint8_t Length,
  uint8_t Data[]
  )
{
  CAN_message_t txmsg;
  txmsg.id = Id;
  txmsg.len = Length;
  for (uint8_t i = 0; i < Length; i++ ) txmsg.buf[i] = Data[i];
  CANBus.write(txmsg);

  /*char txt[50];
  sprintf(txt, "Tx: %3.3X %d ", Id, Length);
  Serial.print(txt);
  for (uint8_t i = 0; i < Length; i++)
  {
    sprintf(txt, "%2.2X ", Data[i]);
    Serial.print(txt);
  }
  Serial.println();*/
}

// transmits a bootup message
static void TxBootup
  (
  void
  )
{
  uint8_t Data[1];

  Data[0] = NMT_STATE_BOOTUP;
  TxCANMessage(0x700 + NODE_ID, 1, Data);
}

// transmits a heartbeat message
static void TxHeartbeat
  (
  void
  )
{
  uint8_t Data[1];

  Data[0] = NMT_STATE_OPERATIONAL;
  TxCANMessage(0x700 + NODE_ID, 1, Data);
}

// transmits PDO containing the measurements and status information
static void TxPDOs
  (
  euler_t *pMeasurements
  )
{
  uint8_t Data[8];
  int16_t Value;

  Value = (int16_t)(pMeasurements->yaw * 100);
  Data[0] = ((uint16_t)Value) & 0xFF;
  Data[1] = (((uint16_t)Value) >> 8) & 0xFF;

  Value = (int16_t)(pMeasurements->pitch * 100);
  Data[2] = ((uint16_t)Value) & 0xFF;
  Data[3] = (((uint16_t)Value) >> 8) & 0xFF;

  Value = (int16_t)(pMeasurements->roll * 100);
  Data[4] = ((uint16_t)Value) & 0xFF;
  Data[5] = (((uint16_t)Value) >> 8) & 0xFF;

  Value = (int16_t)(pMeasurements->yawrate * 100);
  Data[6] = ((uint16_t)Value) & 0xFF;
  Data[7] = (((uint16_t)Value) >> 8) & 0xFF;

  char txt[50];
  sprintf(txt, "%.2f %.2f %.2f %.2f %d", pMeasurements->pitch, pMeasurements->roll, pMeasurements->yaw, pMeasurements->yawrate, CalibrationStatus);
  Serial.print(txt);
  Serial.println();

  TxCANMessage(0x180 + NODE_ID, 8, Data);

  Data[0] = CalibrationStatus;
  TxCANMessage(0x280 + NODE_ID, 1, Data);
}

// clears the current pith and roll calibration
static void ClearCalibration
  (
  void 
  )
{
  ZeroPitch = 0;
  ZeroRoll = 0;

  EepromConfigSave(ZeroPitch, ZeroRoll, (uint8_t)CurrentOrientation);
}

// Calibrate so current attitude reads pitch = 0, roll = 0 on CAN (any orientation).
// Call with ypr.pitch/roll = measured values after frame fixes and wrap, before subtracting Zero*.
// CAN output uses (measured − Zero); we need Zero = measured so the current pose reads zero.
// Zero += measured would double-count once a trim is already applied (second 0x202 adds error).
static void SetCalibration
  (
  void
  )
{
  ZeroPitch = (double)ypr.pitch;
  ZeroRoll = (double)ypr.roll;

  EepromConfigSave(ZeroPitch, ZeroRoll, (uint8_t)CurrentOrientation);
}

// called when a CAN message is received
static void CANReceiveHandler
  (
  const CAN_message_t &msg
  )
{
  // process NMT message
  if ((msg.id == 0x000) && !msg.flags.extended && (msg.len == 2))
  {
    // reset
    if (msg.buf[0] == NMT_CMD_RESET)
    {
      // this node
      if ((msg.buf[1] == NODE_ID) || (msg.buf[1] == NMT_RESET_ALL))
      {
        Reboot();
      }
    }
  }
  // process RPDO1 (DLC may be 8; only first 4 bytes are the password, little-endian)
  else if ((msg.id == 0x200 + NODE_ID) && !msg.flags.extended && (msg.len >= 4))
  {
    if ((msg.buf[0] == (SET_ZERO_PASSWORD & 0xFF)) &&
        (msg.buf[1] == ((SET_ZERO_PASSWORD >> 8) & 0xFF)) &&
        (msg.buf[2] == ((SET_ZERO_PASSWORD >> 16) & 0xFF)) &&
        (msg.buf[3] == ((SET_ZERO_PASSWORD >> 24) & 0xFF)))
    {
      PendingCalibrateRequest = true;
    }
  }
  // process RPDO2
  else if ((msg.id == 0x300 + NODE_ID) && !msg.flags.extended && (msg.len >= 5))
  {
    // if password is correct
    if ((msg.buf[0] == (SET_ORIENTATION_PASSWORD & 0xFF)) &&
        (msg.buf[1] == ((SET_ORIENTATION_PASSWORD >> 8) & 0xFF)) &&
        (msg.buf[2] == ((SET_ORIENTATION_PASSWORD >> 16) & 0xFF)) &&
        (msg.buf[3] == ((SET_ORIENTATION_PASSWORD >> 24) & 0xFF)))
    {
      orientation_t o = (orientation_t)msg.buf[4];
      if (o > ORIENTATION_VERTICAL_A)
      {
        o = ORIENTATION_HORIZONTAL_A;
      }
      SetOrientation(o);
      ClearCalibration();
    }
  }
}

// Apply mount mode to hub + gyro axes only (does not change ZeroPitch/ZeroRoll). Safe from wasReset.
static void ApplyOrientationMode
  (
  orientation_t Orientation
  )
{
  CurrentOrientation = Orientation;

  GyrRot[0][0] = 1.0f;
  GyrRot[0][1] = 0.0f;
  GyrRot[0][2] = 0.0f;
  GyrRot[1][0] = 0.0f;
  GyrRot[1][1] = 1.0f;
  GyrRot[1][2] = 0.0f;
  GyrRot[2][0] = 0.0f;
  GyrRot[2][1] = 0.0f;
  GyrRot[2][2] = 1.0f;

  if (Orientation == ORIENTATION_VERTICAL_A)
  {
    // Package +Y toward world up: heading rate is mostly rotation about body Y (rad/s -> deg/s below).
    YawRateAxis[0] = 0.0f;
    YawRateAxis[1] = 1.0f;
    YawRateAxis[2] = 0.0f;
  }
  else
  {
    // Flat board: spin about package Z when level.
    YawRateAxis[0] = 0.0f;
    YawRateAxis[1] = 0.0f;
    YawRateAxis[2] = 1.0f;
  }

  // Undo any prior hub tare/reorientation so roll/pitch/yaw stay consistent with our math.
  sh2_Quaternion_t qId;
  qId.x = 0.0;
  qId.y = 0.0;
  qId.z = 0.0;
  qId.w = 1.0;
  if (sh2_setReorientation(&qId) != SH2_OK)
  {
    Serial.println(F("sh2_setReorientation(identity) failed"));
  }
}

// User/CAN: change mount mode, clear pitch/roll trim in RAM, persist orientation
// Do not call from wasReset — that would erase calibration every hub reset.
static void SetOrientation
  (
  orientation_t Orientation
  )
{
  ZeroPitch = 0;
  ZeroRoll = 0;

  ApplyOrientationMode(Orientation);

  EepromConfigSave(ZeroPitch, ZeroRoll, (uint8_t)Orientation);
}

// initialize the hardware
void setup()
{
  Serial.begin(115200);
  Serial.println("Tandem Leveller IMU");
  Serial.print(F("CAN NODE_ID=0x"));
  Serial.print(NODE_ID, HEX);
  Serial.print(F("  RPDO2 orientation COB-ID=0x"));
  Serial.println(0x300u + NODE_ID, HEX);

  // set up watchdog
  WDT_timings_t config;
  config.trigger = 2; // in seconds, 0->128
  config.timeout = 3; // in seconds, 0->128
  wdt.begin(config);

  CANBus.begin();
  CANBus.setBaudRate(125000);
  CANBus.setMaxMB(64);
  CANBus.enableFIFO();
  CANBus.onReceive(FIFO, CANReceiveHandler);
  CANBus.enableFIFOInterrupt();
  CANBus.setFIFOFilter(0, 0x000, STD);
  CANBus.setFIFOFilter(1, 0x200 + NODE_ID, STD);
  CANBus.setFIFOFilter(2, 0x300 + NODE_ID, STD);
  CANBus.setMB(MB63, TX); // Set mailbox as transmit

  // FlexCAN_T4: until events() runs once, RX invokes onReceive from the CAN ISR (see
  // struct2queueRx isEventsUsed). RPDO2 calls sh2_setReorientation (SPI) — must not run in ISR.
  CANBus.events();

  TxBootup();
  HBTime = 0;

  TPDOTxTime = 0;

  CalibrationStatus = 0;

  // find sensor
  if (bno08x.begin_SPI(BNO08X_CS, BNO08X_INT))
  {
    SensorFound = true;
    // Hub open: apply mount + trim from EEPROM; do not call SetOrientation here or we clear calibration.
    uint8_t OriStored = EEPROMCONFIG_DEFAULT_ORIENTATION;
    EepromConfigLoad(&ZeroPitch, &ZeroRoll, &OriStored);
    if (OriStored > ORIENTATION_VERTICAL_A)
    {
      OriStored = ORIENTATION_HORIZONTAL_A;
    }
    CurrentOrientation = (orientation_t)OriStored;
    ApplyOrientationMode(CurrentOrientation);
    setReports(reportType, reportIntervalUs);
    setReports(SH2_GYROSCOPE_CALIBRATED, reportIntervalUs);
    setReports(SH2_GEOMAGNETIC_ROTATION_VECTOR, reportIntervalUs);
  }
}

// main loop
// perform background tasks
void loop
  (
  void
  )
{
  // FlexCAN_T4 queues RX in software; events() pops one frame per call. Drain the
  // whole queue each pass so RPDOs are not dropped when loop() runs slower than bus traffic.
  while (CANBus.getRXQueueCount() > 0)
  {
    CANBus.events();
  }

  if (SensorFound)
  {
    if (bno08x.wasReset())
    {
      Serial.println("Sensor was reset");
      // Restore hub + gyro axes only; keep ZeroPitch/ZeroRoll in RAM (same as EEPROM when last saved).
      ApplyOrientationMode(CurrentOrientation);
      setReports(reportType, reportIntervalUs);
      setReports(SH2_GYROSCOPE_CALIBRATED, reportIntervalUs);
      setReports(SH2_GEOMAGNETIC_ROTATION_VECTOR, reportIntervalUs);
    }

    if (SensorEnabled)
    {
      if (bno08x.getSensorEvent(&sensorValue))
      {
        bool orientationUpdated = false;
        switch (sensorValue.sensorId)
        {
          case SH2_ROTATION_VECTOR:
            fusedQuaternionToYpr(
              sensorValue.un.rotationVector.i,
              sensorValue.un.rotationVector.j,
              sensorValue.un.rotationVector.k,
              sensorValue.un.rotationVector.real,
              &ypr,
              true);
            CalibrationStatus = sensorValue.status & 0x03;
            orientationUpdated = true;
            break;

          case SH2_GYROSCOPE_CALIBRATED:
            {
              // Gyro is always in package frame; GyrRot is identity for horizontal, full R for vertical.
              float wx = sensorValue.un.gyroscope.x;
              float wy = sensorValue.un.gyroscope.y;
              float wz = sensorValue.un.gyroscope.z;
              float ox = GyrRot[0][0] * wx + GyrRot[0][1] * wy + GyrRot[0][2] * wz;
              float oy = GyrRot[1][0] * wx + GyrRot[1][1] * wy + GyrRot[1][2] * wz;
              float oz = GyrRot[2][0] * wx + GyrRot[2][1] * wy + GyrRot[2][2] * wz;
              ypr.yawrate = (ox * YawRateAxis[0] + oy * YawRateAxis[1] + oz * YawRateAxis[2]) * 180.0f / PI;
            }
            break;

          case SH2_GEOMAGNETIC_ROTATION_VECTOR:
            //quarternionToEulerGRV(&sensorValue.un.geoMagRotationVector, &ypr, true);
            //ypr.yaw = FindHeading(&sensorValue.un.geoMagRotationVector);
            break;

          case SH2_ARVR_STABILIZED_RV:     
            /*fusedQuaternionToYpr(
              sensorValue.un.arvrStabilizedRV.i,
              sensorValue.un.arvrStabilizedRV.j,
              sensorValue.un.arvrStabilizedRV.k,
              sensorValue.un.arvrStabilizedRV.real,
              &ypr,
              true);
            CalibrationStatus = sensorValue.status & 0x03;
            orientationUpdated = true;*/
            break;
          
          case SH2_GYRO_INTEGRATED_RV:
            /*fusedQuaternionToYpr(
              sensorValue.un.gyroIntegratedRV.i,
              sensorValue.un.gyroIntegratedRV.j,
              sensorValue.un.gyroIntegratedRV.k,
              sensorValue.un.gyroIntegratedRV.real,
              &ypr,
              true);
            orientationUpdated = true;*/
            break;
        }

        // Only apply when yaw/pitch/roll were refreshed from a quaternion; gyro-only events must not re-invert.
        if (orientationUpdated)
        {
          // perform corrections so that heading increases clockwise when viewed from above
          if (ypr.yaw < 0) ypr.yaw += 360;
          ypr.yaw = 360.0 - ypr.yaw;

          // perform corrections so that downhill travel is a positive pitch
          ypr.pitch = -ypr.pitch;

          // Pitch and roll on CAN: [-180, +180] deg (both orientations). Yaw stays 0..360 below.
          wrapDegrees180(&ypr.pitch);
          wrapDegrees180(&ypr.roll);

          if (PendingCalibrateRequest)
          {
            PendingCalibrateRequest = false;
            SetCalibration();
            ypr.pitch = 0.0f;
            ypr.roll = 0.0f;
          }
          else
          {
            // Level zero: CAN = measured - Zero (when measured == Zero*, bus pitch/roll are 0).
            ypr.pitch -= (float)ZeroPitch;
            ypr.roll -= (float)ZeroRoll;
            wrapDegrees180(&ypr.pitch);
            wrapDegrees180(&ypr.roll);
          }
        }

        /*static long last = 0;
        long now = micros();
        Serial.print(now - last);             Serial.print("\t");
        last = now;
        Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
        Serial.print(ypr.yaw);                Serial.print("\t");
        Serial.print(ypr.pitch);              Serial.print("\t");
        Serial.println(ypr.roll);*/
      }
    }
  }

  // transmit TPDOs
  if (TPDOTxTime >= TPDO_TX_PERIOD_MS)
  {
    TPDOTxTime = 0;

    TxPDOs(&ypr);
  }

  // transmit heartbeats
  if (HBTime >= HB_PRODUCER_TIME_MS)
  {
    HBTime = 0;

    TxHeartbeat();
    wdt.feed();
  }
}
