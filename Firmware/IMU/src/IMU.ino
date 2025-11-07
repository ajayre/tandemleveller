#include <Arduino.h>
#include "FlexCAN_T4-master/FlexCAN_T4.h"
#include "WDT_T4-master/Watchdog_t4.h"
#include <Adafruit_BNO08x.h>

// roll left = negative, roll right = positive
// pitch forwards = positive, pitch backwards = negative
// yaw clockwise (viewed from above) = increases angle, anti-clockwise = decreases angle
//   angle wraps around, e.g clockwise rotation = 0...-10...-180.180...10...0

// "CANopen" node id
#if TRACTOR == 1
  #define NODE_ID 0x02
#elif FRONTSCRAPER == 1
  #define NODE_ID 0x03
#else
  #define NODE_ID 0x04
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

struct euler_t
{
  float yaw;
  float pitch;
  float roll;
  float yawrate;  // deg/s
} ypr;

static Adafruit_BNO08x bno08x(BNO08X_RESET);
static sh2_SensorValue_t sensorValue;
static sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
static long reportIntervalUs = 50000;
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANBus;
static bool SensorFound = false;
static bool SensorEnabled = false;
elapsedMillis HBTime;
static WDT_T4<WDT1> wdt;
static uint8_t CalibrationStatus;

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

// transmits a PDO containing the measurements
static void TxPDO
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
}

// initialize the hardware
void setup()
{
  Serial.begin(115200);
  Serial.println("Tandem Leveller IMU");

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
  CANBus.setMB(MB63, TX); // Set mailbox as transmit

  TxBootup();

  CalibrationStatus = 0;

  // find sensor
  if (bno08x.begin_SPI(BNO08X_CS, BNO08X_INT))
  {
    SensorFound = true;
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
  // process can module
  CANBus.events();

  if (SensorFound)
  {
    if (bno08x.wasReset())
    {
      Serial.println("Sensor was reset");
      setReports(reportType, reportIntervalUs);
      setReports(SH2_GYROSCOPE_CALIBRATED, reportIntervalUs);
      setReports(SH2_GEOMAGNETIC_ROTATION_VECTOR, reportIntervalUs);
    }

    char msg[30];

    if (SensorEnabled)
    {
      if (bno08x.getSensorEvent(&sensorValue))
      {
        switch (sensorValue.sensorId)
        {
          case SH2_GYROSCOPE_CALIBRATED:
            ypr.yawrate = sensorValue.un.gyroscope.z * 180.0 / PI;
            break;

          case SH2_GEOMAGNETIC_ROTATION_VECTOR:
            //quarternionToEulerGRV(&sensorValue.un.geoMagRotationVector, &ypr, true);
            //ypr.yaw = FindHeading(&sensorValue.un.geoMagRotationVector);
            break;

          case SH2_ARVR_STABILIZED_RV:     
            quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
            CalibrationStatus = sensorValue.status & 0x03;
            break;
          
          case SH2_GYRO_INTEGRATED_RV:
            quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
            break;
        }

        if (ypr.yaw < 0) ypr.yaw += 360;

        TxPDO(&ypr);

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

  // transmit heartbeats
  if (HBTime >= HB_PRODUCER_TIME_MS)
  {
    HBTime -= HB_PRODUCER_TIME_MS;

    TxHeartbeat();
    wdt.feed();
  }
}
