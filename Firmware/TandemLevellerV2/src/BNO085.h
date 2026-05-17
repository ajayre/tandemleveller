// Bosch BNO08x (BNO085-class) SPI driver: fusion output and calibration for IMU.cpp

#ifndef _BNO085H_
#define _BNO085H_

#include <Arduino.h>
#include <stdint.h>
#include <Adafruit_BNO08x.h>

// Mount / output frame selection. Application +X = magnetic north; modes differ in
// which chip body axis is nominally "up"
// WARNING: must match imu_orientation_t
typedef enum _bno085_orientation_t
{
  // PCB flat: application +Z is up (out of the board).
  BNO085_ORIENTATION_HORIZONTAL_A = 0,
  // Board edge-on (package +Y toward world up); fusion quaternion encodes attitude.
  BNO085_ORIENTATION_VERTICAL_A   = 1
} bno085_orientation_t;

class BNO085
{
  public:
    // Constructor — configures the Adafruit driver for the wired reset pin only;
    // SPI and reports are started in Init(void).
    BNO085
      (
      void
      );

    // Initializes Teensy SPI0 (MOSI 11, MISO 12, SCK 13), chip select /CS (10),
    // interrupt /INT (41), reset /RST (40), loads pitch/roll trim and orientation from
    // EEPROM, applies mount mode, and enables rotation-vector, calibrated gyro, and
    // geomagnetic rotation reports
    void Init
      (
      void
      );

    // Services the BNO08x (hub reset recovery, sh2_service via getSensorEvent), runs the
    // quaternion to heading/pitch/roll/yaw-rate pipeline, and writes: heading (degrees,
    // 0..360 clockwise from above), pitch and roll (degrees, -180..+180), yaw rate
    // (deg/s), fusion calibration status (low two bits of the report status nibble).
    // All pointer parameters must be non-NULL.
    void Read
      (
      float *pHeading,
      float *pPitch,
      float *pRoll,
      float *pYawRate,
      uint8_t *pCalibrationStatus
      );

    // Selects physical mount mode: clears pitch/roll zero trim in RAM, reapplies hub axis
    // mapping and sh2 reorientation (identity), persists orientation and zero trim to
    // EEPROM
    void SetOrientation
      (
      bno085_orientation_t NewOrientation
      );

    // Queues a level calibration: on the next rotation-vector report, stores the current
    // pitch/roll (after mount conventions, before trim) as zero_pitch_/zero_roll_,
    // saves EEPROM, and reports pitch/roll as zero for that sample (same as IMU.ino.txt
    // SET_ZERO / SetCalibration path).
    void SetZero
      (
      void
      );

  private:
    // Teensy 4.x SPI0 Default pins: SCK 13, MOSI 11, MISO 12.
    static const uint8_t kPinCs  = 10;
    static const uint8_t kPinInt = 41;
    static const int8_t  kPinRst = 40;

    // Fusion report interval (50 ms = 20 Hz), same as legacy standalone IMU sketch.
    static const sh2_SensorId_t kReportTypeRotation = SH2_ROTATION_VECTOR;
    static const uint32_t       kReportIntervalUs     = 50000u;

    // Latest attitude/gyro in application convention (degrees, deg/s for rate).
    struct euler_t
    {
      float yaw;
      float pitch;
      float roll;
      float yawrate;
    };

    // Requests one SH-2 sensor report at the given interval.
    void SetReports
      (
      sh2_SensorId_t report_type,
      uint32_t       report_interval_us
      );

    // Converts a unit quaternion (w,x,y,z as qr,qi,qj,qk) to intrinsic yaw/pitch/roll.
    void QuaternionToEuler
      (
      float    qr,
      float    qi,
      float    qj,
      float    qk,
      euler_t *ypr,
      bool     degrees
      );

    // Maps fused game rotation quaternion to application yaw/pitch/roll.
    void FusedQuaternionToYpr
      (
      float    qi,
      float    qj,
      float    qk,
      float    qr,
      euler_t *out_ypr,
      bool     degrees
      );

    // Wraps a degree value to [-180, +180].
    void WrapDegrees180
      (
      float *a_deg
      );

    // Sets mount mode, gyro yaw-rate axis selection, and sh2 reorientation identity.
    void ApplyOrientationMode
      (
      bno085_orientation_t Orientation
      );

    // Decodes one sensor report and updates internal state.
    void ProcessSensorValue
      (
      sh2_SensorValue_t *p_val
      );

    Adafruit_BNO08x      bno08x_;
    sh2_SensorValue_t    sensor_value_{};

    bool                 sensor_found_          = false;
    bool                 sensor_enabled_        = false;
    uint8_t              calibration_status_    = 0U;

    double               zero_pitch_          = 0.0;
    double               zero_roll_           = 0.0;
    bno085_orientation_t current_orientation_{ BNO085_ORIENTATION_HORIZONTAL_A };

    euler_t              ypr_{};

    // Rotation matrix: gyro in calibrated sensor frame -> application frame (rad/s).
    float                gyr_rot_[3][3]{};
    float                yaw_rate_axis_[3]{};

    // When true, next rotation-vector pipeline uses measured pitch/roll as new EEPROM trim
    // (clears displayed attitude to level for that report); matches PendingCalibrateRequest.
    bool                 pending_set_zero_        = false;
};

#endif // _BNO085H_
