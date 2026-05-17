// IMU handling

#ifndef _IMUH_
#define _IMUH_

#include <Arduino.h>
#include "Global.h"
#include "BNO085.h"

// define to 1 to use on-board IMU for tractor
// define to 0 to use CAN bus IMU for tractor
#define USE_INTEGRATED_TRACTOR_IMU 1

typedef struct _imu_t
{
  float Roll;
  float Pitch;
  float Heading;
  float YawRate;
  uint8_t CalibrationStatus;
} imu_t;

// supported orientations
typedef enum _imu_orientation_t
{
  IMU_ORIENTATION_HORIZONTAL_A = 0,
  IMU_ORIENTATION_VERTICAL_A   = 1
} imu_orientation_t;

// callback types
typedef void (*imu_changed_callback_t)(uint8_t Index, imu_t *pimu);
typedef void (*imu_txcanmessage_callback_t)(uint16_t Id, uint8_t Length, uint8_t Data[]);

// Timestamped IMU sample for circular buffer
typedef struct _imu_timestamped_t
{
  imu_t values;
  uint32_t timestamp_ms;
} imu_timestamped_t;

// ~320ms of history at 10ms sample rate
#define IMU_BUFFER_SIZE 32

class IMU
{
  public:
    imu_t IMUValues[NUM_BLADES + 1];

    // Retrieves the buffered sample closest to target_ms. Returns false if
    // the buffer is empty for the given index.
    bool GetSampleAtTime
      (
      uint8_t Index,                  // blade/tractor index
      uint32_t TargetMs,              // millis() timestamp to match
      imu_t *pResult                  // output: closest sample
      ) const;

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
      imu_changed_callback_t _IMUChanged,        // called when an IMU has changed
      imu_txcanmessage_callback_t _TxCANMessage  // called when IMU module wants to send a CAN message
      );

    // sets the pitch and roll to zero
    void SetZero
      (
      uint8_t NodeId             // node to zero
      );

    // sets the orientation
    void SetOrientation
      (
      uint8_t NodeId,                 // node to set orientation
      imu_orientation_t Orientation   // orientation to use
      );

    // perform periodic processing
    void Process
      (
      void  
      );

  private:
    imu_changed_callback_t IMUChanged;
    imu_txcanmessage_callback_t TxCANMessage;
    elapsedMillis OnboardIMUReadTimestamp;
#if USE_INTEGRATED_TRACTOR_IMU == 1
    BNO085 bno085;  
#endif

    // Prototype IMU low-pass (EMA) before fusion; tune alphas in IMU.cpp. Move to IMU node later.
    struct ImuLpState
    {
      bool  init = false;
      float roll = 0.0f;
      float pitch = 0.0f;
      float yaw_rate = 0.0f;
      float h_cos = 1.0f;
      float h_sin = 0.0f;
    };

    ImuLpState imu_lp_[NUM_BLADES + 1];

    imu_timestamped_t ImuBuffer[NUM_BLADES + 1][IMU_BUFFER_SIZE];
    int ImuBufferHead[NUM_BLADES + 1];
    int ImuBufferCount[NUM_BLADES + 1];

    void ApplyInputLowPass
      (
      uint8_t Index,
      float Heading,
      float Pitch,
      float Roll,
      float YawRate
      );

    // stores a set of IMU readings
    void StoreReadings
      (
      uint8_t idx,       // idx for readings (TRACTOR_IDX, etc)
      float Heading,     // current heading
      float Pitch,       // current pitch
      float Roll,        // current roll
      float YawRate      // current yaw rate
      );

#if USE_INTEGRATED_TRACTOR_IMU == 1
    // get the IMU reading from the on-board tractor IMU
    void ReadOnboardIMU
      (
      void
      );
#endif // USE_INTEGRATED_TRACTOR_IMU
};

#endif // _IMUH_
