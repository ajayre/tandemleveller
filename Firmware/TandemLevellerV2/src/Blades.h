// blade control

#ifndef _BLADESH_
#define _BLADESH_

#include <Arduino.h>
#include "Global.h"

// number of blades we support
#define NUM_BLADES 2

// the height of the blade that represents ground level
// when communicating with AgGrade
#define BLADE_HEIGHT_GROUND_LEVEL 200

typedef enum _blade_direction_t
{
  BLADE_DIR_DOWN = 0,
  BLADE_DIR_UP   = 1
} blade_direction_t;

// configuration of blade control
typedef struct _blade_config_t
{
  int PWMGainUp;
  int PWMGainDown;
  uint8_t PWMMinUp;
  uint8_t PWMMinDown;
  uint8_t PWMMaxUp;
  uint8_t PWMMaxDown;
  int IntegralMultiplier;
  int Deadband;
} blade_config_t;

// current status of the blade
typedef struct _blade_status_t
{
  int BladePWM;
  int BladeCommand;
  blade_direction_t BladeDirection;
  bool BladeAuto;
  int16_t SlaveOffset;
} blade_status_t;

// movement command for blade
typedef struct _blade_command_t
{
  int CutValve;       // target blade height in mm. BLADE_HEIGHT_GROUND_LEVEL = on target, < BLADE_HEIGHT_GROUND_LEVEL below target, > BLADE_HEIGHT_GROUND_LEVEL above target. Range is 0 - 400
} blade_command_t;

typedef void (*blades_blade_changed_callback_t)(int BladeIndex, int PWM, blade_direction_t Direction);

class Blades
{
  public:
    // constructor
    Blades
      (
      void
      );

    blade_config_t BladeConfig[NUM_BLADES];
    blade_status_t BladeStatus[NUM_BLADES];
    blade_command_t BladeCommand[NUM_BLADES];
    int BladeHeight[NUM_BLADES];  // in mm
    elapsedMillis LastJogTime[NUM_BLADES];

    // calculate new output for blade
    void ControlBlade
      (
      int BladeIndex              // xxx_BLADE_IDX
      );

    // sets blades into emergency stop state
    void EmergencyStop
      (
      void
      );

    // sets the callback function
    void SetCallback
      (
      blades_blade_changed_callback_t BladeChangedCallback
      );

    // jogs a blade
    void JogBlade
      (
      int BladeIndex,                 // index of blade to jog xxx_BLADE_IDX
      blade_direction_t Direction     // jog direction
      );

    // jogs a blade offset
    // returns true if jog occurred
    bool JogOffset
      (
      int BladeIndex,                 // index of blade to jog offset xxx_BLADE_IDX
      blade_direction_t Direction     // jog direction
      );

  private:
    int pwm1ago[NUM_BLADES] = { 0 };
    int pwm2ago[NUM_BLADES] = { 0 };
    int pwm3ago[NUM_BLADES] = { 0 };
    int pwm4ago[NUM_BLADES] = { 0 };
    int pwm5ago[NUM_BLADES] = { 0 };
    blades_blade_changed_callback_t BladeChangedCallback = NULL;

    // sets the PWM value for the front valve
    void SetFrontValvePWM
      (
      uint8_t Value          // new valve PWM setting 0 - 255
      );

    // sets the PWM value for the rear valve
    void SetRearValvePWM
      (
      uint8_t Value          // new valve PWM setting 0 - 255
      );
};

#endif // _BLADESH_
