// blade control

#include "Blades.h"

// GPIO pins
#define FRONT_HEIGHT_DIR 0
#define FRONT_HEIGHT_PWM 1
#define REAR_HEIGHT_DIR  2
#define REAR_HEIGHT_PWM  3
#define FRONT_DUMP_DIR   4
#define FRONT_DUMP_PWM   5
#define REAR_DUMP_DIR    6
#define REAR_DUMP_PWM    7

// from EHPR98-G35 specs
#define PWM_FREQUENCY_HZ 120

// allowed range for the cutvalve command
#define CUTVALVE_MIN 0
#define CUTVALVE_MAX 400

// mimumum time between two jog moves per mm
#define MIN_TIME_BETWEEN_JOGS_MS 200

// allowed range for slave offset
#define SLAVE_OFFSET_MIN (-128)
#define SLAVE_OFFSET_MAX 127


///////////////////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTIONS

// sets the PWM value for the front valve
void Blades::SetFrontValvePWM
  (
  uint8_t Value          // new valve PWM setting 0 - 255
  )
{
  // if value has changed
  if (abs(Value) != BladeStatus[FRONT_BLADE_IDX].BladePWM)
  {
    // set to 0 - 255
    BladeStatus[FRONT_BLADE_IDX].BladePWM = abs(Value);
    analogWrite(FRONT_HEIGHT_PWM, BladeStatus[FRONT_BLADE_IDX].BladePWM);

    if (BladeChangedCallback != NULL)
    {
      BladeChangedCallback(FRONT_BLADE_IDX, BladeStatus[FRONT_BLADE_IDX].BladePWM, BladeHeight[FRONT_BLADE_IDX], digitalRead(FRONT_HEIGHT_DIR) ? BLADE_DIR_UP : BLADE_DIR_DOWN);
    }
  }
}

// sets the PWM value for the rear valve
void Blades::SetRearValvePWM
  (
  uint8_t Value          // new valve PWM setting 0 - 255
  )
{
  // if value has changed
  if (abs(Value) != BladeStatus[REAR_BLADE_IDX].BladePWM)
  {
    // set to 0 - 255
    BladeStatus[REAR_BLADE_IDX].BladePWM = abs(Value);
    analogWrite(REAR_HEIGHT_PWM, BladeStatus[REAR_BLADE_IDX].BladePWM);

    if (BladeChangedCallback != NULL)
    {
      BladeChangedCallback(REAR_BLADE_IDX, BladeStatus[REAR_BLADE_IDX].BladePWM, BladeHeight[REAR_BLADE_IDX], digitalRead(REAR_HEIGHT_DIR) ? BLADE_DIR_UP : BLADE_DIR_DOWN);
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTIONS

// constructor
Blades::Blades
  (
  void
  )
{
  // set PWM frequency to 120Hz (from EHPR98-G35 specs)
  analogWriteFrequency(FRONT_HEIGHT_PWM, PWM_FREQUENCY_HZ);
  analogWriteFrequency(REAR_HEIGHT_PWM,  PWM_FREQUENCY_HZ);
  analogWriteFrequency(FRONT_DUMP_PWM,   PWM_FREQUENCY_HZ);
  analogWriteFrequency(REAR_DUMP_PWM,    PWM_FREQUENCY_HZ);
  analogWriteResolution(8); // 0 - 255

  // set up direction control signals
  pinMode(FRONT_HEIGHT_DIR, OUTPUT);
  pinMode(REAR_HEIGHT_DIR,  OUTPUT);
  pinMode(FRONT_DUMP_DIR,   OUTPUT);
  pinMode(REAR_DUMP_DIR,    OUTPUT);

  // initalize direction
  digitalWrite(FRONT_HEIGHT_DIR, LOW);
  digitalWrite(REAR_HEIGHT_DIR,  LOW);
  digitalWrite(FRONT_DUMP_DIR,   LOW);
  digitalWrite(REAR_DUMP_DIR,    LOW);

    // reset blade heights
  for (int b = 0; b < NUM_BLADES; b++)
  {
    BladeHeight[b] = BLADE_HEIGHT_GROUND_LEVEL;
  }

  // initial blade status
  memset(&BladeStatus, 0, sizeof(blade_status_t));
  BladeStatus[FRONT_BLADE_IDX].Cutting = false;
  BladeStatus[REAR_BLADE_IDX].Cutting  = false;

  // initial state is no movement
  BladeCommand[FRONT_BLADE_IDX].CutValve = BLADE_HEIGHT_GROUND_LEVEL;
  BladeCommand[REAR_BLADE_IDX].CutValve  = BLADE_HEIGHT_GROUND_LEVEL;
  BladeStatus[FRONT_BLADE_IDX].BladeCommand = BLADE_HEIGHT_GROUND_LEVEL;
  BladeStatus[REAR_BLADE_IDX].BladeCommand = BLADE_HEIGHT_GROUND_LEVEL;

  // default PWM configuruation
  for(int b = 0; b < NUM_BLADES; b++)
  {
    BladeConfig[b].PWMGainUp          = 4;
    BladeConfig[b].PWMGainDown        = 3;
    BladeConfig[b].PWMMinUp           = 50;
    BladeConfig[b].PWMMinDown         = 50;
    BladeConfig[b].PWMMaxUp           = 180;
    BladeConfig[b].PWMMaxDown         = 180;
    BladeConfig[b].IntegralMultiplier = 20;
    BladeConfig[b].Deadband           = 3;
  }

  for (int b = 0; b < NUM_BLADES; b++)
  {
    LastJogTime[b] = 0;
  }
}

// calculate new output for blade
void Blades::ControlBlade
  (
  int BladeIndex              // xxx_BLADE_IDX
  )
{
  int PWMValue;

  // store command for status reporting
  BladeStatus[BladeIndex].BladeCommand = BladeCommand[BladeIndex].CutValve;

  // position error (mm): positive → blade is above target grade → lower;
  //                       negative → blade is below target grade → raise
  int error = BladeCommand[BladeIndex].CutValve - (int)BladeHeight[BladeIndex];

  // reset integrator whenever the setpoint changes to avoid stale windup
  if (BladeCommand[BladeIndex].CutValve != prevCommand[BladeIndex])
  {
    integralAccumulator[BladeIndex] = 0.0f;
    prevError[BladeIndex]           = 0;
    prevCommand[BladeIndex]         = BladeCommand[BladeIndex].CutValve;
  }

  // within deadband: stop the valve and clear PID state
  if (abs(error) <= BladeConfig[BladeIndex].Deadband)
  {
    integralAccumulator[BladeIndex] = 0.0f;
    prevError[BladeIndex]           = 0;
    switch (BladeIndex)
    {
      case FRONT_BLADE_IDX: SetFrontValvePWM(0); break;
      case REAR_BLADE_IDX:  SetRearValvePWM(0);  break;
    }
    return;
  }

  // select direction-dependent gains and limits
  bool  goingDown = (error > 0);
  float Kp        = goingDown ? (float)BladeConfig[BladeIndex].PWMGainDown : (float)BladeConfig[BladeIndex].PWMGainUp;
  float Ki        = (float)BladeConfig[BladeIndex].IntegralMultiplier / 100.0f;
  int   PWMMin    = goingDown ? (int)BladeConfig[BladeIndex].PWMMinDown : (int)BladeConfig[BladeIndex].PWMMinUp;
  int   PWMMax    = goingDown ? (int)BladeConfig[BladeIndex].PWMMaxDown : (int)BladeConfig[BladeIndex].PWMMaxUp;

  // reset integral on zero crossing to prevent windup carrying across direction changes
  if (prevError[BladeIndex] != 0 && ((error > 0) != (prevError[BladeIndex] > 0)))
    integralAccumulator[BladeIndex] = 0.0f;
  else
    integralAccumulator[BladeIndex] += (float)error;

  // anti-windup: clamp integral so its contribution never exceeds PWMMax
  if (Ki > 0.0f)
  {
    float maxAccum = (float)PWMMax / Ki;
    if (integralAccumulator[BladeIndex] >  maxAccum) integralAccumulator[BladeIndex] =  maxAccum;
    if (integralAccumulator[BladeIndex] < -maxAccum) integralAccumulator[BladeIndex] = -maxAccum;
  }

  prevError[BladeIndex] = error;

  // P + I output (both error and accumulator carry the same sign, so abs gives magnitude)
  PWMValue = (int)(Kp * (float)abs(error) + Ki * fabsf(integralAccumulator[BladeIndex]));

  // clamp to [PWMMin, PWMMax]; PWMMin is the valve cracking threshold
  if (PWMValue < PWMMin) PWMValue = PWMMin;
  if (PWMValue > PWMMax) PWMValue = PWMMax;

  // set direction and apply PWM
  if (goingDown)
  {
    digitalWrite(BladeIndex == FRONT_BLADE_IDX ? FRONT_HEIGHT_DIR : REAR_HEIGHT_DIR, HIGH);
    BladeStatus[BladeIndex].BladeDirection = BLADE_DIR_DOWN;
  }
  else
  {
    digitalWrite(BladeIndex == FRONT_BLADE_IDX ? FRONT_HEIGHT_DIR : REAR_HEIGHT_DIR, LOW);
    BladeStatus[BladeIndex].BladeDirection = BLADE_DIR_UP;
  }

  switch (BladeIndex)
  {
    case FRONT_BLADE_IDX: SetFrontValvePWM((uint8_t)PWMValue); break;
    case REAR_BLADE_IDX:  SetRearValvePWM((uint8_t)PWMValue);  break;
  }
}

// sets blades into emergency stop state
void Blades::EmergencyStop
  (
  void
  )
{
  // switch to manual control, stop movement
  for (int b = 0; b < NUM_BLADES; b++)
  {
    BladeStatus[b].Cutting = false;
    BladeStatus[b].BladeCommand = BladeCommand[b].CutValve = BLADE_HEIGHT_GROUND_LEVEL;
  }

  SetFrontValvePWM(0);
  SetRearValvePWM(0);
}

// sets the callback function
void Blades::SetCallback
  (
  blades_blade_changed_callback_t _BladeChangedCallback
  )
{
  BladeChangedCallback = _BladeChangedCallback;
}

// jogs a blade
void Blades::JogBlade
  (
  int BladeIndex,                 // index of blade to jog xxx_BLADE_IDX
  blade_direction_t Direction     // jog direction
  )
{
  if (LastJogTime[BladeIndex] >= MIN_TIME_BETWEEN_JOGS_MS)
  {
    if (Direction == BLADE_DIR_UP)
    {
      BladeCommand[BladeIndex].CutValve += 1;
      if (BladeCommand[BladeIndex].CutValve > CUTVALVE_MAX) BladeCommand[BladeIndex].CutValve = CUTVALVE_MAX;
    }
    else
    {
      BladeCommand[BladeIndex].CutValve -= 1;
      if (BladeCommand[BladeIndex].CutValve < CUTVALVE_MIN) BladeCommand[BladeIndex].CutValve = CUTVALVE_MIN;
    }

    LastJogTime[BladeIndex] = 0;
  }
}

// jogs a blade offset
// returns true if jog occurred
bool Blades::JogOffset
  (
  int BladeIndex,                 // index of blade to jog offset xxx_BLADE_IDX
  blade_direction_t Direction     // jog direction
  )
{
  if (LastJogTime[BladeIndex] >= MIN_TIME_BETWEEN_JOGS_MS)
  {
    if (Direction == BLADE_DIR_UP)
    {
      BladeStatus[BladeIndex].SlaveOffset += 1;
      if (BladeStatus[BladeIndex].SlaveOffset > SLAVE_OFFSET_MAX) BladeStatus[BladeIndex].SlaveOffset = SLAVE_OFFSET_MAX;
    }
    else
    {
      BladeStatus[BladeIndex].SlaveOffset -= 1;
      if (BladeStatus[BladeIndex].SlaveOffset < SLAVE_OFFSET_MIN) BladeStatus[BladeIndex].SlaveOffset = SLAVE_OFFSET_MIN;
    }
    LastJogTime[BladeIndex] = 0;

    return true;
  }

  return false;
}
