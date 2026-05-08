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
  float PWMHist;

  // if already at the requested height then nothing to do
  if (BladeCommand[BladeIndex].CutValve == BladeHeight[BladeIndex]) return;

  // store command
  BladeStatus[BladeIndex].BladeCommand = BladeCommand[BladeIndex].CutValve;

  // fixme - remove and replace with reading actual blade height from sensors
  // this is currently open-loop
  BladeHeight[BladeIndex] = BladeStatus[BladeIndex].BladeCommand;

  // lower the blade
  if (BladeCommand[BladeIndex].CutValve >= (BLADE_HEIGHT_GROUND_LEVEL + BladeConfig[BladeIndex].Deadband))
  {
    // PWM value is negative
    PWMValue = -((BladeCommand[BladeIndex].CutValve - BLADE_HEIGHT_GROUND_LEVEL - BladeConfig[BladeIndex].Deadband) * BladeConfig[BladeIndex].PWMGainDown + BladeConfig[BladeIndex].PWMMinDown);
  }
  // lift the blade
  else if (BladeCommand[BladeIndex].CutValve <= (BLADE_HEIGHT_GROUND_LEVEL - BladeConfig[BladeIndex].Deadband))
  {
    // PWM value is positive
    PWMValue = -((BladeCommand[BladeIndex].CutValve - BLADE_HEIGHT_GROUND_LEVEL + BladeConfig[BladeIndex].Deadband) * BladeConfig[BladeIndex].PWMGainUp - BladeConfig[BladeIndex].PWMMinUp);
  }
  else
  {
    PWMValue = 0;
  }

  // calculate a derivative
  if (BladeCommand[BladeIndex].CutValve != BLADE_HEIGHT_GROUND_LEVEL && PWMValue != 0)
  {
    PWMHist = ((((pwm1ago[BladeIndex]) + pwm2ago[BladeIndex] + (pwm3ago[BladeIndex]) + (pwm4ago[BladeIndex]) + (pwm5ago[BladeIndex] / 2.000)) * (sq(BladeConfig[BladeIndex].IntegralMultiplier) / 100.0000)) / sq(BladeCommand[BladeIndex].CutValve - (float)BLADE_HEIGHT_GROUND_LEVEL));

    //put pwmHist to 0 when the blade cross the line.
    if (BladeCommand[BladeIndex].CutValve > BLADE_HEIGHT_GROUND_LEVEL && (pwm1ago[BladeIndex] + pwm2ago[BladeIndex] + pwm3ago[BladeIndex] + pwm4ago[BladeIndex] + pwm5ago[BladeIndex]) > 0) PWMHist = 0;
    if (BladeCommand[BladeIndex].CutValve < BLADE_HEIGHT_GROUND_LEVEL && (pwm1ago[BladeIndex] + pwm2ago[BladeIndex] + pwm3ago[BladeIndex] + pwm4ago[BladeIndex] + pwm5ago[BladeIndex]) < 0) PWMHist = 0;

    PWMValue = (PWMValue - PWMHist);
  }

  // shuffle samples down
  pwm5ago[BladeIndex] = pwm4ago[BladeIndex];
  pwm4ago[BladeIndex] = pwm3ago[BladeIndex];
  pwm3ago[BladeIndex] = pwm2ago[BladeIndex];
  pwm2ago[BladeIndex] = pwm1ago[BladeIndex];
  pwm1ago[BladeIndex] = PWMValue;
  
  // enforce limits
  if (BladeCommand[BladeIndex].CutValve > BLADE_HEIGHT_GROUND_LEVEL && PWMValue > 0) PWMValue = 0;
  if (BladeCommand[BladeIndex].CutValve > BLADE_HEIGHT_GROUND_LEVEL && PWMValue < -(BladeConfig[BladeIndex].PWMMaxDown)) PWMValue = -(BladeConfig[BladeIndex].PWMMaxDown);
  if (BladeCommand[BladeIndex].CutValve < BLADE_HEIGHT_GROUND_LEVEL && PWMValue < 0) PWMValue = 0;
  if (BladeCommand[BladeIndex].CutValve < BLADE_HEIGHT_GROUND_LEVEL && PWMValue > BladeConfig[BladeIndex].PWMMaxUp) PWMValue = BladeConfig[BladeIndex].PWMMaxUp;
  if (PWMValue > 0 && PWMValue < BladeConfig[BladeIndex].PWMMinUp) PWMValue = 0;
  if (PWMValue < 0 && PWMValue > -(BladeConfig[BladeIndex].PWMMinDown)) PWMValue = 0;

  if (PWMValue < 0)
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
    case FRONT_BLADE_IDX:
      SetFrontValvePWM(abs(PWMValue));
      break;
      
    case REAR_BLADE_IDX:
      SetRearValvePWM(abs(PWMValue));
      break;
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
    BladeCommand[b].CutValve = BLADE_HEIGHT_GROUND_LEVEL;
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
