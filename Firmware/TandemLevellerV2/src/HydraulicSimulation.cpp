// Pilot-operated hydraulic cylinder simulation for front scraper angle

#include <math.h>
#include <stddef.h>
#include "HydraulicSimulation.h"


///////////////////////////////////////////////////////////////////////////////////
// STATIC MEMBER DEFINITIONS

// pilot current fraction samples for kPilotPressuresPsi lookup (0..1)
const float HydraulicSimulation::kPilotCurrentFractions[] =
{
  0.00f,
  0.40f,
  0.50f,
  0.60f,
  0.80f,
  1.00f,
};

// pilot pressure (psi) paired with kPilotCurrentFractions
const float HydraulicSimulation::kPilotPressuresPsi[] =
{
  0.0f,
  60.0f,
  100.0f,
  130.0f,
  225.0f,
  320.0f,
};

// measured cylinder extension samples (mm) for kAngleCalDeg lookup
const float HydraulicSimulation::kExtensionCalMm[] =
{
  0.000f,
  3.898f,
  26.970f,
  58.671f,
  91.994f,
  132.408f,
  169.557f,
  207.627f,
  231.550f,
  237.038f,
  258.179f,
  262.255f,
  296.624f,
  299.265f,
  304.700f,
  313.225f,
  320.861f,
  347.398f,
};

// corrected blade angle (deg) paired with kExtensionCalMm
const float HydraulicSimulation::kAngleCalDeg[] =
{
  -33.38f,
  -32.86f,
  -29.82f,
  -25.60f,
  -21.11f,
  -15.55f,
  -10.31f,
  -4.76f,
  -1.15f,
  0.31f,
  2.98f,
  3.63f,
  9.24f,
  9.69f,
  10.61f,
  12.08f,
  13.42f,
  18.27f,
};


///////////////////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTIONS

// returns -1, 0, or +1 for the sign of Value
float HydraulicSimulation::Sign
  (
  float Value                      // input value
  )
{
  if (Value > 0.0f)
    return 1.0f;

  if (Value < 0.0f)
    return -1.0f;

  return 0.0f;
}

// clamps a float to inclusive limits
float HydraulicSimulation::ClipFloat
  (
  float Value,                     // value to clamp
  float MinValue,                  // lower limit
  float MaxValue                   // upper limit
  )
{
  if (Value < MinValue)
    return MinValue;

  if (Value > MaxValue)
    return MaxValue;

  return Value;
}

// piecewise-linear interpolation with endpoint clamping
float HydraulicSimulation::InterpolateLinear
  (
  float X,                         // lookup value
  const float *pXTable,            // ascending X samples
  const float *pYTable,            // Y samples paired with X
  size_t Count,                    // number of table entries
  float LeftClamp,                 // Y when X is below table
  float RightClamp                 // Y when X is above table
  )
{
  if ((Count == 0u) || (pXTable == NULL) || (pYTable == NULL))
    return 0.0f;

  if (X <= pXTable[0])
    return LeftClamp;

  if (X >= pXTable[Count - 1u])
    return RightClamp;

  for (size_t Index = 1u; Index < Count; ++Index)
  {
    if (X <= pXTable[Index])
    {
      const float X0 = pXTable[Index - 1u];
      const float X1 = pXTable[Index];
      const float Y0 = pYTable[Index - 1u];
      const float Y1 = pYTable[Index];
      const float Span = X1 - X0;

      if (Span <= 0.0f)
        return Y0;

      const float Fraction = (X - X0) / Span;
      return Y0 + (Fraction * (Y1 - Y0));
    }
  }

  return RightClamp;
}

// first-order lag toward target
float HydraulicSimulation::FirstOrderLag
  (
  float Current,                   // present value
  float Target,                    // target value
  float DtS,                       // timestep (seconds)
  float TauS                       // time constant (seconds)
  )
{
  if (TauS <= 0.0f)
    return Target;

  const float Alpha = 1.0f - expf(-DtS / TauS);
  return Current + (Alpha * (Target - Current));
}

// converts 8-bit PWM into signed pilot current fraction
float HydraulicSimulation::PwmToSignedCurrentFraction
  (
  uint8_t Pwm,                     // 8-bit PWM command
  uint8_t Center                   // neutral PWM value
  )
{
  const float PwmValue = ClipFloat((float)Pwm, 0.0f, 255.0f);

  if (PwmValue > (float)Center)
    return (PwmValue - (float)Center) / (255.0f - (float)Center);

  if (PwmValue < (float)Center)
    return -((float)Center - PwmValue) / (float)Center;

  return 0.0f;
}

// maps absolute pilot current fraction to pilot pressure
float HydraulicSimulation::CurrentFractionToPilotPressurePsi
  (
  float AbsCurrentFraction         // absolute pilot current fraction 0..1
  )
{
  const float CurrentFraction = ClipFloat(AbsCurrentFraction, 0.0f, 1.0f);

  return InterpolateLinear(
    CurrentFraction,
    kPilotCurrentFractions,
    kPilotPressuresPsi,
    kPilotCalCount,
    kPilotPressuresPsi[0],
    kPilotPressuresPsi[kPilotCalCount - 1u]);
}

// maps PWM to signed target pilot pressure
float HydraulicSimulation::PwmToTargetPilotPressurePsi
  (
  uint8_t Pwm                      // 8-bit PWM command
  )
{
  const float SignedCurrent = PwmToSignedCurrentFraction(Pwm, CenterPwm);
  const float SignValue = Sign(SignedCurrent);
  const float AbsCurrent = fabsf(SignedCurrent);
  const float Pressure = CurrentFractionToPilotPressurePsi(AbsCurrent);

  return SignValue * Pressure;
}

// maps signed pilot pressure to signed main flow fraction
float HydraulicSimulation::PilotPressureToMainFlowFraction
  (
  float PilotPressurePsi           // signed pilot pressure (psi)
  )
{
  const float SignValue = Sign(PilotPressurePsi);
  const float Pressure = fabsf(PilotPressurePsi);

  if (Pressure <= MainPilotCrackingPsi)
    return 0.0f;

  float Fraction =
    (Pressure - MainPilotCrackingPsi)
    / (MainPilotFullFlowPsi - MainPilotCrackingPsi);

  Fraction = ClipFloat(Fraction, 0.0f, 1.0f);

  return SignValue * Fraction;
}

// maps signed flow fraction to cylinder velocity
float HydraulicSimulation::FlowFractionToVelocityMmS
  (
  float FlowFraction               // signed main flow fraction
  )
{
  if (FlowFraction >= 0.0f)
    return FlowFraction * MaxExtendSpeedMmS;

  return FlowFraction * MaxRetractSpeedMmS;
}

// converts cylinder extension to corrected angle
float HydraulicSimulation::ExtensionToAngleDeg
  (
  float ExtensionMm                // cylinder extension (mm)
  )
{
  return InterpolateLinear(
    ExtensionMm,
    kExtensionCalMm,
    kAngleCalDeg,
    kExtensionAngleCalCount,
    kAngleCalDeg[0],
    kAngleCalDeg[kExtensionAngleCalCount - 1u]);
}

// converts corrected angle to required cylinder extension
float HydraulicSimulation::AngleToExtensionMm
  (
  float AngleDeg                   // corrected angle (degrees)
  )
{
  return InterpolateLinear(
    AngleDeg,
    kAngleCalDeg,
    kExtensionCalMm,
    kExtensionAngleCalCount,
    kExtensionCalMm[0],
    kExtensionCalMm[kExtensionAngleCalCount - 1u]);
}

// resets model state from InitialAngleDeg
void HydraulicSimulation::ResetModel
  (
  void
  )
{
  ActualPilotPressurePsi = 0.0f;
  ActualMainFlowFraction = 0.0f;
  PwmCommand = CenterPwm;
  LastStepMs = 0u;

  State.TimeS = 0.0f;
  State.Pwm = CenterPwm;
  State.TargetPilotPressurePsi = 0.0f;
  State.ActualPilotPressurePsi = 0.0f;
  State.MainFlowFraction = 0.0f;
  State.MainFlowGpm = 0.0f;
  State.VelocityMmS = 0.0f;
  State.ExtensionMm = ClipFloat(
    AngleToExtensionMm(InitialAngleDeg),
    0.0f,
    MaxExtensionMm);
  State.AngleDeg = ExtensionToAngleDeg(State.ExtensionMm);
}

// advances the model by one simulation timestep
void HydraulicSimulation::StepSimulation
  (
  float DtS                        // simulation timestep (seconds)
  )
{
  const float TargetPilotPressurePsi = PwmToTargetPilotPressurePsi(PwmCommand);

  ActualPilotPressurePsi = FirstOrderLag(
    ActualPilotPressurePsi,
    TargetPilotPressurePsi,
    DtS,
    PilotValveTauS);

  const float TargetMainFlowFraction =
    PilotPressureToMainFlowFraction(ActualPilotPressurePsi);

  ActualMainFlowFraction = FirstOrderLag(
    ActualMainFlowFraction,
    TargetMainFlowFraction,
    DtS,
    MainValveTauS);

  float VelocityMmS = FlowFractionToVelocityMmS(ActualMainFlowFraction);
  const float PreviousExtensionMm = State.ExtensionMm;

  State.ExtensionMm = ClipFloat(
    State.ExtensionMm + (VelocityMmS * DtS),
    0.0f,
    MaxExtensionMm);

  if (State.ExtensionMm == PreviousExtensionMm)
  {
    if (((State.ExtensionMm <= 0.0f) && (VelocityMmS < 0.0f))
      || ((State.ExtensionMm >= MaxExtensionMm) && (VelocityMmS > 0.0f)))
    {
      VelocityMmS = 0.0f;
    }
  }

  State.TimeS += DtS;
  State.Pwm = PwmCommand;
  State.TargetPilotPressurePsi = TargetPilotPressurePsi;
  State.ActualPilotPressurePsi = ActualPilotPressurePsi;
  State.MainFlowFraction = ActualMainFlowFraction;
  State.MainFlowGpm = ActualMainFlowFraction * MainMaxFlowGpm;
  State.VelocityMmS = VelocityMmS;
  State.AngleDeg = ExtensionToAngleDeg(State.ExtensionMm);
}


///////////////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTIONS

// constructor; InitialAngleDeg sets starting blade angle (default 0 deg)
HydraulicSimulation::HydraulicSimulation
  (
  float _InitialAngleDeg             // starting corrected angle (degrees)
  )
  : InitialAngleDeg(_InitialAngleDeg)
{
  ResetModel();
}

// starts the simulation and resets model state
void HydraulicSimulation::Start
  (
  void
  )
{
  ResetModel();
  Running = true;
  LastStepMs = millis();
}

// stops the simulation
void HydraulicSimulation::Stop
  (
  void
  )
{
  Running = false;
}

// sets the 8-bit PWM command (0..255, 127 = neutral)
void HydraulicSimulation::SetPWM
  (
  uint8_t Pwm                        // PWM command
  )
{
  PwmCommand = Pwm;
}

// sets the callback invoked every 5 ms while running
void HydraulicSimulation::SetCallback
  (
  hydraulicsimulation_callback_t _Callback // callback function
  )
{
  Callback = _Callback;
}

// call from the main loop; advances simulation and invokes callback every 5 ms
void HydraulicSimulation::Process
  (
  void
  )
{
  if (!Running)
    return;

  const uint32_t NowMs = millis();

  while ((NowMs - LastStepMs) >= HYDRAULICSIMULATION_CALLBACK_PERIOD_MS)
  {
    LastStepMs += HYDRAULICSIMULATION_CALLBACK_PERIOD_MS;
    StepSimulation(SimDtS);

    if (Callback != NULL)
      Callback(this, &State);
  }
}
