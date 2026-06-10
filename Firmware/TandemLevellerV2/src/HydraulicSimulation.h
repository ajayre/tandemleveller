// Pilot-operated hydraulic cylinder simulation for front scraper angle

#ifndef _HYDRAULICSIMULATION_H_
#define _HYDRAULICSIMULATION_H_

#include <stdint.h>
#include <stddef.h>
#include <Arduino.h>

class HydraulicSimulation;

// Process/callback step period (milliseconds)
#define HYDRAULICSIMULATION_CALLBACK_PERIOD_MS (5u)

/////////////////////////////////////////////////////////////////////////////////////
// PUBLIC — exported types and class

// snapshot of hydraulic model outputs passed to the simulation callback
typedef struct _hydraulicsimulation_state_t
{
  float TimeS;                         // simulation time (seconds)
  uint8_t Pwm;                         // 8-bit PWM command 0..255
  float TargetPilotPressurePsi;        // commanded pilot pressure (psi)
  float ActualPilotPressurePsi;        // lagged pilot pressure (psi)
  float MainFlowFraction;              // signed main-valve flow fraction -1..1
  float MainFlowGpm;                   // signed main flow (gpm)
  float VelocityMmS;                   // cylinder velocity (mm/s), + extend
  float ExtensionMm;                   // cylinder extension (mm)
  float AngleDeg;                      // corrected blade angle (degrees)
} hydraulicsimulation_state_t;

// invoked every HYDRAULICSIMULATION_CALLBACK_PERIOD_MS while the simulation runs
typedef void (*hydraulicsimulation_callback_t)
  (
  HydraulicSimulation *pSimulation,    // simulation instance
  const hydraulicsimulation_state_t *pState // current model state
  );

class HydraulicSimulation
{
  public:
    hydraulicsimulation_state_t State = { 0 }; // latest model outputs updated each step

    // constructor; InitialAngleDeg sets starting blade angle (default 0 deg)
    HydraulicSimulation
      (
      float InitialAngleDeg = 0.0f         // starting corrected angle (degrees)
      );

    // starts the simulation and resets model state
    void Start
      (
      void
      );

    // stops the simulation
    void Stop
      (
      void
      );

    // sets the 8-bit PWM command (0..255, 127 = neutral)
    void SetPWM
      (
      uint8_t Pwm                        // PWM command
      );

    // sets the callback invoked every 5 ms while running
    void SetCallback
      (
      hydraulicsimulation_callback_t _Callback // callback function
      );

    // call from the main loop; advances simulation and invokes callback every 5 ms
    void Process
      (
      void
      );

  private:
    bool Running                        = false; // true while simulation is active
    uint8_t PwmCommand                  = 127u; // current 8-bit PWM command
    uint32_t LastStepMs                 = 0u; // millis() timestamp of last simulation step
    float ActualPilotPressurePsi        = 0.0f; // lagged pilot pressure (psi)
    float ActualMainFlowFraction        = 0.0f; // lagged main-valve flow fraction
    hydraulicsimulation_callback_t Callback = NULL; // simulation step callback
    float InitialAngleDeg               = 0.0f; // starting angle for ResetModel (degrees)

    static constexpr uint8_t CenterPwm = 127u; // neutral 8-bit PWM command
    static constexpr float MaxExtensionMm = 350.0f; // maximum cylinder stroke (mm)
    static constexpr float FullRetractStrokeTimeS = 2.0f; // full retract stroke time (s)
    static constexpr float ExtendSpeedFactor = 0.95f; // extend speed vs retract speed ratio
    static constexpr float MainPilotCrackingPsi = 100.0f; // main valve cracking pressure (psi)
    static constexpr float MainPilotFullFlowPsi = 300.0f; // main valve full-flow pilot pressure (psi)
    static constexpr float MainMaxFlowGpm = 24.0f; // main valve full flow (gpm)
    static constexpr float PilotValveT90S = 0.080f; // pilot valve 90% response time (s)
    static constexpr float MainValveT90S = 0.150f; // main valve 90% response time (s)
    static constexpr float SimDtS = 0.005f; // internal simulation timestep (s)
    static constexpr float MaxRetractSpeedMmS = MaxExtensionMm / FullRetractStrokeTimeS; // max retract speed (mm/s)
    static constexpr float MaxExtendSpeedMmS = MaxRetractSpeedMmS * ExtendSpeedFactor; // max extend speed (mm/s)
    static constexpr float PilotValveTauS = PilotValveT90S / 2.30258509299404568402f; // pilot valve time constant (s)
    static constexpr float MainValveTauS = MainValveT90S / 2.30258509299404568402f; // main valve time constant (s)

    static const float kPilotCurrentFractions[]; // pilot current fraction calibration samples
    static const float kPilotPressuresPsi[]; // pilot pressure (psi) paired with kPilotCurrentFractions
    static const float kExtensionCalMm[]; // measured cylinder extension samples (mm)
    static const float kAngleCalDeg[]; // corrected angle (deg) paired with kExtensionCalMm
    static constexpr size_t kPilotCalCount = 6u; // entries in pilot calibration tables
    static constexpr size_t kExtensionAngleCalCount = 18u; // entries in extension/angle tables

    // resets model state from InitialAngleDeg
    void ResetModel
      (
      void
      );

    // advances the model by one simulation timestep
    void StepSimulation
      (
      float DtS                        // simulation timestep (seconds)
      );

    // first-order lag toward target
    static float FirstOrderLag
      (
      float Current,                   // present value
      float Target,                    // target value
      float DtS,                       // timestep (seconds)
      float TauS                       // time constant (seconds)
      );

    // converts 8-bit PWM into signed pilot current fraction
    static float PwmToSignedCurrentFraction
      (
      uint8_t Pwm,                     // 8-bit PWM command
      uint8_t Center                   // neutral PWM value
      );

    // maps absolute pilot current fraction to pilot pressure
    static float CurrentFractionToPilotPressurePsi
      (
      float AbsCurrentFraction         // absolute pilot current fraction 0..1
      );

    // maps PWM to signed target pilot pressure
    static float PwmToTargetPilotPressurePsi
      (
      uint8_t Pwm                      // 8-bit PWM command
      );

    // maps signed pilot pressure to signed main flow fraction
    static float PilotPressureToMainFlowFraction
      (
      float PilotPressurePsi           // signed pilot pressure (psi)
      );

    // maps signed flow fraction to cylinder velocity
    static float FlowFractionToVelocityMmS
      (
      float FlowFraction               // signed main flow fraction
      );

    // converts cylinder extension to corrected angle
    static float ExtensionToAngleDeg
      (
      float ExtensionMm                // cylinder extension (mm)
      );

    // converts corrected angle to required cylinder extension
    static float AngleToExtensionMm
      (
      float AngleDeg                   // corrected angle (degrees)
      );

    // piecewise-linear interpolation with endpoint clamping
    static float InterpolateLinear
      (
      float X,                         // lookup value
      const float *pXTable,            // ascending X samples
      const float *pYTable,            // Y samples paired with X
      size_t Count,                    // number of table entries
      float LeftClamp,                 // Y when X is below table
      float RightClamp                 // Y when X is above table
      );

    // clamps a float to inclusive limits
    static float ClipFloat
      (
      float Value,                     // value to clamp
      float MinValue,                  // lower limit
      float MaxValue                   // upper limit
      );

    // returns -1, 0, or +1 for the sign of Value
    static float Sign
      (
      float Value                      // input value
      );
};

#endif // _HYDRAULICSIMULATION_H_
