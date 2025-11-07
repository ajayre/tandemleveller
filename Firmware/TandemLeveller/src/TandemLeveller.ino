#include <Arduino.h>
#include "FlexCAN_T4-master/FlexCAN_T4.h"
#include "SerialTransfer.h"

// front height:
//  dir = low, PWM output on M1A
//  dir = high, PWM output on M1B
// rear height:
//  dir = low, PWM output on M2A
//  dir = high, PWM output on M2B

// GPIO pins
#define FRONT_HEIGHT_DIR 0
#define FRONT_HEIGHT_PWM 1
#define REAR_HEIGHT_DIR  2
#define REAR_HEIGHT_PWM  3
#define FRONT_DUMP_DIR   4
#define FRONT_DUMP_PWM   5
#define REAR_DUMP_DIR    6
#define REAR_DUMP_PWM    7
#define LED              13

// node IDs
#define CONTROLLER_NODE_ID       0x01
#define TRACTOR_IMU_NODE_ID      0x02
#define FRONTSCRAPER_IMU_NODE_ID 0x03
#define REARSCRAPER_IMU_NODE_ID  0x04
#define PENDANT_NODE_ID          0x05
#define FRONT_ANGLE_NODE_ID      0x06
#define REAR_ANGLE_NODE_ID       0x07

// from EHPR98-G35 specs
#define PWM_FREQUENCY_HZ 120

// number of blades we support
#define NUM_BLADES 2

// indices into arrays
#define FRONT_BLADE_IDX 0
#define REAR_BLADE_IDX  1
#define TRACTOR_IDX     2

// how often to toggle the LED
#define LED_FLASH_PERIOD_MS 1000

// how often to transmit TPDOs
#define TPDO_OUTPUT_PERIOD_MS 50

#define NMT_RESET_CMD 0x81
#define NMT_RESET_ALL 0x00

// NMT states
#define NMT_STATE_BOOTUP      0x00
#define NMT_STATE_OPERATIONAL 0x05

// number of nodes to monitor
#define NUM_NODES 8

// maximum time to wait for a heartbeat before declaring a node as missing
#define MAX_HEARTBEAT_TIME 300

// time between heartbeats in millseconds
#define HB_PRODUCER_TIME_MS 100

// CANopen error code for estop
#define ESTOP_ERROR_CODE 0x1000

// max time to find the pendant before we assume emergency stop
#define MAX_PENDANT_SEARCH_TIME 4000

// serial connection speed expected by opengrade3d
#define OPENGRADE3D_BAUDRATE 38400

// CAN bus speed
#define CAN_BITRATE_BPS 125000

// perform blade control periodically
#define BLADE_CONTROL_PERIOD_MS 50

// mimumum time between two jog moves per mm
#define MIN_TIME_BETWEEN_JOGS_MS 200

// allowed range for the cutvalve command
#define CUTVALVE_MIN 0
#define CUTVALVE_MAX 200

// allowed range for slave offset
#define SLAVE_OFFSET_MIN (-128)
#define SLAVE_OFFSET_MAX 127

// how often to send ping to OpenGrade3D
#define PING_PERIOD_MS 1000

// time to wait before deciding that OG3D has disconnected
#define PING_TIMEOUT_PERIOD_MS 3000

// supported PGNs
typedef enum _pgn_t : uint16_t
{
  // misc
  PGN_ESTOP                    = 0x0000,
  PGN_RESET                    = 0x0001,
  PGN_OG3D_STARTED             = 0x0002,
  PGN_PING                     = 0x0003,

  // blade control
  PGN_FRONT_CUT_VALVE          = 0x1000,   // CUTVALVE_MIN -> CUTVALVE_MAX
  PGN_REAR_CUT_VALVE           = 0x1001,   // CUTVALVE_MIN -> CUTVALVE_MAX
  PGN_FRONT_ZERO_BLADE_HEIGHT  = 0x1002,
  PGN_REAR_ZERO_BLADE_HEIGHT   = 0x1003,

  // blade configuration
  PGN_FRONT_PWM_GAIN_UP        = 0x2002,
  PGN_FRONT_PWM_GAIN_DOWN      = 0x2003,
  PGN_FRONT_PWM_MIN_UP         = 0x2004,
  PGN_FRONT_PWM_MIN_DOWN       = 0x2005,
  PGN_FRONT_PWM_MAX_UP         = 0x2006,
  PGN_FRONT_PWM_MAX_DOWN       = 0x2007,
  PGN_FRONT_INTEGRAL_MULTPLIER = 0x2008,
  PGN_FRONT_DEADBAND           = 0x2009,
  PGN_REAR_PWM_GAIN_UP         = 0x200A,
  PGN_REAR_PWM_GAIN_DOWN       = 0x200B,
  PGN_REAR_PWM_MIN_UP          = 0x200C,
  PGN_REAR_PWM_MIN_DOWN        = 0x200D,
  PGN_REAR_PWM_MAX_UP          = 0x200E,
  PGN_REAR_PWM_MAX_DOWN        = 0x200F,
  PGN_REAR_INTEGRAL_MULTPLIER  = 0x2010,
  PGN_REAR_DEADBAND            = 0x2011,

  // autosteer control
  PGN_AUTOSTEER_RELAY          = 0x3000,
  PGN_AUTOSTEER_SPEED          = 0x3001,
  PGN_AUTOSTEER_DISTANCE       = 0x3002,
  PGN_AUTOSTEER_ANGLE          = 0x3003,

  // autosteer configuration
  PGN_AUTOSTEER_KP             = 0x4000,
  PGN_AUTOSTEER_KI             = 0x4001,
  PGN_AUTOSTEER_KD             = 0x4002,
  PGN_AUTOSTEER_KO             = 0x4003,
  PGN_AUTOSTEER_OFFSET         = 0x4004,
  PGN_AUTOSTEER_MIN_PWM        = 0x4005,
  PGN_AUTOSTEER_MAX_INTEGRAL   = 0x4006,
  PGN_AUTOSTEER_COUNTS_PER_DEG = 0x4007,

  // blade status
  PGN_FRONT_BLADE_OFFSET_SLAVE = 0x5000,
  PGN_FRONT_BLADE_PWMVALUE     = 0x5001,
  PGN_FRONT_BLADE_DIRECTION    = 0x5002,
  PGN_FRONT_BLADE_AUTO         = 0x5003,
  PGN_REAR_BLADE_OFFSET_SLAVE  = 0x5004,
  PGN_REAR_BLADE_PWMVALUE      = 0x5005,
  PGN_REAR_BLADE_DIRECTION     = 0x5006,
  PGN_REAR_BLADE_AUTO          = 0x5007,
  PGN_FRONT_BLADE_HEIGHT       = 0x5008,
  PGN_REAR_BLADE_HEIGHT        = 0x5009,

  // IMU
  PGN_TRACTOR_PITCH            = 0x6000,
  PGN_TRACTOR_ROLL             = 0x6001,
  PGN_TRACTOR_HEADING          = 0x6002,
  PGN_TRACTOR_YAWRATE          = 0x6003,
  PGN_FRONT_PITCH              = 0x6004,
  PGN_FRONT_ROLL               = 0x6005,
  PGN_FRONT_HEADING            = 0x6006,
  PGN_FRONT_YAWRATE            = 0x6007,
  PGN_REAR_PITCH               = 0x6008,
  PGN_REAR_ROLL                = 0x6009,
  PGN_REAR_HEADING             = 0x600A,
  PGN_REAR_YAWRATE             = 0x600B,
} pgn_t;

typedef enum _blade_direction_t
{
  BLADE_DIR_DOWN = 0,
  BLADE_DIR_UP   = 1
} blade_direction_t;

typedef enum _state_t
{
  STATE_RUN,
  STATE_ESTOP
} state_t;

// status information that is transmitted to OpenGrade3D
typedef struct _controllerstatus_t
{
  pgn_t PGN;
  uint32_t Value;
} controllerstatus_t;

// commands that are received from OpenGrade3D
typedef struct _opengrade3dcommand_t
{
  pgn_t PGN;
  uint32_t Value;
} opengrade3dcommand_t;

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
  int CutValve;       // target blade height in mm. 100 = on target, < 100 below target, > 100 above target. Range is 0 - 200
} blade_command_t;

typedef union _button_state_t
{
  uint16_t RawValue;
  struct
  {
    unsigned int Button1Pressed   : 1;
    unsigned int Button2Pressed   : 1;
    unsigned int Button3Pressed   : 1;
    unsigned int Button4Pressed   : 1;
    unsigned int EStopArmed       : 1;
    unsigned int Reserved1        : 3;
    unsigned int Joystick1Pressed : 1;
    unsigned int Joystick2Pressed : 1;
    unsigned int Reserved2        : 6;
  } Fields;
} button_state_t;

typedef union _joystick_state_t
{
  uint8_t RawValue;
  struct
  {
    unsigned int Joystick1Up    : 1;
    unsigned int Joystick1Down  : 1;
    unsigned int Joystick1Right : 1;
    unsigned int Joystick1Left  : 1;
    unsigned int Joystick2Up    : 1;
    unsigned int Joystick2Down  : 1;
    unsigned int Joystick2Right : 1;
    unsigned int Joystick2Left  : 1;
  } Fields;
} joystick_state_t;

typedef struct _imu_t
{
  float Roll;
  float Pitch;
  float Heading;
  float YawRate;
} imu_t;

static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANBus;
static elapsedMillis LEDFlashTimestamp;
static elapsedMillis HBTime[NUM_NODES];
static bool NodeFound[NUM_NODES];
static elapsedMillis PendantSearchTimestamp;
static button_state_t ButtonState;
static joystick_state_t JoystickState;
static bool PendantSearch;
static SerialTransfer OpenGrade3D;
static blade_config_t BladeConfig[NUM_BLADES];
static blade_status_t BladeStatus[NUM_BLADES];
static blade_command_t BladeCommand[NUM_BLADES];
static elapsedMillis BladeControlTimestamp;
static int pwm1ago[NUM_BLADES] = { 0 } , pwm2ago[NUM_BLADES] = { 0 }, pwm3ago[NUM_BLADES] = { 0 }, pwm4ago[NUM_BLADES] = { 0 }, pwm5ago[NUM_BLADES] = { 0 };
static elapsedMillis TPDOTimestamp;
static elapsedMillis HBTimestamp;
static state_t State;
static elapsedMillis LastJogTime[NUM_BLADES];
static imu_t IMUValues[NUM_BLADES + 1];
static int BladeHeight[NUM_BLADES];  // in mm
static elapsedMillis PingTimestamp;
static elapsedMillis LastPingRxTimestamp;
static bool OG3DFound = false;

// resets the controller
static void Reset
  (
  void
  )
{
  SCB_AIRCR = 0x05FA0004;
  while(1);
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
}

// transmits a bootup message
static void TxBootup
  (
  void
  )
{
  uint8_t Data[1];

  Data[0] = NMT_STATE_BOOTUP;
  TxCANMessage(0x700 + CONTROLLER_NODE_ID, 1, Data);
}

// transmits a heartbeat message
static void TxHeartbeat
  (
  void
  )
{
  uint8_t Data[1];

  Data[0] = NMT_STATE_OPERATIONAL;
  TxCANMessage(0x700 + CONTROLLER_NODE_ID, 1, Data);
}

// transmit TPDO1
static void TxTPDO1
  (
  void
  )
{
  uint8_t Data[8];

  Data[0] = BladeStatus[FRONT_BLADE_IDX].BladePWM & 0xFF;
  Data[1] = (BladeStatus[FRONT_BLADE_IDX].BladePWM >> 8) & 0xFF;
  Data[2] = BladeStatus[FRONT_BLADE_IDX].BladeCommand;
  Data[3] = (BladeStatus[FRONT_BLADE_IDX].BladeDirection & 0x01) | ((BladeStatus[FRONT_BLADE_IDX].BladeAuto & 0x01) << 1);

  Data[4] = BladeStatus[REAR_BLADE_IDX].BladePWM & 0xFF;
  Data[5] = (BladeStatus[REAR_BLADE_IDX].BladePWM >> 8) & 0xFF;
  Data[6] = BladeStatus[REAR_BLADE_IDX].BladeCommand;
  Data[7] = (BladeStatus[REAR_BLADE_IDX].BladeDirection & 0x01) | ((BladeStatus[REAR_BLADE_IDX].BladeAuto & 0x01) << 1);

  TxCANMessage(0x180 + CONTROLLER_NODE_ID, 8, Data);
}

// transmit TPDO2
static void TxTPDO2
  (
  void
  )
{
  uint8_t Data[2];

  Data[0] = (uint8_t)BladeStatus[FRONT_BLADE_IDX].SlaveOffset;
  Data[1] = (uint8_t)BladeStatus[REAR_BLADE_IDX].SlaveOffset;

  TxCANMessage(0x280 + CONTROLLER_NODE_ID, 2, Data);
}

// perform an emergency stop of blade control
static void EmergencyStop
  (
  int LineNumber
  )
{
  if (State == STATE_RUN)
  {
    State = STATE_ESTOP;

    // tell OpenGrade3D
    controllerstatus_t Status;
    Status.PGN = PGN_ESTOP;
    Status.Value = 0;
    SendStatus(&Status);

    // send emergency message so all CAN nodes are aware of the stop
    CAN_message_t txmsg;
    txmsg.id = 0x080 + CONTROLLER_NODE_ID;
    txmsg.len = 8;
    txmsg.buf[0] =  ESTOP_ERROR_CODE       & 0xFF;
    txmsg.buf[1] = (ESTOP_ERROR_CODE >> 8) & 0xFF;
    txmsg.buf[2] = 0x80;  // manufacturer-specific error
    txmsg.buf[3] =  LineNumber        & 0xFF;
    txmsg.buf[4] = (LineNumber >> 8)  & 0xFF;
    txmsg.buf[5] = (LineNumber >> 16) & 0xFF;
    txmsg.buf[6] = (LineNumber >> 24) & 0xFF;
    txmsg.buf[7] = 0x00;
    CANBus.write(txmsg);

    Serial.println("ESTOP!");

    // switch to manual control, stop movement
    for (int b = 0; b < NUM_BLADES; b++)
    {
      BladeStatus[b].BladeAuto = false;
      BladeCommand[b].CutValve = 100;
    }
    SetFrontValvePWM(0);
    SetRearValvePWM(0);

    TxTPDO1();
    TxTPDO2();

    TxFrontBladeAuto();
    TxRearBladeAuto();
  }
}

// process TPDO from angle sensors
static void ProcessAngleTPDO
  (
  uint8_t NodeId,
  uint8_t Length,
  const uint8_t *pData
  )
{
  // fixme - to do

  // send updated blade heights
  TxFrontBladeHeight();
  TxRearBladeHeight();
}

// process TPDO from IMU
static void ProcessIMUTPDO
  (
  uint8_t NodeId,
  uint8_t Length,
  const uint8_t *pData
  )
{
  if (Length == 8)
  {
    float Heading = ((uint16_t)(pData[0] | ((uint16_t)pData[1] << 8))) / 100.0;
    float Pitch   = ((int16_t)(pData[2] | ((uint16_t)pData[3] << 8))) / 100.0;
    float Roll    = ((int16_t)(pData[4] | ((uint16_t)pData[5] << 8))) / 100.0;
    float YawRate = ((int16_t)(pData[6] | ((uint16_t)pData[7] << 8))) / 100.0;

    switch (NodeId)
    {
      case TRACTOR_IMU_NODE_ID:
        IMUValues[TRACTOR_IDX].Heading = Heading;
        IMUValues[TRACTOR_IDX].Pitch   = Pitch;
        IMUValues[TRACTOR_IDX].Roll    = Roll;
        IMUValues[TRACTOR_IDX].YawRate = YawRate;
        TxTractorIMU();
        break;

      case FRONTSCRAPER_IMU_NODE_ID:
        IMUValues[FRONT_BLADE_IDX].Heading = Heading;
        IMUValues[FRONT_BLADE_IDX].Pitch   = Pitch;
        IMUValues[FRONT_BLADE_IDX].Roll    = Roll;
        IMUValues[FRONT_BLADE_IDX].YawRate = YawRate;
        TxFrontScraperIMU();
        break;

      case REARSCRAPER_IMU_NODE_ID:
        IMUValues[REAR_BLADE_IDX].Heading = Heading;
        IMUValues[REAR_BLADE_IDX].Pitch   = Pitch;
        IMUValues[REAR_BLADE_IDX].Roll    = Roll;
        IMUValues[REAR_BLADE_IDX].YawRate = YawRate;
        TxRearScraperIMU();
        break;
    }
  }
}

// sends the front blade height to OpenGrade3D
static void TxFrontBladeHeight
  (
  void
  )
{
  controllerstatus_t Status;

  Status.PGN = PGN_FRONT_BLADE_HEIGHT;
  Status.Value = BladeHeight[FRONT_BLADE_IDX];
  SendStatus(&Status);
}

// sends the rear blade height to OpenGrade3D
static void TxRearBladeHeight
  (
  void
  )
{
  controllerstatus_t Status;

  Status.PGN = PGN_REAR_BLADE_HEIGHT;
  Status.Value = BladeHeight[REAR_BLADE_IDX];
  SendStatus(&Status);
}

// send tractor IMU values to OpenGrade3D
static void TxTractorIMU
  (
  void
  )
{
  controllerstatus_t Status;

  Status.PGN = PGN_TRACTOR_ROLL;
  Status.Value = (int32_t)(IMUValues[TRACTOR_IDX].Roll * 100);
  SendStatus(&Status);
  Status.PGN = PGN_TRACTOR_PITCH;
  Status.Value = (int32_t)(IMUValues[TRACTOR_IDX].Pitch * 100);
  SendStatus(&Status);
  Status.PGN = PGN_TRACTOR_HEADING;
  Status.Value = (uint32_t)(IMUValues[TRACTOR_IDX].Heading * 100);
  SendStatus(&Status);
  Status.PGN = PGN_TRACTOR_YAWRATE;
  Status.Value = (int32_t)(IMUValues[TRACTOR_IDX].YawRate * 100);
  SendStatus(&Status);
}

// send front scraper IMU values to OpenGrade3D
static void TxFrontScraperIMU
  (
  void
  )
{
  controllerstatus_t Status;

  Status.PGN = PGN_FRONT_ROLL;
  Status.Value = (int32_t)(IMUValues[FRONT_BLADE_IDX].Roll * 100);
  SendStatus(&Status);
  Status.PGN = PGN_FRONT_PITCH;
  Status.Value = (int32_t)(IMUValues[FRONT_BLADE_IDX].Pitch * 100);
  SendStatus(&Status);
  Status.PGN = PGN_FRONT_HEADING;
  Status.Value = (uint32_t)(IMUValues[FRONT_BLADE_IDX].Heading * 100);
  SendStatus(&Status);
  Status.PGN = PGN_FRONT_YAWRATE;
  Status.Value = (int32_t)(IMUValues[FRONT_BLADE_IDX].YawRate * 100);
  SendStatus(&Status);
}

// send rear scraper IMU values to OpenGrade3D
static void TxRearScraperIMU
  (
  void
  )
{
  controllerstatus_t Status;

  Status.PGN = PGN_REAR_ROLL;
  Status.Value = (int32_t)(IMUValues[REAR_BLADE_IDX].Roll * 100);
  SendStatus(&Status);
  Status.PGN = PGN_REAR_PITCH;
  Status.Value = (int32_t)(IMUValues[REAR_BLADE_IDX].Pitch * 100);
  SendStatus(&Status);
  Status.PGN = PGN_REAR_HEADING;
  Status.Value = (uint32_t)(IMUValues[REAR_BLADE_IDX].Heading * 100);
  SendStatus(&Status);
  Status.PGN = PGN_REAR_YAWRATE;
  Status.Value = (int32_t)(IMUValues[REAR_BLADE_IDX].YawRate * 100);
  SendStatus(&Status);
}

// send front blade slave offset to OpenGrade3D
static void TxFrontBladeSlaveOffset
  (
  void
  )
{
  controllerstatus_t Status;

  Status.PGN = PGN_FRONT_BLADE_OFFSET_SLAVE;
  Status.Value = BladeStatus[FRONT_BLADE_IDX].SlaveOffset;
  SendStatus(&Status);
}

// send rear blade slave offset to OpenGrade3D
static void TxRearBladeSlaveOffset
  (
  void
  )
{
  controllerstatus_t Status;

  Status.PGN = PGN_REAR_BLADE_OFFSET_SLAVE;
  Status.Value = BladeStatus[REAR_BLADE_IDX].SlaveOffset;
  SendStatus(&Status);
}

// send front blade auto state to OpenGrade3D
static void TxFrontBladeAuto
  (
  void
  )
{
  controllerstatus_t Status;

  Status.PGN = PGN_FRONT_BLADE_AUTO;
  Status.Value = BladeStatus[FRONT_BLADE_IDX].BladeAuto;
  SendStatus(&Status);
}

// send rear blade auto state to OpenGrade3D
static void TxRearBladeAuto
  (
  void
  )
{
  controllerstatus_t Status;

  Status.PGN = PGN_REAR_BLADE_AUTO;
  Status.Value = BladeStatus[REAR_BLADE_IDX].BladeAuto;
  SendStatus(&Status);
}

// process TPDO from pendant
static void ProcessPendantTPDO
  (
  uint8_t Length,
  const uint8_t *pData
  )
{
  if (Length == 3)
  {
    unsigned int LastButton1State = ButtonState.Fields.Button1Pressed;
    unsigned int LastButton2State = ButtonState.Fields.Button2Pressed;

    ButtonState.RawValue = (pData[0] | ((uint16_t)pData[1] << 8));
    JoystickState.RawValue = pData[2];

    // ESTOP PRESSED
    if (!ButtonState.Fields.EStopArmed)
    {
      EmergencyStop(__LINE__);
    }

    if (State == STATE_RUN)
    {
      // toggle auto mode for front blade
      if (ButtonState.Fields.Button1Pressed && !LastButton1State)
      {
        if (BladeStatus[FRONT_BLADE_IDX].BladeAuto)
        {
          BladeStatus[FRONT_BLADE_IDX].BladeAuto = false;
        }
        else
        {
          BladeStatus[FRONT_BLADE_IDX].BladeAuto = true;
        }
        TxFrontBladeAuto();
      }

      // toggle auto mode for rear blade
      if (ButtonState.Fields.Button2Pressed && !LastButton2State)
      {
        if (BladeStatus[REAR_BLADE_IDX].BladeAuto)
        {
          BladeStatus[REAR_BLADE_IDX].BladeAuto = false;
        }
        else
        {
          BladeStatus[REAR_BLADE_IDX].BladeAuto = true;
        }
        TxRearBladeAuto();
      }

      // if joystick 1 is moved up or down in auto mode then exit auto mode
      if ((JoystickState.Fields.Joystick1Up || JoystickState.Fields.Joystick1Down) && BladeStatus[FRONT_BLADE_IDX].BladeAuto)
      {
        BladeStatus[FRONT_BLADE_IDX].BladeAuto = false;
        TxFrontBladeAuto();
      }

      // if joystick 2 is moved up or down in auto mode then exit auto mode
      if ((JoystickState.Fields.Joystick2Up || JoystickState.Fields.Joystick2Down) && BladeStatus[REAR_BLADE_IDX].BladeAuto)
      {
        BladeStatus[REAR_BLADE_IDX].BladeAuto = false;
        TxRearBladeAuto();
      }

      if (!BladeStatus[FRONT_BLADE_IDX].BladeAuto)
      {
        // jog front blade (joystick button not pressed)
        if (JoystickState.Fields.Joystick1Up && !ButtonState.Fields.Joystick1Pressed)
        {
          if (LastJogTime[FRONT_BLADE_IDX] >= MIN_TIME_BETWEEN_JOGS_MS)
          {
            BladeCommand[FRONT_BLADE_IDX].CutValve += 1;
            if (BladeCommand[FRONT_BLADE_IDX].CutValve > CUTVALVE_MAX) BladeCommand[FRONT_BLADE_IDX].CutValve = CUTVALVE_MAX;
            LastJogTime[FRONT_BLADE_IDX] = 0;
          }
        }
        else if (JoystickState.Fields.Joystick1Down && !ButtonState.Fields.Joystick1Pressed)
        {
          if (LastJogTime[FRONT_BLADE_IDX] >= MIN_TIME_BETWEEN_JOGS_MS)
          {
            BladeCommand[FRONT_BLADE_IDX].CutValve -= 1;
            if (BladeCommand[FRONT_BLADE_IDX].CutValve < CUTVALVE_MIN) BladeCommand[FRONT_BLADE_IDX].CutValve = CUTVALVE_MIN;
            LastJogTime[FRONT_BLADE_IDX] = 0;
          }
        }
        // adjust slave offset (joystick button is pressed)
        else if (JoystickState.Fields.Joystick1Up && ButtonState.Fields.Joystick1Pressed)
        {
          if (LastJogTime[FRONT_BLADE_IDX] >= MIN_TIME_BETWEEN_JOGS_MS)
          {
            BladeStatus[FRONT_BLADE_IDX].SlaveOffset += 1;
            if (BladeStatus[FRONT_BLADE_IDX].SlaveOffset > SLAVE_OFFSET_MAX) BladeStatus[FRONT_BLADE_IDX].SlaveOffset = SLAVE_OFFSET_MAX;
            LastJogTime[FRONT_BLADE_IDX] = 0;
            TxFrontBladeSlaveOffset();
          }
        }
        else if (JoystickState.Fields.Joystick1Down && ButtonState.Fields.Joystick1Pressed)
        {
          if (LastJogTime[FRONT_BLADE_IDX] >= MIN_TIME_BETWEEN_JOGS_MS)
          {
            BladeStatus[FRONT_BLADE_IDX].SlaveOffset -= 1;
            if (BladeStatus[FRONT_BLADE_IDX].SlaveOffset < SLAVE_OFFSET_MIN) BladeStatus[FRONT_BLADE_IDX].SlaveOffset = SLAVE_OFFSET_MIN;
            LastJogTime[FRONT_BLADE_IDX] = 0;
            TxFrontBladeSlaveOffset();
          }
        }
      }

      if (!BladeStatus[REAR_BLADE_IDX].BladeAuto)
      {
        // jog rear blade (joystick button not pressed)
        if (JoystickState.Fields.Joystick2Up && !ButtonState.Fields.Joystick2Pressed)
        {
          if (LastJogTime[REAR_BLADE_IDX] >= MIN_TIME_BETWEEN_JOGS_MS)
          {
            BladeCommand[REAR_BLADE_IDX].CutValve += 1;
            if (BladeCommand[REAR_BLADE_IDX].CutValve > CUTVALVE_MAX) BladeCommand[REAR_BLADE_IDX].CutValve = CUTVALVE_MAX;
            LastJogTime[REAR_BLADE_IDX] = 0;
          }
        }
        else if (JoystickState.Fields.Joystick2Down && !ButtonState.Fields.Joystick2Pressed)
        {
          if (LastJogTime[REAR_BLADE_IDX] >= MIN_TIME_BETWEEN_JOGS_MS)
          {
            BladeCommand[REAR_BLADE_IDX].CutValve -= 1;
            if (BladeCommand[REAR_BLADE_IDX].CutValve < CUTVALVE_MIN) BladeCommand[REAR_BLADE_IDX].CutValve = CUTVALVE_MIN;
            LastJogTime[REAR_BLADE_IDX] = 0;
          }
        }
        // adjust slave offset (joystick button is pressed)
        else if (JoystickState.Fields.Joystick2Up && ButtonState.Fields.Joystick2Pressed)
        {
          if (LastJogTime[REAR_BLADE_IDX] >= MIN_TIME_BETWEEN_JOGS_MS)
          {
            BladeStatus[REAR_BLADE_IDX].SlaveOffset += 1;
            if (BladeStatus[REAR_BLADE_IDX].SlaveOffset > SLAVE_OFFSET_MAX) BladeStatus[REAR_BLADE_IDX].SlaveOffset = SLAVE_OFFSET_MAX;
            LastJogTime[REAR_BLADE_IDX] = 0;
            TxRearBladeSlaveOffset();
          }
        }
        else if (JoystickState.Fields.Joystick2Down && ButtonState.Fields.Joystick2Pressed)
        {
          if (LastJogTime[REAR_BLADE_IDX] >= MIN_TIME_BETWEEN_JOGS_MS)
          {
            BladeStatus[REAR_BLADE_IDX].SlaveOffset -= 1;
            if (BladeStatus[REAR_BLADE_IDX].SlaveOffset < SLAVE_OFFSET_MIN) BladeStatus[REAR_BLADE_IDX].SlaveOffset = SLAVE_OFFSET_MIN;
            LastJogTime[REAR_BLADE_IDX] = 0;
            TxRearBladeSlaveOffset();
          }
        }
      }
    }
  }
}

// process the heartbeat from a node
static void ProcessHeartbeat
  (
  uint8_t NodeId,
  uint8_t Length,
  const uint8_t *pData
  )
{
  if (Length == 1)
  {
    uint8_t NMTState = pData[0];

    // if we see a bootup message then we have found a node
    if (NMTState == NMT_STATE_BOOTUP)
    {
      NodeFound[NodeId - 1] = true;
      HBTime[NodeId - 1] = 0;
      Serial.println("Found node");
    }

    // on every heartbeat reset the timer
    if (NMTState == NMT_STATE_OPERATIONAL)
    {
      // must have missed the bootup message
      if (!NodeFound[NodeId -1])
      {
        NodeFound[NodeId - 1] = true;
      }

      HBTime[NodeId - 1] = 0;
    }
  }
}

// checks to see if any nodes are missing
static void CheckForMissingNodes
  (
  void
  )
{
  for (int n = 0; n < NUM_NODES; n++)
  {
    if (NodeFound[n])
    {
      if (HBTime[n] >= MAX_HEARTBEAT_TIME)
      {
        Serial.println("NODE MISSING");
        NodeFound[n] = false;

        // if the pendant has dissappeared then perform emergency stop
        // as we can't see the ESTOP button
        if ((n + 1) == PENDANT_NODE_ID)
        {
          EmergencyStop(__LINE__);
        }
      }
    }
  }
}

// called when a CAN message is received
static void CANReceiveHandler
  (
  const CAN_message_t &msg
  )
{
  switch (msg.id)
  {
    // PDOs
    case 0x180 + PENDANT_NODE_ID:
      ProcessPendantTPDO(msg.len, msg.buf);
      break;
    case 0x180 + TRACTOR_IMU_NODE_ID:
      ProcessIMUTPDO(TRACTOR_IMU_NODE_ID, msg.len, msg.buf);
      break;
    case 0x180 + FRONTSCRAPER_IMU_NODE_ID:
      ProcessIMUTPDO(FRONTSCRAPER_IMU_NODE_ID, msg.len, msg.buf);
      break;
    case 0x180 + REARSCRAPER_IMU_NODE_ID:
      ProcessIMUTPDO(REARSCRAPER_IMU_NODE_ID, msg.len, msg.buf);
      break;
    case 0x180 + FRONT_ANGLE_NODE_ID:
      ProcessAngleTPDO(FRONT_ANGLE_NODE_ID, msg.len, msg.buf);
      break;
    case 0x180 + REAR_ANGLE_NODE_ID:
      ProcessAngleTPDO(REAR_ANGLE_NODE_ID, msg.len, msg.buf);
      break;

    // heartbeats
    case 0x700 + PENDANT_NODE_ID:
      ProcessHeartbeat(PENDANT_NODE_ID, msg.len, msg.buf);
      break;
  }
}

// resets all nodes on the CAN bus
static void ResetAllNodes
  (
  void
  )
{
  CAN_message_t txmsg;
  
  txmsg.id = 0x000;
  txmsg.len = 2;
  txmsg.buf[0] = NMT_RESET_CMD;
  txmsg.buf[1] = NMT_RESET_ALL;

  CANBus.write(txmsg);
}

// sends status value to OpenGrade3D
static void SendStatus
  (
  controllerstatus_t *pStatus
  )
{
  OpenGrade3D.packet.txBuff[0] = (byte)(pStatus->PGN & 0xFF);
  OpenGrade3D.packet.txBuff[1] = (byte)((pStatus->PGN >> 8) & 0xFF);
  OpenGrade3D.packet.txBuff[2] = (byte)(pStatus->Value & 0xFF);
  OpenGrade3D.packet.txBuff[3] = (byte)((pStatus->Value >> 8) & 0xFF);
  OpenGrade3D.packet.txBuff[4] = (byte)((pStatus->Value >> 16) & 0xFF);
  OpenGrade3D.packet.txBuff[5] = (byte)((pStatus->Value >> 24) & 0xFF);
  OpenGrade3D.sendData(6);
}

// gets a command from OpenGrade3D
static opengrade3dcommand_t GetCommand
  (
  void
  )
{
  opengrade3dcommand_t Command;

  Command.PGN = (pgn_t)(((uint16_t)OpenGrade3D.packet.rxBuff[1] << 8) | OpenGrade3D.packet.rxBuff[0]);
  Command.Value = ((uint32_t)OpenGrade3D.packet.rxBuff[5] << 24) |
                  ((uint32_t)OpenGrade3D.packet.rxBuff[4] << 16) |
                  ((uint32_t)OpenGrade3D.packet.rxBuff[3] << 8)  |
                   (uint32_t)OpenGrade3D.packet.rxBuff[2];

  return Command;
}

// calculate new output for blade
static void ControlBlade
  (
  int BladeIndex              // xxx_BLADE_IDX
  )
{
  int PWMValue;
  float PWMHist;

  // only control the blade if we are in the run state
  if (State != STATE_RUN)
  {
    return;
  }

  // store command
  BladeStatus[BladeIndex].BladeCommand = BladeCommand[BladeIndex].CutValve;

  // lower the blade
  if (BladeCommand[BladeIndex].CutValve >= (100 + BladeConfig[BladeIndex].Deadband))
  {
    // PWM value is negative
    PWMValue = -((BladeCommand[BladeIndex].CutValve - 100 - BladeConfig[BladeIndex].Deadband) * BladeConfig[BladeIndex].PWMGainDown + BladeConfig[BladeIndex].PWMMinDown);
  }
  // lift the blade
  else if (BladeCommand[BladeIndex].CutValve <= (100 - BladeConfig[BladeIndex].Deadband))
  {
    // PWM value is positive
    PWMValue = -((BladeCommand[BladeIndex].CutValve - 100 + BladeConfig[BladeIndex].Deadband) * BladeConfig[BladeIndex].PWMGainUp - BladeConfig[BladeIndex].PWMMinUp);
  }
  else
  {
    PWMValue = 0;
  }

  // calculate a derivative
  if (BladeCommand[BladeIndex].CutValve != 100 && PWMValue != 0)
  {
    PWMHist = ((((pwm1ago[BladeIndex]) + pwm2ago[BladeIndex] + (pwm3ago[BladeIndex]) + (pwm4ago[BladeIndex]) + (pwm5ago[BladeIndex] / 2.000)) * (sq(BladeConfig[BladeIndex].IntegralMultiplier) / 100.0000)) / sq(BladeCommand[BladeIndex].CutValve - 100.0000));

    //put pwmHist to 0 when the blade cross the line.
    if (BladeCommand[BladeIndex].CutValve > 100 && (pwm1ago[BladeIndex] + pwm2ago[BladeIndex] + pwm3ago[BladeIndex] + pwm4ago[BladeIndex] + pwm5ago[BladeIndex]) > 0) PWMHist = 0;
    if (BladeCommand[BladeIndex].CutValve < 100 && (pwm1ago[BladeIndex] + pwm2ago[BladeIndex] + pwm3ago[BladeIndex] + pwm4ago[BladeIndex] + pwm5ago[BladeIndex]) < 0) PWMHist = 0;

    PWMValue = (PWMValue - PWMHist);
  }

  // shuffle samples down
  pwm5ago[BladeIndex] = pwm4ago[BladeIndex];
  pwm4ago[BladeIndex] = pwm3ago[BladeIndex];
  pwm3ago[BladeIndex] = pwm2ago[BladeIndex];
  pwm2ago[BladeIndex] = pwm1ago[BladeIndex];
  pwm1ago[BladeIndex] = PWMValue;
  
  // enforce limits
  if (BladeCommand[BladeIndex].CutValve > 100 && PWMValue > 0) PWMValue = 0;
  if (BladeCommand[BladeIndex].CutValve > 100 && PWMValue < -(BladeConfig[BladeIndex].PWMMaxDown)) PWMValue = -(BladeConfig[BladeIndex].PWMMaxDown);
  if (BladeCommand[BladeIndex].CutValve < 100 && PWMValue < 0) PWMValue = 0;
  if (BladeCommand[BladeIndex].CutValve < 100 && PWMValue > BladeConfig[BladeIndex].PWMMaxUp) PWMValue = BladeConfig[BladeIndex].PWMMaxUp;
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

// sets the PWM value for the front valve
static void SetFrontValvePWM
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

    // update OG3D
    controllerstatus_t Status;
    Status.PGN = PGN_FRONT_BLADE_PWMVALUE;
    Status.Value = BladeStatus[FRONT_BLADE_IDX].BladePWM;
    SendStatus(&Status);

    Status.PGN = PGN_FRONT_BLADE_DIRECTION;
    Status.Value = digitalRead(FRONT_HEIGHT_DIR);
    SendStatus(&Status);
  }
}

// sets the PWM value for the rear valve
static void SetRearValvePWM
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

    // update OG3D
    controllerstatus_t Status;
    Status.PGN = PGN_REAR_BLADE_PWMVALUE;
    Status.Value = BladeStatus[REAR_BLADE_IDX].BladePWM;
    SendStatus(&Status);

    Status.PGN = PGN_REAR_BLADE_DIRECTION;
    Status.Value = digitalRead(REAR_HEIGHT_DIR);
    SendStatus(&Status);
  }
}

// initialize the hardware
void setup()
{
  // fixme - change back to Serial5
  // configure connection to opengrade3d
  Serial.begin(OPENGRADE3D_BAUDRATE);
  OpenGrade3D.begin(Serial);

  // configure CAN bus
  CANBus.begin();
  CANBus.setBaudRate(CAN_BITRATE_BPS);
  CANBus.setMaxMB(64);
  CANBus.enableFIFO();
  CANBus.onReceive(FIFO, CANReceiveHandler);
  CANBus.enableFIFOInterrupt();
  
  // TPDO1s
  CANBus.setFIFOFilterRange(0, 0x181, 0x1FF, STD);
  // TPDO2s
  CANBus.setFIFOFilterRange(1, 0x281, 0x2FF, STD);
  // Heartbeats
  CANBus.setFIFOFilterRange(2, 0x701, 0x77F, STD);

  CANBus.setMB(MB63, TX); // Set mailbox as transmit

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

  // set up LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  // reset heartbeat timers and flags
  for (int n = 0; n < NUM_NODES; n++)
  {
    HBTime[n] = 0;
    NodeFound[n] = false;
  }

  // start looking for pendant, we can't operate without it
  PendantSearchTimestamp = 0;
  PendantSearch = true;

  // reset buttons and joysticks
  ButtonState.RawValue = 0;
  JoystickState.RawValue = 0;

  HBTimestamp = 0;
  TPDOTimestamp = 0;

  for (int b = 0; b < NUM_BLADES; b++)
  {
    LastJogTime[b] = 0;
  }

  // clear all IMU values
  for (int i = 0; i < NUM_BLADES + 1; i++)
  {
    memset(&IMUValues[i], 0, sizeof(imu_t));
  }

  // reset blade heights
  for (int b = 0; b < NUM_BLADES; b++)
  {
    BladeHeight[b] = 0;
  }

  // initial blade status
  memset(&BladeStatus, 0, sizeof(blade_status_t));
  BladeStatus[FRONT_BLADE_IDX].BladeAuto = false;
  BladeStatus[REAR_BLADE_IDX].BladeAuto  = false;

  // initial state is no movement
  BladeCommand[FRONT_BLADE_IDX].CutValve = 100;
  BladeCommand[REAR_BLADE_IDX].CutValve  = 100;

  BladeControlTimestamp = 0;

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

  PingTimestamp = 0;
  LastPingRxTimestamp = 0;

  State = STATE_RUN;

  TxBootup();
  TxTPDO1();
  TxTPDO2();

  TxFrontBladeSlaveOffset();
  TxRearBladeSlaveOffset();

  TxFrontBladeAuto();
  TxRearBladeAuto();

  TxFrontBladeHeight();
  TxRearBladeHeight();

  ResetAllNodes();
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

  CheckForMissingNodes();

  // get data from OpenGrade3D
  if (OpenGrade3D.available())
  {
    opengrade3dcommand_t Command;

    OG3DFound = true;

    Command = GetCommand();

    switch (Command.PGN)
    {
      // misc
      case PGN_RESET:
        Reset();
        break;

      case PGN_OG3D_STARTED:
        // send current states
        TxFrontBladeAuto();
        TxRearBladeAuto();
        TxFrontBladeSlaveOffset();
        TxRearBladeSlaveOffset();
        break;

      case PGN_PING:
        LastPingRxTimestamp = 0;
        break;

      // reset blade height
      case PGN_FRONT_ZERO_BLADE_HEIGHT:
        BladeHeight[FRONT_BLADE_IDX] = 0;
        break;
      case PGN_REAR_ZERO_BLADE_HEIGHT:
        BladeHeight[REAR_BLADE_IDX] = 0;
        break;

        // front blade configuration
      case PGN_FRONT_PWM_GAIN_UP:
        BladeConfig[FRONT_BLADE_IDX].PWMGainUp = Command.Value;
        break;
      case PGN_FRONT_PWM_GAIN_DOWN:
        BladeConfig[FRONT_BLADE_IDX].PWMGainDown = Command.Value;
        break;
      case PGN_FRONT_PWM_MIN_UP:
        BladeConfig[FRONT_BLADE_IDX].PWMMinUp = Command.Value;
        break;
      case PGN_FRONT_PWM_MIN_DOWN:
        BladeConfig[FRONT_BLADE_IDX].PWMMinDown = Command.Value;
        break;
      case PGN_FRONT_PWM_MAX_UP:
        BladeConfig[FRONT_BLADE_IDX].PWMMaxUp = Command.Value;
        break;
      case PGN_FRONT_PWM_MAX_DOWN:
        BladeConfig[FRONT_BLADE_IDX].PWMMaxDown = Command.Value;
        break;
      case PGN_FRONT_INTEGRAL_MULTPLIER:
        BladeConfig[FRONT_BLADE_IDX].IntegralMultiplier = Command.Value;
        break;
      case PGN_FRONT_DEADBAND:
        BladeConfig[FRONT_BLADE_IDX].Deadband = Command.Value;
        break;

      // rear blade configuration
      case PGN_REAR_PWM_GAIN_UP:
        BladeConfig[REAR_BLADE_IDX].PWMGainUp = Command.Value;
        break;
      case PGN_REAR_PWM_GAIN_DOWN:
        BladeConfig[REAR_BLADE_IDX].PWMGainDown = Command.Value;
        break;
      case PGN_REAR_PWM_MIN_UP:
        BladeConfig[REAR_BLADE_IDX].PWMMinUp = Command.Value;
        break;
      case PGN_REAR_PWM_MIN_DOWN:
        BladeConfig[REAR_BLADE_IDX].PWMMinDown = Command.Value;
        break;
      case PGN_REAR_PWM_MAX_UP:
        BladeConfig[REAR_BLADE_IDX].PWMMaxUp = Command.Value;
        break;
      case PGN_REAR_PWM_MAX_DOWN:
        BladeConfig[REAR_BLADE_IDX].PWMMaxDown = Command.Value;
        break;
      case PGN_REAR_INTEGRAL_MULTPLIER:
        BladeConfig[REAR_BLADE_IDX].IntegralMultiplier = Command.Value;
        break;
      case PGN_REAR_DEADBAND:
        BladeConfig[REAR_BLADE_IDX].Deadband = Command.Value;
        break;
        
      // front blade commands
      case PGN_FRONT_CUT_VALVE:
        if (BladeStatus[FRONT_BLADE_IDX].BladeAuto)
        {
          // store for use on next calculation pass
          BladeCommand[FRONT_BLADE_IDX].CutValve = Command.Value;
        }
        break;

      // rear blade commands
      case PGN_REAR_CUT_VALVE:
        if (BladeStatus[REAR_BLADE_IDX].BladeAuto)
        {
          // store for use on next calculation pass
          BladeCommand[REAR_BLADE_IDX].CutValve = Command.Value;
        }
        break;
    }
  }

  // perform blade control
  if (BladeControlTimestamp >= BLADE_CONTROL_PERIOD_MS)
  {
    BladeControlTimestamp = 0;

    for (int b = 0; b < NUM_BLADES; b++)
    {
      ControlBlade(b);
    }
  }

  // periodically transmit data onto the CAN bus
  if (TPDOTimestamp >= TPDO_OUTPUT_PERIOD_MS)
  {
    TPDOTimestamp = 0;

    TxTPDO1();
    TxTPDO2();
  }

  // check to see if OpenGrade3D has disappeared
  if ((LastPingRxTimestamp >= PING_TIMEOUT_PERIOD_MS) && OG3DFound)
  {
    OG3DFound = false;

    EmergencyStop(__LINE__);
  }

  // check for pendant
  if (PendantSearch && (PendantSearchTimestamp >= MAX_PENDANT_SEARCH_TIME))
  {
    // stop search
    PendantSearch = false;

    // not found
    if (!NodeFound[PENDANT_NODE_ID - 1])
    {
      EmergencyStop(__LINE__);
    }
    // found pendant but emergency stop is not armed
    else if (!ButtonState.Fields.EStopArmed)
    {
      EmergencyStop(__LINE__);
    }
  }

  // transmit heartbeats
  if (HBTimestamp >= HB_PRODUCER_TIME_MS)
  {
    HBTimestamp = 0;

    TxHeartbeat();
  }

  // flash LED
  if (LEDFlashTimestamp >= LED_FLASH_PERIOD_MS)
  {
    LEDFlashTimestamp -= LED_FLASH_PERIOD_MS;
    
    digitalToggle(LED);
  }

  // tell OG3D we are alive
  if (PingTimestamp >= PING_PERIOD_MS)
  {
    PingTimestamp = 0;

    controllerstatus_t Status;
    Status.PGN   = PGN_PING;
    Status.Value = 0;
    SendStatus(&Status);
  }
}
