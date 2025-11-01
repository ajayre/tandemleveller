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

// from EHPR98-G35 specs
#define PWM_FREQUENCY_HZ 120

// number of blades we support
#define NUM_BLADES 2

// blade indices into arrays
#define FRONT_BLADE_IDX 0
#define REAR_BLADE_IDX  1

// how often to toggle the LED
#define LED_FLASH_PERIOD_MS 1000

// how often to send status to OpenGrade3D
#define STATUS_OUTPUT_PERIOD_MS 1000

// how often to transmit TPDO1
#define TPDO1_OUTPUT_PERIOD_MS 50

#define NMT_RESET_CMD 0x81
#define NMT_RESET_ALL 0x00

// NMT states
#define NMT_STATE_BOOTUP      0x00
#define NMT_STATE_OPERATIONAL 0x05

// number of nodes to monitor
#define NUM_NODES 5

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

// supported PGNs
typedef enum _pgn_t : uint16_t
{
  // misc
  PGN_ESTOP                    = 0x0000,

  // blade control
  PGN_FRONT_CUT_VALVE          = 0x1000,   // 0 - 200
  PGN_FRONT_BLADE_OFFSET       = 0x1001,
  PGN_REAR_CUT_VALVE           = 0x1002,   // 0 - 200
  PGN_REAR_BLADE_OFFSET        = 0x1003,

  // blade configuration
  PGN_FRONT_PWM_GAIN_UP        = 0x2002,
  PGN_FRONT_PWM_GAIN_DOWN      = 0x2003,
  PGN_FRONT_PWM_MIN_UP         = 0x2004,
  PGN_FRONT_PWM_MIN_DOWN       = 0x2005,
  PGN_FRONT_PWM_MAX_UP         = 0x2006,
  PGN_FRONT_PWM_MAX_DOWN       = 0x2007,
  PGN_FRONT_INTEGRAL_MULTPLIER = 0x2008,
  PGN_FRONT_DEADBAND           = 0x2009,
  PGN_REAR_PWM_GAIN_UP         = 0x2002,
  PGN_REAR_PWM_GAIN_DOWN       = 0x2003,
  PGN_REAR_PWM_MIN_UP          = 0x2004,
  PGN_REAR_PWM_MIN_DOWN        = 0x2005,
  PGN_REAR_PWM_MAX_UP          = 0x2006,
  PGN_REAR_PWM_MAX_DOWN        = 0x2007,
  PGN_REAR_INTEGRAL_MULTPLIER  = 0x2008,
  PGN_REAR_DEADBAND            = 0x2009,

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

  // controller status
  PGN_FRONT_BLADE_OFFSET_SLAVE = 0x5000,
  PGN_FRONT_BLADE_PWMVALUE     = 0x5001,
  PGN_FRONT_BLADE_DIRECTION    = 0x5002,
  PGN_REAR_BLADE_OFFSET_SLAVE  = 0x5000,
  PGN_REAR_BLADE_PWMVALUE      = 0x5001,
  PGN_REAR_BLADE_DIRECTION     = 0x5002,
} pgn_t;

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

// current status of the blade
typedef struct _blade_status_t
{
  int BladePWM;
  int BladeCommand;
  blade_direction_t BladeDirection;
  bool BladeAuto;
  int Offset;
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

static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANBus;
static elapsedMillis LEDFlashTimestamp;
static elapsedMillis HBTime[NUM_NODES];
static bool NodeFound[NUM_NODES];
static elapsedMillis PendantSearchTimestamp;
static button_state_t ButtonState;
static joystick_state_t JoystickState;
static bool PendantSearch;
static elapsedMillis StatusOutputTimestamp;
static SerialTransfer OpenGrade3D;
static blade_config_t BladeConfig[NUM_BLADES];
static blade_status_t BladeStatus[NUM_BLADES];
static blade_command_t BladeCommand[NUM_BLADES];
static elapsedMillis BladeControlTimestamp;
static int pwm1ago = 0, pwm2ago = 0, pwm3ago = 0, pwm4ago = 0, pwm5ago = 0;
static elapsedMillis TPDO1Timestamp;
static elapsedMillis HBTimestamp;
static state_t State;
static elapsedMillis LastJogTime;

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

// transmit PDO1
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

// perform an emergency stop of blade control
static void EmergencyStop
  (
  void
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
    txmsg.buf[3] = 0x00;
    txmsg.buf[4] = 0x00;
    txmsg.buf[5] = 0x00;
    txmsg.buf[6] = 0x00;
    txmsg.buf[7] = 0x00;
    CANBus.write(txmsg);

    Serial.println("ESTOP!");

    // switch to manual control, stop movement
    BladeStatus[FRONT_BLADE_IDX].BladeAuto = false;
    BladeStatus[REAR_BLADE_IDX].BladeAuto  = false;
    BladeCommand[FRONT_BLADE_IDX].CutValve = 100;
    BladeCommand[REAR_BLADE_IDX].CutValve  = 100;
    SetValvePWM(0);

    TxTPDO1();
  }
}

// process TPDO from IMU
static void ProcessIMUTPDO
  (
  uint8_t NodeId,
  uint8_t Length,
  const uint8_t *pData
  )
{
  if (Length == 6)
  {
    float Yaw   = ((int16_t)(pData[0] | ((uint16_t)pData[1] << 8))) / 100.0;
    float Pitch = ((int16_t)(pData[2] | ((uint16_t)pData[3] << 8))) / 100.0;
    float Roll  = ((int16_t)(pData[4] | ((uint16_t)pData[5] << 8))) / 100.0;
  }
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

    //Serial.printf("%s %s %s %s",
    //  JoystickState.Fields.Joystick1Up    ? "U" : "-",
    //  JoystickState.Fields.Joystick1Down  ? "D" : "-",
    //  JoystickState.Fields.Joystick1Left  ? "L" : "-",
    //  JoystickState.Fields.Joystick1Right ? "R" : "-"
    //);
    //Serial.println();

    // ESTOP PRESSED
    if (!ButtonState.Fields.EStopArmed)
    {
      EmergencyStop();
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
      }

      if (!BladeStatus[REAR_BLADE_IDX].BladeAuto)
      {
        // manual jogging of front blade
        if (JoystickState.Fields.Joystick1Up)
        {
          if (LastJogTime >= MIN_TIME_BETWEEN_JOGS_MS)
          {
            BladeCommand[FRONT_BLADE_IDX].CutValve += 1;
            if (BladeCommand[FRONT_BLADE_IDX].CutValve > 200) BladeCommand[FRONT_BLADE_IDX].CutValve = 200;
            LastJogTime = 0;
          }
        }
        else if (JoystickState.Fields.Joystick1Down)
        {
          if (LastJogTime >= MIN_TIME_BETWEEN_JOGS_MS)
          {
            BladeCommand[FRONT_BLADE_IDX].CutValve -= 1;
            if (BladeCommand[FRONT_BLADE_IDX].CutValve < 0) BladeCommand[FRONT_BLADE_IDX].CutValve = 0;
            LastJogTime = 0;
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
          EmergencyStop();
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
  void
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
  BladeStatus[FRONT_BLADE_IDX].BladeCommand = BladeCommand[FRONT_BLADE_IDX].CutValve;

  // lower the blade
  if (BladeCommand[FRONT_BLADE_IDX].CutValve >= (100 + BladeConfig[FRONT_BLADE_IDX].Deadband))
  {
    // PWM value is negative
    PWMValue = -((BladeCommand[FRONT_BLADE_IDX].CutValve - 100 - BladeConfig[FRONT_BLADE_IDX].Deadband) * BladeConfig[FRONT_BLADE_IDX].PWMGainDown + BladeConfig[FRONT_BLADE_IDX].PWMMinDown);
  }
  // lift the blade
  else if (BladeCommand[FRONT_BLADE_IDX].CutValve <= (100 - BladeConfig[FRONT_BLADE_IDX].Deadband))
  {
    // PWM value is positive
    PWMValue = -((BladeCommand[FRONT_BLADE_IDX].CutValve - 100 + BladeConfig[FRONT_BLADE_IDX].Deadband) * BladeConfig[FRONT_BLADE_IDX].PWMGainUp - BladeConfig[FRONT_BLADE_IDX].PWMMinUp);
  }
  else
  {
    PWMValue = 0;
  }

  // calculate a derivative
  if (BladeCommand[FRONT_BLADE_IDX].CutValve != 100 && PWMValue != 0)
  {
    PWMHist = ((((pwm1ago) + pwm2ago + (pwm3ago) + (pwm4ago) + (pwm5ago / 2.000)) * (sq(BladeConfig[FRONT_BLADE_IDX].IntegralMultiplier) / 100.0000)) / sq(BladeCommand[FRONT_BLADE_IDX].CutValve - 100.0000));

    //put pwmHist to 0 when the blade cross the line.
    if (BladeCommand[FRONT_BLADE_IDX].CutValve > 100 && (pwm1ago + pwm2ago + pwm3ago + pwm4ago + pwm5ago) > 0) PWMHist = 0;
    if (BladeCommand[FRONT_BLADE_IDX].CutValve < 100 && (pwm1ago + pwm2ago + pwm3ago + pwm4ago + pwm5ago) < 0) PWMHist = 0;

    PWMValue = (PWMValue - PWMHist);
  }

  // shuffle samples down
  pwm5ago = pwm4ago;
  pwm4ago = pwm3ago;
  pwm3ago = pwm2ago;
  pwm2ago = pwm1ago;
  pwm1ago = PWMValue;
  
  // enforce limits
  if (BladeCommand[FRONT_BLADE_IDX].CutValve > 100 && PWMValue > 0) PWMValue = 0;
  if (BladeCommand[FRONT_BLADE_IDX].CutValve > 100 && PWMValue < -(BladeConfig[FRONT_BLADE_IDX].PWMMaxDown)) PWMValue = -(BladeConfig[FRONT_BLADE_IDX].PWMMaxDown);
  if (BladeCommand[FRONT_BLADE_IDX].CutValve < 100 && PWMValue < 0) PWMValue = 0;
  if (BladeCommand[FRONT_BLADE_IDX].CutValve < 100 && PWMValue > BladeConfig[FRONT_BLADE_IDX].PWMMaxUp) PWMValue = BladeConfig[FRONT_BLADE_IDX].PWMMaxUp;
  if (PWMValue > 0 && PWMValue < BladeConfig[FRONT_BLADE_IDX].PWMMinUp) PWMValue = 0;
  if (PWMValue < 0 && PWMValue > -(BladeConfig[FRONT_BLADE_IDX].PWMMinDown)) PWMValue = 0;

  if (PWMValue < 0)
  {
    digitalWrite(FRONT_HEIGHT_DIR, HIGH);
    BladeStatus[FRONT_BLADE_IDX].BladeDirection = BLADE_DIR_DOWN;
  }
  else
  {
    digitalWrite(FRONT_HEIGHT_DIR, LOW);
    BladeStatus[FRONT_BLADE_IDX].BladeDirection = BLADE_DIR_UP;
  }

  SetValvePWM(abs(PWMValue));
}

// sets the PWM value for the valve
static void SetValvePWM
  (
  uint8_t Value          // new valve PWM setting 0 - 255
  )
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

// initialize the hardware
void setup()
{
  //debug.begin(Serial);
  //halt_cpu(); 

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
  CANBus.setFIFOFilter(0, 0x180 + TRACTOR_IMU_NODE_ID,      STD);
  CANBus.setFIFOFilter(1, 0x180 + FRONTSCRAPER_IMU_NODE_ID, STD);
  CANBus.setFIFOFilter(2, 0x180 + REARSCRAPER_IMU_NODE_ID,  STD);
  CANBus.setFIFOFilter(3, 0x180 + PENDANT_NODE_ID,          STD);
  CANBus.setFIFOFilter(4, 0x700 + TRACTOR_IMU_NODE_ID,      STD);
  CANBus.setFIFOFilter(5, 0x700 + FRONTSCRAPER_IMU_NODE_ID, STD);
  CANBus.setFIFOFilter(6, 0x700 + REARSCRAPER_IMU_NODE_ID,  STD);
  CANBus.setFIFOFilter(7, 0x700 + PENDANT_NODE_ID,          STD);
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

  StatusOutputTimestamp = 0;

  // reset buttons and joysticks
  ButtonState.RawValue = 0;
  JoystickState.RawValue = 0;

  HBTimestamp = 0;
  TPDO1Timestamp = 0;

  LastJogTime = 0;

  // initial blade status
  memset(&BladeStatus, 0, sizeof(blade_status_t));
  BladeStatus[FRONT_BLADE_IDX].BladeAuto = false;
  BladeStatus[REAR_BLADE_IDX].BladeAuto  = false;

  // initial state is no movement
  BladeCommand[FRONT_BLADE_IDX].CutValve = 100;
  BladeCommand[REAR_BLADE_IDX].CutValve  = 100;

  BladeControlTimestamp = 0;

  // default PWM configuruation
  BladeConfig[FRONT_BLADE_IDX].PWMGainUp          = 4;
  BladeConfig[FRONT_BLADE_IDX].PWMGainDown        = 3;
  BladeConfig[FRONT_BLADE_IDX].PWMMinUp           = 50;
  BladeConfig[FRONT_BLADE_IDX].PWMMinDown         = 50;
  BladeConfig[FRONT_BLADE_IDX].PWMMaxUp           = 180;
  BladeConfig[FRONT_BLADE_IDX].PWMMaxDown         = 180;
  BladeConfig[FRONT_BLADE_IDX].IntegralMultiplier = 20;
  BladeConfig[FRONT_BLADE_IDX].Deadband           = 3;

  State = STATE_RUN;

  TxBootup();
  TxTPDO1();

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

    Command = GetCommand();

    switch (Command.PGN)
    {
      // blade configuration
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

      // blade commands
      case PGN_FRONT_CUT_VALVE:
        if (BladeStatus[FRONT_BLADE_IDX].BladeAuto)
        {
          // store for use on next calculation pass
          BladeCommand[FRONT_BLADE_IDX].CutValve = Command.Value;
        }
        break;
      case PGN_FRONT_BLADE_OFFSET:
        // not used
        break;
    }
  }

  // perform blade control
  if (BladeControlTimestamp >= BLADE_CONTROL_PERIOD_MS)
  {
    BladeControlTimestamp = 0;

    ControlBlade();
  }

  // periodically send status to OpenGrade3D
  if (StatusOutputTimestamp >= STATUS_OUTPUT_PERIOD_MS)
  {
    StatusOutputTimestamp = 0;

    controllerstatus_t Status;

    // output blade offset
    Status.PGN = PGN_FRONT_BLADE_OFFSET_SLAVE;
    Status.Value = BladeStatus[FRONT_BLADE_IDX].Offset;
    SendStatus(&Status);
  }

  // periodically transmit data onto the CAN bus
  if (TPDO1Timestamp >= TPDO1_OUTPUT_PERIOD_MS)
  {
    TPDO1Timestamp = 0;

    TxTPDO1();
  }

  // check for pendant
  if (PendantSearch && (PendantSearchTimestamp >= MAX_PENDANT_SEARCH_TIME))
  {
    // stop search
    PendantSearch = false;

    // not found
    if (!NodeFound[PENDANT_NODE_ID - 1])
    {
      EmergencyStop();
    }
    // found pendant but emergency stop is not armed
    else if (!ButtonState.Fields.EStopArmed)
    {
      EmergencyStop();
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
}
