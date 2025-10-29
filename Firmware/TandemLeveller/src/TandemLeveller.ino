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

// how often to toggle the LED
#define LED_FLASH_PERIOD_MS 1000

// how often to send status to OpenGrade3D
#define STATUS_OUTPUT_PERIOD_MS 1000

#define NMT_RESET_CMD 0x81
#define NMT_RESET_ALL 0x00

// NMT states
#define NMT_STATE_BOOTUP      0x00
#define NMT_STATE_OPERATIONAL 0x05

// number of nodes to monitor
#define NUM_NODES 5

// maximum time to wait for a heartbeat before declaring a node as missing
#define MAX_HEARTBEAT_TIME 300

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

// supported PGNs
typedef enum _pgn_t : uint16_t
{
  // misc
  PGN_ESTOP                    = 0x0000,

  // blade control
  PGN_CUT_VALVE                = 0x1000,   // 0 - 200
  PGN_BLADE_OFFSET             = 0x1001,

  // blade configuration
  PGN_PWM_GAIN_UP              = 0x2002,
  PGN_PWM_GAIN_DOWN            = 0x2003,
  PGN_PWM_MIN_UP               = 0x2004,
  PGN_PWM_MIN_DOWN             = 0x2005,
  PGN_PWM_MAX_UP               = 0x2006,
  PGN_PWM_MAX_DOWN             = 0x2007,
  PGN_INTEGRAL_MULTPLIER       = 0x2008,
  PGN_DEADBAND                 = 0x2009,

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
  PGN_BLADE_OFFSET_SLAVE       = 0x5000,
  PGN_FRONT_BLADE_PWMVALUE     = 0x5001,
  PGN_FRONT_BLADE_DIRECTION    = 0x5002,
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

// current status of the blade
typedef struct _blade_status_t
{
  int Offset;
} blade_status_t;

// movement command for blade
typedef struct _blade_command_t
{
  int Offset;
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
    unsigned int Joystick1Left  : 1;
    unsigned int Joystick1Right : 1;
    unsigned int Joystick1Up    : 1;
    unsigned int Joystick1Down  : 1;
    unsigned int Joystick2Left  : 1;
    unsigned int Joystick2Right : 1;
    unsigned int Joystick2Up    : 1;
    unsigned int Joystick2Down  : 1;
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
static blade_config_t BladeConfig;
static blade_status_t BladeStatus;
static blade_command_t BladeCommand;
static bool AutoBlade;
static elapsedMillis BladeControlTimestamp;
static int pwm1ago = 0, pwm2ago = 0, pwm3ago = 0, pwm4ago = 0, pwm5ago = 0;

// processes the current blade command
static void ProcessBladeCommand
  (
  blade_command_t *pCommand     // command to process
  )
{
  // update status
  BladeStatus.Offset = pCommand->Offset;
  controllerstatus_t Status;
  Status.PGN = PGN_BLADE_OFFSET_SLAVE;
  Status.Value = BladeStatus.Offset;
  SendStatus(&Status);
}

// perform an emergency stop of blade control
static void EmergencyStop
  (
  void
  )
{
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
    ButtonState.RawValue = (pData[0] | ((uint16_t)pData[1] << 8));
    JoystickState.RawValue = pData[2];

    // ESTOP PRESSED
    if (!ButtonState.Fields.EStopArmed)
    {
      EmergencyStop();
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
      Serial.println("hb reset");
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
  byte PWMDrive;

  if (AutoBlade)
  {
    // lower the blade
    if (BladeCommand.CutValve >= (100 + BladeConfig.Deadband))
    {
      // PWM value is negative
      PWMValue = -((BladeCommand.CutValve - 100 - BladeConfig.Deadband) * BladeConfig.PWMGainDown + BladeConfig.PWMMinDown);
    }
    // lift the blade
    else if (BladeCommand.CutValve <= (100 - BladeConfig.Deadband))
    {
      // PWM value is positive
      PWMValue = -((BladeCommand.CutValve - 100 + BladeConfig.Deadband) * BladeConfig.PWMGainUp - BladeConfig.PWMMinUp);
    }
    else
    {
      PWMValue = 0;
    }

    // calculate a derivative
    if (BladeCommand.CutValve != 100 && PWMValue != 0)
    {
      PWMHist = ((((pwm1ago) + pwm2ago + (pwm3ago) + (pwm4ago) + (pwm5ago / 2.000)) * (sq(BladeConfig.IntegralMultiplier) / 100.0000)) / sq(BladeCommand.CutValve - 100.0000));

      //put pwmHist to 0 when the blade cross the line.
      if (BladeCommand.CutValve > 100 && (pwm1ago + pwm2ago + pwm3ago + pwm4ago + pwm5ago) > 0) PWMHist = 0;
      if (BladeCommand.CutValve < 100 && (pwm1ago + pwm2ago + pwm3ago + pwm4ago + pwm5ago) < 0) PWMHist = 0;

      PWMValue = (PWMValue - PWMHist);
    }

    // shuffle samples down
    pwm5ago = pwm4ago;
    pwm4ago = pwm3ago;
    pwm3ago = pwm2ago;
    pwm2ago = pwm1ago;
    pwm1ago = PWMValue;
  
    // enforce limits
    if (BladeCommand.CutValve > 100 && PWMValue > 0) PWMValue = 0;
    if (BladeCommand.CutValve > 100 && PWMValue < -(BladeConfig.PWMMaxDown)) PWMValue = -(BladeConfig.PWMMaxDown);
    if (BladeCommand.CutValve < 100 && PWMValue < 0) PWMValue = 0;
    if (BladeCommand.CutValve < 100 && PWMValue > BladeConfig.PWMMaxUp) PWMValue = BladeConfig.PWMMaxUp;
    if (PWMValue > 0 && PWMValue < BladeConfig.PWMMinUp) PWMValue = 0;
    if (PWMValue < 0 && PWMValue > -(BladeConfig.PWMMinDown)) PWMValue = 0;

    if (PWMValue < 0)
    {
      digitalWrite(FRONT_HEIGHT_DIR, HIGH);
    }
    else
    {
      digitalWrite(FRONT_HEIGHT_DIR, LOW);
    }

    // set to 0 - 255
    // scale to 0 - 32767
    PWMDrive = abs(PWMValue);
    analogWrite(FRONT_HEIGHT_PWM, (int)(32767.0 / 255.0 * PWMDrive));

    controllerstatus_t Status;
    Status.PGN = PGN_FRONT_BLADE_PWMVALUE;
    Status.Value = PWMDrive;
    SendStatus(&Status);

    Status.PGN = PGN_FRONT_BLADE_DIRECTION;
    Status.Value = digitalRead(FRONT_HEIGHT_DIR);
    SendStatus(&Status);
  }
}

// initialize the hardware
void setup()
{
  //debug.begin(Serial);
  //halt_cpu(); 

  // configure connection to opengrade3d
  //Serial5.begin(OPENGRADE3D_BAUDRATE);
  //OpenGrade3D.begin(Serial5);
  // fixme - swap back
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
  analogWriteResolution(15); // 0 - 32767

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

  // initial blade status
  // fixme - to do - read from angle sensor
  BladeStatus.Offset = 100;

  // fixme - change to false
  AutoBlade = true;

  BladeControlTimestamp = 0;

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
      case PGN_PWM_GAIN_UP:
        BladeConfig.PWMGainUp = Command.Value;
        break;
      case PGN_PWM_GAIN_DOWN:
        BladeConfig.PWMGainDown = Command.Value;
        break;
      case PGN_PWM_MIN_UP:
        BladeConfig.PWMMinUp = Command.Value;
        break;
      case PGN_PWM_MIN_DOWN:
        BladeConfig.PWMMinDown = Command.Value;
        break;
      case PGN_PWM_MAX_UP:
        BladeConfig.PWMMaxUp = Command.Value;
        break;
      case PGN_PWM_MAX_DOWN:
        BladeConfig.PWMMaxDown = Command.Value;
        break;
      case PGN_INTEGRAL_MULTPLIER:
        BladeConfig.IntegralMultiplier = Command.Value;
        break;
      case PGN_DEADBAND:
        BladeConfig.Deadband = Command.Value;
        break;

      // blade commands
      case PGN_CUT_VALVE:
        BladeCommand.CutValve = Command.Value;
        ProcessBladeCommand(&BladeCommand);
        break;
      case PGN_BLADE_OFFSET:
        BladeCommand.Offset = Command.Value;
        ProcessBladeCommand(&BladeCommand);
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
    Status.PGN = PGN_BLADE_OFFSET_SLAVE;
    Status.Value = BladeStatus.Offset;
    SendStatus(&Status);
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

  // flash LED
  if (LEDFlashTimestamp >= LED_FLASH_PERIOD_MS)
  {
    LEDFlashTimestamp -= LED_FLASH_PERIOD_MS;
    
    digitalToggle(LED);
  }
}
