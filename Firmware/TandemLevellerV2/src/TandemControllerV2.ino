#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Global.h"
#include "AgGrade.h"
#include "GNSS.h"
#include "Blades.h"
#include "CANopen.h"
#include "IMU.h"

// front height:
//  dir = low, PWM output on M1A
//  dir = high, PWM output on M1B
// rear height:
//  dir = low, PWM output on M2A
//  dir = high, PWM output on M2B

// GPIO pins
#define LED 13

// how often to toggle the LED
#define LED_FLASH_PERIOD_MS 1000

// max time to find the pendant before we assume emergency stop
#define MAX_PENDANT_SEARCH_TIME 4000

// perform blade control periodically
#define BLADE_CONTROL_PERIOD_MS 50

// how often to send ping to AgGrade
#define PING_PERIOD_MS 1000

// time to wait before deciding that AgGrade has disconnected
#define PING_TIMEOUT_PERIOD_MS 3000

// how often to transmit TPDOs
#define TPDO_OUTPUT_PERIOD_MS 50

typedef enum _state_t
{
  STATE_RUN,
  STATE_ESTOP
} state_t;

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

// MAC address for this device
static byte MACAddress[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

// our IP address
static IPAddress OurIPAddress(192, 168, 1, 100);
// local port to listen on
static unsigned int LocalPort = 8888;

// remote IP address
static IPAddress RemoteIPAddress(192, 168, 1, 10);
// remote port
static unsigned int RemotePort = 6000;

// access to AgGrade
static AgGrade agGrade;

// access to GNSS data streams and processing
static GNSS NavData;

// blade control
static Blades BladeControl;

// CAN bus
static CANopen CANopn;

// IMU handling
static IMU IMUHandler;

// State variables
static elapsedMillis LEDFlashTimestamp;
static elapsedMillis PendantSearchTimestamp;
static button_state_t ButtonState;
static joystick_state_t JoystickState;
static bool PendantSearch;
static elapsedMillis BladeControlTimestamp;
static state_t State;
static elapsedMillis PingTimestamp;
static elapsedMillis LastPingRxTimestamp;
static bool AgGradeFound = false;
static elapsedMillis TPDOTimestamp;

// resets the controller
static void Reset
  (
  void
  )
{
  SCB_AIRCR = 0x05FA0004;
  while(1);
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

    // tell AgGrade
    pgnpacket_t Status;
    Status.PGN = PGN_ESTOP;
    agGrade.SendStatus(&Status);

    CANopn.EmergencyStop(LineNumber);

    BladeControl.EmergencyStop();

    CANopn.TxTPDO1(&(BladeControl.BladeStatus[FRONT_BLADE_IDX]), &(BladeControl.BladeStatus[REAR_BLADE_IDX]));
    CANopn.TxTPDO2(&(BladeControl.BladeStatus[FRONT_BLADE_IDX]), &(BladeControl.BladeStatus[REAR_BLADE_IDX]));

    TxFrontBladeAuto();
    TxRearBladeAuto();
  }
}

// called when a blade changes height or direction
static void Blades_BladeChanged
  (
  int BladeIndex,                // index of blade that changed xx_BLADE_IDX
  int PWM,                       // current PWM output to blade
  blade_direction_t Direction    // direction of blade movement
  )
{
    // update AgGrade
    pgnpacket_t Status;
    if (BladeIndex == FRONT_BLADE_IDX)
      Status.PGN = PGN_FRONT_BLADE_PWMVALUE;
    else
      Status.PGN = PGN_REAR_BLADE_PWMVALUE;
    SetPGNPacketUInt32(&Status, PWM);
    agGrade.SendStatus(&Status);

    if (BladeIndex == FRONT_BLADE_IDX)
      Status.PGN = PGN_FRONT_BLADE_DIRECTION;
    else
      Status.PGN = PGN_REAR_BLADE_DIRECTION;
    Status.Data[0] = Direction;
    agGrade.SendStatus(&Status);
}

// Stores a 16-bit value into a pgn packet
static void SetPGNPacketUInt16
  (
  pgnpacket_t *Packet,
  uint16_t Value
  )
{
  Packet->Data[0] = Value & 0xFF;
  Packet->Data[1] = (Value >> 8) & 0xFF;
}

// Stores a 32-bit value into a pgn packet
static void SetPGNPacketUInt32
  (
  pgnpacket_t *Packet,
  uint32_t Value
  )
{
  Packet->Data[0] = Value & 0xFF;
  Packet->Data[1] = (Value >> 8) & 0xFF;
  Packet->Data[2] = (Value >> 16) & 0xFF;
  Packet->Data[3] = (Value >> 24) & 0xFF;
}

// Stores a 32-bit value into a pgn packet at a specific byte offset
static void SetPGNPacketUInt32AtOffset
  (
  pgnpacket_t *Packet,
  uint8_t Offset,
  uint32_t Value
  )
{
  Packet->Data[Offset + 0] = Value & 0xFF;
  Packet->Data[Offset + 1] = (Value >> 8) & 0xFF;
  Packet->Data[Offset + 2] = (Value >> 16) & 0xFF;
  Packet->Data[Offset + 3] = (Value >> 24) & 0xFF;
}

// gets a 32-bit value from a pgn packet
static uint32_t GetPGNPacketUInt32
  (
  pgnpacket_t *Packet
  )
{
  return ((uint32_t)(Packet->Data[3]) << 24) | 
         ((uint32_t)(Packet->Data[2]) << 16) | 
         ((uint32_t)(Packet->Data[1]) << 8) | 
         Packet->Data[0];
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

// sends the front blade height to AgGrade
static void TxFrontBladeHeight
  (
  void
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_FRONT_BLADE_HEIGHT;
  SetPGNPacketUInt32(&Status, BladeControl.BladeHeight[FRONT_BLADE_IDX]);
  agGrade.SendStatus(&Status);
}

// sends the rear blade height to AgGrade
static void TxRearBladeHeight
  (
  void
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_REAR_BLADE_HEIGHT;
  SetPGNPacketUInt32(&Status, BladeControl.BladeHeight[REAR_BLADE_IDX]);
  agGrade.SendStatus(&Status);
}

// called when an IMU has changed
static void IMU_IMUChanged
  (
  uint8_t Index,             // index of IMU xxx_IDX
  imu_t *pIMUValue           // new IMU values
  )
{
  pgnpacket_t Status;

  switch (Index)
  {
    case TRACTOR_IDX:     Status.PGN = PGN_TRACTOR_IMU; break;
    case FRONT_BLADE_IDX: Status.PGN = PGN_FRONT_IMU;   break;
    case REAR_BLADE_IDX:  Status.PGN = PGN_REAR_IMU;    break;
  }

  SetPGNPacketUInt32AtOffset(&Status, 0,  (uint32_t)(pIMUValue->Pitch   * 100));
  SetPGNPacketUInt32AtOffset(&Status, 4,  (uint32_t)(pIMUValue->Roll    * 100));
  SetPGNPacketUInt32AtOffset(&Status, 8,  (uint32_t)(pIMUValue->Heading * 100));
  SetPGNPacketUInt32AtOffset(&Status, 12, (uint32_t)(pIMUValue->YawRate * 100));
  Status.Data[16] = pIMUValue->CalibrationStatus;
  agGrade.SendStatus(&Status);
}

// send front blade slave offset to AgGrade
static void TxFrontBladeSlaveOffset
  (
  void
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_FRONT_BLADE_OFFSET_SLAVE;
  SetPGNPacketUInt16(&Status, BladeControl.BladeStatus[FRONT_BLADE_IDX].SlaveOffset);
  agGrade.SendStatus(&Status);
}

// send rear blade slave offset to AgGrade
static void TxRearBladeSlaveOffset
  (
  void
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_REAR_BLADE_OFFSET_SLAVE;
  SetPGNPacketUInt16(&Status, BladeControl.BladeStatus[REAR_BLADE_IDX].SlaveOffset);
  agGrade.SendStatus(&Status);
}

// send front blade auto state to AgGrade
static void TxFrontBladeAuto
  (
  void
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_FRONT_CUTTING;
  Status.Data[0] = BladeControl.BladeStatus[FRONT_BLADE_IDX].BladeAuto;
  agGrade.SendStatus(&Status);
}

// send rear blade auto state to AgGrade
static void TxRearBladeAuto
  (
  void
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_REAR_CUTTING;
  Status.Data[0] = BladeControl.BladeStatus[REAR_BLADE_IDX].BladeAuto;
  agGrade.SendStatus(&Status);
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
        if (BladeControl.BladeStatus[FRONT_BLADE_IDX].BladeAuto)
        {
          BladeControl.BladeStatus[FRONT_BLADE_IDX].BladeAuto = false;
        }
        else
        {
          BladeControl.BladeStatus[FRONT_BLADE_IDX].BladeAuto = true;
        }
        TxFrontBladeAuto();
      }

      // toggle auto mode for rear blade
      if (ButtonState.Fields.Button2Pressed && !LastButton2State)
      {
        if (BladeControl.BladeStatus[REAR_BLADE_IDX].BladeAuto)
        {
          BladeControl.BladeStatus[REAR_BLADE_IDX].BladeAuto = false;
        }
        else
        {
          BladeControl.BladeStatus[REAR_BLADE_IDX].BladeAuto = true;
        }
        TxRearBladeAuto();
      }

      // if joystick 1 is moved up or down in auto mode then exit auto mode
      if ((JoystickState.Fields.Joystick1Up || JoystickState.Fields.Joystick1Down) && BladeControl.BladeStatus[FRONT_BLADE_IDX].BladeAuto)
      {
        BladeControl.BladeStatus[FRONT_BLADE_IDX].BladeAuto = false;
        TxFrontBladeAuto();
      }

      // if joystick 2 is moved up or down in auto mode then exit auto mode
      if ((JoystickState.Fields.Joystick2Up || JoystickState.Fields.Joystick2Down) && BladeControl.BladeStatus[REAR_BLADE_IDX].BladeAuto)
      {
        BladeControl.BladeStatus[REAR_BLADE_IDX].BladeAuto = false;
        TxRearBladeAuto();
      }

      if (!BladeControl.BladeStatus[FRONT_BLADE_IDX].BladeAuto)
      {
        // jog front blade (joystick button not pressed)
        if (JoystickState.Fields.Joystick1Up && !ButtonState.Fields.Joystick1Pressed)
        {
          BladeControl.JogBlade(FRONT_BLADE_IDX, BLADE_DIR_UP);
        }
        else if (JoystickState.Fields.Joystick1Down && !ButtonState.Fields.Joystick1Pressed)
        {
          BladeControl.JogBlade(FRONT_BLADE_IDX, BLADE_DIR_DOWN);
        }
        // adjust slave offset (joystick button is pressed)
        else if (JoystickState.Fields.Joystick1Up && ButtonState.Fields.Joystick1Pressed)
        {
          if (BladeControl.JogOffset(FRONT_BLADE_IDX, BLADE_DIR_UP))
          {
            TxFrontBladeSlaveOffset();
          }
        }
        else if (JoystickState.Fields.Joystick1Down && ButtonState.Fields.Joystick1Pressed)
        {
          if (BladeControl.JogOffset(FRONT_BLADE_IDX, BLADE_DIR_DOWN))
          {
            TxFrontBladeSlaveOffset();
          }
        }
      }

      if (!BladeControl.BladeStatus[REAR_BLADE_IDX].BladeAuto)
      {
        // jog rear blade (joystick button not pressed)
        if (JoystickState.Fields.Joystick2Up && !ButtonState.Fields.Joystick2Pressed)
        {
          BladeControl.JogBlade(REAR_BLADE_IDX, BLADE_DIR_UP);
        }
        else if (JoystickState.Fields.Joystick2Down && !ButtonState.Fields.Joystick2Pressed)
        {
          BladeControl.JogBlade(REAR_BLADE_IDX, BLADE_DIR_DOWN);
        }
        // adjust slave offset (joystick button is pressed)
        else if (JoystickState.Fields.Joystick2Up && ButtonState.Fields.Joystick2Pressed)
        {
          if (BladeControl.JogOffset(REAR_BLADE_IDX, BLADE_DIR_UP))
          {
            TxRearBladeSlaveOffset();
          }
        }
        else if (JoystickState.Fields.Joystick2Down && ButtonState.Fields.Joystick2Pressed)
        {
          if (BladeControl.JogOffset(REAR_BLADE_IDX, BLADE_DIR_DOWN))
          {
            TxRearBladeSlaveOffset();
          }
        }
      }
    }
    // if in emergency stop
    else if (State == STATE_ESTOP)
    {
      // if all four buttons pressed then reboot to exit ESTOP
      if (ButtonState.Fields.Button1Pressed && ButtonState.Fields.Button2Pressed &&
          ButtonState.Fields.Button3Pressed && ButtonState.Fields.Button4Pressed)
      {
        // notify AgGrade
        pgnpacket_t Status;
        Status.PGN = PGN_CLEAR_ESTOP;
        agGrade.SendStatus(&Status);

        elapsedMillis Delay = 0;
        while (Delay < 2000);
        Reset();
      }
    }
  }
}

// sends an NMEA sentence over UDP with packet framing
static void GNSS_SendNMEASentence
  (
  pgn_t PGN,
  char *sentence,
  uint8_t length
  )
{
  pgnpacket_t NMEAPacket;
  
  NMEAPacket.PGN = PGN;
  
  // Copy as much of the sentence as will fit
  uint8_t copyLen = (length > MAX_PGN_LEN) ? MAX_PGN_LEN : length;
  memcpy(NMEAPacket.Data, sentence, copyLen);
  
  agGrade.SendStatus(&NMEAPacket);
}

// initialization
void setup
  (
  )
{
  // secondary tablet
  Serial5.begin(115200);

  // Open serial communications for debug
  Serial.begin(115200);
  Serial.println("AgGrade Controller");
  //while (!Serial)
  //{
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}

  Serial.println("Starting UDP...");

  agGrade.Connect(MACAddress, OurIPAddress, LocalPort, RemoteIPAddress, RemotePort);

  NavData.Connect();
  NavData.SetCallback(GNSS_SendNMEASentence);

  BladeControl.SetCallback(Blades_BladeChanged);

  TPDOTimestamp = 0;

  CANopn.Init();
  CANopn.SetCallbacks(CANopen_ProcessPDO, CANopen_NodeLost, CANopen_RequestReset);

  // set up LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  // start looking for pendant, we can't operate without it
  PendantSearchTimestamp = 0;
  PendantSearch = true;

  // reset buttons and joysticks
  ButtonState.RawValue = 0;
  JoystickState.RawValue = 0;

  IMUHandler.Init();
  IMUHandler.SetCallbacks(IMU_IMUChanged);

  BladeControlTimestamp = 0;
  PingTimestamp = 0;
  LastPingRxTimestamp = 0;

  State = STATE_RUN;

  CANopn.TxBootup();
  CANopn.TxTPDO1(&(BladeControl.BladeStatus[FRONT_BLADE_IDX]), &(BladeControl.BladeStatus[REAR_BLADE_IDX]));
  CANopn.TxTPDO2(&(BladeControl.BladeStatus[FRONT_BLADE_IDX]), &(BladeControl.BladeStatus[REAR_BLADE_IDX]));

  TxFrontBladeSlaveOffset();
  TxRearBladeSlaveOffset();

  TxFrontBladeAuto();
  TxRearBladeAuto();

  TxFrontBladeHeight();
  TxRearBladeHeight();

  CANopn.ResetAllNodes();

  /*// fixme - remove
  // test connection to secondary tablet
  Serial.println("Sec tablet test...");
  int RxByte;
  Serial5.write('A');
  while ((RxByte = Serial5.read()) == -1);
  if (RxByte == 'A')
  {
    Serial.println("Secondary tablet echo received");
  }
  else
  {
    Serial.print("Secondary tablet echo failed with: ");
    Serial.println(RxByte);
  }*/

  Serial.println("Ready!");
}

// called when canopen module requests a reset
static void CANopen_RequestReset
  (
  void
  )
{
  Reset();
}

// called when a node dissappears from the CAN bus
static void CANopen_NodeLost
  (
  uint8_t NodeId             // id of node that was lost
  )
{
  // if the pendant has dissappeared then perform emergency stop
  // as we can't see the ESTOP button
  if ((NodeId == PENDANT_NODE_ID) && (PendantSearchTimestamp >= MAX_PENDANT_SEARCH_TIME))
  {
    EmergencyStop(__LINE__);
  }
}

// processes a received PDO
static void CANopen_ProcessPDO
  (
  uint8_t NodeId,            // id of node that transmitted the pdo
  uint16_t PDONumber,        // TPDO number
  size_t DataLength,         // length of data in PDO
  const uint8_t *pData       // PDO data
  )
{
  switch (NodeId)
  {
    case PENDANT_NODE_ID:
      ProcessPendantTPDO(DataLength, pData);
      break;

    case TRACTOR_IMU_NODE_ID:
    case FRONTSCRAPER_IMU_NODE_ID:
    case REARSCRAPER_IMU_NODE_ID:
      if (PDONumber == 1)
        IMUHandler.ProcessIMUTPDO1(NodeId, DataLength, pData);
      else
        IMUHandler.ProcessIMUTPDO2(NodeId, DataLength, pData);
      break;

    case FRONT_ANGLE_NODE_ID:
    case REAR_ANGLE_NODE_ID:
      ProcessAngleTPDO(NodeId, DataLength, pData);
      break;
  }
}

// continually executes
void loop
  (
  )
{  
    // periodically transmit data onto the CAN bus
  if (TPDOTimestamp >= TPDO_OUTPUT_PERIOD_MS)
  {
    TPDOTimestamp = 0;

    CANopn.TxTPDO1(&(BladeControl.BladeStatus[FRONT_BLADE_IDX]), &(BladeControl.BladeStatus[REAR_BLADE_IDX]));
    CANopn.TxTPDO2(&(BladeControl.BladeStatus[FRONT_BLADE_IDX]), &(BladeControl.BladeStatus[REAR_BLADE_IDX]));
  }

  CANopn.Process();

  // Check for incoming commands
  if (agGrade.IsCommandAvailable())
  {
    // We received a complete command
    pgnpacket_t Command = agGrade.GetCommand();
    
    AgGradeFound = true;
    
    // Process the command based on PGN
    switch (Command.PGN)
    {
      // misc
      case PGN_RESET:
        Reset();
        break;

      case PGN_AGGRADE_STARTED:
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
        BladeControl.BladeHeight[FRONT_BLADE_IDX] = 0;
        break;
      case PGN_REAR_ZERO_BLADE_HEIGHT:
        BladeControl.BladeHeight[REAR_BLADE_IDX] = 0;
        break;

        // front blade configuration
      case PGN_FRONT_PWM_GAIN_UP:
        BladeControl.BladeConfig[FRONT_BLADE_IDX].PWMGainUp = GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_PWM_GAIN_DOWN:
        BladeControl.BladeConfig[FRONT_BLADE_IDX].PWMGainDown = GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_PWM_MIN_UP:
        BladeControl.BladeConfig[FRONT_BLADE_IDX].PWMMinUp = GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_PWM_MIN_DOWN:
        BladeControl.BladeConfig[FRONT_BLADE_IDX].PWMMinDown = GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_PWM_MAX_UP:
        BladeControl.BladeConfig[FRONT_BLADE_IDX].PWMMaxUp = GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_PWM_MAX_DOWN:
        BladeControl.BladeConfig[FRONT_BLADE_IDX].PWMMaxDown = GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_INTEGRAL_MULTPLIER:
        BladeControl.BladeConfig[FRONT_BLADE_IDX].IntegralMultiplier = GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_DEADBAND:
        BladeControl.BladeConfig[FRONT_BLADE_IDX].Deadband = GetPGNPacketUInt32(&Command);
        break;

      // rear blade configuration
      case PGN_REAR_PWM_GAIN_UP:
        BladeControl.BladeConfig[REAR_BLADE_IDX].PWMGainUp = GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_PWM_GAIN_DOWN:
        BladeControl.BladeConfig[REAR_BLADE_IDX].PWMGainDown = GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_PWM_MIN_UP:
        BladeControl.BladeConfig[REAR_BLADE_IDX].PWMMinUp = GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_PWM_MIN_DOWN:
        BladeControl.BladeConfig[REAR_BLADE_IDX].PWMMinDown = GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_PWM_MAX_UP:
        BladeControl.BladeConfig[REAR_BLADE_IDX].PWMMaxUp = GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_PWM_MAX_DOWN:
        BladeControl.BladeConfig[REAR_BLADE_IDX].PWMMaxDown = GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_INTEGRAL_MULTPLIER:
        BladeControl.BladeConfig[REAR_BLADE_IDX].IntegralMultiplier = GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_DEADBAND:
        BladeControl.BladeConfig[REAR_BLADE_IDX].Deadband = GetPGNPacketUInt32(&Command);
        break;
        
      // front blade commands
      case PGN_FRONT_CUT_VALVE:
        if (BladeControl.BladeStatus[FRONT_BLADE_IDX].BladeAuto)
        {
          // store for use on next calculation pass
          BladeControl.BladeCommand[FRONT_BLADE_IDX].CutValve = GetPGNPacketUInt32(&Command);
        }
        break;

      // rear blade commands
      case PGN_REAR_CUT_VALVE:
        if (BladeControl.BladeStatus[REAR_BLADE_IDX].BladeAuto)
        {
          // store for use on next calculation pass
          BladeControl.BladeCommand[REAR_BLADE_IDX].CutValve = GetPGNPacketUInt32(&Command);
        }
        break;
        
      default:
        Serial.print("Unknown PGN: 0x");
        Serial.println(Command.PGN, HEX);
        break;
    }
  }

  // perform blade control
  if (BladeControlTimestamp >= BLADE_CONTROL_PERIOD_MS)
  {
    BladeControlTimestamp = 0;

    // only control the blade if we are in the run state
    if (State == STATE_RUN)
    {
      for (int b = 0; b < NUM_BLADES; b++)
      {
        BladeControl.ControlBlade(b);
      }
    }
  }

  // check to see if AgGrade has disappeared
  if ((LastPingRxTimestamp >= PING_TIMEOUT_PERIOD_MS) && AgGradeFound)
  {
    AgGradeFound = false;

    EmergencyStop(__LINE__);
  }

  // check for pendant
  if (PendantSearch && (PendantSearchTimestamp >= MAX_PENDANT_SEARCH_TIME))
  {
    // stop search
    PendantSearch = false;

    // not found
    if (!CANopn.IsNodeFound(PENDANT_NODE_ID))
    {
      EmergencyStop(__LINE__);
    }
    // found pendant but emergency stop is not armed
    else if (!ButtonState.Fields.EStopArmed)
    {
      EmergencyStop(__LINE__);
    }
  }

  // flash LED
  if (LEDFlashTimestamp >= LED_FLASH_PERIOD_MS)
  {
    LEDFlashTimestamp -= LED_FLASH_PERIOD_MS;
    
    digitalToggle(LED);
  }

  // tell AgGrade we are alive
  if (PingTimestamp >= PING_PERIOD_MS)
  {
    PingTimestamp = 0;

    pgnpacket_t Status;
    Status.PGN   = PGN_PING;
    agGrade.SendStatus(&Status);
  }

  NavData.Process();
}
