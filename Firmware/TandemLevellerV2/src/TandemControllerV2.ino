#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "FlexCAN_T4-master/FlexCAN_T4.h"
#include "Global.h"
#include "AgGrade.h"
#include "GNSS.h"

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
#define FRONT_ANGLE_NODE_ID      0x10
#define REAR_ANGLE_NODE_ID       0x11

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

// CAN bus speed
#define CAN_BITRATE_BPS 125000

// the height of the blade that represents ground level
// when communicating with AgGrade
#define BLADE_HEIGHT_GROUND_LEVEL 200

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

// how often to send ping to AgGrade
#define PING_PERIOD_MS 1000

// time to wait before deciding that AgGrade has disconnected
#define PING_TIMEOUT_PERIOD_MS 3000

// NMEA 0183 special characters
#define LF 0x0A
#define CR 0x0D

// GGA: talker + max 14 comma-separated data fields (split buffer)
#define MAX_GGA_FIELDS 16

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
  uint8_t CalibrationStatus;
} imu_t;

// data structure for reading GNSS streams
typedef struct _gnss_reader_t
{
  bool Synced;
  uint8_t NextWritePos;
  char Buffer[MAX_NMEA_LENGTH];
} gnss_reader_t;

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

// CAN bus instance
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANBus;

// State variables
static elapsedMillis LEDFlashTimestamp;
static elapsedMillis HBTime[NUM_NODES];
static bool NodeFound[NUM_NODES];
static elapsedMillis PendantSearchTimestamp;
static button_state_t ButtonState;
static joystick_state_t JoystickState;
static bool PendantSearch;
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
static bool AgGradeFound = false;

// GNSS stream readers
static gnss_reader_t TractorGNSS      = { 0 };
static gnss_reader_t FrontScraperGNSS = { 0 };
static gnss_reader_t RearScraperGNSS  = { 0 };

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

    // tell AgGrade
    pgnpacket_t Status;
    Status.PGN = PGN_ESTOP;
    agGrade.SendStatus(&Status);

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

    // switch to manual control, stop movement
    for (int b = 0; b < NUM_BLADES; b++)
    {
      BladeStatus[b].BladeAuto = false;
      BladeCommand[b].CutValve = BLADE_HEIGHT_GROUND_LEVEL;
    }
    SetFrontValvePWM(0);
    SetRearValvePWM(0);

    TxTPDO1();
    TxTPDO2();

    TxFrontBladeAuto();
    TxRearBladeAuto();
  }
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

// process TPDO1 from IMU
static void ProcessIMUTPDO1
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

// process TPDO2 from IMU
static void ProcessIMUTPDO2
  (
  uint8_t NodeId,
  uint8_t Length,
  const uint8_t *pData
  )
{
  if (Length == 1)
  {
    switch (NodeId)
    {
      case TRACTOR_IMU_NODE_ID:
        IMUValues[TRACTOR_IDX].CalibrationStatus = pData[0];
        break;
      case FRONTSCRAPER_IMU_NODE_ID:
        IMUValues[FRONT_BLADE_IDX].CalibrationStatus = pData[0];
        break;
      case REARSCRAPER_IMU_NODE_ID:
        IMUValues[REAR_BLADE_IDX].CalibrationStatus = pData[0];
        break;
    }
  }
}

// sends the front blade height to AgGrade
static void TxFrontBladeHeight
  (
  void
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_FRONT_BLADE_HEIGHT;
  SetPGNPacketUInt32(&Status, BladeHeight[FRONT_BLADE_IDX]);
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
  SetPGNPacketUInt32(&Status, BladeHeight[REAR_BLADE_IDX]);
  agGrade.SendStatus(&Status);
}

// send tractor IMU values to AgGrade
static void TxTractorIMU
  (
  void
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_TRACTOR_IMU;
  SetPGNPacketUInt32AtOffset(&Status, 0, (uint32_t)(IMUValues[TRACTOR_IDX].Pitch * 100));
  SetPGNPacketUInt32AtOffset(&Status, 4, (uint32_t)(IMUValues[TRACTOR_IDX].Roll * 100));
  SetPGNPacketUInt32AtOffset(&Status, 8, (uint32_t)(IMUValues[TRACTOR_IDX].Heading * 100));
  SetPGNPacketUInt32AtOffset(&Status, 12, (uint32_t)(IMUValues[TRACTOR_IDX].YawRate * 100));
  Status.Data[16] = IMUValues[TRACTOR_IDX].CalibrationStatus;
  agGrade.SendStatus(&Status);
}

// send front scraper IMU values to AgGrade
static void TxFrontScraperIMU
  (
  void
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_FRONT_IMU;
  SetPGNPacketUInt32AtOffset(&Status, 0, (uint32_t)(IMUValues[FRONT_BLADE_IDX].Pitch * 100));
  SetPGNPacketUInt32AtOffset(&Status, 4, (uint32_t)(IMUValues[FRONT_BLADE_IDX].Roll * 100));
  SetPGNPacketUInt32AtOffset(&Status, 8, (uint32_t)(IMUValues[FRONT_BLADE_IDX].Heading * 100));
  SetPGNPacketUInt32AtOffset(&Status, 12, (uint32_t)(IMUValues[FRONT_BLADE_IDX].YawRate * 100));
  Status.Data[16] = IMUValues[FRONT_BLADE_IDX].CalibrationStatus;
  agGrade.SendStatus(&Status);
}

// send rear scraper IMU values to AgGrade
static void TxRearScraperIMU
  (
  void
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_REAR_IMU;
  SetPGNPacketUInt32AtOffset(&Status, 0, (uint32_t)(IMUValues[REAR_BLADE_IDX].Pitch * 100));
  SetPGNPacketUInt32AtOffset(&Status, 4, (uint32_t)(IMUValues[REAR_BLADE_IDX].Roll * 100));
  SetPGNPacketUInt32AtOffset(&Status, 8, (uint32_t)(IMUValues[REAR_BLADE_IDX].Heading * 100));
  SetPGNPacketUInt32AtOffset(&Status, 12, (uint32_t)(IMUValues[REAR_BLADE_IDX].YawRate * 100));
  Status.Data[16] = IMUValues[REAR_BLADE_IDX].CalibrationStatus;
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
  SetPGNPacketUInt16(&Status, BladeStatus[FRONT_BLADE_IDX].SlaveOffset);
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
  SetPGNPacketUInt16(&Status, BladeStatus[REAR_BLADE_IDX].SlaveOffset);
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
  Status.Data[0] = BladeStatus[FRONT_BLADE_IDX].BladeAuto;
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
  Status.Data[0] = BladeStatus[REAR_BLADE_IDX].BladeAuto;
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
        NodeFound[n] = false;

        // if the pendant has dissappeared then perform emergency stop
        // as we can't see the ESTOP button
        if (((n + 1) == PENDANT_NODE_ID) && (PendantSearchTimestamp >= MAX_PENDANT_SEARCH_TIME))
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
      ProcessIMUTPDO1(TRACTOR_IMU_NODE_ID, msg.len, msg.buf);
      break;
    case 0x180 + FRONTSCRAPER_IMU_NODE_ID:
      ProcessIMUTPDO1(FRONTSCRAPER_IMU_NODE_ID, msg.len, msg.buf);
      break;
    case 0x180 + REARSCRAPER_IMU_NODE_ID:
      ProcessIMUTPDO1(REARSCRAPER_IMU_NODE_ID, msg.len, msg.buf);
      break;
    case 0x180 + FRONT_ANGLE_NODE_ID:
      ProcessAngleTPDO(FRONT_ANGLE_NODE_ID, msg.len, msg.buf);
      break;
    case 0x180 + REAR_ANGLE_NODE_ID:
      ProcessAngleTPDO(REAR_ANGLE_NODE_ID, msg.len, msg.buf);
      break;
    case 0x280 + TRACTOR_IMU_NODE_ID:
      ProcessIMUTPDO2(TRACTOR_IMU_NODE_ID, msg.len, msg.buf);
      break;
    case 0x280 + FRONTSCRAPER_IMU_NODE_ID:
      ProcessIMUTPDO2(FRONTSCRAPER_IMU_NODE_ID, msg.len, msg.buf);
      break;
    case 0x280 + REARSCRAPER_IMU_NODE_ID:
      ProcessIMUTPDO2(REARSCRAPER_IMU_NODE_ID, msg.len, msg.buf);
      break;

    // heartbeats
    case 0x700 + PENDANT_NODE_ID:
      ProcessHeartbeat(PENDANT_NODE_ID, msg.len, msg.buf);
      break;
  }

  // process NMT message
  if ((msg.id == 0x000) && !msg.flags.extended && (msg.len == 2))
  {
    // reset
    if (msg.buf[0] == NMT_RESET_CMD)
    {
      // this node
      if ((msg.buf[1] == CONTROLLER_NODE_ID) || (msg.buf[1] == NMT_RESET_ALL))
      {
        Reset();
      }
    }
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

    // update AgGrade
    pgnpacket_t Status;
    Status.PGN = PGN_FRONT_BLADE_PWMVALUE;
    SetPGNPacketUInt32(&Status, BladeStatus[FRONT_BLADE_IDX].BladePWM);
    agGrade.SendStatus(&Status);

    Status.PGN = PGN_FRONT_BLADE_DIRECTION;
    Status.Data[0] = digitalRead(FRONT_HEIGHT_DIR);
    agGrade.SendStatus(&Status);
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

    // update AgGrade
    pgnpacket_t Status;
    Status.PGN = PGN_REAR_BLADE_PWMVALUE;
    SetPGNPacketUInt32(&Status, BladeStatus[REAR_BLADE_IDX].BladePWM);
    agGrade.SendStatus(&Status);

    Status.PGN = PGN_REAR_BLADE_DIRECTION;
    Status.Data[0] = digitalRead(REAR_HEIGHT_DIR);
    agGrade.SendStatus(&Status);
  }
}

// XOR checksum for NMEA 0183: bytes from first after '$' up to (exclusive) '*'
static uint8_t NMEAChecksumBytes
  (
  const char *startAfterDollar,
  const char *endBeforeStar
  )
{
  uint8_t cs = 0;
  for (const char *p = startAfterDollar; p < endBeforeStar; p++)
  {
    cs ^= (uint8_t)(*p);
  }
  return cs;
}

static int NMEAHexNibble
  (
  char c
  )
{
  if (c >= '0' && c <= '9')
  {
    return (int)(c - '0');
  }
  if (c >= 'A' && c <= 'F')
  {
    return (int)(c - 'A' + 10);
  }
  if (c >= 'a' && c <= 'f')
  {
    return (int)(c - 'a' + 10);
  }
  return -1;
}

// Split payload (between '$' and '*') on commas; writes '\0' into payload; returns field count
static int NMEASplitFields
  (
  char *payload,
  char *fields[],
  int maxFields
  )
{
  int n = 0;
  fields[n++] = payload;
  for (char *p = payload; *p != '\0' && n < maxFields; p++)
  {
    if (*p == ',')
    {
      *p = '\0';
      if (n < maxFields)
      {
        fields[n++] = p + 1;
      }
    }
  }
  return n;
}

// Parse NMEA ddmm.mmmmm (lat) or dddmm.mmmmm (lon) field to signed decimal degrees using hemisphere
static double NMEAParseLatLonToDecimal
  (
  const char *dmField,
  const char *hemi          // "N"/"S" or "E"/"W"
  )
{
  if (dmField == NULL || *dmField == '\0')
  {
    return 0.0;
  }
  double dm = strtod(dmField, NULL);
  int ideg = (int)(dm / 100.0);
  double minutes = dm - (double)ideg * 100.0;
  double dec = (double)ideg + minutes / 60.0;
  if (hemi != NULL)
  {
    char h = hemi[0];
    if (h == 'S' || h == 's' || h == 'W' || h == 'w')
    {
      dec = -fabs(dec);
    }
    else
    {
      dec = fabs(dec);
    }
  }
  return dec;
}

// Append minutes as mm.mmmmmmm (2 integer digits + 7 fractional digits, no spaces) after degree digits in dmOut
static int NMEAAppendMinutes
  (
  char *dmOut,
  size_t dmLen,
  size_t offset,
  double minutes
  )
{
  if (minutes < 0.0)
  {
    minutes = 0.0;
  }
  if (minutes >= 60.0)
  {
    minutes = 59.9999999;
  }
  long scaled = lround(minutes * 10000000.0);
  int minInt = (int)(scaled / 10000000L);
  int minFrac = (int)(scaled % 10000000L);
  if (minInt > 59)
  {
    minInt = 59;
    minFrac = 9999999;
  }
  int n2 = snprintf(dmOut + offset, dmLen - offset, "%02d.%07d", minInt, minFrac);
  if (n2 < 0 || offset + (size_t)n2 >= dmLen)
  {
    return -1;
  }
  return (int)(offset + (size_t)n2);
}

// Format signed decimal latitude to ddmm.mmmmmmm (7 dp on minutes) and N/S
static int NMEAFormatLatitude
  (
  double latDec,
  char *dmOut,
  size_t dmLen,
  char latNS[2]
  )
{
  double a = fabs(latDec);
  int deg = (int)floor(a + 1e-9);
  if (deg > 90)
  {
    deg = 90;
  }
  double min = (a - (double)deg) * 60.0;
  latNS[0] = (latDec < 0.0) ? 'S' : 'N';
  latNS[1] = '\0';
  int n = snprintf(dmOut, dmLen, "%02d", deg);
  if (n < 0 || (size_t)n >= dmLen)
  {
    return -1;
  }
  return NMEAAppendMinutes(dmOut, dmLen, (size_t)n, min);
}

// Format signed decimal longitude to dddmm.mmmmmmm (7 dp on minutes) and E/W
static int NMEAFormatLongitude
  (
  double lonDec,
  char *dmOut,
  size_t dmLen,
  char lonEW[2]
  )
{
  double a = fabs(lonDec);
  int deg = (int)floor(a + 1e-9);
  if (deg > 180)
  {
    deg = 180;
  }
  double min = (a - (double)deg) * 60.0;
  lonEW[0] = (lonDec < 0.0) ? 'W' : 'E';
  lonEW[1] = '\0';
  int n = snprintf(dmOut, dmLen, "%03d", deg);
  if (n < 0 || (size_t)n >= dmLen)
  {
    return -1;
  }
  return NMEAAppendMinutes(dmOut, dmLen, (size_t)n, min);
}

static void ProcessNMEASentence
  (
  pgn_t PGN,
  const char *sentence,
  uint8_t length
  )
{
  bool isGga = (strncmp(sentence, "$GPGGA,", 7) == 0) ||
               (strncmp(sentence, "$GNGGA,", 7) == 0);

  if (!isGga)
  {
    SendNMEASentence(PGN, sentence, length);
    return;
  }

  const char *star = strchr(sentence, '*');
  if (star == NULL || star[1] == '\0' || star[2] == '\0')
  {
    return;
  }

  int hi = NMEAHexNibble(star[1]);
  int lo = NMEAHexNibble(star[2]);
  if (hi < 0 || lo < 0)
  {
    return;
  }
  uint8_t wantCs = (uint8_t)((hi << 4) | lo);
  uint8_t gotCs = NMEAChecksumBytes(sentence + 1, star);
  if (gotCs != wantCs)
  {
    return;
  }

  char work[MAX_NMEA_LENGTH];
  if (length >= sizeof(work))
  {
    return;
  }
  memcpy(work, sentence, length);
  work[length] = '\0';

  char *wstar = strchr(work, '*');
  if (wstar == NULL)
  {
    return;
  }
  *wstar = '\0';

  char *fields[MAX_GGA_FIELDS];
  int nfields = NMEASplitFields(work + 1, fields, MAX_GGA_FIELDS);
  if (nfields < 6)
  {
    return;
  }

  double lat = NMEAParseLatLonToDecimal(fields[2], fields[3]);
  double lon = NMEAParseLatLonToDecimal(fields[4], fields[5]);

  char latDm[16];
  char latNS[2];
  char lonDm[16];
  char lonEW[2];

  if (NMEAFormatLatitude(lat, latDm, sizeof(latDm), latNS) < 0 ||
      NMEAFormatLongitude(lon, lonDm, sizeof(lonDm), lonEW) < 0)
  {
    return;
  }

  char out[MAX_NMEA_LENGTH];
  int o = snprintf(out, sizeof(out), "$%s,%s,%s,%s,%s,%s",
                     fields[0], fields[1], latDm, latNS, lonDm, lonEW);
  if (o < 0 || o >= (int)sizeof(out))
  {
    return;
  }

  for (int f = 6; f < nfields; f++)
  {
    int add = snprintf(out + o, sizeof(out) - (size_t)o, ",%s", fields[f]);
    if (add < 0 || (size_t)add >= sizeof(out) - (size_t)o)
    {
      return;
    }
    o += add;
  }

  uint8_t newCs = NMEAChecksumBytes(out + 1, out + o);
  int fin = snprintf(out + o, sizeof(out) - (size_t)o, "*%02X\r\n", (unsigned)newCs);
  if (fin < 0 || (size_t)fin >= sizeof(out) - (size_t)o)
  {
    return;
  }
  o += fin;

  if ((uint8_t)o != length || memcmp(sentence, out, length) != 0)
  {
    Serial.println("NMEA GGA round-trip mismatch");
    Serial.print("orig: ");
    Serial.println(sentence);
    Serial.print("recon: ");
    Serial.println(out);
    while (true)
    {
      ;
    }
  }
  Serial.print(".");

  SendNMEASentence(PGN, out, (uint8_t)o);
}

// sends an NMEA sentence over UDP with packet framing
static void SendNMEASentence
  (
  pgn_t PGN,
  const char *sentence,
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

// processes a byte read from a GNSS stream
static void ProcessGNSSByte
  (
  int RxByte,                 // byte to process
  gnss_reader_t *pReader,     // reader to use
  pgn_t PGN                   // PGN to use when sending
  )
{
  // byte received
  if (RxByte != -1)
  {
    // waiting to sync, got start of sentence
    if (!pReader->Synced && (RxByte == '$'))
    {
      pReader->Synced = true;
      pReader->NextWritePos = 0;
      pReader->Buffer[pReader->NextWritePos++] = RxByte;
    }
    // synced, store character
    else if (pReader->Synced)
    {
      pReader->Buffer[pReader->NextWritePos++] = (char)RxByte;

      // if end of sentence
      if (RxByte == LF)
      {
        // add null terminator
        pReader->Buffer[pReader->NextWritePos++] = NULL;
        
        // process the sentence
        ProcessNMEASentence(PGN, pReader->Buffer, strlen(pReader->Buffer));

        // start again
        pReader->Synced = false;
      }
      // sentence too long - must be mangled, discard
      else if (pReader->NextWritePos == MAX_NMEA_LENGTH)
      {
        pReader->Synced = false;
      }
    }
  }
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
  BladeCommand[FRONT_BLADE_IDX].CutValve = BLADE_HEIGHT_GROUND_LEVEL;
  BladeCommand[REAR_BLADE_IDX].CutValve  = BLADE_HEIGHT_GROUND_LEVEL;

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

// continually executes
void loop
  (
  )
{
  int RxByte;
  
  // process can module
  CANBus.events();

  CheckForMissingNodes();
  
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
        BladeHeight[FRONT_BLADE_IDX] = 0;
        break;
      case PGN_REAR_ZERO_BLADE_HEIGHT:
        BladeHeight[REAR_BLADE_IDX] = 0;
        break;

        // front blade configuration
      case PGN_FRONT_PWM_GAIN_UP:
        BladeConfig[FRONT_BLADE_IDX].PWMGainUp = GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_PWM_GAIN_DOWN:
        BladeConfig[FRONT_BLADE_IDX].PWMGainDown = GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_PWM_MIN_UP:
        BladeConfig[FRONT_BLADE_IDX].PWMMinUp = GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_PWM_MIN_DOWN:
        BladeConfig[FRONT_BLADE_IDX].PWMMinDown = GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_PWM_MAX_UP:
        BladeConfig[FRONT_BLADE_IDX].PWMMaxUp = GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_PWM_MAX_DOWN:
        BladeConfig[FRONT_BLADE_IDX].PWMMaxDown = GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_INTEGRAL_MULTPLIER:
        BladeConfig[FRONT_BLADE_IDX].IntegralMultiplier = GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_DEADBAND:
        BladeConfig[FRONT_BLADE_IDX].Deadband = GetPGNPacketUInt32(&Command);
        break;

      // rear blade configuration
      case PGN_REAR_PWM_GAIN_UP:
        BladeConfig[REAR_BLADE_IDX].PWMGainUp = GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_PWM_GAIN_DOWN:
        BladeConfig[REAR_BLADE_IDX].PWMGainDown = GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_PWM_MIN_UP:
        BladeConfig[REAR_BLADE_IDX].PWMMinUp = GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_PWM_MIN_DOWN:
        BladeConfig[REAR_BLADE_IDX].PWMMinDown = GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_PWM_MAX_UP:
        BladeConfig[REAR_BLADE_IDX].PWMMaxUp = GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_PWM_MAX_DOWN:
        BladeConfig[REAR_BLADE_IDX].PWMMaxDown = GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_INTEGRAL_MULTPLIER:
        BladeConfig[REAR_BLADE_IDX].IntegralMultiplier = GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_DEADBAND:
        BladeConfig[REAR_BLADE_IDX].Deadband = GetPGNPacketUInt32(&Command);
        break;
        
      // front blade commands
      case PGN_FRONT_CUT_VALVE:
        if (BladeStatus[FRONT_BLADE_IDX].BladeAuto)
        {
          // store for use on next calculation pass
          BladeCommand[FRONT_BLADE_IDX].CutValve = GetPGNPacketUInt32(&Command);
        }
        break;

      // rear blade commands
      case PGN_REAR_CUT_VALVE:
        if (BladeStatus[REAR_BLADE_IDX].BladeAuto)
        {
          // store for use on next calculation pass
          BladeCommand[REAR_BLADE_IDX].CutValve = GetPGNPacketUInt32(&Command);
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

  // tell AgGrade we are alive
  if (PingTimestamp >= PING_PERIOD_MS)
  {
    PingTimestamp = 0;

    pgnpacket_t Status;
    Status.PGN   = PGN_PING;
    agGrade.SendStatus(&Status);
  }

  // process tractor GNSS
  RxByte = Serial6.read();
  ProcessGNSSByte(RxByte, &TractorGNSS, PGN_TRACTOR_NMEA);

  // process front scraper GNSS
  RxByte = Serial7.read();
  ProcessGNSSByte(RxByte, &FrontScraperGNSS, PGN_FRONT_NMEA);
  
  // process rear scraper GNSS
  RxByte = Serial8.read();
  ProcessGNSSByte(RxByte, &RearScraperGNSS, PGN_REAR_NMEA);
}
