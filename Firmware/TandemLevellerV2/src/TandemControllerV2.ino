// AgGrade Controller

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
#include "Pendent.h"
#include "SecondaryTablet.h"
#include "SensorFusor.h"

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

// pendant reading
static Pendent Pend;

// secondary tablet
static SecondaryTablet SecTablet;

// sensor fusors
static SensorFusor Fusors[NUM_BLADES + 1];

// State variables
static elapsedMillis LEDFlashTimestamp;
static elapsedMillis PendantSearchTimestamp;
static bool PendantSearch;
static elapsedMillis BladeControlTimestamp;
static state_t State;
static elapsedMillis PingTimestamp;
static elapsedMillis LastPingRxTimestamp;
static bool AgGradeFound = false;
static elapsedMillis TPDOTimestamp;
static bool SecTabletPresent;
static antenna_location_t AntennaLocations[NUM_BLADES + 1]; // fixme - to do - get the values from AgGrade

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
    agGrade.EmergencyStop();

    // tell CAN bus nodes
    CANopn.EmergencyStop(LineNumber);

    BladeControl.EmergencyStop();

    CANopn.TxTPDO1(&(BladeControl.BladeStatus[FRONT_BLADE_IDX]), &(BladeControl.BladeStatus[REAR_BLADE_IDX]));
    CANopn.TxTPDO2(&(BladeControl.BladeStatus[FRONT_BLADE_IDX]), &(BladeControl.BladeStatus[REAR_BLADE_IDX]));
    CANopn.TxTPDO3(NavData.TractorLocation.Latitude, NavData.TractorLocation.Longitude);
    CANopn.TxTPDO4(NavData.TractorLocation.Altitude, NavData.TractorLocation.RtkStatus);
    CANopn.TxTPDO6(NavData.RawTractorLocation.Latitude, NavData.RawTractorLocation.Longitude);

    BladeControl.BladeStatus[FRONT_BLADE_IDX].Cutting = false;
    agGrade.SendFrontBladeCuttingRequest(BladeControl.BladeStatus[FRONT_BLADE_IDX].Cutting);
    BladeControl.BladeStatus[REAR_BLADE_IDX].Cutting = false;
    agGrade.SendRearBladeCuttingRequest(BladeControl.BladeStatus[REAR_BLADE_IDX].Cutting);
  }
}

// called when a blade changes height or direction
static void Blades_BladeChanged
  (
  int BladeIndex,                // index of blade that changed xx_BLADE_IDX
  int PWM,                       // current PWM output to blade
  uint32_t Height,               // current blade height
  blade_direction_t Direction    // direction of blade movement
  )
{
    agGrade.SendBladeState(BladeIndex, PWM, Height, Direction);
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
  agGrade.SendFrontBladeHeight(BladeControl.BladeHeight[FRONT_BLADE_IDX]);
  agGrade.SendRearBladeHeight(BladeControl.BladeHeight[REAR_BLADE_IDX]);
}

// called when an IMU has changed
static void IMU_IMUChanged
  (
  uint8_t Index,             // index of IMU xxx_IDX
  imu_t *pIMUValue           // new IMU values
  )
{
  agGrade.SendIMUState(Index, pIMUValue);
}

// called when IMU module wants to transmit a CAN message
static void IMU_TxCANMessage
  (
  uint16_t Id,               // id of message to send
  uint8_t Length,            // length of message to send
  uint8_t Data[]             // message data to send
  )
{
  CANopn.TxCANMessage(Id, Length, Data);
}

// process TPDO from pendant
static void ProcessPendantTPDO
  (
  uint8_t Length,
  const uint8_t *pData
  )
{
  if (Pend.ProcessPendentTPDO(Length, pData))
  {
    // ESTOP PRESSED
    if (Pend.IsESTOPPressed())
    {
      EmergencyStop(__LINE__);
    }

    if (State == STATE_RUN)
    {
      // toggle cutting mode for front blade
      if (Pend.IsButton1Pressed())
      {
        agGrade.SendFrontBladeCuttingRequest(!BladeControl.BladeStatus[FRONT_BLADE_IDX].Cutting);
      }

      // toggle cutting mode for rear blade
      if (Pend.IsButton2Pressed())
      {
        agGrade.SendRearBladeCuttingRequest(!BladeControl.BladeStatus[REAR_BLADE_IDX].Cutting);
      }

      // if joystick 1 is moved up or down in cutting mode then exit cutting mode
      if (Pend.IsJoystick1UpOrDown() && BladeControl.BladeStatus[FRONT_BLADE_IDX].Cutting)
      {
        BladeControl.BladeStatus[FRONT_BLADE_IDX].Cutting = false;
        agGrade.SendFrontBladeCuttingRequest(BladeControl.BladeStatus[FRONT_BLADE_IDX].Cutting);
      }

      // if joystick 2 is moved up or down in cutting mode then exit cutting mode
      if (Pend.IsJoystick2UpOrDown() && BladeControl.BladeStatus[REAR_BLADE_IDX].Cutting)
      {
        BladeControl.BladeStatus[REAR_BLADE_IDX].Cutting = false;
        agGrade.SendRearBladeCuttingRequest(BladeControl.BladeStatus[REAR_BLADE_IDX].Cutting);
      }

      // if not cutting then jog blade if joystick is moved
      if (!BladeControl.BladeStatus[FRONT_BLADE_IDX].Cutting)
      {
        // jog front blade (joystick button not pressed)
        if (Pend.IsJoystick1Up() && !Pend.IsJoystick1Pressed())
        {
          BladeControl.JogBlade(FRONT_BLADE_IDX, BLADE_DIR_UP);
        }
        else if (Pend.IsJoystick1Down() && !Pend.IsJoystick1Pressed())
        {
          BladeControl.JogBlade(FRONT_BLADE_IDX, BLADE_DIR_DOWN);
        }
        // adjust slave offset (joystick button is pressed)
        else if (Pend.IsJoystick1Up() && Pend.IsJoystick1Pressed())
        {
          if (BladeControl.JogOffset(FRONT_BLADE_IDX, BLADE_DIR_UP))
          {
            agGrade.TxFrontBladeSlaveOffset(BladeControl.BladeStatus[FRONT_BLADE_IDX].SlaveOffset);
          }
        }
        else if (Pend.IsJoystick1Down() && Pend.IsJoystick1Pressed())
        {
          if (BladeControl.JogOffset(FRONT_BLADE_IDX, BLADE_DIR_DOWN))
          {
            agGrade.TxFrontBladeSlaveOffset(BladeControl.BladeStatus[FRONT_BLADE_IDX].SlaveOffset);
          }
        }
      }

      // if not cutting then jog blade if joystick is moved
      if (!BladeControl.BladeStatus[REAR_BLADE_IDX].Cutting)
      {
        // jog rear blade (joystick button not pressed)
        if (Pend.IsJoystick2Up() && !Pend.IsJoystick2Pressed())
        {
          BladeControl.JogBlade(REAR_BLADE_IDX, BLADE_DIR_UP);
        }
        else if (Pend.IsJoystick2Down() && !Pend.IsJoystick2Pressed())
        {
          BladeControl.JogBlade(REAR_BLADE_IDX, BLADE_DIR_DOWN);
        }
        // adjust slave offset (joystick button is pressed)
        else if (Pend.IsJoystick2Up() && Pend.IsJoystick2Pressed())
        {
          if (BladeControl.JogOffset(REAR_BLADE_IDX, BLADE_DIR_UP))
          {
            agGrade.TxRearBladeSlaveOffset(BladeControl.BladeStatus[REAR_BLADE_IDX].SlaveOffset);
          }
        }
        else if (Pend.IsJoystick2Down() && Pend.IsJoystick2Pressed())
        {
          if (BladeControl.JogOffset(REAR_BLADE_IDX, BLADE_DIR_DOWN))
          {
            agGrade.TxRearBladeSlaveOffset(BladeControl.BladeStatus[REAR_BLADE_IDX].SlaveOffset);
          }
        }
      }
    }
    // if in emergency stop
    else if (State == STATE_ESTOP)
    {
      // if all four buttons pressed then reboot to exit ESTOP
      if (Pend.AreAllButtonsPressed())
      {
        // notify AgGrade
        agGrade.ClearEmergencyStop();

        elapsedMillis Delay = 0;
        while (Delay < 2000);
        Reset();
      }
    }
  }
}

// called when sensor fusing needs to happen
// updates the location with fused data
static void GNSS_RequestFuse
  (
  pgn_t PGN,                          // PGN source of the location
  gnss_location_t *pLocation          // unfused location
  )
{
  switch (PGN)
  {
    default:
    case PGN_TRACTOR_NMEA:
        Fusors[TRACTOR_IDX].Fuse(pLocation, IMUHandler, TRACTOR_IDX, AntennaLocations[TRACTOR_IDX]);
        break;

    case PGN_FRONT_NMEA:
        Fusors[FRONT_BLADE_IDX].Fuse(pLocation, IMUHandler, FRONT_BLADE_IDX, AntennaLocations[FRONT_BLADE_IDX]);
        break;

    case PGN_REAR_NMEA:
        Fusors[REAR_BLADE_IDX].Fuse(pLocation, IMUHandler, REAR_BLADE_IDX, AntennaLocations[REAR_BLADE_IDX]);
        break;
  }
}

// called when an NMEA sentence has been received
// sends to AgGrade
static void GNSS_ReceivedNMEASentence
  (
  pgn_t PGN,
  char *sentence,
  uint8_t length
  )
{
  agGrade.SendNMEASentence(PGN, sentence, length);
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
    Serial.println("Pendent lost");
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

// called when the fusor has applied a fuse
void SensorFusor_FuseApplied
  (
  SensorFusor *pFusor,       // the fusor making this call
  int EastingMm,             // correction in easting in mm
  int NorthingMm,            // correction in northing in mm
  int AltitudeMm             // correction in altitude in mm
  )
{
  if (pFusor == &Fusors[TRACTOR_IDX])
  {
    CANopn.TxTPDO5(EastingMm, NorthingMm, AltitudeMm);
  }
}

// initialization
void setup
  (
  )
{
  // Open serial communications for debug
  Serial.begin(115200);
  Serial.println("AgGrade Controller");
  //while (!Serial)
  //{
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}

  SecTablet.Init();
  SecTabletPresent = SecTablet.IsPresent();

  Serial.print("Secondary tablet found: ");
  Serial.println(SecTabletPresent ? "yes" : "no");

  Serial.println("Starting UDP...");

  agGrade.Connect(MACAddress, OurIPAddress, LocalPort, RemoteIPAddress, RemotePort);

  NavData.Connect();
  NavData.SetCallbacks(GNSS_ReceivedNMEASentence, GNSS_RequestFuse);

  BladeControl.SetCallback(Blades_BladeChanged);

  TPDOTimestamp = 0;

  CANopn.Init();
  CANopn.SetCallbacks(CANopen_ProcessPDO, CANopen_NodeLost, CANopen_RequestReset);

  for (int i = 0; i < NUM_BLADES + 1; i++)
  {
    Fusors[i].SetCallbacks(SensorFusor_FuseApplied);
  }

  // set up LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  // start looking for pendant, we can't operate without it
  PendantSearchTimestamp = 0;
  PendantSearch = true;

  Pend.Init();

  IMUHandler.Init();
  IMUHandler.SetCallbacks(IMU_IMUChanged, IMU_TxCANMessage);

  BladeControlTimestamp = 0;
  PingTimestamp = 0;
  LastPingRxTimestamp = 0;

  State = STATE_RUN;

  CANopn.TxBootup();
  CANopn.TxTPDO1(&(BladeControl.BladeStatus[FRONT_BLADE_IDX]), &(BladeControl.BladeStatus[REAR_BLADE_IDX]));
  CANopn.TxTPDO2(&(BladeControl.BladeStatus[FRONT_BLADE_IDX]), &(BladeControl.BladeStatus[REAR_BLADE_IDX]));
  CANopn.TxTPDO3(NavData.TractorLocation.Latitude, NavData.TractorLocation.Longitude);
  CANopn.TxTPDO4(NavData.TractorLocation.Altitude, NavData.TractorLocation.RtkStatus);
  CANopn.TxTPDO6(NavData.RawTractorLocation.Latitude, NavData.RawTractorLocation.Longitude);

  agGrade.TxFrontBladeSlaveOffset(BladeControl.BladeStatus[FRONT_BLADE_IDX].SlaveOffset);
  agGrade.TxRearBladeSlaveOffset(BladeControl.BladeStatus[REAR_BLADE_IDX].SlaveOffset);

  agGrade.SendFrontBladeHeight(BladeControl.BladeHeight[FRONT_BLADE_IDX]);
  agGrade.SendRearBladeHeight(BladeControl.BladeHeight[REAR_BLADE_IDX]);

  CANopn.ResetAllNodes();

  // fixme - remove
  AntennaLocations[TRACTOR_IDX].HeightMm = 2593;

  Serial.println("Ready");
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
    CANopn.TxTPDO3(NavData.TractorLocation.Latitude, NavData.TractorLocation.Longitude);
    CANopn.TxTPDO4(NavData.TractorLocation.Altitude, NavData.TractorLocation.RtkStatus);
    CANopn.TxTPDO6(NavData.RawTractorLocation.Latitude, NavData.RawTractorLocation.Longitude);
  }

  CANopn.Process();
  IMUHandler.Process();

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
        agGrade.TxFrontBladeSlaveOffset(BladeControl.BladeStatus[FRONT_BLADE_IDX].SlaveOffset);
        agGrade.TxRearBladeSlaveOffset(BladeControl.BladeStatus[REAR_BLADE_IDX].SlaveOffset);
        break;

      case PGN_PING:
        LastPingRxTimestamp = 0;
        break;

      // reset blade height
      case PGN_FRONT_ZERO_BLADE_HEIGHT:
        BladeControl.BladeHeight[FRONT_BLADE_IDX] = BLADE_HEIGHT_GROUND_LEVEL;
        agGrade.SendFrontBladeHeight(BladeControl.BladeHeight[FRONT_BLADE_IDX]);
        break;
      case PGN_REAR_ZERO_BLADE_HEIGHT:
        BladeControl.BladeHeight[REAR_BLADE_IDX] = BLADE_HEIGHT_GROUND_LEVEL;
        agGrade.SendRearBladeHeight(BladeControl.BladeHeight[FRONT_BLADE_IDX]);
        break;

      // request blade height
      case PGN_FRONT_BLADE_HEIGHT:
        agGrade.SendFrontBladeHeight(BladeControl.BladeHeight[FRONT_BLADE_IDX]);
        break;        
      case PGN_REAR_BLADE_HEIGHT:
        agGrade.SendRearBladeHeight(BladeControl.BladeHeight[REAR_BLADE_IDX]);
        break;

        // front blade configuration
      case PGN_FRONT_PWM_GAIN_UP:
        BladeControl.BladeConfig[FRONT_BLADE_IDX].PWMGainUp = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_PWM_GAIN_DOWN:
        BladeControl.BladeConfig[FRONT_BLADE_IDX].PWMGainDown = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_PWM_MIN_UP:
        BladeControl.BladeConfig[FRONT_BLADE_IDX].PWMMinUp = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_PWM_MIN_DOWN:
        BladeControl.BladeConfig[FRONT_BLADE_IDX].PWMMinDown = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_PWM_MAX_UP:
        BladeControl.BladeConfig[FRONT_BLADE_IDX].PWMMaxUp = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_PWM_MAX_DOWN:
        BladeControl.BladeConfig[FRONT_BLADE_IDX].PWMMaxDown = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_INTEGRAL_MULTPLIER:
        BladeControl.BladeConfig[FRONT_BLADE_IDX].IntegralMultiplier = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_DEADBAND:
        BladeControl.BladeConfig[FRONT_BLADE_IDX].Deadband = agGrade.GetPGNPacketUInt32(&Command);
        break;

      // rear blade configuration
      case PGN_REAR_PWM_GAIN_UP:
        BladeControl.BladeConfig[REAR_BLADE_IDX].PWMGainUp = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_PWM_GAIN_DOWN:
        BladeControl.BladeConfig[REAR_BLADE_IDX].PWMGainDown = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_PWM_MIN_UP:
        BladeControl.BladeConfig[REAR_BLADE_IDX].PWMMinUp = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_PWM_MIN_DOWN:
        BladeControl.BladeConfig[REAR_BLADE_IDX].PWMMinDown = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_PWM_MAX_UP:
        BladeControl.BladeConfig[REAR_BLADE_IDX].PWMMaxUp = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_PWM_MAX_DOWN:
        BladeControl.BladeConfig[REAR_BLADE_IDX].PWMMaxDown = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_INTEGRAL_MULTPLIER:
        BladeControl.BladeConfig[REAR_BLADE_IDX].IntegralMultiplier = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_DEADBAND:
        BladeControl.BladeConfig[REAR_BLADE_IDX].Deadband = agGrade.GetPGNPacketUInt32(&Command);
        break;
        
      // front blade commands
      case PGN_FRONT_CUT_VALVE:
        if (BladeControl.BladeStatus[FRONT_BLADE_IDX].Cutting)
        {
          // store for use on next calculation pass
          BladeControl.BladeCommand[FRONT_BLADE_IDX].CutValve = agGrade.GetPGNPacketUInt32(&Command);
        }
        break;

      // rear blade commands
      case PGN_REAR_CUT_VALVE:
        if (BladeControl.BladeStatus[REAR_BLADE_IDX].Cutting)
        {
          // store for use on next calculation pass
          BladeControl.BladeCommand[REAR_BLADE_IDX].CutValve = agGrade.GetPGNPacketUInt32(&Command);
        }
        break;

      // front blade status
      case PGN_FRONT_STATE:
        {
          blade_modes_t Mode = (blade_modes_t)Command.Data[0];

          if (Mode == BLADE_MODE_AUTOCUTTING)
            BladeControl.BladeStatus[FRONT_BLADE_IDX].Cutting = true;
          else
            BladeControl.BladeStatus[FRONT_BLADE_IDX].Cutting = false;
        }
        break;

      // rear blade status
      case PGN_REAR_STATE:
        {
          blade_modes_t Mode = (blade_modes_t)Command.Data[0];

          if (Mode == BLADE_MODE_AUTOCUTTING)
            BladeControl.BladeStatus[REAR_BLADE_IDX].Cutting = true;
          else
            BladeControl.BladeStatus[REAR_BLADE_IDX].Cutting = false;
        }
        break;

      // antenna locations
      case PGN_TRACTOR_ANTENNA_HEIGHT:
        AntennaLocations[TRACTOR_IDX].HeightMm = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_TRACTOR_ANTENNA_LEFTOFF:
        AntennaLocations[TRACTOR_IDX].LeftMm = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_TRACTOR_ANTENNA_FORWARDOFF:
        AntennaLocations[TRACTOR_IDX].ForwardMm = agGrade.GetPGNPacketUInt32(&Command);
        break;
        
      case PGN_FRONT_ANTENNA_HEIGHT:
        AntennaLocations[FRONT_BLADE_IDX].HeightMm = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_ANTENNA_LEFTOFF:
        AntennaLocations[FRONT_BLADE_IDX].LeftMm = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_FRONT_ANTENNA_FORWARDOFF:
        AntennaLocations[FRONT_BLADE_IDX].ForwardMm = agGrade.GetPGNPacketUInt32(&Command);
        break;

      case PGN_REAR_ANTENNA_HEIGHT:
        AntennaLocations[REAR_BLADE_IDX].HeightMm = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_ANTENNA_LEFTOFF:
        AntennaLocations[REAR_BLADE_IDX].LeftMm = agGrade.GetPGNPacketUInt32(&Command);
        break;
      case PGN_REAR_ANTENNA_FORWARDOFF:
        AntennaLocations[REAR_BLADE_IDX].ForwardMm = agGrade.GetPGNPacketUInt32(&Command);
        break;

      case PGN_MAGNETIC_DECLINATION:
        {
          for (int i = 0; i < NUM_BLADES + 1; i++)
          {
            Fusors[i].SetMagneticDeclination(agGrade.GetPGNPacketUInt32(&Command));
          }
        }
        break;

      // IMUs
      case PGN_TRACTOR_IMU_LEVEL:
        IMUHandler.SetZero(TRACTOR_IMU_NODE_ID);
        break;
      case PGN_FRONT_IMU_LEVEL:
        IMUHandler.SetZero(FRONTSCRAPER_IMU_NODE_ID);
        break;
      case PGN_REAR_IMU_LEVEL:
        IMUHandler.SetZero(FRONTSCRAPER_IMU_NODE_ID);
        break;
      case PGN_FRONT_BUCKET_IMU_LEVEL:
        IMUHandler.SetZero(FRONT_BUCKET_IMU_NODE_ID);
        break;
      case PGN_REAR_BUCKET_IMU_LEVEL:
        IMUHandler.SetZero(REAR_BUCKET_IMU_NODE_ID);
        break;

      case PGN_TRACTOR_IMU_ORIENT:
        {
          uint32_t Orientation = agGrade.GetPGNPacketUInt32(&Command);
          IMUHandler.SetOrientation(TRACTOR_IMU_NODE_ID, (imu_orientation_t)Orientation);
        }
        break;
      case PGN_FRONT_IMU_ORIENT:
        {
          uint32_t Orientation = agGrade.GetPGNPacketUInt32(&Command);
          IMUHandler.SetOrientation(FRONTSCRAPER_IMU_NODE_ID, (imu_orientation_t)Orientation);
        }
        break;
      case PGN_REAR_IMU_ORIENT:
        {
          uint32_t Orientation = agGrade.GetPGNPacketUInt32(&Command);
          IMUHandler.SetOrientation(REARSCRAPER_IMU_NODE_ID, (imu_orientation_t)Orientation);
        }
        break;
      case PGN_FRONT_BUCKET_IMU_ORIENT:
        {
          uint32_t Orientation = agGrade.GetPGNPacketUInt32(&Command);
          IMUHandler.SetOrientation(FRONT_BUCKET_IMU_NODE_ID, (imu_orientation_t)Orientation);
        }
        break;
      case PGN_REAR_BUCKET_IMU_ORIENT:
        {
          uint32_t Orientation = agGrade.GetPGNPacketUInt32(&Command);
          IMUHandler.SetOrientation(REAR_BUCKET_IMU_NODE_ID, (imu_orientation_t)Orientation);
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
    else if (Pend.IsESTOPPressed())
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

    agGrade.SendPing();
  }

  NavData.Process();
}
