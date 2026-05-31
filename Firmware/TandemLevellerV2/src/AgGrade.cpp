// interface to AgGrade

#include "AgGrade.h"
#include "UDPTransfer.h"

///////////////////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTIONS

// starts ethernet
void AgGrade::StartEthernet
  (
  byte MACAddress[],           // our MAC address
  IPAddress LocalIPAddress     // our IP address
  )
{
    // look for working Ethernet connection
  bool LinkUp = false;
  do
  {
    // start Ethernet
    Ethernet.begin(MACAddress, LocalIPAddress);

    Serial.println("Checking for Ethernet...");
 
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
      Serial.println("Ethernet hardware not found");
      continue;
    }

    Serial.println("Checking for link...");

    if (Ethernet.linkStatus() == LinkOFF)
    {
      Serial.println("Ethernet cable is not connected.");
    }
    else
    {
      LinkUp = true;
    }
  } while (!LinkUp);
}


///////////////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTIONS

// constructor
AgGrade::AgGrade
  (
  void
  )
{
}

// connects to AgGrade
void AgGrade::Connect
  (
  byte MACAddress[],            // our MAC address
  IPAddress LocalIPAddress,     // our IP address
  unsigned int LocalPort,       // port that we listen on
  IPAddress RemoteIPAddress,    // IP address of AgGrade
  unsigned int RemotePort       // port that AgGrade is listening on
  )
{
  IPAddress gateway = LocalIPAddress;
  IPAddress dns = LocalIPAddress;
  IPAddress subnet(255, 255, 255, 0);

  gateway[3] = 1;
  dns[3] = 1;

  StartEthernet(MACAddress, LocalIPAddress);

  Ethernet.enableLinkRecovery(MACAddress, LocalIPAddress, subnet, gateway, dns);

  this->LocalPort = LocalPort;

  // start UDP
  Udp.begin(LocalPort);

  // Initialize UDP Transfer for packet-based communication
  UdpTransfer.begin(Udp);
  UdpTransfer.setRemote(RemoteIPAddress, RemotePort);
}

void AgGrade::MaintainEthernet
  (
  void
  )
{
  switch (Ethernet.maintainLinkRecovery())
  {
    case EthernetLinkRecoveryNetifReinitialized:
      Udp.stop();
      Udp.begin(LocalPort);
      UdpTransfer.begin(Udp);
      break;

    case EthernetLinkRecoveryLinkRestored:
    case EthernetLinkRecoveryIdle:
    default:
      break;
  }
}

// sends status value over UDP using packet framing
void AgGrade::Send
  (
  pgnpacket_t *pStatus
  )
{
  // Pack the PGN
  UdpTransfer.packet.txBuff[0] = (uint8_t)(pStatus->PGN & 0xFF);
  UdpTransfer.packet.txBuff[1] = (uint8_t)((pStatus->PGN >> 8) & 0xFF);

  // Pack the data
  for (int b = 0; b < MAX_PGN_LEN; b++)
  {
    UdpTransfer.packet.txBuff[2 + b] = pStatus->Data[b];
  }

  // Send the packet
  UdpTransfer.sendData(MAX_PGN_LEN + 2);
}

// checks if a command is waiting to be read
// returns true if command waiting
bool AgGrade::IsCommandAvailable
  (
  void  
  )
{
  if (UdpTransfer.available() > 0) return true;

  return false;
}

// gets a command from AgGrade
pgnpacket_t AgGrade::GetCommand
  (
  void
  )
{
  pgnpacket_t Command;

  Command.PGN = (pgn_t)(((uint16_t)UdpTransfer.packet.rxBuff[1] << 8) | 
                         UdpTransfer.packet.rxBuff[0]);
  for (int b = 0; b < MAX_PGN_LEN; b++)
  {
    Command.Data[b] = UdpTransfer.packet.rxBuff[2 + b];
  }

  return Command;
}

// call when a blade changes height or direction
// sends the new state to AgGrade.
void AgGrade::SendBladeState
  (
  int BladeIndex,                // index of blade that changed xx_BLADE_IDX
  int PWM,                       // current PWM output to blade
  uint32_t Height,               // current blade height
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
    Send(&Status);

    if (BladeIndex == FRONT_BLADE_IDX)
      Status.PGN = PGN_FRONT_BLADE_DIRECTION;
    else
      Status.PGN = PGN_REAR_BLADE_DIRECTION;
    Status.Data[0] = Direction;
    Send(&Status);

    if (BladeIndex == FRONT_BLADE_IDX)
      Status.PGN = PGN_FRONT_BLADE_HEIGHT;
    else
      Status.PGN = PGN_REAR_BLADE_HEIGHT;
    SetPGNPacketUInt32(&Status, Height);
    Send(&Status);
}

// tells AgGrade that the on-board tractor IMU is being used
void AgGrade::SendUsingOnBoardTractorIMU
  (
  void  
  )
{
  pgnpacket_t Status;
  Status.PGN = PGN_ONBOARD_TRACTOR_IMU;
  Send(&Status);
}

// sends the front blade height to AgGrade
void AgGrade::SendFrontBladeHeight
  (
  uint32_t Height            // blade height (0 -> BLADE_HEIGHT_GROUND_LEVEL * 2)
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_FRONT_BLADE_HEIGHT;
  SetPGNPacketUInt32(&Status, Height);
  Send(&Status);
}

// sends the rear blade height to AgGrade
void AgGrade::SendRearBladeHeight
  (
  uint32_t Height            // blade height (0 -> BLADE_HEIGHT_GROUND_LEVEL * 2)
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_REAR_BLADE_HEIGHT;
  SetPGNPacketUInt32(&Status, Height);
  Send(&Status);
}

// sends an IMU state to AgGrade
void AgGrade::SendIMUState
  (
  int IMUIndex,              // index of IMU xxx_IDX
  imu_t *pIMUValue           // new IMU values
  )
{
  pgnpacket_t Status;

  switch (IMUIndex)
  {
    case TRACTOR_IDX:     Status.PGN = PGN_TRACTOR_IMU; break;
    case FRONT_BLADE_IDX: Status.PGN = PGN_FRONT_IMU;   break;
    case REAR_BLADE_IDX:  Status.PGN = PGN_REAR_IMU;    break;
  }

  int32_t pitchCenti = (int32_t)lrintf(pIMUValue->Pitch * 100.0f);
  SetPGNPacketUInt32AtOffset(&Status, 0, (uint32_t)pitchCenti);

  int32_t rollCenti = (int32_t)lrintf(pIMUValue->Roll * 100.0f);
  SetPGNPacketUInt32AtOffset(&Status, 4, (uint32_t)rollCenti);

  int32_t headingCenti = (int32_t)lrintf(pIMUValue->Heading * 100.0f);
  SetPGNPacketUInt32AtOffset(&Status, 8, (uint32_t)headingCenti);

  int32_t yawRateCenti = (int32_t)lrintf(pIMUValue->YawRate * 100.0f);
  SetPGNPacketUInt32AtOffset(&Status, 12, (uint32_t)yawRateCenti);

  Status.Data[16] = pIMUValue->CalibrationStatus;
  Send(&Status);
}

// sends front blade slave offset to AgGrade
void AgGrade::TxFrontBladeSlaveOffset
  (
  int16_t Offset
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_FRONT_BLADE_OFFSET_SLAVE;
  SetPGNPacketUInt16(&Status, Offset);
  Send(&Status);
}

// sends rear blade slave offset to AgGrade
void AgGrade::TxRearBladeSlaveOffset
  (
  int16_t Offset
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_REAR_BLADE_OFFSET_SLAVE;
  SetPGNPacketUInt16(&Status, Offset);
  Send(&Status);
}

// sends front blade cutting request to AgGrade
void AgGrade::SendFrontBladeCuttingRequest
  (
  bool StartCutting
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_FRONT_CUTTING_REQUEST;
  Status.Data[0] = StartCutting;
  Send(&Status);
}

// sends rear blade cutting request to AgGrade
void AgGrade::SendRearBladeCuttingRequest
  (
  bool StartCutting
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_REAR_CUTTING_REQUEST;
  Status.Data[0] = StartCutting;
  Send(&Status);
}

// sends an emergency stop notification to AgGrade
void AgGrade::EmergencyStop
  (
  void  
  )
{
  pgnpacket_t Status;
  Status.PGN = PGN_ESTOP;
  Send(&Status);
}

// sends a clear emergency stop notification to AgGrade
void AgGrade::ClearEmergencyStop
  (
  void  
  )
{
  pgnpacket_t Status;
  Status.PGN = PGN_CLEAR_ESTOP;
  Send(&Status);
}

// sends a ping to AgGrade
void AgGrade::SendPing
  (
  void
  )
{
    pgnpacket_t Status;
    Status.PGN   = PGN_PING;
    Send(&Status);
}

// sends an NMEA sentence to AgGrade
void AgGrade::SendNMEASentence
  (
  pgn_t PGN,                 // PGN for sentence
  char *sentence,            // sentence
  uint8_t length             // sentence length
  )
{
  pgnpacket_t NMEAPacket;
  
  NMEAPacket.PGN = PGN;
  
  // Copy as much of the sentence as will fit
  uint8_t copyLen = (length > MAX_PGN_LEN) ? MAX_PGN_LEN : length;
  memcpy(NMEAPacket.Data, sentence, copyLen);
  
  Send(&NMEAPacket);
}

// Stores a 16-bit value into a pgn packet
void AgGrade::SetPGNPacketUInt16
  (
  pgnpacket_t *Packet,
  uint16_t Value
  )
{
  Packet->Data[0] = Value & 0xFF;
  Packet->Data[1] = (Value >> 8) & 0xFF;
}

// Stores a 32-bit value into a pgn packet
void AgGrade::SetPGNPacketUInt32
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
void AgGrade::SetPGNPacketUInt32AtOffset
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
uint32_t AgGrade::GetPGNPacketUInt32
  (
  pgnpacket_t *Packet
  )
{
  return ((uint32_t)(Packet->Data[3]) << 24) | 
         ((uint32_t)(Packet->Data[2]) << 16) | 
         ((uint32_t)(Packet->Data[1]) << 8) | 
         Packet->Data[0];
}
