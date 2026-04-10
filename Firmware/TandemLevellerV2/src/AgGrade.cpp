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
  StartEthernet(MACAddress, LocalIPAddress);

  // start UDP
  Udp.begin(LocalPort);

  // Initialize UDP Transfer for packet-based communication
  UdpTransfer.begin(Udp);
  UdpTransfer.setRemote(RemoteIPAddress, RemotePort);
}

// sends status value over UDP using packet framing
void AgGrade::SendStatus
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
    SendStatus(&Status);

    if (BladeIndex == FRONT_BLADE_IDX)
      Status.PGN = PGN_FRONT_BLADE_DIRECTION;
    else
      Status.PGN = PGN_REAR_BLADE_DIRECTION;
    Status.Data[0] = Direction;
    SendStatus(&Status);
}

// sends the front blade height to AgGrade
void AgGrade::SendFrontBladeHeight
  (
  int Height                 // blade height
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_FRONT_BLADE_HEIGHT;
  SetPGNPacketUInt32(&Status, Height);
  SendStatus(&Status);
}

// sends the rear blade height to AgGrade
void AgGrade::SendRearBladeHeight
  (
  int Height                 // blade height
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_REAR_BLADE_HEIGHT;
  SetPGNPacketUInt32(&Status, Height);
  SendStatus(&Status);
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

  SetPGNPacketUInt32AtOffset(&Status, 0,  (uint32_t)(pIMUValue->Pitch   * 100));
  SetPGNPacketUInt32AtOffset(&Status, 4,  (uint32_t)(pIMUValue->Roll    * 100));
  SetPGNPacketUInt32AtOffset(&Status, 8,  (uint32_t)(pIMUValue->Heading * 100));
  SetPGNPacketUInt32AtOffset(&Status, 12, (uint32_t)(pIMUValue->YawRate * 100));
  Status.Data[16] = pIMUValue->CalibrationStatus;
  SendStatus(&Status);
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
  SendStatus(&Status);
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
  SendStatus(&Status);
}

// sends front blade auto state to AgGrade
void AgGrade::SendFrontBladeAuto
  (
  bool Auto
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_FRONT_CUTTING;
  Status.Data[0] = Auto;
  SendStatus(&Status);
}

// sends rear blade auto state to AgGrade
void AgGrade::SendRearBladeAuto
  (
  bool Auto
  )
{
  pgnpacket_t Status;

  Status.PGN = PGN_REAR_CUTTING;
  Status.Data[0] = Auto;
  SendStatus(&Status);
}

// sends an emergency stop notification to AgGrade
void AgGrade::EmergencyStop
  (
  void  
  )
{
  pgnpacket_t Status;
  Status.PGN = PGN_CLEAR_ESTOP;
  SendStatus(&Status);
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
  
  SendStatus(&NMEAPacket);
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
