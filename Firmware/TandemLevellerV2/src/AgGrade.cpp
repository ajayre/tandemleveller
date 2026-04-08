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
