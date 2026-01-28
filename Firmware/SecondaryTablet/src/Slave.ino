#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include "UDPTransfer.h"

// GPIO pins
#define LED 13

// how often to toggle the LED
#define LED_FLASH_PERIOD_MS 1000

// maximum amount of data for a single PGN
#define MAX_PGN_LEN 16

// how often to send ping to AgGrade
#define PING_PERIOD_MS 1000

// time to wait before deciding that AgGrade has disconnected
#define PING_TIMEOUT_PERIOD_MS 3000

// supported PGNs
typedef enum _pgn_t : uint16_t
{
  // misc
  PGN_ESTOP                    = 0x0000,
  PGN_RESET                    = 0x0001,
  PGN_AGGRADE_STARTED          = 0x0002,
  PGN_PING                     = 0x0003,

  // GNSS
  PGN_TRACTOR_NMEA             = 0x7000,
  PGN_FRONT_NMEA               = 0x7001,
  PGN_REAR_NMEA                = 0x7002
} pgn_t;

// information that is transmitted to/from AgGrade
typedef struct _pgnpacket_t
{
  pgn_t PGN;
  uint8_t Data[MAX_PGN_LEN] = { 0 };
} pgnpacket_t;

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

// An EthernetUDP instance to let us send and receive packets over UDP
static EthernetUDP Udp;

// UDP Transfer for packet-based communication
static UDPTransfer UdpTransfer;

// State variables
static elapsedMillis LEDFlashTimestamp;
static elapsedMillis PingTimestamp;
static elapsedMillis LastPingRxTimestamp;
static bool AgGradeFound = false;

// resets the controller
static void Reset
  (
  void
  )
{
  SCB_AIRCR = 0x05FA0004;
  while(1);
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

// sends status value over UDP using packet framing
static void SendStatus
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

// gets a command from UDP packet
static pgnpacket_t GetCommand
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

// initialization
void setup
  (
  )
{
  // primary tablet
  Serial1.begin(115200);
 
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  Serial.println("Slave with UDP");
  //while (!Serial)
  //{
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}

  /*// look for working Ethernet connection
  bool LinkUp = false;
  do
  {
    // start Ethernet
    Ethernet.begin(MACAddress, OurIPAddress);

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

  Serial.println("Starting UDP...");

  // start UDP
  Udp.begin(LocalPort);

  // Initialize UDP Transfer for packet-based communication
  UdpTransfer.begin(Udp);
  UdpTransfer.setRemote(RemoteIPAddress, RemotePort);*/

  // set up LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  PingTimestamp = 0;
  LastPingRxTimestamp = 0;

  Serial.println("Ready!");
}

// continually executes
void loop
  (
  )
{
  int RxByte;
  
  /*// Check for incoming UDP packets with packet framing
  if (UdpTransfer.available() > 0)
  {
    // We received a complete packet
    pgnpacket_t Command = GetCommand();
    
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
        // fixme - to do - ?
        break;

      case PGN_PING:
        LastPingRxTimestamp = 0;
        break;
        
      default:
        Serial.print("Unknown PGN: 0x");
        Serial.println(Command.PGN, HEX);
        break;
    }
  }*/

  // check to see if AgGrade has disappeared
  if ((LastPingRxTimestamp >= PING_TIMEOUT_PERIOD_MS) && AgGradeFound)
  {
    AgGradeFound = false;

    // fixme - to do - ?
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
    SendStatus(&Status);
  }

  // process data from primary tablet
  RxByte = Serial1.read();
  if (RxByte != -1)
  {
    // fixme - to do
    // echo back
    Serial1.write(RxByte);
  }
}
