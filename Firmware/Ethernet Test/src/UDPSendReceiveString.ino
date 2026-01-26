#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include "UDPTransfer.h"

// NMEA 0183 special characters
#define LF 0x0A
#define CR 0x0D

// maximum length of an NMEA 0183 sentence
#define MAX_NMEA_LENGTH 83

// maximum number of bytes in a PGN packet payload
#define MAX_PGN_LEN (MAX_NMEA_LENGTH + 2)

// supported PGNs (subset from TandemLeveller)
typedef enum _pgn_t : uint16_t
{
  // misc
  PGN_ESTOP                    = 0x0000,
  PGN_RESET                    = 0x0001,
  PGN_AGGRADE_STARTED          = 0x0002,
  PGN_PING                     = 0x0003,

  // blade control
  PGN_FRONT_CUT_VALVE          = 0x1000,
  PGN_REAR_CUT_VALVE           = 0x1001,
  PGN_FRONT_ZERO_BLADE_HEIGHT  = 0x1002,
  PGN_REAR_ZERO_BLADE_HEIGHT   = 0x1003,

  // blade status
  PGN_FRONT_BLADE_OFFSET_SLAVE = 0x5000,
  PGN_FRONT_BLADE_PWMVALUE     = 0x5001,
  PGN_FRONT_BLADE_DIRECTION    = 0x5002,
  PGN_FRONT_CUTTING            = 0x5003,
  PGN_REAR_BLADE_OFFSET_SLAVE  = 0x5004,
  PGN_REAR_BLADE_PWMVALUE      = 0x5005,
  PGN_REAR_BLADE_DIRECTION     = 0x5006,
  PGN_REAR_CUTTING             = 0x5007,
  PGN_FRONT_BLADE_HEIGHT       = 0x5008,
  PGN_REAR_BLADE_HEIGHT        = 0x5009,

  // IMU
  PGN_TRACTOR_PITCH            = 0x6000,
  PGN_TRACTOR_ROLL             = 0x6001,
  PGN_TRACTOR_HEADING          = 0x6002,
  PGN_TRACTOR_YAWRATE          = 0x6003,
  PGN_TRACTOR_IMUCALIBRATION   = 0x6004,
  PGN_FRONT_PITCH              = 0x6005,
  PGN_FRONT_ROLL               = 0x6006,
  PGN_FRONT_HEADING            = 0x6007,
  PGN_FRONT_YAWRATE            = 0x6008,
  PGN_FRONT_IMUCALIBRATION     = 0x6009,
  PGN_REAR_PITCH               = 0x600A,
  PGN_REAR_ROLL                = 0x600B,
  PGN_REAR_HEADING             = 0x600C,
  PGN_REAR_YAWRATE             = 0x600D,
  PGN_REAR_IMUCALIBRATION      = 0x600E,

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

// An EthernetUDP instance to let us send and receive packets over UDP
static EthernetUDP Udp;

// UDP Transfer for packet-based communication
static UDPTransfer UdpTransfer;

// GNSS stream readers
static gnss_reader_t TractorGNSS      = { 0 };
static gnss_reader_t FrontScraperGNSS = { 0 };
static gnss_reader_t RearScraperGNSS  = { 0 };

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

// sends an NMEA sentence over UDP with packet framing
static void SendNMEASentence
  (
  pgn_t PGN,
  const char* sentence,
  uint8_t length
  )
{
  pgnpacket_t NMEAPacket;
  
  NMEAPacket.PGN = PGN;
  
  // Copy as much of the sentence as will fit
  uint8_t copyLen = (length > MAX_PGN_LEN) ? MAX_PGN_LEN : length;
  memcpy(NMEAPacket.Data, sentence, copyLen);
  
  SendStatus(&NMEAPacket);
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

        // debug output
        Serial.print(pReader->Buffer);
        
        // Send using packet-based UDP
        // Note: For full NMEA sentences (up to 83 bytes), 
        // you may want to use a larger packet or raw UDP
        SendNMEASentence(PGN, pReader->Buffer, strlen(pReader->Buffer));
        // fixme - remove
        //Udp.beginPacket(RemoteIPAddress, RemotePort);
        //Udp.write(pReader->Buffer);
        //Udp.endPacket();

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
  // tractor GNSS
  Serial6.begin(115200);
  // front scraper GNSS
  Serial7.begin(115200);
  // rear scraper GNSS
  Serial8.begin(115200);

  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  Serial.println("UDP Test with Packet Framing");
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // look for working Ethernet connection
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
  UdpTransfer.setRemote(RemoteIPAddress, RemotePort);

  Serial.println("Ready!");
}

// continually executes
void loop
  (
  )
{
  int RxByte;
  
  // Check for incoming UDP packets with packet framing
  if (UdpTransfer.available() > 0)
  {
    // We received a complete packet
    pgnpacket_t Command = GetCommand();
    
    Serial.print("Received PGN: 0x");
    Serial.println(Command.PGN, HEX);
    
    // Process the command based on PGN
    switch (Command.PGN)
    {
      case PGN_PING:
        // Respond with a ping
        {
          pgnpacket_t Response;
          Response.PGN = PGN_PING;
          SendStatus(&Response);
        }
        break;
        
      case PGN_RESET:
        // Reset the device
        Serial.println("Reset requested");
        // SCB_AIRCR = 0x05FA0004;  // Uncomment to enable reset
        break;
        
      // Add more command handlers as needed
      
      default:
        Serial.print("Unknown PGN: 0x");
        Serial.println(Command.PGN, HEX);
        break;
    }
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
