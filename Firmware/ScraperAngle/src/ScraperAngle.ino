#include <Arduino.h>
#include "FlexCAN_T4-master/FlexCAN_T4.h"
#include "WDT_T4-master/Watchdog_t4.h"

// "CANopen" node id
#define NODE_ID 0x06

#define NMT_STATE_BOOTUP      0x00
#define NMT_STATE_OPERATIONAL 0x05

// NMT commands
#define NMT_CMD_RESET 0x81

// special value to reset all nodes
#define NMT_RESET_ALL 0x00

// time between heartbeats in millseconds
#define HB_PRODUCER_TIME_MS 100

// time between transmission of TPDO in milliseconds
#define TPDO_EVENT_TIME_MS 10

#define JOYSTICK1_X_PIN A2

static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANBus;
elapsedMillis HBTime;
static WDT_T4<WDT1> wdt;
static uint16_t Angle = 0;
elapsedMillis TPDOTime;

// reboot the device
static void Reboot
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

  char txt[50];
  sprintf(txt, "Tx: %3.3X %d ", Id, Length);
  Serial.print(txt);
  for (uint8_t i = 0; i < Length; i++)
  {
    sprintf(txt, "%2.2X ", Data[i]);
    Serial.print(txt);
  }
  Serial.println();
}

// transmits a bootup message
static void TxBootup
  (
  void
  )
{
  uint8_t Data[1];

  Data[0] = NMT_STATE_BOOTUP;
  TxCANMessage(0x700 + NODE_ID, 1, Data);
}

// transmits a heartbeat message
static void TxHeartbeat
  (
  void
  )
{
  uint8_t Data[1];

  Data[0] = NMT_STATE_OPERATIONAL;
  TxCANMessage(0x700 + NODE_ID, 1, Data);
}

// transmits a PDO containing the button states
static void TxPDO
  (
  void
  )
{
  uint8_t Data[2];

  Data[0] =  Angle       & 0xFF;
  Data[1] = (Angle >> 8) & 0xFF;
  TxCANMessage(0x180 + NODE_ID, 2, Data);
}

// called when a CAN message is received
static void CANReceiveHandler
  (
  const CAN_message_t &msg
  )
{
  // process NMT message
  if ((msg.id == 0x000) && !msg.flags.extended && (msg.len == 2))
  {
    // reset
    if (msg.buf[0] == NMT_CMD_RESET)
    {
      // this node
      if ((msg.buf[1] == NODE_ID) || (msg.buf[1] == NMT_RESET_ALL))
      {
        Reboot();
      }
    }
  }
}

// sensor handling
static void PollSensor
  (
  void
  )
{
  int X = analogRead(JOYSTICK1_X_PIN);

  // convert voltage to an angle
  Angle = (uint16_t)(X * 100);

  // transmit PDO periodically
  if (TPDOTime >= TPDO_EVENT_TIME_MS)
  {
    TPDOTime = 0;
    TxPDO();
  }
}

// initialize the hardware
void setup()
{
  Serial.begin(115200);
  Serial.println("Tandem Leveller Scraper Angle");

  // set up watchdog
  WDT_timings_t config;
  config.trigger = 2; // in seconds, 0->128
  config.timeout = 3; // in seconds, 0->128
  wdt.begin(config);

  // set up I/O
  pinMode(JOYSTICK1_X_PIN, INPUT);

  analogReadAveraging(16);

  CANBus.begin();
  CANBus.setBaudRate(125000);
  CANBus.setMaxMB(64);
  CANBus.enableFIFO();
  CANBus.onReceive(FIFO, CANReceiveHandler);
  CANBus.enableFIFOInterrupt();
  CANBus.setFIFOFilter(0, 0x000, STD);
  CANBus.setMB(MB63, TX); // Set mailbox as transmit

  // tell everyone we are ready
  TxBootup();

  HBTime = 0;
  TPDOTime = 0;
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

  PollSensor();

  // transmit heartbeats
  if (HBTime >= HB_PRODUCER_TIME_MS)
  {
    HBTime -= HB_PRODUCER_TIME_MS;

    TxHeartbeat();
    wdt.feed();
  }
}
