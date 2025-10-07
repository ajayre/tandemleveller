#include <Arduino.h>
#include "FlexCAN_T4-master/FlexCAN_T4.h"
#include "WDT_T4-master/Watchdog_t4.h"
#include "button-debounce-main/src/debounce.h"

// "CANopen" node id
#define NODE_ID 0x05

#define NMT_STATE_BOOTUP      0x00
#define NMT_STATE_OPERATIONAL 0x05

// NMT commands
#define NMT_CMD_RESET 0x81

// special value to reset all nodes
#define NMT_RESET_ALL 0x00

// time between heartbeats in millseconds
#define HB_PRODUCER_TIME_MS 1000

#define BUTTON1_INPUT_PIN 0
#define BUTTON1_LED_PIN   1
#define BUTTON1_ID        0

// time for power on self test
#define POST_DELAY_MS 1000

// supported LED states
typedef enum _led_state_t
{
  LED_ON,
  LED_OFF
} led_state_t;

// prototypes
static void buttonHandler(uint8_t btnId, uint8_t btnState);

static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANBus;
elapsedMillis HBTime;
static WDT_T4<WDT1> wdt;
static Button Button1(BUTTON1_ID, buttonHandler);
static elapsedMillis POSTTimer;
static bool POSTCompleted = false;

static void buttonHandler
  (
  uint8_t btnId,
  uint8_t btnState
  )
{
  if (btnState == BTN_PRESSED) {
    Serial.println("Pushed button");
    SetButtonLED(BUTTON1_LED_PIN, LED_ON);
  } else {
    // btnState == BTN_OPEN.
    Serial.println("Released button");
    SetButtonLED(BUTTON1_LED_PIN, LED_OFF);
  }
}

// reboot the device
static void Reboot
  (
  void
  )
{
  SCB_AIRCR = 0x05FA0004;
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
  uint8_t Data[6] = { 0 };

  TxCANMessage(0x180 + NODE_ID, 6, Data);
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

// button debounce handling
static void pollButtons
  (
  void  
  )
{
  Button1.update(digitalRead(BUTTON1_INPUT_PIN));
}

// sets the LED on a button
static void SetButtonLED
  (
  uint8_t Pin,
  led_state_t State
  )
{
  digitalWrite(Pin, State == LED_ON ? LOW : HIGH);
}

// initialize the hardware
void setup()
{
  Serial.begin(115200);
  Serial.println("Tandem Leveller Pendant");

  // set up watchdog
  WDT_timings_t config;
  config.trigger = 2; // in seconds, 0->128
  config.timeout = 3; // in seconds, 0->128
  wdt.begin(config);

  // set up I/O
  pinMode(BUTTON1_INPUT_PIN, INPUT_PULLUP);
  pinMode(BUTTON1_LED_PIN, OUTPUT);

  // turn on all LEDs for POST
  SetButtonLED(BUTTON1_LED_PIN, LED_ON);

  POSTTimer = 0;

  CANBus.begin();
  CANBus.setBaudRate(125000);
  CANBus.setMaxMB(64);
  CANBus.enableFIFO();
  CANBus.onReceive(FIFO, CANReceiveHandler);
  CANBus.enableFIFOInterrupt();
  CANBus.setFIFOFilter(0, 0x000, STD);
  CANBus.setMB(MB63, TX); // Set mailbox as transmit
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

  if (!POSTCompleted && (POSTTimer >= POST_DELAY_MS))
  {
    POSTCompleted = true;
    
    // turn off all LEDs
    SetButtonLED(BUTTON1_LED_PIN, LED_OFF);

    // tell everyone we are ready
    TxBootup();

    HBTime = 0;
    wdt.feed();
  }

  // only execute once we have done the power on self test
  if (POSTCompleted)
  {
    // process buttons
    pollButtons();

    // transmit heartbeats
    if (HBTime >= HB_PRODUCER_TIME_MS)
    {
      HBTime -= HB_PRODUCER_TIME_MS;

      TxHeartbeat();
      wdt.feed();
    }
  }
}
