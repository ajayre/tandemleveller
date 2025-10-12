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
#define HB_PRODUCER_TIME_MS 100

#define BUTTON1_INPUT_PIN 0
#define BUTTON1_LED_PIN   1
#define BUTTON1_ID        0

#define BUTTON2_INPUT_PIN 2
#define BUTTON2_LED_PIN   3
#define BUTTON2_ID        1

#define BUTTON3_INPUT_PIN 4
#define BUTTON3_LED_PIN   5
#define BUTTON3_ID        2

#define BUTTON4_INPUT_PIN 6
#define BUTTON4_LED_PIN   7
#define BUTTON4_ID        3

#define BUTTON5_INPUT_PIN 8
#define BUTTON5_LED_PIN   9
#define BUTTON5_ID        4

#define JOYSTICK1_X_PIN      A2
#define JOYSTICK1_Y_PIN      A1
#define JOYSTICK1_BUTTON_PIN 14
#define JOYSTICK1_ID         9

#define JOYSTICK2_X_PIN      A5
#define JOYSTICK2_Y_PIN      A4
#define JOYSTICK2_BUTTON_PIN 17
#define JOYSTICK2_ID         10

// time for power on self test
#define POST_DELAY_MS 1000

// supported LED states
typedef enum _led_state_t
{
  LED_ON,
  LED_OFF
} led_state_t;

// prototypes
static void ButtonHandler(uint8_t btnId, uint8_t btnState);

static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANBus;
elapsedMillis HBTime;
static WDT_T4<WDT1> wdt;
static Button Button1(BUTTON1_ID, ButtonHandler);
static Button Button2(BUTTON2_ID, ButtonHandler);
static Button Button3(BUTTON3_ID, ButtonHandler);
static Button Button4(BUTTON4_ID, ButtonHandler);
static Button Button5(BUTTON5_ID, ButtonHandler);
static Button Joystick1Button(JOYSTICK1_ID, ButtonHandler);
static Button Joystick2Button(JOYSTICK2_ID, ButtonHandler);
static elapsedMillis POSTTimer;
static bool POSTCompleted = false;
static uint16_t ButtonStates = 0x0000;
static uint8_t JoystickStates = 0x00;

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
  uint8_t Data[3];

  // transmit button and joystick states
  Data[0] =  ButtonStates       & 0xFF;
  Data[1] = (ButtonStates >> 8) & 0xFF;
  Data[2] = JoystickStates;
  TxCANMessage(0x180 + NODE_ID, 3, Data);
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

// called when a button is pressed or released
static void ButtonHandler
  (
  uint8_t btnId,     // ID of button that changed
  uint8_t btnState   // new state of button (BTN_PRESSED, BTN_OPEN)
  )
{
  if (btnState == BTN_PRESSED)
  {
    Serial.println("Pushed button");

    switch (btnId)
    {
      case BUTTON1_ID:   SetButtonLED(BUTTON1_LED_PIN, LED_ON); ButtonStates |= (1 << 0); break;
      case BUTTON2_ID:   SetButtonLED(BUTTON2_LED_PIN, LED_ON); ButtonStates |= (1 << 1); break;
      case BUTTON3_ID:   SetButtonLED(BUTTON3_LED_PIN, LED_ON); ButtonStates |= (1 << 2); break;
      case BUTTON4_ID:   SetButtonLED(BUTTON4_LED_PIN, LED_ON); ButtonStates |= (1 << 3); break;
      case BUTTON5_ID:   SetButtonLED(BUTTON5_LED_PIN, LED_ON); ButtonStates |= (1 << 4); break;
      case JOYSTICK1_ID:                                        ButtonStates |= (1 << 8); break;
      case JOYSTICK2_ID:                                        ButtonStates |= (1 << 9); break;
    }
  }
  else
  {
    // btnState == BTN_OPEN
    Serial.println("Released button");

    switch (btnId)
    {
      case BUTTON1_ID:   SetButtonLED(BUTTON1_LED_PIN, LED_OFF); ButtonStates &= ~(1 << 0); break;
      case BUTTON2_ID:   SetButtonLED(BUTTON2_LED_PIN, LED_OFF); ButtonStates &= ~(1 << 1); break;
      case BUTTON3_ID:   SetButtonLED(BUTTON3_LED_PIN, LED_OFF); ButtonStates &= ~(1 << 2); break;
      case BUTTON4_ID:   SetButtonLED(BUTTON4_LED_PIN, LED_OFF); ButtonStates &= ~(1 << 3); break;
      case BUTTON5_ID:   SetButtonLED(BUTTON5_LED_PIN, LED_OFF); ButtonStates &= ~(1 << 4); break;
      case JOYSTICK1_ID:                                         ButtonStates &= ~(1 << 8); break;
      case JOYSTICK2_ID:                                         ButtonStates &= ~(1 << 9); break;
    }
  }

  TxPDO();
}

// button debounce handling
static void pollButtons
  (
  void  
  )
{
  Button1.update(digitalRead(BUTTON1_INPUT_PIN));
  Button2.update(digitalRead(BUTTON2_INPUT_PIN));
  Button3.update(digitalRead(BUTTON3_INPUT_PIN));
  Button4.update(digitalRead(BUTTON4_INPUT_PIN));
  Button5.update(digitalRead(BUTTON5_INPUT_PIN));
  Joystick1Button.update(digitalRead(JOYSTICK1_BUTTON_PIN));
  Joystick2Button.update(digitalRead(JOYSTICK2_BUTTON_PIN));
}

// joystick handling
static void pollJoysticks
  (
  void
  )
{
  int X = analogRead(JOYSTICK1_X_PIN);
  int Y = analogRead(JOYSTICK1_Y_PIN);

  bool Changed = false;

  if (X <= 100)
  {
    if (!(JoystickStates & (1 << 0)))
    {
      JoystickStates |= (1 << 0); // left
      Changed = true;
    }
  }
  else if (X >= 923)
  {
    if (!(JoystickStates & (1 << 1)))
    {
      JoystickStates |= (1 << 1); // right
      Changed = true;
    }
  }
  else
  {
    if (JoystickStates & (1 << 0))
    {
      JoystickStates &= ~(1 << 0);
      Changed = true;
    }
    if (JoystickStates & (1 << 1))
    {
      JoystickStates &= ~(1 << 1);
      Changed = true;
    }
  }

  if (Y <= 100)
  {
    if (!(JoystickStates & (1 << 2)))
    {
      JoystickStates |= (1 << 2); // up
      Changed = true;
    }
  }
  else if (Y >= 923)
  {
    if (!(JoystickStates & (1 << 3)))
    {
      JoystickStates |= (1 << 3); // down
      Changed = true;
    }
  }
  else
  {
    if (JoystickStates & (1 << 2))
    {
      JoystickStates &= ~(1 << 2);
      Changed = true;
    }
    if (JoystickStates & (1 << 3))
    {
      JoystickStates &= ~(1 << 3);
      Changed = true;
    }
  }

  X = analogRead(JOYSTICK2_X_PIN);
  Y = analogRead(JOYSTICK2_Y_PIN);

  if (X <= 100)
  {
    if (!(JoystickStates & (1 << 4)))
    {
      JoystickStates |= (1 << 4); // left
      Changed = true;
    }
  }
  else if (X >= 923)
  {
    if (!(JoystickStates & (1 << 5)))
    {
      JoystickStates |= (1 << 5); // right
      Changed = true;
    }
  }
  else
  {
    if (JoystickStates & (1 << 4))
    {
      JoystickStates &= ~(1 << 4);
      Changed = true;
    }
    if (JoystickStates & (1 << 5))
    {
      JoystickStates &= ~(1 << 5);
      Changed = true;
    }
  }

  if (Y <= 100)
  {
    if (!(JoystickStates & (1 << 6)))
    {
      JoystickStates |= (1 << 6); // up
      Changed = true;
    }
  }
  else if (Y >= 923)
  {
    if (!(JoystickStates & (1 << 7)))
    {
      JoystickStates |= (1 << 7); // down
      Changed = true;
    }
  }
  else
  {
    if (JoystickStates & (1 << 6))
    {
      JoystickStates &= ~(1 << 6);
      Changed = true;
    }
    if (JoystickStates & (1 << 7))
    {
      JoystickStates &= ~(1 << 7);
      Changed = true;
    }
  }

  if (Changed) TxPDO();
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
  pinMode(BUTTON1_LED_PIN,   OUTPUT);

  pinMode(BUTTON2_INPUT_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_LED_PIN,   OUTPUT);

  pinMode(BUTTON3_INPUT_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_LED_PIN,   OUTPUT);

  pinMode(BUTTON4_INPUT_PIN, INPUT_PULLUP);
  pinMode(BUTTON4_LED_PIN,   OUTPUT);

  pinMode(BUTTON5_INPUT_PIN, INPUT_PULLUP);
  pinMode(BUTTON5_LED_PIN,   OUTPUT);

  pinMode(JOYSTICK1_X_PIN, INPUT);
  pinMode(JOYSTICK1_Y_PIN, INPUT);
  pinMode(JOYSTICK1_BUTTON_PIN, INPUT_PULLUP);

  pinMode(JOYSTICK2_X_PIN, INPUT);
  pinMode(JOYSTICK2_Y_PIN, INPUT);
  pinMode(JOYSTICK2_BUTTON_PIN, INPUT_PULLUP);

  // button 5 is the estop
  Button5.setNormallyClosed();
  ButtonStates |= (1 << 4);

  // turn on all LEDs for POST
  SetButtonLED(BUTTON1_LED_PIN, LED_ON);
  SetButtonLED(BUTTON2_LED_PIN, LED_ON);
  SetButtonLED(BUTTON3_LED_PIN, LED_ON);
  SetButtonLED(BUTTON4_LED_PIN, LED_ON);
  SetButtonLED(BUTTON5_LED_PIN, LED_ON);

  POSTTimer = 0;

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

  // process buttons
  pollButtons();
  // process joysticks
  pollJoysticks();

  if (!POSTCompleted && (POSTTimer >= POST_DELAY_MS))
  {
    POSTCompleted = true;
    
    // turn off all LEDs
    SetButtonLED(BUTTON1_LED_PIN, LED_OFF);
    SetButtonLED(BUTTON2_LED_PIN, LED_OFF);
    SetButtonLED(BUTTON3_LED_PIN, LED_OFF);
    SetButtonLED(BUTTON4_LED_PIN, LED_OFF);
    SetButtonLED(BUTTON5_LED_PIN, LED_OFF);
  }

  // transmit heartbeats
  if (HBTime >= HB_PRODUCER_TIME_MS)
  {
    HBTime -= HB_PRODUCER_TIME_MS;

    TxHeartbeat();
    wdt.feed();
  }
}
