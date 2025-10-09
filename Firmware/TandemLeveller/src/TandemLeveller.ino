#include <Arduino.h>
#include "FlexCAN_T4-master/FlexCAN_T4.h"

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
#define TRACTOR_IMU_NODE_ID      0x02
#define FRONTSCRAPER_IMU_NODE_ID 0x03
#define REARSCRAPER_IMU_NODE_ID  0x04
#define PENDANT_NODE_ID          0x05

// from EHPR98-G35 specs
#define PWM_FREQUENCY_HZ 120

// how often to toggle the LED
#define LED_FLASH_PERIOD_MS 1000

#define NMT_RESET_CMD 0x81
#define NMT_RESET_ALL 0x00

// NMT states
#define NMT_STATE_BOOTUP      0x00
#define NMT_STATE_OPERATIONAL 0x05

// number of nodes to monitor
#define NUM_NODES 5

// maximum time to wait for a heartbeat before declaring a node as missing
#define MAX_HEARTBEAT_TIME 6000

static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANBus;
static elapsedMillis LEDFlashTimestamp;
static elapsedMillis HBTime[NUM_NODES];
static bool NodeFound[NUM_NODES];

typedef union _button_state_t
{
  uint16_t RawValue;
  struct
  {
    unsigned int Button1Pressed   : 1;
    unsigned int Button2Pressed   : 1;
    unsigned int Button3Pressed   : 1;
    unsigned int Button4Pressed   : 1;
    unsigned int Reserved1        : 4;
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
    unsigned int Joystick1Left  : 1;
    unsigned int Joystick1Right : 1;
    unsigned int Joystick1Up    : 1;
    unsigned int Joystick1Down  : 1;
    unsigned int Joystick2Left  : 1;
    unsigned int Joystick2Right : 1;
    unsigned int Joystick2Up    : 1;
    unsigned int Joystick2Down  : 1;
  } Fields;
} joystick_state_t;

// process TPDO from IMU
static void ProcessIMUTPDO
  (
  uint8_t NodeId,
  uint8_t Length,
  const uint8_t *pData
  )
{
  if (Length == 6)
  {
    float Yaw   = ((int16_t)(pData[0] | ((uint16_t)pData[1] << 8))) / 100.0;
    float Pitch = ((int16_t)(pData[2] | ((uint16_t)pData[3] << 8))) / 100.0;
    float Roll  = ((int16_t)(pData[4] | ((uint16_t)pData[5] << 8))) / 100.0;
  }
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
    button_state_t ButtonState;
    joystick_state_t JoystickState;

    ButtonState.RawValue = (pData[0] | ((uint16_t)pData[1] << 8));
    JoystickState.RawValue = pData[2];
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
      Serial.println("Found node");
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
      Serial.println("hb reset");
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
        Serial.println("NODE MISSING");
        NodeFound[n] = false;
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
    case 0x180 + PENDANT_NODE_ID:
      ProcessPendantTPDO(msg.len, msg.buf);
      break;
    case 0x180 + TRACTOR_IMU_NODE_ID:
      ProcessIMUTPDO(TRACTOR_IMU_NODE_ID, msg.len, msg.buf);
      break;
    case 0x180 + FRONTSCRAPER_IMU_NODE_ID:
      ProcessIMUTPDO(FRONTSCRAPER_IMU_NODE_ID, msg.len, msg.buf);
      break;
    case 0x180 + REARSCRAPER_IMU_NODE_ID:
      ProcessIMUTPDO(REARSCRAPER_IMU_NODE_ID, msg.len, msg.buf);
      break;

    case 0x700 + PENDANT_NODE_ID:
      ProcessHeartbeat(PENDANT_NODE_ID, msg.len, msg.buf);
      break;
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

// initialize the hardware
void setup()
{
  //debug.begin(Serial);
  //halt_cpu(); 

  Serial5.begin(9600);    // opens serial port, sets data rate to 9600 bps

  CANBus.begin();
  CANBus.setBaudRate(125000);
  CANBus.setMaxMB(64);
  CANBus.enableFIFO();
  CANBus.onReceive(FIFO, CANReceiveHandler);
  CANBus.enableFIFOInterrupt();
  CANBus.setFIFOFilter(0, 0x180 + TRACTOR_IMU_NODE_ID,      STD);
  CANBus.setFIFOFilter(1, 0x180 + FRONTSCRAPER_IMU_NODE_ID, STD);
  CANBus.setFIFOFilter(2, 0x180 + REARSCRAPER_IMU_NODE_ID,  STD);
  CANBus.setFIFOFilter(3, 0x180 + PENDANT_NODE_ID,          STD);
  CANBus.setFIFOFilter(4, 0x700 + TRACTOR_IMU_NODE_ID,      STD);
  CANBus.setFIFOFilter(5, 0x700 + FRONTSCRAPER_IMU_NODE_ID, STD);
  CANBus.setFIFOFilter(6, 0x700 + REARSCRAPER_IMU_NODE_ID,  STD);
  CANBus.setFIFOFilter(7, 0x700 + PENDANT_NODE_ID,          STD);
  CANBus.setMB(MB63, TX); // Set mailbox as transmit

  // set PWM frequency to 120Hz (from EHPR98-G35 specs)
  analogWriteFrequency(FRONT_HEIGHT_PWM, PWM_FREQUENCY_HZ);
  analogWriteFrequency(REAR_HEIGHT_PWM,  PWM_FREQUENCY_HZ);
  analogWriteFrequency(FRONT_DUMP_PWM,   PWM_FREQUENCY_HZ);
  analogWriteFrequency(REAR_DUMP_PWM,    PWM_FREQUENCY_HZ);
  analogWriteResolution(15); // 0 - 32767

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

  //analogWrite(REAR_HEIGHT_PWM, (int)(32767 * 0.50));
  analogWrite(REAR_DUMP_PWM, (int)(32767 * 0.50));
  //digitalWrite(REAR_HEIGHT_DIR, HIGH);

  // reset heartbeat timers and flags
  for (int n = 0; n < NUM_NODES; n++)
  {
    HBTime[n] = 0;
    NodeFound[n] = false;
  }

  ResetAllNodes();
}

// main loop
// perform background tasks
void loop
  (
  void
  )
{
  int c;

  // process can module
  CANBus.events();

  CheckForMissingNodes();

  // echo serial data
  if (Serial5.available() > 0)
  {
    // read the incoming byte:
    c = Serial5.read();
    Serial5.print((char)c);
    Serial5.print("Y");
  }

  // flash LED
  if (LEDFlashTimestamp >= LED_FLASH_PERIOD_MS)
  {
    LEDFlashTimestamp -= LED_FLASH_PERIOD_MS;
    
    digitalToggle(LED);
  }
}
