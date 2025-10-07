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

// from EHPR98-G35 specs
#define PWM_FREQUENCY_HZ 120

// how often to toggle the LED
#define LED_FLASH_PERIOD_MS 1000

static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANBus;
static elapsedMillis LEDFlashTimestamp;

// called when a CAN message is received
static void CANReceiveHandler
  (
  const CAN_message_t &msg
  )
{
  /*CAN_message_t txmsg;
  txmsg.id = 0x201;
  for (uint8_t i = 0; i < 8; i++ ) txmsg.buf[i] = msg.buf[i];
  CANBus.write(txmsg);*/
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
  CANBus.setFIFOFilter(0, 0x181, STD);
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
