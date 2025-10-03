#include <Arduino.h>
#include "FlexCAN_T4-master/FlexCAN_T4.h"

// GPIO pins
#define LED 13

// how often to toggle the LED
#define LED_FLASH_PERIOD_MS 400

static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANBus;
static elapsedMillis LEDFlashTimestamp;

// called when a CAN message is received
static void CANReceiveHandler
  (
  const CAN_message_t &msg
  )
{
  CAN_message_t txmsg;
  txmsg.id = 0x201;
  for (uint8_t i = 0; i < 8; i++ ) txmsg.buf[i] = msg.buf[i];
  CANBus.write(txmsg);
}

// initialize the hardware
void setup()
{
  //debug.begin(Serial);
  //halt_cpu(); 

  CANBus.begin();
  CANBus.setBaudRate(125000);
  CANBus.setMaxMB(64);
  CANBus.enableFIFO();
  CANBus.onReceive(FIFO, CANReceiveHandler);
  CANBus.enableFIFOInterrupt();
  CANBus.setFIFOFilter(0, 0x181, STD);
  CANBus.setMB(MB63, TX); // Set mailbox as transmit

  // set up LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
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

  // flash LED
  if (LEDFlashTimestamp >= LED_FLASH_PERIOD_MS)
  {
    LEDFlashTimestamp -= LED_FLASH_PERIOD_MS;
    
    digitalToggle(LED);
  }
}
