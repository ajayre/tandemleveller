#include <Arduino.h>
#include "FlexCAN_T4-master/FlexCAN_T4.h"
//#include "TeensyDebug/TeensyDebug.h"
//#include <usb_serial.h>

// GPIO pins
#define FRONT_HEIGHT_DIR 0
#define FRONT_HEIGHT_PWM 1
#define REAR_HEIGHT_DIR  2
#define REAR_HEIGHT_PWM  3
#define FRONT_DUMP_DIR   4
#define FRONT_DUMP_PWM   5
#define REAR_DUMP_DIR    6
#define REAR_DUMP_PWM    7

// from EHPR98-G35 specs
#define PWM_FREQUENCY_HZ 120

static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANBus;

void canSniff
  (
  const CAN_message_t &msg
  )
{
  CAN_message_t txmsg;
  txmsg.id = 0x201;
  for (uint8_t i = 0; i < 8; i++ ) txmsg.buf[i] = msg.buf[i];
  CANBus.write(txmsg);
}

void setup()
{
  //debug.begin(Serial);
  //halt_cpu(); 

  Serial5.begin(9600);    // opens serial port, sets data rate to 9600 bps

  CANBus.begin();
  CANBus.setBaudRate(125000);
  CANBus.setMaxMB(64);
  CANBus.enableFIFO();
  CANBus.onReceive(FIFO, canSniff);
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
  pinMode(REAR_HEIGHT_DIR, OUTPUT);
  pinMode(FRONT_DUMP_DIR, OUTPUT);
  pinMode(REAR_DUMP_DIR, OUTPUT);

  // initalize direction
  digitalWrite(FRONT_HEIGHT_DIR, LOW);
  digitalWrite(REAR_HEIGHT_DIR, LOW);
  digitalWrite(FRONT_DUMP_DIR, LOW);
  digitalWrite(REAR_DUMP_DIR, LOW);

  // set to 50% PWM
  analogWrite(FRONT_HEIGHT_PWM, 32767 / 2);
}

void loop()
{
  CANBus.events();

  /*// send data only when you receive data:
  if (Serial5.available() > 0)
  {
  
    // read the incoming byte:
    incomingByte = Serial5.read();
  
    // say what you got:
    Serial5.print((char)incomingByte);
    Serial5.print("Y");
  }*/ 
}
