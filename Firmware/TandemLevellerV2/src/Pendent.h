// reading of pendent

#ifndef _PENDENTH_
#define _PENDENTH_

#include <Arduino.h>
#include "Global.h"

typedef union _button_state_t
{
  uint16_t RawValue;
  struct
  {
    unsigned int Button1Pressed   : 1;
    unsigned int Button2Pressed   : 1;
    unsigned int Button3Pressed   : 1;
    unsigned int Button4Pressed   : 1;
    unsigned int EStopArmed       : 1;
    unsigned int Reserved1        : 3;
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
    unsigned int Joystick1Up    : 1;
    unsigned int Joystick1Down  : 1;
    unsigned int Joystick1Right : 1;
    unsigned int Joystick1Left  : 1;
    unsigned int Joystick2Up    : 1;
    unsigned int Joystick2Down  : 1;
    unsigned int Joystick2Right : 1;
    unsigned int Joystick2Left  : 1;
  } Fields;
} joystick_state_t;

class Pendent
{
  public:
    // constructor
    Pendent
      (
      void
      );

    // initializes the module
    void Init
      (
      void
      );

    // process TPDO from pendent
    // returns true if TPDO was process, false if not processed
    bool ProcessPendentTPDO
      (
      uint8_t Length,                    // length of PDO data
      const uint8_t *pData               // PDO data
      );

    // checks if button 1 has been pressed
    // returns true if pressed, false if not pressed
    bool IsButton1Pressed
      (
      void
      );

    // checks if button 2 has been pressed
    // returns true if pressed, false if not pressed
    bool IsButton2Pressed
      (
      void
      );

    // checks if all buttons are pressed
    // true if all buttons are pressed, false if not
    bool AreAllButtonsPressed
      (
      void  
      );

    // checks if the ESTOP button has been pressed
    // true if ESTOP has been pressed, false if not
    bool IsESTOPPressed
      (
      void
      );

    // checks if joystick 1 is pushed up
    // returns true if pushed up, otherwise returns false
    bool IsJoystick1Up
      (
      void  
      );

    // checks if joystick 2 is pushed up
    // returns true if pushed up, otherwise returns false
    bool IsJoystick2Up
      (
      void  
      );

    // checks if joystick 1 is pushed down
    // returns true if pushed down, otherwise returns false
    bool IsJoystick1Down
      (
      void  
      );

    // checks if joystick 2 is pushed down
    // returns true if pushed down, otherwise returns false
    bool IsJoystick2Down
      (
      void  
      );

    // checks if joystick 1 is pushed up or down
    // returns true if pushed up or down, otherwise returns false
    bool IsJoystick1UpOrDown
      (
      void  
      );

    // checks if joystick 2 is pushed up or down
    // returns true if pushed up or down, otherwise returns false
    bool IsJoystick2UpOrDown
      (
      void  
      );

    // checks if joystick 1 is pressed
    // returns true if pressed, false if not pressed
    bool IsJoystick1Pressed
      (
      void
      );

    // checks if joystick 2 is pressed
    // returns true if pressed, false if not pressed
    bool IsJoystick2Pressed
      (
      void
      );

  private:
    button_state_t ButtonState;
    joystick_state_t JoystickState;
    unsigned int LastButton1State;
    unsigned int LastButton2State;
};

#endif // _PENDENTH_
