// reading of pendent

#include "Pendent.h"

// constructor
Pendent::Pendent
  (
  void
  )
{
}

// initializes the module
void Pendent::Init
  (
  void
  )
{
  // reset buttons and joysticks
  ButtonState.RawValue = 0;
  JoystickState.RawValue = 0;

  LastButton1State = 0;
  LastButton2State = 0;
}

// process TPDO from pendent
// returns true if TPDO was process, false if not processed
bool Pendent::ProcessPendentTPDO
  (
  uint8_t Length,                    // length of PDO data
  const uint8_t *pData               // PDO data
  )
{
  if (Length == 3)
  {
    LastButton1State = ButtonState.Fields.Button1Pressed;
    LastButton2State = ButtonState.Fields.Button2Pressed;

    ButtonState.RawValue = (pData[0] | ((uint16_t)pData[1] << 8));
    JoystickState.RawValue = pData[2];

    return true;
  }

  return false;
}

// checks if button 1 has been pressed
// returns true if pressed, false if not pressed
bool Pendent::IsButton1Pressed
  (
  void
  )
{
  if (ButtonState.Fields.Button1Pressed && !LastButton1State) return true;

  return false;
}

// checks if button 2 has been pressed
// returns true if pressed, false if not pressed
bool Pendent::IsButton2Pressed
  (
  void
  )
{
  if (ButtonState.Fields.Button2Pressed && !LastButton2State) return true;

  return false;
}

// checks if all buttons are pressed
// true if all buttons are pressed, false if not
bool Pendent::AreAllButtonsPressed
  (
  void  
  )
{
  if (ButtonState.Fields.Button1Pressed && ButtonState.Fields.Button2Pressed &&
      ButtonState.Fields.Button3Pressed && ButtonState.Fields.Button4Pressed)
    return true;

  return false;
}

// checks if the ESTOP button has been pressed
// true if ESTOP has been pressed, false if not
bool Pendent::IsESTOPPressed
  (
  void
  )
{
  if (!ButtonState.Fields.EStopArmed) return true;

  return false;
}

// checks if joystick 1 is pushed up
// returns true if pushed up, otherwise returns false
bool Pendent::IsJoystick1Up
  (
  void  
  )
{
  if (JoystickState.Fields.Joystick1Up) return true;

  return false;
}

// checks if joystick 2 is pushed up
// returns true if pushed up, otherwise returns false
bool Pendent::IsJoystick2Up
  (
  void  
  )
{
  if (JoystickState.Fields.Joystick2Up) return true;

  return false;
}

// checks if joystick 1 is pushed down
// returns true if pushed down, otherwise returns false
bool Pendent::IsJoystick1Down
  (
  void  
  )
{
  if (JoystickState.Fields.Joystick1Down) return true;

  return false;
}

// checks if joystick 2 is pushed down
// returns true if pushed down, otherwise returns false
bool Pendent::IsJoystick2Down
  (
  void  
  )
{
  if (JoystickState.Fields.Joystick2Down) return true;

  return false;
}

// checks if joystick 1 is pushed up or down
// returns true if pushed up or down, otherwise returns false
bool Pendent::IsJoystick1UpOrDown
  (
  void  
  )
{
  return IsJoystick1Up() || IsJoystick1Down();
}

// checks if joystick 2 is pushed up or down
// returns true if pushed up or down, otherwise returns false
bool Pendent::IsJoystick2UpOrDown
  (
  void  
  )
{
  return IsJoystick2Up() || IsJoystick2Down();
}

// checks if joystick 1 is pressed
// returns true if pressed, false if not pressed
bool Pendent::IsJoystick1Pressed
  (
  void
  )
{
  return ButtonState.Fields.Joystick1Pressed;
}

// checks if joystick 2 is pressed
// returns true if pressed, false if not pressed
bool Pendent::IsJoystick2Pressed
  (
  void
  )
{
  return ButtonState.Fields.Joystick2Pressed;
}
