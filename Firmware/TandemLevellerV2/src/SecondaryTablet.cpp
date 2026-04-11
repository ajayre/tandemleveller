// interface to secondary tablet

#include "SecondaryTablet.h"

// constructor
SecondaryTablet::SecondaryTablet
  (
  void
  )
{
}

// initializes the module
void SecondaryTablet::Init
  (
  void
  )
{
  // secondary tablet
  Serial5.begin(115200);
}

// checks if the secondary tablet is present
// returns true for present, false for not present
bool SecondaryTablet::IsPresent
  (
  void  
  )
{
  /*// fixme - remove
  // test connection to secondary tablet
  Serial.println("Sec tablet test...");
  int RxByte;
  Serial5.write('A');
  while ((RxByte = Serial5.read()) == -1);
  if (RxByte == 'A')
  {
    Serial.println("Secondary tablet echo received");
  }
  else
  {
    Serial.print("Secondary tablet echo failed with: ");
    Serial.println(RxByte);
  }*/

  return false;
}
