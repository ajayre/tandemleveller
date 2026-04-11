// interface to secondary tablet

#ifndef _SECTABLETH_
#define _SECTABLETH_

#include <Arduino.h>
#include "Global.h"

class SecondaryTablet
{
  public:
    // constructor
    SecondaryTablet
      (
      void
      );

    // initializes the module
    void SecondaryTablet::Init
      (
      void
      );

    // checks if the secondary tablet is present
    // returns true for present, false for not present
    bool SecondaryTablet::IsPresent
      (
      void  
      );

  private:
};

#endif // _SECTABLETH_
