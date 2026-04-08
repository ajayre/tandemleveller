// GNSS interface and processing

#include <Arduino.h>
#include "GNSS.h"

// constructor
GNSS::GNSS
  (
  void
  )
{
  // tractor GNSS
  Serial6.begin(115200);
  // front scraper GNSS
  Serial7.begin(115200);
  // rear scraper GNSS
  Serial8.begin(115200);
}
