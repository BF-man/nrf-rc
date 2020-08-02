#ifndef DoubleBTS7960HBridePWMController_h
#define DoubleBTS7960HBridePWMController_h

#include "Arduino.h"

class DoubleBTS7960HBridePWMController
{
public:
  void write(byte value);
  void attach(byte leftPwmPin, byte rightPwmPin);

private:
  byte lpwPin;
  byte rpwPin;
};

#endif
