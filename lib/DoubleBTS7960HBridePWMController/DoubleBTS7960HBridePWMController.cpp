#include "Arduino.h"
#include "DoubleBTS7960HBridePWMController.h"

void DoubleBTS7960HBridePWMController::attach(byte leftPwmPin,
                                              byte rightPwmPin)
{
  lpwPin = leftPwmPin;
  rpwPin = rightPwmPin;
}

void DoubleBTS7960HBridePWMController::write(byte value)
{
  if (value > 128)
  {
    analogWrite(lpwPin, map(value, 128, 255, 0, 255));
    analogWrite(rpwPin, 0);
    return;
  }
  if (value < 127)
  {
    analogWrite(lpwPin, 0);
    analogWrite(rpwPin, map(value, 0, 127, 255, 0));
    return;
  }

  analogWrite(lpwPin, 0);
  analogWrite(rpwPin, 0);
}
