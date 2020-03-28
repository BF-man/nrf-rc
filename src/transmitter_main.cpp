#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

#define JOYSTICK_TRANSMITTING_OFFSET 200
// TODO auto calibration
// left stick
#define X1_MIN 0
#define X1_IDLE 514
#define X1_MAX 1023

#define Y1_MIN 27
#define Y1_MAX 1023
// right stick
#define X2_MIN 0
#define X2_MAX 1017

#define Y2_MIN 0
#define Y2_IDLE 505
#define Y2_MAX 1023

#define STICK_MIN -1023
#define STICK_MAX 1023

const int X_pin = 3;  // analog pin connected to X output
const int Y_pin = 2;  // analog pin connected to Y output
const int X2_pin = 1; // analog pin connected to X output
const int Y2_pin = 0; // analog pin connected to Y output

int mapStickState(int value, int minVal, int idleVal, int maxVal);
void debugSent();

struct StickState
{
  int x;
  int y;
};
struct ControllerState
{
  StickState leftStick;
  StickState rightStick;
};

ControllerState controllerState;

// Pro mini SPI: 10 (SS), 11 (MOSI), 12 (MISO), 13 (SCK)
RF24 radio(8, 7);

//address through which two modules communicate.
const byte transmitterAddress[6] = "clie1";
const byte receiverAddress[6] = "serv1";
int channel = 10;

void setup()
{
  Serial.begin(9600);

  radio.begin();
  radio.setChannel(channel);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(receiverAddress);
  radio.stopListening(); // Set module as transmitter

  Serial.print("Payload size: ");
  Serial.println(sizeof(controllerState));
  Serial.println("Beginning ... ");
}

void loop()
{
  // debugSent();

  // steering
  controllerState.leftStick.x = mapStickState(analogRead(X_pin), X1_MIN, X1_IDLE, X1_MAX);
  // controllerState.leftStick.y = getStickState(X_pin, X1_MIN, X1_MAX);
  // throttle
  controllerState.rightStick.y = (-1) * mapStickState(analogRead(Y2_pin), Y2_MIN, Y2_IDLE, Y2_MAX);
  // setJoystickY(analogRead(Y2_pin), Y2_MAX, Y2_MIN);

  bool result = radio.write(&controllerState, sizeof(controllerState));

  if (!result)
    Serial.println("Fail");
}

int mapStickState(int value, int minVal, int idleVal, int maxVal)
{
  // TODO: not sure why, but it is still maps to -1 while idle(+1 for inversed y)
  const int fixedIdle = value >= idleVal ? map(value, idleVal, maxVal, maxVal / 2, maxVal) : map(value, 0, idleVal, 0, maxVal / 2);
  return map(fixedIdle, minVal, maxVal, STICK_MIN, STICK_MAX);
}

void debugSent()
{
  Serial.print(" leftStick.x: ");
  Serial.print("(");
  Serial.print(analogRead(X_pin));
  Serial.print(")");
  Serial.print(controllerState.leftStick.x);
  Serial.print("; ");

  Serial.print("rightStick.y: ");
  Serial.print("(");
  Serial.print(analogRead(Y2_pin));
  Serial.print(")");
  Serial.print(controllerState.rightStick.y);
  Serial.println(";");
}
