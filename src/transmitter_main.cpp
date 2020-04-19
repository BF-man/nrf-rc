#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include <Statistics.h>

#define JOYSTICK_TRANSMITTING_OFFSET 200
// TODO auto calibration
// left stick
#define X1_MIN 0
#define X1_IDLE 514
#define X1_MAX 1023

#define Y1_MIN 27
#define Y1_IDLE 514
#define Y1_MAX 1023

// right stick
#define X2_MIN 0
#define X2_IDLE 505
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

void displayStatistics(unsigned int iterationsCount, byte failsCount, float failRate, byte avgMs, byte maxMs, byte momentMs);

Statistics statistics;

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
  

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  Serial.println("Beginning ... ");
}

void loop()
{
  statistics.registerIteration();
  // debugSent();

  // steering
  controllerState.leftStick.x = mapStickState(analogRead(X_pin), X1_MIN, X1_IDLE, X1_MAX);
  controllerState.leftStick.y = mapStickState(analogRead(Y_pin), Y1_MIN, Y1_IDLE, Y1_MAX);
  // throttle
  controllerState.rightStick.y = (-1) * mapStickState(analogRead(Y2_pin), Y2_MIN, Y2_IDLE, Y2_MAX);
  controllerState.rightStick.x = (-1) * mapStickState(analogRead(X2_pin), X2_MIN, X2_IDLE, X2_MAX);
  // setJoystickY(analogRead(Y2_pin), Y2_MAX, Y2_MIN);

  bool result = radio.write(&controllerState, sizeof(controllerState));

  if (!result)
    statistics.registerTransmissionError();

  statistics.registerTransmissionEnd();

  displayStatistics(
      statistics.getIterationsCount(),
      statistics.getFailsCount(),
      statistics.getFailRate(),
      statistics.getAvgMs(),
      statistics.getMaxMs(),
      statistics.getMomentMs());

  statistics.registerIterationEnd();
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

void displayStatistics(unsigned int iterationsCount, byte failsCount, float failRate, byte avgMs, byte maxMs, byte momentMs)
{
  static long refreshCounterMs = millis();
  if (millis() - refreshCounterMs < 1000)
    return;

  refreshCounterMs = millis();
  display.clearDisplay();

  display.setTextSize(1);              // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);             // Start at top-left corner
  // display.print(iterationsCount);
  display.print(F("Err: "));
  display.println(failsCount);
  display.print(F("Err rate: "));
  display.println(failRate);
  display.print(F("avgMs: "));
  display.println(avgMs);
  display.print(F("maxMs: "));
  display.println(maxMs);
  display.print(F("momentMs: "));
  display.println(momentMs);

  display.display();
}
