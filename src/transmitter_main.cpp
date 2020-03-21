#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

#define STX 0x02
#define ETX 0x03
#define JOYSTICK_TRANSMITTING_OFFSET 200
// TODO auto calibration
#define X1_MIN 0
#define X1_MAX 1019
#define Y1_MIN 37
#define Y1_MAX 1023
#define X2_MIN 0
#define X2_MAX 1017
#define Y2_MIN 0
#define Y2_MAX 1020

#define TRANSMITTING_DELAY 50
byte cmd[8] = {STX, 0, 0, 0, 0, 0, 0, 0};
byte cmd2[8] = {0, 0, 0, 0, 0, 0, 0, 0};
const int SW_pin = 2; // digital pin connected to switch output
const int X_pin = 3;  // analog pin connected to X output
const int Y_pin = 2;  // analog pin connected to Y output
const int X2_pin = 1; // analog pin connected to X output
const int Y2_pin = 0; // analog pin connected to Y output

void setJoystickY(int yValue, int minVal, int maxVal);
void setJoystickX(int xValue, int minVal, int maxVal);
int getJoystickY(byte data[8]);
int getJoystickX(byte data[8]);
void debugSticks();

// Pro mini SPI: 10 (SS), 11 (MOSI), 12 (MISO), 13 (SCK)
RF24 radio(8, 7);

//address through which two modules communicate.
const byte address[6] = "clie1";
int channel = 0;

void setup()
{
  Serial.begin(9600);

  pinMode(SW_pin, INPUT);
  digitalWrite(SW_pin, HIGH);

  radio.begin();

  //set the address
  radio.openWritingPipe(address);
  radio.setChannel(channel); 
  
  // radio.setDataRate (RF24_2MBPS); 
  radio.setDataRate (RF24_1MBPS); 
  // radio.setDataRate (RF24_250KBPS); 
  //Set module as transmitter
  radio.stopListening();
  Serial.println("Beginning ... ");
}

void loop()
{
  //debugSticks();

  setJoystickX(analogRead(X_pin), X1_MIN, X1_MAX);
  setJoystickY(analogRead(Y2_pin), Y2_MAX, Y2_MIN);

  bool result = radio.write(&cmd,sizeof(cmd));

  if (result)
  {
    Serial.println("Success");
  }
  else
  {
    Serial.println("Fail");
    Serial.println(channel);
    radio.setChannel(channel++); 
  }

  delay(TRANSMITTING_DELAY);
}

void setJoystickY(int yValue, int minVal, int maxVal)
{
  int yValueMapped = map(yValue, minVal, maxVal, -100, 100) + JOYSTICK_TRANSMITTING_OFFSET;
  char chars[3];
  String str = String(yValueMapped);
  str.toCharArray(chars, DEC);

  cmd[4] = chars[0];
  cmd[5] = chars[1];
  cmd[6] = chars[2];

  //Serial.println(str);
  //Serial.println("---------setJoystickY-----------");
  //Serial.print(data4);
  //Serial.print('|');
  //Serial.print(data5);
  //Serial.print('|');
  //Serial.print(data6);
  //Serial.println('|');
  //Serial.println("--------------------");
}

void setJoystickX(int xValue, int minVal, int maxVal)
{
  int xValueMapped = map(xValue, minVal, maxVal, -100, 100) + JOYSTICK_TRANSMITTING_OFFSET;

  char chars[3];
  String str = String(xValueMapped);
  str.toCharArray(chars, DEC);

  cmd[1] = chars[0];
  cmd[2] = chars[1];
  cmd[3] = chars[2];

  //Serial.println(str);
  //Serial.println("-----------setJoystickX---------");
  //Serial.print(data1);
  //Serial.print('|');
  //Serial.print(data2);
  //Serial.print('|');
  //Serial.print(data3);
  //Serial.println('|');
  //Serial.println("--------------------");
}

int getJoystickY(byte data[8])
{
  return (data[4] - 48) * 100 + (data[5] - 48) * 10 + (data[6] - 48) - JOYSTICK_TRANSMITTING_OFFSET; // obtain the Int from the ASCII representation
}

int getJoystickX(byte data[8])
{
  return (data[1] - 48) * 100 + (data[2] - 48) * 10 + (data[3] - 48) - JOYSTICK_TRANSMITTING_OFFSET; // obtain the Int from the ASCII representation
}

void debugSticks()
{
  Serial.print("Switch:  ");
  Serial.print(digitalRead(SW_pin));
  Serial.print('|');
  Serial.print("1-st: ");
  Serial.print(analogRead(X_pin));
  Serial.print('|');
  Serial.print(analogRead(Y_pin));
  Serial.print("     ");
  Serial.print("2-nd: ");
  Serial.print(analogRead(X2_pin));
  Serial.print('|');
  Serial.println(analogRead(Y2_pin));
}
