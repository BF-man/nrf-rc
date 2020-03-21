#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
// #include "SoftwareSerial.h"
#include <Servo.h>

#define MOVE_FORWARD 'F'
#define MOVE_BACKWARD 'B'
#define STOP 'S'

#define STEERING_MIN_ANGLE 130 //150
#define STEERING_MAX_ANGLE 50  //90
#define MOTOR_MIN_SPEED_PWM 30
#define MOTOR_MAX_SPEED_PWM 250
#define JOYSTICK_TRANSMITTING_OFFSET 200

#define SERVO_PIN 6
#define MOTOR_LPWM 5
#define MOTOR_RPWM 3

#define STX 0x02
#define ETX 0x03
#define ledPin 13
#define SLOW 750 // Datafields refresh rate (ms)
#define FAST 250 // Datafields refresh rate (ms)

byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // bytes received
// byte cmd2[8] = {0, 0, 0, 0, 0, 0, 0, 0};

Servo servo;

void movementController(int joystickX, int joystickY);
void mainMotorController(int speed);
void motorController(int direction,
                     byte speed,
                     byte lpwPin,
                     byte rpwPin);
void flushSerials();
int getJoystickY(byte data[8]);
int getJoystickX(byte data[8]);

const byte transmitterAddress[6] = "clie1";
const byte receiverAddress[6] = "serv1";
int channel = 10;
RF24 radio(8, 7);

void setup()
{
  Serial.begin(9600);
  servo.attach(SERVO_PIN);

  pinMode(MOTOR_LPWM, OUTPUT);
  pinMode(MOTOR_RPWM, OUTPUT);

  radio.begin();
  radio.setChannel(channel);
  radio.setDataRate(RF24_250KBPS);
  // radio.setDataRate (RF24_2MBPS);
  // radio.setDataRate (RF24_1MBPS);
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);

  radio.openReadingPipe(1, receiverAddress);
  radio.startListening();

  Serial.println("Listening...");
}

void loop()
{
  if (radio.available())
  {
    // Variable for the received timestamp
    while (radio.available())
    {                                // While there is data ready
      radio.read(&cmd, sizeof(cmd)); // Get the payload
    }

    delay(2);

    // if (cmd[0] != STX)
    //   return flushSerials();

    for (int i = 1; i < 8; i++)
    {
      delay(1);

      // if (cmd[i] > 127 || i > 7)
      //   break; // Communication error
      // if ((cmd[i] == ETX) && (i == 2 || i == 7))
      //   break; // Button or Joystick data

      if (i == 7)
        movementController(getJoystickX(cmd), getJoystickY(cmd)); // 6 Bytes  ex: < STX "200" "180" ETX >
      // if (i == 7)
      //   cmd2[7] = getJoystickX(cmd);
      // if (i == 7)
      //   cmd2[6] = getJoystickY(cmd);
    }
    // flushSerials();
  }
  delay(50);
}

void movementController(int joystickX, int joystickY)
{
  if (joystickX < -100 || joystickX > 100 || joystickY < -100 || joystickY > 100)
    return;
  mainMotorController(joystickY);
  servo.write(map(joystickX, -99, 99, STEERING_MIN_ANGLE, STEERING_MAX_ANGLE));
}

void mainMotorController(int speed)
{
  if (speed > 5)
  {
    motorController(MOVE_FORWARD, map(speed, 10, 99, MOTOR_MIN_SPEED_PWM, MOTOR_MAX_SPEED_PWM), MOTOR_LPWM, MOTOR_RPWM);
    return;
  }
  if (speed < -5)
  {
    motorController(MOVE_BACKWARD, map((-1) * speed, 10, 99, MOTOR_MIN_SPEED_PWM, MOTOR_MAX_SPEED_PWM), MOTOR_LPWM, MOTOR_RPWM);
    return;
  }
  motorController(STOP, 0, MOTOR_LPWM, MOTOR_RPWM);
}

void motorController(int direction,
                     byte speed,
                     byte lpwPin,
                     byte rpwPin)
{
  switch (direction)
  {
  case MOVE_FORWARD:
    analogWrite(lpwPin, speed);
    analogWrite(rpwPin, 0);
    break;
  case MOVE_BACKWARD:
    analogWrite(lpwPin, 0);
    analogWrite(rpwPin, speed);
    break;
  case STOP:
    analogWrite(lpwPin, 0);
    analogWrite(rpwPin, 0);
    break;
  default:
    break;
  }
}

void flushSerials()
{
  Serial.flush();
}

int getJoystickY(byte data[8])
{
  return (data[4] - 48) * 100 + (data[5] - 48) * 10 + (data[6] - 48) - JOYSTICK_TRANSMITTING_OFFSET; // obtain the Int from the ASCII representation
}

int getJoystickX(byte data[8])
{
  return (data[1] - 48) * 100 + (data[2] - 48) * 10 + (data[3] - 48) - JOYSTICK_TRANSMITTING_OFFSET; // obtain the Int from the ASCII representation
}

//void getButtonState(int bStatus)  {
// switch (bStatus) {
//// -----------------  BUTTON #1  -----------------------
//   case 'A':
//     buttonStatus |= B000001;        // ON
//     Serial.println("\n** Button_1: ON **");
//     // your code...
//     displayStatus = "LED <ON>";
//     Serial.println(displayStatus);
//     digitalWrite(ledPin, HIGH);
//     break;
//   case 'B':
//     buttonStatus &= B111110;        // OFF
//     Serial.println("\n** Button_1: OFF **");
//     // your code...
//     displayStatus = "LED <OFF>";
//     Serial.println(displayStatus);
//     digitalWrite(ledPin, LOW);
//     break;
//
//// -----------------  BUTTON #2  -----------------------
//   case 'C':
//     buttonStatus |= B000010;        // ON
//     Serial.println("\n** Button_2: ON **");
//     // your code...
//     displayStatus = "Button2 <ON>";
//     Serial.println(displayStatus);
//     break;
//   case 'D':
//     buttonStatus &= B111101;        // OFF
//     Serial.println("\n** Button_2: OFF **");
//     // your code...
//     displayStatus = "Button2 <OFF>";
//     Serial.println(displayStatus);
//     break;
//
//// -----------------  BUTTON #3  -----------------------
//   case 'E':
//     buttonStatus |= B000100;        // ON
//     Serial.println("\n** Button_3: ON **");
//     // your code...
//     displayStatus = "Motor #1 enabled"; // Demo text message
//     Serial.println(displayStatus);
//     break;
//   case 'F':
//     buttonStatus &= B111011;      // OFF
//     Serial.println("\n** Button_3: OFF **");
//     // your code...
//     displayStatus = "Motor #1 stopped";
//     Serial.println(displayStatus);
//     break;
//
//// -----------------  BUTTON #4  -----------------------
//   case 'G':
//     buttonStatus |= B001000;       // ON
//     Serial.println("\n** Button_4: ON **");
//     // your code...
//     displayStatus = "Datafield update <FAST>";
//     Serial.println(displayStatus);
//     sendInterval = FAST;
//     break;
//   case 'H':
//     buttonStatus &= B110111;    // OFF
//     Serial.println("\n** Button_4: OFF **");
//     // your code...
//     displayStatus = "Datafield update <SLOW>";
//     Serial.println(displayStatus);
//     sendInterval = SLOW;
//    break;
//
//// -----------------  BUTTON #5  -----------------------
//   case 'I':           // configured as momentary button
////      buttonStatus |= B010000;        // ON
//     Serial.println("\n** Button_5: ++ pushed ++ **");
//     // your code...
//     displayStatus = "Button5: <pushed>";
//     break;
////   case 'J':
////     buttonStatus &= B101111;        // OFF
////     // your code...
////     break;
//
//// -----------------  BUTTON #6  -----------------------
//   case 'K':
//     buttonStatus |= B100000;        // ON
//     Serial.println("\n** Button_6: ON **");
//     // your code...
//      displayStatus = "Button6 <ON>"; // Demo text message
//    break;
//   case 'L':
//     buttonStatus &= B011111;        // OFF
//     Serial.println("\n** Button_6: OFF **");
//     // your code...
//     displayStatus = "Button6 <OFF>";
//     break;
// }
//// ---------------------------------------------------------------
//}
