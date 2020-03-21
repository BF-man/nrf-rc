#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
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

// RADIO
const byte transmitterAddress[6] = "clie1";
const byte receiverAddress[6] = "serv1";
const uint8_t radioChannel = 10;
RF24 radio(8, 7);

byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // bytes received

Servo servo;

const uint32_t timeoutMs = 1000;
uint32_t connectionLostTimer =  millis();
const byte loopDelayMs = 50;

void movementController(int joystickX, int joystickY);
void mainMotorController(int speed);
void motorController(int direction,
                     byte speed,
                     byte lpwPin,
                     byte rpwPin);
void flushSerials();
int getJoystickY(byte data[8]);
int getJoystickX(byte data[8]);
void stop();

void setup()
{
  Serial.begin(9600);
  servo.attach(SERVO_PIN);

  pinMode(MOTOR_LPWM, OUTPUT);
  pinMode(MOTOR_RPWM, OUTPUT);

  radio.begin();
  radio.setChannel(radioChannel);
  radio.setDataRate(RF24_250KBPS);

  radio.openReadingPipe(1, receiverAddress);
  radio.startListening();

  Serial.println("Listening...");
  connectionLostTimer = millis();
}

void loop()
{
   if(millis() - connectionLostTimer > timeoutMs){
    stop();
    delay(loopDelayMs);
  }

  if (radio.available())
  {
    connectionLostTimer = millis();
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
    }
  }
  delay(loopDelayMs);
}

void movementController(int joystickX, int joystickY)
{
  if (joystickX < -100 || joystickX > 100 || joystickY < -100 || joystickY > 100)
    return;
  mainMotorController(joystickY);
  servo.write(map(joystickX, -99, 99, STEERING_MIN_ANGLE, STEERING_MAX_ANGLE));
}

void stop() {
  movementController(0, 0);
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

