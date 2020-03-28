#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

#define MOVE_FORWARD 'F'
#define MOVE_BACKWARD 'B'
#define STOP 'S'

#define STEERING_MIN_ANGLE 130 //150
#define STEERING_MAX_ANGLE 50  //90
#define MOTOR_MIN_SPEED_PWM 20
#define MOTOR_MAX_SPEED_PWM 250

#define STICK_MIN -1023
#define STICK_MAX 1023

#define SERVO_PIN 6
#define MOTOR_LPWM 5
#define MOTOR_RPWM 3
#define MOTOR_THROTTLE_THRESHOLD 5

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

// RADIO
const byte transmitterAddress[6] = "clie1";
const byte receiverAddress[6] = "serv1";
const uint8_t radioChannel = 10;
RF24 radio(8, 7);

Servo servo;

const uint32_t timeoutMs = 1000;
uint32_t connectionLostTimer = millis();

void movementController(int joystickX, int joystickY);
void mainMotorController(int throttle, int maxSpeed);
int getMotorSpeed(int throttle, int maxThrottle);
void motorController(int direction,
                     byte speed,
                     byte lpwPin,
                     byte rpwPin);

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
  if (millis() - connectionLostTimer > timeoutMs)
  {
    Serial.println("Connection lost");
    movementController(0, 0);
    connectionLostTimer = millis();
  }

  if (radio.available())
  {
    connectionLostTimer = millis();

    while (radio.available())
    {                                // While there is data ready
      radio.read(&controllerState, sizeof(controllerState)); // Get the payload
    }
    movementController(controllerState.leftStick.x, controllerState.rightStick.y);
  }
}

void movementController(int joystickX, int joystickY)
{
  if (joystickX < STICK_MIN || joystickX > STICK_MAX || joystickY < STICK_MIN || joystickY > STICK_MAX) {
    // Serial.println('Stick overload!');
    return;
  }
    
  mainMotorController(joystickY, STICK_MAX);
  servo.write(map(joystickX, STICK_MIN, STICK_MAX, STEERING_MIN_ANGLE, STEERING_MAX_ANGLE));
}

void mainMotorController(int throttle, int maxSpeed)
{
  const int speed = getMotorSpeed(throttle, STICK_MAX);
  if (speed > 0) return motorController(MOVE_FORWARD, speed, MOTOR_LPWM, MOTOR_RPWM);
  if (speed < 0) return motorController(MOVE_BACKWARD, speed, MOTOR_LPWM, MOTOR_RPWM);
  motorController(STOP, speed, MOTOR_LPWM, MOTOR_RPWM);
}

int getMotorSpeed(int throttle, int maxThrottle) {
  if (throttle > MOTOR_THROTTLE_THRESHOLD)
  {
    return map(throttle, 0, maxThrottle, MOTOR_MIN_SPEED_PWM, MOTOR_MAX_SPEED_PWM);
  }
  if (throttle < -MOTOR_THROTTLE_THRESHOLD) return (-1) * getMotorSpeed((-1) * throttle, maxThrottle);
  return 0;
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
