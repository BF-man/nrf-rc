#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <nrfrc_config.h>

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

byte SOFT_START_MULTIPLEXER = 80;
byte SOFT_START_CURRENT_SPEED_MULTIPLEXER = 1;
byte SOFT_START_BURST_ADD_PERCENTAGE = 20;

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
RF24 radio(8, 7);

Servo servo;

const uint32_t timeoutMs = 1000;
uint32_t connectionLostTimer = millis();
uint32_t loopTimer = millis();

void movementController(int joystickX, int joystickY);
void mainMotorController(int throttle, int maxSpeed);
int getMotorSpeed(int throttle, int maxThrottle);
void motorController(int direction,
                     byte speed,
                     byte lpwPin,
                     byte rpwPin);
int smoothThrottleChange(int throttle);

void setup()
{
  Serial.begin(9600);
  servo.attach(SERVO_PIN);

  pinMode(MOTOR_LPWM, OUTPUT);
  pinMode(MOTOR_RPWM, OUTPUT);

  radio.begin();
  radio.setChannel(NRFRC_CONFIG_RADIO_CHANNEL);
  radio.setDataRate(NRFRC_CONFIG_RADIO_DATA_RATE);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(NRFRC_CONFIG_RADIO_ACK_ENABLED);

  radio.openReadingPipe(1, receiverAddress);
  radio.startListening();

  Serial.println("Listening...");
  connectionLostTimer = millis();
}

void loop()
{
  loopTimer = millis();
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
    {                                                        // While there is data ready
      radio.read(&controllerState, sizeof(controllerState)); // Get the payload
    }
    movementController(controllerState.leftStick.x, controllerState.rightStick.y);
  }

  // uint32_t looptime = millis() - loopTimer;
  // if (looptime > 5) {
  //   Serial.println("Loop: " + String(looptime));
  // }
}

void movementController(int joystickX, int joystickY)
{
  if (joystickX < STICK_MIN || joystickX > STICK_MAX || joystickY < STICK_MIN || joystickY > STICK_MAX)
  {
    // Serial.println('Stick overload!');
    return;
  }

  mainMotorController(joystickY, STICK_MAX);
  servo.write(map(joystickX, STICK_MIN, STICK_MAX, STEERING_MIN_ANGLE, STEERING_MAX_ANGLE));
}

void mainMotorController(int throttle, int maxSpeed)
{
  static int direction = STOP;
  const int smoothedThrottle = smoothThrottleChange(throttle);

  const int speed = getMotorSpeed(smoothedThrottle, STICK_MAX);
  const byte absSpeed = abs(speed);
  // if (speed != 0) Serial.println(speed);
  if (speed > 0)
    direction = MOVE_FORWARD;
  if (speed < 0)
    direction = MOVE_BACKWARD;
  if (speed == 0)
    direction = STOP;

  // Serial.println("sp: " + String(speed) + "; dir: " + String(direction) + "; smSp: " + String(absSpeed));
  return motorController(direction, absSpeed, MOTOR_LPWM, MOTOR_RPWM);
}

int getMotorSpeed(int throttle, int maxThrottle)
{
  if (throttle > MOTOR_THROTTLE_THRESHOLD)
  {
    return map(throttle, 0, maxThrottle, MOTOR_MIN_SPEED_PWM, MOTOR_MAX_SPEED_PWM);
  }
  if (throttle < -MOTOR_THROTTLE_THRESHOLD)
    return (-1) * getMotorSpeed((-1) * throttle, maxThrottle);
  return 0;
}

void motorController(int direction,
                     byte speed,
                     byte lpwPin,
                     byte rpwPin)
{
  // return;
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

int smoothThrottleChange(int rawThrottle)
{
  static uint32_t smoothStartUpTimer = 0;
  static uint32_t smoothStartDownTimer = 0;
  static int currentSpeed = map(rawThrottle, -1025, 1025, 0, 1024);
  ;
  const int throttle = map(rawThrottle, -1025, 1025, 0, 1024);

  if (throttle > currentSpeed && smoothStartUpTimer == 0)
  {
    // currentSpeed = min(currentSpeed + (throttle / (currentSpeed + 1)) / SOFT_START_BURST_ADD_PERCENTAGE, throttle);
    smoothStartUpTimer = millis();
    smoothStartDownTimer = 0;
    // Serial.print(" IF ");
  }

  if (throttle <= currentSpeed && smoothStartDownTimer == 0)
  {
    // Serial.print(" ELSE ");
    smoothStartDownTimer = millis();
    smoothStartUpTimer = 0;
    // currentSpeed = max(floor(currentSpeed * 0.8), throttle);
  }

  if (throttle == currentSpeed)
  {
    smoothStartDownTimer = 0;
    smoothStartUpTimer = 0;
  }

  if (smoothStartUpTimer > 0)
  {
    // Serial.println("up " + String(currentSpeed) + " " + String(throttle));

    int devider = max((throttle - currentSpeed * SOFT_START_CURRENT_SPEED_MULTIPLEXER), 1);
    uint32_t timeDIff = millis() - smoothStartUpTimer;
    currentSpeed += timeDIff * SOFT_START_MULTIPLEXER / devider;
    currentSpeed = min(currentSpeed, throttle);
    // Serial.println("speed: " + String(speed) + "; currentSpeed: " + String(currentSpeed) + "; timeDIff: " + String(timeDIff) + "; devider: " + String(devider));
    // Serial.print(speed);
    // Serial.print("; currentSpeed: ");
    // // Serial.print(nextSpeed);
    // // Serial.print(" ");
    // Serial.println(currentSpeed);
    // if (currentSpeed == throttle) Serial.println(timeDIff);

    // Serial.print("; timeDIff: ");
    // Serial.print(timeDIff);
    // Serial.print("; devider: ");
    // Serial.println(devider);
    // Serial.println("up2 " + String(currentSpeed) + " " + String(throttle));
  }

  if (smoothStartDownTimer > 0)
  {
    // Serial.println("down " + String(currentSpeed) + " " + String(throttle));
    uint32_t timeDIff = millis() - smoothStartDownTimer;
    // = МАКС(D1 - A2 * ((D1 - $H$1) / МАКС((D1 + $H$1); 1)); $H$1)
    // = МАКС(D1 - A2 * (D1 + $H$1) / $H$2; $H$1)
    currentSpeed -= timeDIff * 0.2 * (float)abs(currentSpeed - throttle) / 2000.0;
    // Serial.println(String((float)abs(currentSpeed - throttle) / (float)(currentSpeed + throttle + 1)) + " " + String(((float)511 / (float)1536)));
    currentSpeed = max(currentSpeed, throttle);
    // Serial.println("down3 " + String(currentSpeed) + " " + String(throttle));
  }

  // Serial.print(speed);
  // Serial.print(" ");
  // Serial.print(nextSpeed);
  // Serial.print(" ");
  // Serial.println(currentSpeed);
  if (currentSpeed != 512)
    Serial.println(String(currentSpeed) + " " + String(throttle));
  return map(currentSpeed, 0, 1024, -1024, 1024);

  // return currentSpeed;
}
