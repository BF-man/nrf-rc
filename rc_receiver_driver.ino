#include "SoftwareSerial.h"
#include <Servo.h>

#define MOVE_FORWARD 'F'
#define MOVE_BACKWARD 'B'
#define STOP 'S'
#define BLUETOOTH_TX_PIN 10
#define BLUETOOTH_RX_PIN 11
#define SERVO_PIN 6
#define MOTOR_LPWM 5
#define MOTOR_RPWM 3

#define MOTOR_MOSFET_GATE_PIN 9

#define    STX          0x02
#define    ETX          0x03
#define    ledPin       13
#define    SLOW         750 // Datafields refresh rate (ms)
#define    FAST         250 // Datafields refresh rate (ms)

byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};                 // bytes received

// * RX is digital pin 10 (connect to TX of other device)
// * TX is digital pin 11 (connect to RX of other device)
SoftwareSerial mySerial(BLUETOOTH_TX_PIN,BLUETOOTH_RX_PIN); // BlueTooth module: pin#10=TX pin#11=RX
Servo servo;

void setup() {
 Serial.begin(9600);
 while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
 }
 servo.attach(SERVO_PIN);
 mySerial.begin(57600);   // 57600 = max value for SoftwareSerial
 pinMode (MOTOR_LPWM, OUTPUT);
 pinMode (MOTOR_RPWM, OUTPUT);
 pinMode (MOTOR_MOSFET_GATE_PIN, OUTPUT);
// while(mySerial.available())  mySerial.read();         // empty RX buffer
}

void loop() {
 if(mySerial.available())  {                           // data received from smartphone
   delay(2);
   cmd[0] =  mySerial.read();  
   if(cmd[0] == STX)  {
     int i=1;      
     while(mySerial.available())  {
       delay(1);
       cmd[i] = mySerial.read();
       if(cmd[i]>127 || i>7)                 break;     // Communication error
       if((cmd[i]==ETX) && (i==2 || i==7))   break;     // Button or Joystick data
       i++;
     }
//     if     (i==2)          getButtonState(cmd[1]);    // 3 Bytes  ex: < STX "C" ETX >
//     else if(i==7)          getJoystickState(cmd);     // 6 Bytes  ex: < STX "200" "180" ETX >
     if(i==7) {
      getJoystickState(cmd);     // 6 Bytes  ex: < STX "200" "180" ETX >
     }
   }
 }
 mySerial.flush();
 Serial.flush();
// sendBlueToothData(); 
}

void main_motor(int joystick_val) {
  byte motor_speed = 0;
 
  if (joystick_val > 5) {
    motor_speed = map(joystick_val, 10, 99, 30, 200);
    motor_controller(MOVE_FORWARD, motor_speed, MOTOR_LPWM, MOTOR_RPWM);
  }
  if (joystick_val < -5) {
    motor_speed = map((-1) * joystick_val, 10, 99, 30, 200);
    motor_controller(MOVE_BACKWARD, motor_speed, MOTOR_LPWM, MOTOR_RPWM);
  }
  if (joystick_val > -5 && joystick_val < 5) {
    motor_speed = 0;
    motor_controller(STOP, motor_speed, MOTOR_LPWM, MOTOR_RPWM);
  }
}

void motor_controller(int motor_direction,
                      byte motor_speed,
                      byte lpw_pin,
                      byte rpw_pin) {
  switch (motor_direction) {
    case MOVE_FORWARD:
      analogWrite(lpw_pin, motor_speed);
      analogWrite(rpw_pin, 0);
      break;
    case MOVE_BACKWARD:
      analogWrite(lpw_pin, 0);
      analogWrite(rpw_pin, motor_speed);
      break;
    case STOP:
      analogWrite(lpw_pin, 0);
      analogWrite(rpw_pin, 0);
      break; 
    default:
    break;
  }  
}

void getJoystickState(byte data[8])    {
  int joyX = (data[1]-48)*100 + (data[2]-48)*10 + (data[3]-48);       // obtain the Int from the ASCII representation
  int joyY = (data[4]-48)*100 + (data[5]-48)*10 + (data[6]-48);
  joyX = joyX - 200;                                                  // Offset to avoid
  joyY = joyY - 200;                                                  // transmitting negative numbers

  if(joyX<-100 || joyX>100 || joyY<-100 || joyY>100)     return;      // commmunication error
  // Your code here ...
  /////////////////////////////////////////
  main_motor(joyY);
  servo.write(map(joyX, -99, 99, 90, 150));
  /////////////////////////////////////////
}

//void sendBlueToothData()  {
// static long previousMillis = 0;
// long currentMillis = millis();
// if(currentMillis - previousMillis > sendInterval) {   // send data back to smartphone
//   previousMillis = currentMillis;
//
//// Data frame transmitted back from Arduino to Android device:
//// < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >
//// < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example
//
//   mySerial.print((char)STX);                                             // Start of Transmission
//   mySerial.print(getButtonStatusString());  mySerial.print((char)0x1);   // buttons status feedback
//   mySerial.print(GetdataInt1());            mySerial.print((char)0x4);   // datafield #1
//   mySerial.print(GetdataFloat2());          mySerial.print((char)0x5);   // datafield #2
//   mySerial.print(displayStatus);                                         // datafield #3
//   mySerial.print((char)ETX);                                             // End of Transmission
// }
//}

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


