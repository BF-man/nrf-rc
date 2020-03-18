 /**
 * A Mirf example to test the latency between two Ardunio.
 *
 * Pins:
 * Hardware SPI:
 * MISO -> 12
 * MOSI -> 11
 * SCK -> 13
 *
 * Configurable:
 * CE -> 8
 * CSN -> 7
 *
 * Note: To see best case latency comment out all Serial.println
 * statements not displaying the result and load 
 * 'ping_server_interupt' on the server.
 */
 
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>\

#define    STX          0x02
#define    ETX          0x03
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
const int X_pin = 3; // analog pin connected to X output
const int Y_pin = 2; // analog pin connected to Y output
const int X2_pin = 1; // analog pin connected to X output
const int Y2_pin = 0; // analog pin connected to Y output
 
void setup(){
  Serial.begin(9600);
  /*
   * Setup pins / SPI.
   */
 
  /* To change CE / CSN Pins:
   * 
   * Mirf.csnPin = 9;
   * Mirf.cePin = 7;
   */
  /*
  Mirf.cePin = 7;
   Mirf.csnPin = 8;
   */
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
 
  /*
   * Configure reciving address.
   */
 
  Mirf.setRADDR((byte *)"clie1");
 
  /*
   * Set the payload length to sizeof(unsigned long) the
   * return type of millis().
   *
   * NB: payload on client and server must be the same.
   */
 
  Mirf.payload = sizeof(cmd);
 
  /*
   * Write channel and payload config then power up reciver.
   */
 
  /*
   * To change channel:
   * 
   * Mirf.channel = 10;
   *
   * NB: Make sure channel is legal in your area.
   */
 
  Mirf.config();
  pinMode(SW_pin, INPUT);
  digitalWrite(SW_pin, HIGH);
  Serial.println("Beginning ... "); 
}
 
void loop(){
  unsigned long time = millis();
  Mirf.setTADDR((byte *)"serv1");

  //debugSticks();

  //setJoystickX(1455 - 0.7107 * analogRead(X_pin) - 0.7107 * analogRead(Y_pin)); //225'' cos(225) = -0.7071; sin(225) = -0.7071; from 0 to 1454
  //setJoystickY(0.7107 * analogRead(X2_pin) - 0.7107 * analogRead(Y2_pin) + 1455/2 + 2.5); //acceleration. from 730 to 725
  setJoystickX(analogRead(X_pin), X1_MIN, X1_MAX);
  setJoystickY(analogRead(Y2_pin), Y2_MAX, Y2_MIN);
  //Serial.print(analogRead(X_pin));
  //Serial.print('|');
  //Serial.print(analogRead(Y_pin));
  //Serial.print('|');
  //Serial.print(1455 - 0.7107 * analogRead(X_pin) - 0.7107 * analogRead(Y_pin));
  //Serial.print('|');
  //Serial.println(0.7107 * analogRead(X2_pin) - 0.7107 * analogRead(Y2_pin) + 1456/2);
  Mirf.send(cmd);

  //Serial.println("Sent:");
  //Serial.print(cmd[0]);
  //Serial.print('|');
  //Serial.print(cmd[1]);
  //Serial.print('|');
  //Serial.print(cmd[2]);
  //Serial.print('|');
  //Serial.print(cmd[3]);
  //Serial.print('|');
  //Serial.print(cmd[4]);
  //Serial.print('|');
  //Serial.print(cmd[5]);
  //Serial.print('|');
  //Serial.print(cmd[6]);
  //Serial.print('|');
  //Serial.print(cmd[7]);
  //Serial.print('|');
  //Serial.println("--------------------");
  //Serial.print(getJoystickY(cmd));
  //Serial.print('|');
  //Serial.println(getJoystickX(cmd));
  
 
  while(Mirf.isSending()){
  }
  //Serial.println("Finished sending");
  

  //delay(10);
  //while(!Mirf.dataReady()){
  //  Serial.println("Waiting");
  //  if ( ( millis() - time ) > 1000 ) {
  //    Serial.println("Timeout on response from server!");
  //    return;
  //  }
  //}
 
  //Mirf.getData(cmd2);

  //Serial.println("Got packet");
  //Serial.print(cmd2[0]);
  //Serial.print('|');
  //Serial.print(cmd2[1]);
  //Serial.print('|');
  //Serial.print(cmd2[2]);
  //Serial.print('|');
  //Serial.print(cmd2[3]);
  //Serial.print('|');
  //Serial.print(cmd2[4]);
  //Serial.print('|');
  //Serial.print(cmd2[5]);
  //Serial.print('|');
  //Serial.print(cmd2[6]);
  //Serial.print('|');
  //Serial.print(cmd2[7]);
  //Serial.print('|');
  //Serial.println("--------------------");
  //Serial.println(getJoystickY(cmd2));
  //Serial.println(getJoystickX(cmd2));
 
  delay(TRANSMITTING_DELAY);
} 

void setJoystickY (int yValue, int minVal, int maxVal) {
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

void setJoystickX (int xValue, int minVal, int maxVal) {
  int xValueMapped = map(xValue, minVal, maxVal,  -100, 100) + JOYSTICK_TRANSMITTING_OFFSET;

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

int getJoystickY (byte data[8]) {    
  return (data[4] - 48) * 100 + (data[5] - 48) * 10 + (data[6] - 48) - JOYSTICK_TRANSMITTING_OFFSET;   // obtain the Int from the ASCII representation
}

int getJoystickX (byte data[8]) {    
  return (data[1] - 48) * 100 + (data[2] - 48) * 10 + (data[3] - 48) - JOYSTICK_TRANSMITTING_OFFSET;   // obtain the Int from the ASCII representation
}

void debugSticks () {
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
    