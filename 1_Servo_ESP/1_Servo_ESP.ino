#include "ESP32_Servo.h"
#include <WiFi.h>
#include <WiFiUDP.h>
#include "LedControl.h"
/*
#include "XT_DAC_Audio.h"
#include "data/ominous.h"
#include "data/war_x.h"
*/

#define LED_BUILTIN 2

// WiFi network name and password:
char *networkName = "Teletubbies";
char *networkPswd = "NooNooNoo";

//declare WiFiUDP object
WiFiUDP UDPServer;

//set local and target IP addresses and ports
unsigned int carUDPPort = 2401;
unsigned int joystickUDPPort = 2400;
unsigned int servoUDPPort = 2402;
IPAddress carIPAddress(192, 168, 1, 184);
IPAddress joystickIPAddress(192, 168, 1, 176);
IPAddress servoIPAddress(192, 168, 1, 147);
int BAUD_RATE = 115200;

//declare variables to hold incoming and outgoing packets
const int packetSize = 100;
char packetIn[packetSize];
char packetOut[packetSize];

const int pin_hammer_base = 23;
const int pin_hammer_arm = 22;

Servo servo_hammer_base;
Servo servo_hammer_arm;

//LED MATRIX STUFF++++++++++++++++++++++++++++++++++++++++
#define NUMBER_OF_DEVICES 4 //number of led matrix connect in series
#define CS_PIN 15
#define CLK_PIN 14
#define MISO_PIN 2 //we do not use this pin just fill to match constructor
#define MOSI_PIN 12
LedControl lc = LedControl(MOSI_PIN, CLK_PIN, CS_PIN, NUMBER_OF_DEVICES);
int delayTime = 50;

// happy face
byte hf[8]= {B00111100,B01000010,B10100101,B10000001,B10100101,B10011001,B01000010,B00111100};
// neutral face
byte nf[8]={B00111100, B01000010,B10100101,B10000001,B10111101,B10000001,B01000010,B00111100};
// sad face
byte sf[8]= {B00111100,B01000010,B10100101,B10000001,B10011001,B10100101,B01000010,B00111100};
//END LED MATRIX STUFF++++++++++++++++++++++++++++++++++++++++

/*
//SPEAKER STUFF++++++++++++++++++++++++++++++++++++++++
const int pin_audio = 26;
XT_DAC_Audio_Class DacAudio(pin_audio, 0);
XT_Wav_Class ominousAudio(ominous);
XT_Wav_Class war_xAudio(war_x);
//END SPEAKER STUFF++++++++++++++++++++++++++++++++++++++++
*/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);

  initOutputs();

  //Connect to the WiFi network with set local IP address & port
  WiFi.begin(networkName, networkPswd);
  WiFi.config(servoIPAddress, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  UDPServer.begin(servoUDPPort);

  while (WiFi.status() != WL_CONNECTED) { //wait to connect to WiFi
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  digitalWrite(LED_BUILTIN, LOW);

  //use as station
  WiFi.mode(WIFI_STA);

  servo_hammer_base.attach(pin_hammer_base, 900, 2100);
  servo_hammer_arm.attach(pin_hammer_arm, 900, 2100);

  //LED MATRIX STUFF+++++++++++++++++++++++++++++++++++
  //we have already set the number of devices when we created the LedControl
  int devices=lc.getDeviceCount();
  //we have to init all devices in a loop
  for(int address=0;address<devices;address++) {
    /*The MAX72XX is in power-saving mode on startup*/
    lc.shutdown(address,false);
    /* Set the brightness to a medium values */
    lc.setIntensity(address,8);
    /* and clear the display */
    lc.clearDisplay(address);
  }
  //END LED MATRIX STUFF+++++++++++++++++++++++++++++++++++

/*
  //SPEAKER STUFF+++++++++++++++++++++++++++++++++++
  ominousAudio.RepeatForever = false;
  war_xAudio.RepeatForever = false;
  ominousAudio.RepeatIdx = 1;
  war_xAudio.RepeatIdx = 1;
  //END SPEAKER STUFF+++++++++++++++++++++++++++++++++++
  */
}

void loop() {
  bool is_hammer_hit = false; 
  int noBytes = UDPServer.parsePacket();
  if ( noBytes ) {
    Serial.print("Packet received:  ");
    Serial.print(noBytes);

    UDPServer.read(packetIn, packetSize);
    Serial.println();

    int hammer_base_angle = packetIn[0];
    hammer_base_angle  = map(hammer_base_angle, 1, 255, 0, 180);
    int hammer_arm_angle = packetIn[1];
    hammer_arm_angle = map(hammer_arm_angle, 1, 255, 0, 180);
    
    servo_hammer_base.write(hammer_base_angle);
    servo_hammer_arm.write(hammer_arm_angle);

    if(hammer_arm_angle < 90) {
      is_hammer_hit = true;
    }
  }

  //LED MATRIX STUFF++++++++++++++++++++++++++++++++++++++++++++
  if(is_hammer_hit == false) {
      int devices=lc.getDeviceCount();
      for(int address=0;address<devices;address++) {
        showDrivingAnimation(address);
      }
  }
  else {
    int devices=lc.getDeviceCount();
      for(int address=0;address<devices;address++) {
        showAttackAnimation(address);
      }
  }
  delay(delayTime);
  //LED MATRIX STUFF++++++++++++++++++++++++++++++++++++++++++++
}

void initOutputs() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pin_hammer_base, OUTPUT);
  pinMode(pin_hammer_arm, OUTPUT);

  return;
}

void showDrivingAnimation(int addr) {
  // Display happy face
  lc.setRow(addr,0,hf[0]);
  lc.setRow(addr,1,hf[1]);
  lc.setRow(addr,2,hf[2]);
  lc.setRow(addr,3,hf[3]);
  lc.setRow(addr,4,hf[4]);
  lc.setRow(addr,5,hf[5]);
  lc.setRow(addr,6,hf[6]);
  lc.setRow(addr,7,hf[7]);
  return;
}

void showAttackAnimation(int addr) {
  // Display sad face
  lc.setRow(addr,0,sf[0]);
  lc.setRow(addr,1,sf[1]);
  lc.setRow(addr,2,sf[2]);
  lc.setRow(addr,3,sf[3]);
  lc.setRow(addr,4,sf[4]);
  lc.setRow(addr,5,sf[5]);
  lc.setRow(addr,6,sf[6]);
  lc.setRow(addr,7,sf[7]);
  
  return;
}
