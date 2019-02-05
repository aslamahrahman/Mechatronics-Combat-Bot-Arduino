#include <WiFi.h>
#include <WiFiUDP.h>

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

//declare pins
#define LED_BUILTIN 2
const int pin_pot_x = 33;
const int pin_pot_y = 32;
const int sel = 35;
const int pin_hammer_base = 36;
const int pin_hammer_arm = 34;

//declare variables to hold incoming and outgoing packets
const int packetSize = 100;
char packetIn[packetSize];
char packetOut[packetSize];

void setup() {
  Serial.begin(BAUD_RATE);
  initPins();
  digitalWrite(sel, HIGH);
  
  //Connect to the WiFi network with set local IP address & port
  WiFi.begin(networkName, networkPswd);
  WiFi.config(joystickIPAddress, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  UDPServer.begin(joystickUDPPort);

  while(WiFi.status() != WL_CONNECTED) { //wait to connect to WiFi
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  digitalWrite(LED_BUILTIN, LOW);

  //use as station
  WiFi.mode(WIFI_STA);
}

void loop() {
  //read potentiometer inputs & map to a character
  int pot_drive = analogRead(pin_pot_x);
  int velocity = map(pot_drive, 0, 4095, 1, 255);
  int pot_turn = analogRead(pin_pot_y);
  int dir = map(pot_turn, 0, 4095, 1, 255);
  int select = digitalRead(sel);
  int hammer_base = analogRead(pin_hammer_base);
  int hammer_base_angle = map(hammer_base, 0, 4095, 1, 255);
  int hammer_arm = analogRead(pin_hammer_arm);
  int hammer_arm_angle = map(hammer_arm, 0, 4095, 1, 255);

  Serial.print(pot_drive);
  Serial.print("-");
  Serial.print(pot_turn);
  Serial.print("-");
  Serial.print(hammer_base);
  Serial.print("-");
  Serial.println(hammer_arm);

  //create packet for sending
  char out[packetSize];
  out[0] = velocity;
  out[1] = dir;
  out[2] = 0;
  strcpy(packetOut, out);
  UDPServer.beginPacket(carIPAddress, carUDPPort);
  UDPServer.printf("%s", packetOut);
  UDPServer.endPacket();

  //create packet for sending
  out[0] = hammer_base_angle;
  out[1] = hammer_arm_angle;
  out[2] = 0;
  strcpy(packetOut, out);
  UDPServer.beginPacket(servoIPAddress, servoUDPPort);
  UDPServer.printf("%s", packetOut);
  UDPServer.endPacket();
  
  delay(50);
}

void initPins() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pin_pot_x, INPUT);
  pinMode(pin_pot_y, INPUT);
  pinMode(sel, INPUT);
  pinMode(pin_hammer_base, INPUT);
  pinMode(pin_hammer_arm, INPUT);


  return;
}
