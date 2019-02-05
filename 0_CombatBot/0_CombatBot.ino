#include <stdio.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "ESP32_Servo.h"

//LED STUFF++++++++++++++++++++++++++++++++++++++++
#include "FastLED.h"
FASTLED_USING_NAMESPACE

#define ROBOTNUM 2            // robot number
#define RED 0xFF0000          // color for the red team
#define BLUE 0x0000FF         // color for the blue team
#define HEALTHCOLOR 0x00FF00  // color for the health LEDs
#define TEAMCOLOR BLUE        // robot team
#define FLASHHALFPERIOD 250   // the blue team is supposed to flash this is half of the period of that flash
#define READPERIOD 400        // Slows down the I2C
#define WHITE 0xFFFFFF        // white color for healing
#define HALFWHITE 0x7F7F7F    // half white for healing 

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif
//END LED STUFF++++++++++++++++++++++++++++++++++++++++

// I2C STUFF===========================================
#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 128           /*!< Data buffer length of test buffer */
#define W_LENGTH 1                /*!< Data length for w, [0,DATA_LENGTH] */
#define R_LENGTH 16               /*!< Data length for r, [0,DATA_LENGTH] */


#define I2C_MASTER_SCL_IO (gpio_num_t)33             /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO (gpio_num_t)25               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(1) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 10000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define CONFIG_I2C_SLAVE_ADDRESS 0x28
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL I2C_MASTER_ACK                             /*!< I2C ack value */
#define NACK_VAL I2C_MASTER_NACK                           /*!< I2C nack value */

// END I2C STUFF===========================================


//LED STUFF++++++++++++++++++++++++++++++++++++++++
#define DATA_PIN    5  //What pin is the LED ring data on
#define LED_TYPE    WS2812  //APA102
#define COLOR_ORDER GRB  // changes the order so we can use standard RGB for the values we set.
#define NUM_LEDS    24  //Number of LEDs in the ring
CRGB leds[NUM_LEDS];  // this is the place you set the value of the LEDs each LED is 24 bits

#define BRIGHTNESS          60   // lower the brighness a bit so it doesn't look blown out on the camera.
#define FRAMES_PER_SECOND  120   // some number this is likely faster than needed

// -- The core to run FastLED.show()
#define FASTLED_SHOW_CORE 0

// -- Task handles for use in the notifications
static TaskHandle_t FastLEDshowTaskHandle = 0;
static TaskHandle_t userTaskHandle = 0;
//END LED STUFF++++++++++++++++++++++++++++++++++++++++

//WIFI STUFF++++++++++++++++++++++++++++++++++++++++
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
//END WIFI STUFF++++++++++++++++++++++++++++++++++++++++

//MOTOR STUFF++++++++++++++++++++++++++++++++++++++++
//define motor variables
int motor1_pwm = 0;
int motor2_pwm = 0;
int motor1_dir1 = 0;
int motor1_dir2 = 0;
int motor2_dir1 = 0;
int motor2_dir2 = 0;

const int pin_motor1_pwm = 23;
const int pin_motor2_pwm = 22;
const int pin_motor1_dir1 = 19;
const int pin_motor1_dir2 = 18;
const int pin_motor2_dir1 = 17;
const int pin_motor2_dir2 = 16;

//define thresholds to remove noise from pot readings
const int drive_thresh = 50;
const int drive_thresh_forward = drive_thresh;
const int drive_thresh_backward = -drive_thresh;
const int turn_thresh = 25;
const int dir_thresh_right = turn_thresh;
const int dir_thresh_left = -turn_thresh;

//direction pins values, caster wheel on left, motor 2 nearer to you, forward is anti-clockwise rotation, backward is clockwise rotation
const byte bw_dir1 = LOW;
const byte bw_dir2 = HIGH;
const byte fw_dir1 = HIGH;
const byte fw_dir2 = LOW;
const int is_turn_linear = 1;
//END MOTOR STUFF++++++++++++++++++++++++++++++++++++++++d

//AUTONOMOUS STUFF++++++++++++++++++++++++++++++++++++++++
//define echo and trigger pins for ultrasonic sensor
const int pin_echo = 35;
const int pin_trigger = 4;
int auto_distance = 15; //centimeters
int reverse_vel = -255;
int reverse_time = 500; //milliseconds
Servo servo_ussensor;
const int pin_servo_ussensor = 13;
int straight_angle = 90;
int left_angle = 45;
int right_angle = 135;
//END AUTONOMOUS STUFF++++++++++++++++++++++++++++++++++++++++

//LEDC STUFF++++++++++++++++++++++++++++++++++++++++
//define ledc variables
const int freq = 1200;
const int led1Channel = 0;
const int led2Channel = 1;
const int resolution = 8;
//END LEDC STUFF++++++++++++++++++++++++++++++++++++++++

//HEALING STUFF++++++++++++++++++++++++++++++++++++++++
const int pin_light_sensor = 32;
int light_threshold = 50;
unsigned long duration = 0;
volatile int heal_intensity_flag = 0;
//END HEALING STUFF++++++++++++++++++++++++++++++++++++++++

//MISCELLANEOUS STUFF++++++++++++++++++++++++++++++++++++++++
#define LED_BUILTIN 2
//END MISCELLANEOUS STUFF++++++++++++++++++++++++++++++++++++++++

// I2C STUFF===========================================
/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t nsize)
{
    if (nsize == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN); 
    if (nsize > 1) {
        i2c_master_read(cmd, data_rd, nsize - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + nsize - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS); // send all queued commands
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t nsize)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, nsize, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        Serial.printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            Serial.printf("\n");
        }
    }
    Serial.printf("\n");
}

uint8_t data_wr[DATA_LENGTH];
uint8_t data_rd[DATA_LENGTH];

static void i2c_read_test()
{
  int ret;

  ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, DATA_LENGTH);

  if (ret == ESP_ERR_TIMEOUT) {
    ESP_LOGE(TAG, "I2C Timeout");
    Serial.println("I2C Timeout");
  } else if (ret == ESP_OK) {
    Serial.printf(" MASTER READ FROM SLAVE ******\n");
    disp_buf(data_rd, DATA_LENGTH);
    digitalWrite(LED_BUILTIN,LOW);
  } else {
    ESP_LOGW(TAG, " %s: Master read slave error, IO not connected...\n",
             esp_err_to_name(ret));
  }
}

static void i2c_write_test()
{ 
  int ret;
                                                                             
  ret = i2c_master_write_slave(I2C_MASTER_NUM, data_wr, W_LENGTH);
  if (ret == ESP_ERR_TIMEOUT) {
    ESP_LOGE(TAG, "I2C Timeout");
  } else if (ret == ESP_OK) {
    Serial.printf(" MASTER WRITE TO SLAVE\n");
    disp_buf(data_wr, W_LENGTH);
  } else {
    ESP_LOGW(TAG, "%s: Master write slave error, IO not connected....\n",
            esp_err_to_name(ret));
  }
}

// END I2C STUFF===========================================



//LED STUFF++++++++++++++++++++++++++++++++++++++++
/** show() for ESP32
 *  Call this function instead of FastLED.show(). It signals core 0 to issue a show, 
 *  then waits for a notification that it is done.
 */
void FastLEDshowESP32()
{
    if (userTaskHandle == 0) {
        // -- Store the handle of the current task, so that the show task can
        //    notify it when it's done
        userTaskHandle = xTaskGetCurrentTaskHandle();

        // -- Trigger the show task
        xTaskNotifyGive(FastLEDshowTaskHandle);

        // -- Wait to be notified that it's done
        const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 200 );
        ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
        userTaskHandle = 0;
    }
}

/** show Task
 *  This function runs on core 0 and just waits for requests to call FastLED.show()
 */
void FastLEDshowTask(void *pvParameters)
{
    // -- Run forever...
    for(;;) {
        // -- Wait for the trigger
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // -- Do the show (synchronously)
        FastLED.show();

        // -- Notify the calling task
        xTaskNotifyGive(userTaskHandle);
    }
}

void SetupFastLED(void){
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  int core = xPortGetCoreID();
    Serial.print("Main code running on core ");
    Serial.println(core);

    // -- Create the FastLED show task
    xTaskCreatePinnedToCore(FastLEDshowTask, "FastLEDshowTask", 2048, NULL, 2, &FastLEDshowTaskHandle, FASTLED_SHOW_CORE);

}

void ShowRobotNum(void){
  int flashTime = millis(); // what time is it
  static int flashTimeOld = 0; // when was the last toggle
  static bool ledsOn = 1;  // are the robot number LEDs on
  int robotLeds[] = {0,6,12,18};  // location of the LEDs used to display the robot number
//  static int RONUM = 0;
//  RONUM;
  
  if (TEAMCOLOR == BLUE){ // if the team is blue you need to flash the LEDs
    if ((flashTime-FLASHHALFPERIOD) > flashTimeOld){ // if the correct amount of time has passed toggle the robot number LEDs
//      Serial.print("changing LED state from: ");
//      Serial.println(ledsOn);
//      Serial.println();
      if (ledsOn){  // if they are on turn them off
        ledsOn = 0;
      }
      else {  // if they are off turn them on
        ledsOn = 1; 
      }
      flashTimeOld = flashTime;   //store when we changed the state
    }
  }
  
  leds[robotLeds[0]]=TEAMCOLOR*ledsOn;  // The first LED is always displayed with the robot number

  switch (ROBOTNUM){  //Change the LEDs based on the robot number
  case 1:
    leds[robotLeds[1]]=0;
    leds[robotLeds[2]]=0;
    leds[robotLeds[3]]=0;
    break;
  case 2:
    leds[robotLeds[1]]=0;
    leds[robotLeds[2]]=TEAMCOLOR*ledsOn;
    leds[robotLeds[3]]=0;
    break;
  case 3:
    leds[robotLeds[1]]=TEAMCOLOR*ledsOn;
    leds[robotLeds[2]]=0;
    leds[robotLeds[3]]=TEAMCOLOR*ledsOn;
    break;
  case 4:
    leds[robotLeds[1]]=TEAMCOLOR*ledsOn;
    leds[robotLeds[2]]=TEAMCOLOR*ledsOn;
    leds[robotLeds[3]]=TEAMCOLOR*ledsOn;
    //RONUM = 1;
    break;
  }
}

void ShowHealth(int health){
  int healthLeds[] = {1,2,3,4,5,7,8,9,10,11,13,14,15,16,17,19,20,21,22,23}; // the location of the 24 LEDs used for health
  
  leds[healthLeds[0]] = HEALTHCOLOR*(health > 0);  // last LED doesn't go off till the health is 0
  leds[healthLeds[19]] = HEALTHCOLOR*(health == 100);  // first LED goes off as soon as the health is not 100

  for(int i=1; i<19; i++){
    leds[healthLeds[i]] = HEALTHCOLOR*(health > (i*5));  // the other leds go off in increments of 5
  } 
}

void clearLEDs(void){
  for(int i=0; i<NUM_LEDS; i++){
    leds[i] = 0; // Turn off everything 
  }
}
// END LED STUFF++++++++++++++++++++++++++++++++++++++++

//INITIALIZATION STUFF+++++++++++++++++++++++++++++++++++
void initPins() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pin_motor1_pwm, OUTPUT);
  pinMode(pin_motor2_pwm, OUTPUT);
  pinMode(pin_motor1_dir1, OUTPUT);
  pinMode(pin_motor1_dir2, OUTPUT);
  pinMode(pin_motor2_dir1, OUTPUT);
  pinMode(pin_motor2_dir2, OUTPUT);
  pinMode(pin_echo, INPUT);
  pinMode(pin_trigger, OUTPUT);
  pinMode(pin_servo_ussensor, OUTPUT);
  
  return;
}

//END INITIALIZATION STUFF+++++++++++++++++++++++++++++++++++

//MOTOR STUFF+++++++++++++++++++++++++++++++++++
void drive(int velocity) {
  if (velocity > drive_thresh_forward) {
    //go forward
    motor1_pwm = abs(velocity);
    motor2_pwm = abs(velocity);

    digitalWrite(pin_motor1_dir1, fw_dir1);
    digitalWrite(pin_motor1_dir2, fw_dir2);
    ledcWrite(led1Channel, motor1_pwm);

    digitalWrite(pin_motor2_dir1, fw_dir1);
    digitalWrite(pin_motor2_dir2, fw_dir2);
    ledcWrite(led2Channel, motor2_pwm);
  }
  else if (velocity < drive_thresh_backward) {
    //go backward
    motor1_pwm = abs(velocity);
    motor2_pwm = abs(velocity);

    digitalWrite(pin_motor1_dir1, bw_dir1);
    digitalWrite(pin_motor1_dir2, bw_dir2);
    ledcWrite(led1Channel, motor1_pwm);

    digitalWrite(pin_motor2_dir1, bw_dir1);
    digitalWrite(pin_motor2_dir2, bw_dir2);
    ledcWrite(led2Channel, motor2_pwm);
  }
  else {
    //stay right there
    motor1_pwm = 0;
    motor2_pwm = 0;
    ledcWrite(led1Channel, motor1_pwm);
    ledcWrite(led2Channel, motor2_pwm);
  }
}

void turn_left(int velocity, int pot_turn) {
  if (velocity > drive_thresh_forward) {
    //go forward left
    motor1_pwm = abs(velocity);
    motor2_pwm = is_turn_linear*(abs(velocity)*pot_turn/(3*turn_thresh) + 4*abs(velocity)/3);

    digitalWrite(pin_motor1_dir1, fw_dir1);
    digitalWrite(pin_motor1_dir2, fw_dir2);
    ledcWrite(led1Channel, motor1_pwm);

    digitalWrite(pin_motor2_dir1, fw_dir1);
    digitalWrite(pin_motor2_dir2, fw_dir2);
    ledcWrite(led2Channel, motor2_pwm);
  }
  else if (velocity < drive_thresh_backward) {
    //go backward left
    motor1_pwm = abs(velocity);
    motor2_pwm = is_turn_linear*(abs(velocity)*pot_turn/(3*turn_thresh) + 4*abs(velocity)/3);

    digitalWrite(pin_motor1_dir1, bw_dir1);
    digitalWrite(pin_motor1_dir2, bw_dir2);
    ledcWrite(led1Channel, motor1_pwm);

    digitalWrite(pin_motor2_dir1, bw_dir1);
    digitalWrite(pin_motor2_dir2, bw_dir2);
    ledcWrite(led2Channel, motor2_pwm);
  }
  else {
    //stay right there
    motor1_pwm = 0;
    motor2_pwm = 0;
    ledcWrite(led1Channel, motor1_pwm);
    ledcWrite(led2Channel, motor2_pwm);
  }
}

void turn_right(int velocity, int pot_turn) {
  if (velocity > drive_thresh_forward) {
    //go forward right
    motor1_pwm = is_turn_linear*(-abs(velocity)*pot_turn/(3*turn_thresh) + 4*abs(velocity)/3);
    motor2_pwm = abs(velocity);

    digitalWrite(pin_motor1_dir1, fw_dir1);
    digitalWrite(pin_motor1_dir2, fw_dir2);
    ledcWrite(led1Channel, motor1_pwm);

    digitalWrite(pin_motor2_dir1, fw_dir1);
    digitalWrite(pin_motor2_dir2, fw_dir2);
    ledcWrite(led2Channel, motor2_pwm);
  }
  else if (velocity < drive_thresh_backward) {
    //go backward right
    motor1_pwm = is_turn_linear*(-abs(velocity)*pot_turn/(3*turn_thresh) + 4*abs(velocity)/3);
    motor2_pwm = abs(velocity);
    
    digitalWrite(pin_motor1_dir1, bw_dir1);
    digitalWrite(pin_motor1_dir2, bw_dir2);
    ledcWrite(led1Channel, motor1_pwm);

    digitalWrite(pin_motor2_dir1, bw_dir1);
    digitalWrite(pin_motor2_dir2, bw_dir2);
    ledcWrite(led2Channel, motor2_pwm);
  }
  else {
    //stay right there
    motor1_pwm = 0;
    motor2_pwm = 0;
    ledcWrite(led1Channel, motor1_pwm);
    ledcWrite(led2Channel, motor2_pwm);
  }
}

void turn_manual(int pot_x, int pot_y) {
  if(pot_y >= 0 && pot_x >= 0) {
    motor1_pwm = -255*pot_x/205 + 255*255/205;
    motor2_pwm = pot_y;
    
    digitalWrite(pin_motor1_dir1, fw_dir1);
    digitalWrite(pin_motor1_dir2, fw_dir2);
    ledcWrite(led1Channel, motor1_pwm);

    digitalWrite(pin_motor2_dir1, fw_dir1);
    digitalWrite(pin_motor2_dir2, fw_dir2);
    ledcWrite(led2Channel, motor2_pwm);
  }
  else if(pot_y <= 0 && pot_x <= 0) {
    motor1_pwm = abs(pot_y);
    motor2_pwm = 255*pot_x/205 + 255*255/205;
  
    digitalWrite(pin_motor1_dir1, bw_dir1);
    digitalWrite(pin_motor1_dir2, bw_dir2);
    ledcWrite(led1Channel, motor1_pwm);

    digitalWrite(pin_motor2_dir1, bw_dir1);
    digitalWrite(pin_motor2_dir2, bw_dir2);
    ledcWrite(led2Channel, motor2_pwm);  
  }
  else if(pot_y >=0 && pot_x <=0) {
    motor1_pwm = pot_y;
    motor2_pwm = 255*pot_x/205 + 255*255/205;

    digitalWrite(pin_motor1_dir1, fw_dir1);
    digitalWrite(pin_motor1_dir2, fw_dir2);
    ledcWrite(led1Channel, motor1_pwm);

    digitalWrite(pin_motor2_dir1, fw_dir1);
    digitalWrite(pin_motor2_dir2, fw_dir2);
    ledcWrite(led2Channel, motor2_pwm);
  }
  else if(pot_y <=0 && pot_x >=0 ) {
    motor1_pwm = -255*pot_x/205 + 255*255/205;
    motor2_pwm = abs(pot_y);

    digitalWrite(pin_motor1_dir1, bw_dir1);
    digitalWrite(pin_motor1_dir2, bw_dir2);
    ledcWrite(led1Channel, motor1_pwm);

    digitalWrite(pin_motor2_dir1, bw_dir1);
    digitalWrite(pin_motor2_dir2, bw_dir2);
    ledcWrite(led2Channel, motor2_pwm);
  }
  
  return;
}
//END MOTOR STUFF+++++++++++++++++++++++++++++++++++

//AUTONOMOUS STUFF++++++++++++++++++++++++++++++++++++++++
int distance() {
  int duration, distance;
  digitalWrite(pin_trigger, HIGH);
  delayMicroseconds(1000);
  digitalWrite(pin_trigger, LOW);
  duration = pulseIn(pin_echo, HIGH);
  distance = (duration/2) / 29.1; //centimeters
  return distance;
}

void autobreak() {
  motor1_pwm = 0;
  motor2_pwm = 0;
  ledcWrite(led1Channel, motor1_pwm);
  ledcWrite(led2Channel, motor2_pwm);
  return;
}

void autoreverse(int velocity, int pot_turn, int duration) {
  //go backward left
  motor1_pwm = abs(velocity);
  motor2_pwm = is_turn_linear*(abs(velocity)*pot_turn/(3*turn_thresh) + 4*abs(velocity)/3);

  digitalWrite(pin_motor1_dir1, bw_dir1);
  digitalWrite(pin_motor1_dir2, bw_dir2);
  ledcWrite(led1Channel, motor1_pwm);

  digitalWrite(pin_motor2_dir1, bw_dir1);
  digitalWrite(pin_motor2_dir2, bw_dir2);
  ledcWrite(led2Channel, motor2_pwm);

  delay(duration);
  return;
}
//END AUTONOMOUS STUFF++++++++++++++++++++++++++++++++++++++++

//  SETUP###############################################
//  ####################################################
//  ####################################################
//  ####################################################
void setup() {
  //INITIALIZATION STUFF+++++++++++++++++++++++++++++++++++
  Serial.begin(115200);
  initPins();
  //END INITIALIZATION STUFF+++++++++++++++++++++++++++++++++++

  //WIFI STUFF+++++++++++++++++++++++++++++++++++
  //Connect to the WiFi network with set local IP address & port
  WiFi.begin(networkName, networkPswd);
  WiFi.config(carIPAddress, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  UDPServer.begin(carUDPPort);

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
  //END WIFI STUFF+++++++++++++++++++++++++++++++++++

  //AUTONOMOUS STUFF++++++++++++++++++++++++++++++++++++++++
  servo_ussensor.attach(pin_servo_ussensor);
  //END AUTONOMOUS STUFF++++++++++++++++++++++++++++++++++++++++

  //LEDC STUFF+++++++++++++++++++++++++++++++++++
  ledcSetup(led1Channel, freq, resolution);
  ledcAttachPin(pin_motor1_pwm, led1Channel);
  ledcSetup(led2Channel, freq, resolution);
  ledcAttachPin(pin_motor2_pwm, led2Channel);
  //END LEDC STUFF+++++++++++++++++++++++++++++++++++

  // I2C STUFF===========================================
  pinMode(LED_BUILTIN, OUTPUT);  // make the onboard LED an output

  ESP_ERROR_CHECK(i2c_master_init());  // Initialize the I2C
  // END I2C STUFF===========================================
  
  //LED STUFF++++++++++++++++++++++++++++++++++++++++
  SetupFastLED();  // Setup the LEDs
  //END LED STUFF++++++++++++++++++++++++++++++++++++++++
}


//  LOOP $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//  $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//  $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//  $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
void loop() {
  int currentTime = millis();  // Get the current time
  static int readTime = currentTime; // Timewhen we last read
  
  static bool gameStatus = 0;              // game on 1, game off 0
  static bool reset = 0;                   // 1 for reseting, not sure what the intention is here, check with Diego
  static bool autoMode = 0;                // 0 not autonomous, 1 is autonomous
   
  static bool syncStatus = 0;              // 0 sync still hasn't happend, 1 sync has happend
  static byte coolDownStatus = 0;          // 0 ready to hit, 1 cooling down  In the same order as the health (Red 1, 2, 3, 4, Blue 1, 2, 3, 4)

  static byte healthRobot[8];      // health of each robot as a byte (Red 1, 2, 3, 4, Blue 1, 2, 3, 4)
  static byte healthNexus[4];      // health of the two nexi each is 10 bit (Red L, H, Blue L, H)
  static byte towerStatus[2];      // status of the two towers.  Not sure why this needs 4 bits each

  static byte healingFreq = 0;  // First bit is for the low frequency (1), the second bit is for the high frequency (2), zero can be sent to the hat to request the information.

  // I2C STUFF===========================================
  if ((currentTime - READPERIOD) >= readTime){  //if we haven't read for the correct amount of time we can do it now.
      readTime=currentTime;  // update when we last read
      switch (healingFreq) {  //  This just cycles through the different information we can send, students should make it approriate to what they are sensing
        case 1:
          healingFreq = 1;  //the low frequency is present
          break;
        case 2:
          healingFreq = 2;  // the high freq. is present
          break;
        case 0:
          healingFreq = 0;  // no healing but data requested
          break;
      }
      data_wr[0]=healingFreq;  // put the healing information into the buffer
      i2c_write_test();       // write the buffer
      //delay(1);
      i2c_read_test();        // read the data only do this after a write or the slave buffer can fill up

      gameStatus = 1 & (data_rd[0]>>0);      // game on 1, game off 0
      reset = 1 & (data_rd[0]>>1);           // 1 for reseting, not sure what the intention is here, check with Diego
      autoMode = 1 & (data_rd[0]>>2);        // 0 not autonomous, 1 is autonomous
      syncStatus = 1 & (data_rd[0]>>3);      // 0 sync still hasn't happend, 1 sync has happened,  this makes sure the times each robots sends corresponds if this 
      

      if (0xFF != data_rd[6]){// if the robot health is FF something is wrong and disregard the incoming data
        coolDownStatus = data_rd[1];  // 0 ready to hit, 1 cooling down  in robot order red then blue
        
        healthNexus[0] = data_rd[2];
        healthNexus[1] = data_rd[3];
        healthNexus[2] = data_rd[4];
        healthNexus[3] = data_rd[5];
  
        healthRobot[0] = data_rd[6];
        healthRobot[1] = data_rd[7];
        healthRobot[2] = data_rd[8];
        healthRobot[3] = data_rd[9];
  
        healthRobot[4] = data_rd[10];
        healthRobot[5] = data_rd[11];
        healthRobot[6] = data_rd[12];
        healthRobot[7] = data_rd[13];
  
        towerStatus[1] = 0x0F & (data_rd[14]>>0);      // This can be cleaned up because you just need the and for the first one and the shift for the second but I like the consistency.
        towerStatus[2] = 0x0F & (data_rd[14]>>4);
      }
      else{  // blink to show something went wrong
        digitalWrite(LED_BUILTIN,LOW);
        delay(250);
        digitalWrite(LED_BUILTIN,HIGH);
      } 
  }

  if(gameStatus == 1) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  // END I2C STUFF===========================================
  
  //LED STUFF++++++++++++++++++++++++++++++++++++++++
  static int health;  // what this robots health is
  // send the 'leds' array out to the actual LED strip
  health = healthRobot[ROBOTNUM-1+(TEAMCOLOR == BLUE)*4];  // Get the position based on the robot number it is zero indexed so we need to lower everything by 1, if it is a blue robot we need to start after the read robots
  //health = 80; //only for test purposes
  ShowRobotNum();  // set the LEDs for the robot number
  ShowHealth(health); //set the LEDs for the health
  if (0 == health){  // If we are dead turn off the lights.
    clearLEDs();
  }
  Serial.print("health: "); 
  Serial.println(health);
  //END LED STUFF++++++++++++++++++++++++++++++++++++++++

  //DRIVE STUFF+++++++++++++++++++++++++++++++++++
  if(health == 0 || gameStatus == 0) {
    autobreak();
  }
  else {
    if(autoMode == 0) {
      //MANUAL MODE
      int noBytes = UDPServer.parsePacket();
      if ( noBytes ) {
        Serial.print("Packet received:  ");
        Serial.println(noBytes);
    
        UDPServer.read(packetIn, packetSize);
        //DRIVE WITH SLIDER STUFF++++++++++++++++++++++++++++
        /*
        int velocity = packetIn[0];
        velocity = map(velocity, 1, 255, -255, 255);
        int pot_turn = packetIn[1];
        pot_turn = map(pot_turn, 1, 255, -100, 100);
    
        if (pot_turn >= dir_thresh_left && pot_turn <= dir_thresh_right) {
          //go straight
          drive(velocity);
        }
        else if (pot_turn < dir_thresh_left) {
          //turn left
          turn_left(velocity, pot_turn);
        }
        else if (pot_turn > dir_thresh_right) {
          //turn right
          turn_right(velocity, pot_turn);
        }
        */
        //END DRIVE WITH SLIDER STUFF++++++++++++++++++++++++++++

        //DRIVE WITH JOYSTICK STUFF++++++++++++++++++++++++++++
        int pot_x = packetIn[0];
        pot_x = map(pot_x, 1, 255, -255, 255);
        int pot_y = packetIn[1];
        pot_y = map(pot_y, 1, 255, -255, 255);

        if(abs(pot_x) >= drive_thresh && abs(pot_y) >= drive_thresh) {
          turn_manual(pot_x, pot_y); 
        }
        else if(abs(pot_y) >=drive_thresh) {
          drive(pot_y);
        }
        else {
          autobreak();
        }
        //END DRIVE WITH JOYSTICK STUFF++++++++++++++++++++++++++++
      }
    }
    else {
      //AUTONOMOUS MODE
      //JUST DRIVE AROUND
      int distance_0 = distance();
      Serial.println(distance_0);
      if(distance_0 > auto_distance) {
        servo_ussensor.write(straight_angle);
        drive(150);
      }
      else {
        servo_ussensor.write(left_angle); //look left
        delay(500);
        int left_distance = distance(); //look right
        servo_ussensor.write(right_angle);
        delay(500);
        int right_distance = distance(); 
        servo_ussensor.write(straight_angle);

        if(left_distance < right_distance) { //turn right
          autoreverse(reverse_vel, -100, reverse_time);
        }
        else{ //turn left
          autoreverse(reverse_vel, 100, reverse_time);
        }
        drive(150); //start a little slow
      }
    }
  }
  //END DRIVE STUFF+++++++++++++++++++++++++++++++++++

  //HEALING STUFF+++++++++++++++++++++++++++++++++++++
  int flag = 0;
  long time_0 = 0;
  long time_stop = 0;
  int counter = 0;
  int healthLeds[] = {1,2,3,4,5,7,8,9,10,11,13,14,15,16,17,19,20,21,22,23}; // the location of the 24 LEDs used for health

  //heal if light above threshold intensity is detected
  if(analogRead(pin_light_sensor) > light_threshold && health < 100) {
    int time_start = xTaskGetTickCount();
    int time_end = 100;
    do {
      int adc = analogRead(pin_light_sensor);
    
      if(adc < 1000){
        if(flag == 0) {
          time_0 = micros();
          flag = 1;
        }
      } 
      else{
        if(flag == 1) {
          time_stop = micros();
          counter += 1;
          flag = 0;
          duration = time_stop - time_0;

          if(heal_intensity_flag = 0) {
            if(duration >= 2080 && duration <= 2270) { //230=4372
              for(int i=health/5; i<20; i++) {
                leds[healthLeds[i]] = WHITE; 
              }
              healingFreq = 1;
            }
            else if(duration >= 303 && duration <= 350) { //1600=625
              for(int i=health/5; i<20; i++) {
                leds[healthLeds[i]] = WHITE; 
              }
              healingFreq = 2; 
            }
            else {
              healingFreq = 0;
            } 
            heal_intensity_flag = 1;
          }
          else {
            if(duration >= 2080 && duration <= 2270) { //230=4372
              for(int i=health/5; i<20; i++) {
                leds[healthLeds[i]] = HALFWHITE; 
              }
              healingFreq = 1;
            }
            else if(duration >= 303 && duration <= 350) { //1600=625
              for(int i=health/5; i<20; i++) {
                leds[healthLeds[i]] = HALFWHITE; 
              }
              healingFreq = 2; 
            }
            else {
              healingFreq = 0;
            } 
            heal_intensity_flag = 0;
          }
        }
      }
    }while(((xTaskGetTickCount() - time_start) < time_end) && (counter < 5));
  }

  delay(FLASHHALFPERIOD/4);  // wait a bit so the LEDs don't cycle to fast
  FastLEDshowESP32(); //Actually send the values to the ring
  FastLED.delay(1000/FRAMES_PER_SECOND); // insert a delay to keep the framerate modest
}
