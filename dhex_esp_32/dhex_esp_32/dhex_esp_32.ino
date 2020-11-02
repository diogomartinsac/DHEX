//    DHEX: Two wheeled robot
//    Copyright (C) 2020 Renato Magnelli <renatomagnelli@gmail.com>
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "esp32-hal-ledc.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <WiFi.h>

// Model and actuator parameters
#define WHEEL_RADIUS   32.5
#define WHEEL_DISTANCE 106
#define ENCODER_SLITS  20
#define ENCODER_PERIOD 50*portTICK_PERIOD_MS
#define PWM_FREQUENCY  2000
#define PWM_RESOLUTION 8

// Ports configuration
#define ENCODER_L_PIN 34
#define ENCODER_R_PIN 35
#define MOTOR_AIA_PIN 25
#define MOTOR_AIB_PIN 26
#define MOTOR_BIA_PIN 32
#define MOTOR_BIB_PIN 33

// Network configuration
IPAddress server(192, 168, 1, 20);
const uint16_t serverPort = 11411;
const char* password = "senhadanet1";
const char* ssid     = "AP1601";

// ROS variables for easy access
uint32_t left_sensor, right_sensor;
int16_t  left_motor , right_motor ;

// ROS Publisher & Subscriber
ros::NodeHandle nh;
std_msgs::Float32 vl_msg;
std_msgs::Float32 vr_msg;
std_msgs::Int16   ml_msg;
std_msgs::Int16   mr_msg;
ros::Publisher vl_chatter("dhex/vl", &vl_msg);
ros::Publisher vr_chatter("dhex/vr", &vr_msg);
void  updateLeftMotor(const std_msgs::Int16&);
void updateRightMotor(const std_msgs::Int16&);
ros::Subscriber<std_msgs::Int16> ml_chatter("dhex/l_motor", updateLeftMotor );
ros::Subscriber<std_msgs::Int16> mr_chatter("dhex/r_motor", updateRightMotor);

// Subscriber's callback that updates left motor PWM
void updateLeftMotor(const std_msgs::Int16& msg){
      left_motor = msg.data%256;
      if(left_motor < 0){
            ledcWrite(0, 0);
            ledcWrite(1, abs(left_motor));
      } else {
            ledcWrite(0, abs(left_motor));
            ledcWrite(1, 0);
      }
}

// Subscriber's callback that updates right motor PWM
void updateRightMotor(const std_msgs::Int16& msg){
      right_motor = msg.data%256;
      if(right_motor < 0){
            ledcWrite(2, 0);
            ledcWrite(3, abs(right_motor));
      } else {
            ledcWrite(2, abs(right_motor));
            ledcWrite(3, 0);
      }
}

// Interruption counter of the left encoder
void IRAM_ATTR pulseLeftSensor() {
      left_sensor  += ((left_motor  > 0) ? 1 : -1);
}

// Interruption counter of the right encoder
void IRAM_ATTR pulseRightSensor() {
      right_sensor += ((right_motor > 0) ? 1 : -1);
}

// Distance per 500 ms (ENCODER_PERIOD) [mm/s]
void evaluateSpeed(void * pvParameters){
      uint32_t milestone_left, milestone_right;
      while(true){
            milestone_left  =  left_sensor;
            milestone_right = right_sensor;
            vTaskDelay(ENCODER_PERIOD);
            vl_msg.data = (( left_sensor - milestone_left ) * WHEEL_RADIUS/ENCODER_SLITS)
                        * 1000/ENCODER_PERIOD;
            vr_msg.data = ((right_sensor - milestone_right) * WHEEL_RADIUS/ENCODER_SLITS)
                        * 1000/ENCODER_PERIOD;
      }
}

void setup() {
      delay(100);
      Serial.begin(9600);
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.println("Connecting to WiFi..");
      }
      Serial.println("Connected to the WiFi network");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      delay(500);

      ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
      ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION);
      ledcSetup(2, PWM_FREQUENCY, PWM_RESOLUTION);
      ledcSetup(3, PWM_FREQUENCY, PWM_RESOLUTION);
      pinMode(MOTOR_AIA_PIN, OUTPUT);
      pinMode(MOTOR_AIB_PIN, OUTPUT);
      pinMode(MOTOR_BIA_PIN, OUTPUT);
      pinMode(MOTOR_BIB_PIN, OUTPUT);
      pinMode(ENCODER_L_PIN,  INPUT); 
      pinMode(ENCODER_R_PIN,  INPUT); 
      ledcAttachPin(MOTOR_AIA_PIN, 0);
      ledcAttachPin(MOTOR_AIB_PIN, 1);
      ledcAttachPin(MOTOR_BIA_PIN, 2);
      ledcAttachPin(MOTOR_BIB_PIN, 3);

      nh.getHardware()->setConnection(server, serverPort);
      nh.initNode();
      nh.advertise(vl_chatter);
      nh.advertise(vr_chatter);
      nh.subscribe(ml_chatter);
      nh.subscribe(mr_chatter);
      delay(100);

      attachInterrupt(ENCODER_L_PIN, pulseLeftSensor , RISING);
      attachInterrupt(ENCODER_R_PIN, pulseRightSensor, RISING);
      TaskHandle_t xHandle = NULL;
      xTaskCreate(evaluateSpeed, "evaluateSpeed", 20000, ( void * ) 1, tskIDLE_PRIORITY, &xHandle);
}

void loop() {
      if(nh.connected()){
            Serial.println("Connected");
            vl_chatter.publish(&vl_msg);
            vr_chatter.publish(&vr_msg);
      } else {
            Serial.println("Not Connected");
      }
      nh.spinOnce();
      delay(1000);
}

/*  STATE SPACE EQUATIONS FOR TWO WHEELED ROBOT
 *  xdot = cos teta v
 *  ydot = sin teta v
 *  tetadot = omega
 *  v = (vr + vl) / 2
 *  w = (vr + vl) / L
 *  
 *  v = R/2  R/2 wr
 *  w   R/L -R/L wl
 *  
 *  x      = R/2 cos teta   R/2 cos teta   wr
 *  y        R/2 sin teta   R/2 sin teta   wl
 *  teta         R/L           -R/L
 *  
 */
 
