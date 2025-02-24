/*  Hand Sensor Data Collect and Send Code
    by Ralph Lejano and Edcel Abanto (G13)
    
    The purpose of this code is to consolidate data values from different sensors on a glove.
    Five flex sensors and two MPU6050s are attached on a glove to detect finger and wrist flexion.
    Raw data from flex sensors and accelerometers are converted into smaller values and consolidated 
    into one line of data labelled as a gesture.
*/

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BusIO_Register.h>
#include <MPU6050.h>
#include <Wire.h>
#include "Glove.h"
#include <vector>
#include <string>

#include <stdio.h>
#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h> 
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/publisher.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/u_int8.h>

// Code for MPU6050 sensor was referenced from the following repository: https://github.com/ElectronicCats/mpu6050/tree/master/examples
// NOTE: accelerometer data values end up as estimates, if we were to leave the MPU6050 in a fixed position, we expect the outputs to be 0,0 and 9.8ms2 but it is not.
// Flex sensor code was made using simple ADC capture and data adjustments to accomodate for resistance variability
// Important things to note: might have to change the gesture signatures into non-string data to conserve flash space
// Communication code must be eventually integrated into this code unless we decided on using separate esp32s for the functions

//                 calibration
//                      |
//                      v
//                    idle
//                      |
//                      v
//              connect to drones
//                      |
//                      v
//                select drone        (maybe we can sign a certain gesture to select one or the other or both)
//            |         ^         |
//            v         |         v
//        ctrl drone1   |     ctrl drone2
//          |           |         |
//          |-----------|---------| (sent through a specific gesture or set of gestures)
//          v                     v
//     cmd sent (auto)        cmd sent (auto)
//          |                     |
//          V                     v
//        drone 1               drone 2

// Declare MPU6050 objects for the hand
MPU6050 handMPU(0x68);
int16_t handAx, handAy, handAz;

// Declare sensors for the fingers and hand orientation
FlexSensor tmbFing(36); 
FlexSensor indFing(39);
FlexSensor midFing(34);
FlexSensor rngFing(35);
FlexSensor pnkFing(32);
tiltSensor handTilt;

uint8_t dataPackage;       //THIS IS THE DATA PACKAGE THAT WILL BE SENT TO THE COMPUTER

/* Hotspot */
char* ssid = "edcel";
char* pass = "edcel1234";
char* agent_ip = "172.20.10.9";

char* agent_port = "8888";

// uROS Variables
rcl_allocator_t allocator;
rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

//  Messages
std_msgs__msg__UInt8 msg;

//  ROS2 Topic Message for Publishing
rcl_publisher_t publisher;

//  Timer callback function
void publish_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    if (timer == NULL) return;

    //  Fill the message
    msg.data = dataPackage;

    //  Publish the message 
    if (rcl_publish(&publisher, &msg, NULL) == RCL_RET_OK)
    {
      Serial.println("Published!");
    }
    else 
    {
      Serial.println("Failed to Publish! :(");
    }
    
}

void setup(void) {
  Serial.begin(115200);
  Wire.begin();

  handMPU.initialize();
  if(handMPU.testConnection() == false){
    Serial.println("MPU Test for Hand Failed");
  }
  else{
    Serial.println("MPU Test for Hand Successful");
  }

    //  Initialize micro-ROS transport
    set_microros_wifi_transports(ssid, pass, agent_ip, atoi(agent_port));

    //  Ping the microROS Agent 
    Serial.print("Pinging micro-ROS Agent... ");
    if (rmw_uros_ping_agent(2000, 10)) 
    {  // Wait up to 2 seconds, try 10 times
        Serial.println("Connected to micro-ROS Agent");
    } else 
    {
        Serial.println("Failed to connect to micro-ROS Agent");
    }

    //  Initialize micro-ROS
    allocator = rcl_get_default_allocator();

    //  Initialize RCL init options 
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
    if (ret != RCL_RET_OK)
    {
        Serial.println("Failed to initialize RCL init options");
        return;
    }
    else
    {
        Serial.println("RCL init options initialized.");  
    }

    //  Set ROS DOMAIN ID Option to 
    size_t domain_id = 42;
    ret = rcl_init_options_set_domain_id(&init_options, domain_id);
    if (ret != RCL_RET_OK)
    {
        Serial.println("\nFailed to set domain ID");
        return;
    }
    else
    {
        Serial.println("ROS_DOMAIN_ID Set.");  
    }
    
    // Initialize micro-ROS support with default options
    ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    if (ret != RCL_RET_OK) 
    {
        Serial.printf("\nFailed to initialize micro-ROS support, error %d\n", ret);
        return;
    }
    else
    {
        Serial.println("micro-ROS support initialized.");  
    }

    //  Node Creation
    ret = rclc_node_init_default(&node, "esp32_publisher_node", "", &support);
    if (ret != RCL_RET_OK)
    {
        Serial.printf("\nFailed to initialize node, error: %d\n", ret);  
        return;
    }
    else 
    {
        Serial.println("Node Created.");
    }

    //  Create a publisher 
    rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
      "/esp32_glove"
    );

    //  Create a timer (1 second period)
    rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(1000),
      publish_callback
    );

    //  Initialize executor
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    //  Message Initialization
    msg.data = NULL;
    
    Serial.println("microROS setup complete!");
}

void loop() {
  // Read raw flex sensor data from ADCs
  tmbFing.updateRaw();
  indFing.updateRaw();
  midFing.updateRaw();
  rngFing.updateRaw();
  pnkFing.updateRaw();

  tmbFing.adjustScale(0, 100);
  indFing.adjustScale(0, 100);
  midFing.adjustScale(0, 100);
  rngFing.adjustScale(0, 100);
  pnkFing.adjustScale(0, 100);

  // Get the acceleration values from the MPU6050
  handMPU.getAcceleration(&handAx, &handAy, &handAz);
  handTilt.setAccelValues(handAx,handAy,handAz);

  dataPackage = 0;
  //Package the data to be sent
  dataPackage = dataPackage + (indFing.flexCheck() << 7);
  dataPackage = dataPackage + (midFing.flexCheck() << 6);
  dataPackage = dataPackage + (rngFing.flexCheck() << 5);
  dataPackage = dataPackage + (pnkFing.flexCheck() << 4);
  dataPackage = dataPackage + (tmbFing.flexCheck() << 3);
  dataPackage = dataPackage + handTilt.getOrientation();

  Serial.println("Spinning...");
  //  Spin Executor
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  //  Delay to control publishing rate
  delay(10); 
}