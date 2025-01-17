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
#include "Wire.h"
#include "Gesture.h"
#include <vector>
#include <string>

/* #include <stdio.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h> 
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/publisher.h>
#include <rclc/subscription.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h> */

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

MPU6050 handMPU(0x68);
MPU6050 foreMPU(0x69);

int16_t handAx, handAy, handAz;
int16_t foreAx, foreAy, foreAz;

tiltSensor handTilt;
tiltSensor foreTilt;

FlexSensor tmbFing(36); 
FlexSensor indFing(39);
FlexSensor midFing(34);
FlexSensor rngFing(35);
FlexSensor pnkFing(32);

gesture currGest;
gesture GestureSet[6];

float handRoll, handPitch;
float foreRoll, forePitch;

/*
/* Hotspot 
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
std_msgs__msg__Int32 incoming_msg;
std_msgs__msg__Int32MultiArray msg;

//  ROS 2 Topic Message for Subscribing
//rcl_subscription_t subscriber;

//  ROS2 Topic Message for Publishing
rcl_publisher_t publisher;

int32_t data_array[5] = {10, 20, 30, 40, 50};

//  Timer callback function
void publish_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    if (timer == NULL) return;

    //  Fill the message
    msg.data.data = data_array;
    msg.data.size = sizeof(data_array) / sizeof(data_array[0]);
    msg.data.capacity = msg.data.size;

    //  Publish the message 
    if (rcl_publish(&publisher, &msg, NULL) == RCL_RET_OK)
    {
      Serial.println("Array Published!");
    }
    else 
    {
      Serial.println("Failed to Publish! :(");
    }
    
}

*/

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

  foreMPU.initialize();
  if(foreMPU.testConnection() == false){
    Serial.println("MPU Test for Forearm Failed");
  }
  else{
    Serial.println("MPU Test for Forearm Successful");
  }


  //The Gesture set should be defined in Gesture.h
  // Preset Gestures for now, signature will be replaced with non-string in Gesture.h
  GestureSet[0].setGesture("agree", {1, -1, -1, -1, -1});
  GestureSet[1].setGesture("point", {-1, 1, -1, -1, -1});
  GestureSet[2].setGesture("promise", {-1, -1, -1, -1, 1});
  GestureSet[3].setGesture("peace", {-1, 1, 1, -1, -1});
  GestureSet[4].setGesture("okay", {0, 0, 1, 1, 1});
  GestureSet[5].setGesture("idle", {0, 0, 0, 0, 0});

/*     delay(1000);

    //  Attempt to connect to Wi-Fi
    WiFi.begin(ssid, pass);
    while(WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.println(".");
    }
    Serial.println("\nConnected");

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

    //  Print ESP32 Address
    Serial.println("ESP32 IP Address");
    Serial.println(WiFi.localIP());

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
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "esp32_glove"
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
    msg.data.data = NULL;
    msg.data.size = 0;
    msg.data.capacity = 0;
    
    Serial.println("microROS setup complete!");
} */
}

void loop() {
  // Read raw flex sensor data from ADCs
  tmbFing.updateRaw();
  indFing.updateRaw();
  midFing.updateRaw();
  rngFing.updateRaw();
  pnkFing.updateRaw();

  // Calibrate flex sensors, this is to get more accurate max and min values for the sensors reducing effects of variability
  tmbFing.autoCalibrate();
  indFing.autoCalibrate();
  midFing.autoCalibrate();
  rngFing.autoCalibrate();
  pnkFing.autoCalibrate();

  // Scale the raw values to a range of 0-100
  tmbFing.adjustScale(0, 100);
  indFing.adjustScale(0, 100);
  midFing.adjustScale(0, 100);
  rngFing.adjustScale(0, 100);
  pnkFing.adjustScale(0, 100);

  std::vector<int> flexion = {
    tmbFing.flexCheck(),
    indFing.flexCheck(),
    midFing.flexCheck(),
    rngFing.flexCheck(),
    pnkFing.flexCheck()
  };

  handMPU.getAcceleration(&handAx, &handAy, &handAz);
  foreMPU.getAcceleration(&foreAx, &foreAy, &foreAz);

  // Calculate roll and pitch for hand
  handRoll = atan2(handAy, handAz) * 180 / PI;
  handPitch = atan2(-handAx, sqrt(handAy * handAy + handAz * handAz)) * 180 / PI;

  // Calculate roll and pitch for forearm
  foreRoll = atan2(foreAy, foreAz) * 180 / PI;
  forePitch = atan2(-foreAx, sqrt(foreAy * foreAy + foreAz * foreAz)) * 180 / PI;

  Serial.print("Hand Roll: ");
  Serial.print(handRoll);
  Serial.print(" Hand Pitch: ");
  Serial.println(handPitch);

  Serial.print("Forearm Roll: ");
  Serial.print(foreRoll);
  Serial.print(" Forearm Pitch: ");
  Serial.println(forePitch);

  currGest.setGesture("current gesture", flexion);
  currGest.printFingerStates();

// Print the current gesture, this command should be transferred to Gesture.h
bool matchFound = false;
for (int i = 0; i < 6; i++) {
  if (currGest.isThis(GestureSet[i])) {
    Serial.print("Matched Gesture: ");
    Serial.println(GestureSet[i].getName().c_str());
    matchFound = true;
    break;
  }
}
if (!matchFound) {
  Serial.println("No Match");
}

  Serial.println("");
  delay(100);

/*
      Serial.println("Spinning...");
    //  Spin Executor
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    //  Delay to control publishing rate
    delay(10); */
}