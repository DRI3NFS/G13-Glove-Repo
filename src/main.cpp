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

MPU6050 mpuHand(MPU6050_ADDRESS_AD0_LOW);  //hand accelerometer
int16_t handax, handay, handaz;
int16_t handgx, handgy, handgz;
tilt handOrient;

MPU6050 mpuFore(MPU6050_ADDRESS_AD0_HIGH); //usable register for AD0 high or second MPU6050
int16_t foreax, foreay, foreaz;
int16_t foregx, foregy, foregz;
tilt foreOrient;

FlexSensor tmbFing;
FlexSensor indFing;
FlexSensor midFing;
FlexSensor rngFing;
FlexSensor pnkFing;

gesture currGest;
gesture GestureSet[6];

float handRoll, handPitch;
float foreRoll, forePitch;

void setup(void) {
  Serial.begin(115200);
  Wire.begin();

  // initialize both
  mpuHand.initialize();
  mpuFore.initialize();

  // Set up flex sensors and MPU6050
  tmbFing.pin = 36;
  indFing.pin = 39;
  midFing.pin = 34;
  rngFing.pin = 35;
  pnkFing.pin = 32;

  // Preset Gestures for now, signature will be replaced with non-string in Gesture.h
  GestureSet[0].setGesture("agree", {1, -1, -1, -1, -1});
  GestureSet[1].setGesture("point", {-1, 1, -1, -1, -1});
  GestureSet[2].setGesture("promise", {-1, -1, -1, -1, 1});
  GestureSet[3].setGesture("peace", {-1, 1, 1, -1, -1});
  GestureSet[4].setGesture("okay", {0, 0, 1, 1, 1});
  GestureSet[5].setGesture("idle", {0, 0, 0, 0, 0});
}

void loop() {
  // Read raw flex sensor data from ADCs
  tmbFing.rawValue = analogRead(tmbFing.pin);
  indFing.rawValue = analogRead(indFing.pin);
  midFing.rawValue = analogRead(midFing.pin);
  rngFing.rawValue = analogRead(rngFing.pin);
  pnkFing.rawValue = analogRead(pnkFing.pin);

  // Calibrate flex sensors, this is to get more accurate max and min values for the sensors reducing effects of variability
  tmbFing.autoCalibrate();
  indFing.autoCalibrate();
  midFing.autoCalibrate();
  rngFing.autoCalibrate();
  pnkFing.autoCalibrate();

  // Scale the raw values to a range of 0-100
  std::vector<int> flexion = {
    tmbFing.flexCheck(tmbFing.scaleValue(0, 100)),
    indFing.flexCheck(indFing.scaleValue(0, 100)),
    midFing.flexCheck(midFing.scaleValue(0, 100)),
    rngFing.flexCheck(rngFing.scaleValue(0, 100)),
    pnkFing.flexCheck(pnkFing.scaleValue(0, 100))
  };
  currGest.setGesture("current", flexion);

  // Print the current gesture, this command should be transferred to Gesture.h
  Serial.println("Current Gesture:");
  for (int i = 0; i < 5; i++) {
    Serial.print(flexion[i]);
    Serial.print(" ");
  }
  Serial.println("");

  // Check if the current gesture matches any of the preset gestures, this command should be transferred to Gesture.h under class Gesture{}
  for (int i = 0; i < 5; i++) {
    if (currGest.isThis(GestureSet[i].getFingerStates()) != "") {
      Serial.println(GestureSet[i].getName().c_str());
    }
    else {
      Serial.println("No Gesture");
      i = 5;
    }
  }

  // hand accelerometer and gyro acquisition
  mpuHand.getMotion6(&handax, &handay, &handaz, &handgx, &handgy, &handgz);

  /* Following Lines are for testing tilt detection */
  handRoll = atan2(handay, handaz) * 180/PI;
  handPitch = atan2(-handax, sqrt(handay * handay + handaz * handaz)) * 180/PI;

  // Print the accelerometer data
  Serial.printf("Tilt Values from Hand Sensor:\r\n");
  Serial.print("roll: ");
  Serial.println(handRoll,1);
  Serial.print("pitch: ");
  Serial.println(handPitch, 1);

  // print command must be moved to Gesture.h under class tilt{}
  std::string handDir = handOrient.classify(handRoll, handPitch);
  Serial.print("hand is oriented ");
  Serial.println(handDir.c_str());

  mpuFore.getMotion6(&foreax, &foreay, &foreaz, &foregx, &foregy, &foregz);
  foreRoll = atan2(foreay, foreaz) * 180/PI;
  forePitch = atan2(-foreax, sqrt(foreay * foreay + foreaz * foreaz)) * 180/PI;

  // Print the accelerometer data
  Serial.printf("Tilt Values from Forearm Sensor:\r\n");
  Serial.print("roll: ");
  Serial.println(foreRoll,1);
  Serial.print("pitch: ");
  Serial.println(forePitch, 1);
  /* End of tilt detection Test Code*/

  // print command must be moved to Gesture.h under class tilt{}
  std::string foreDir = foreOrient.classify(foreRoll, forePitch);
  Serial.print("forearm is oriented ");
  Serial.println(foreDir.c_str());

  Serial.println("");
  delay(100);
}