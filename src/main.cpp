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

MPU6050 mpuHand(0x68);
int16_t handax, handay, handaz;
int16_t handgx, handgy, handgz;
tilt handOrient;

MPU6050 mpuWrst(0x69); //usable rergister for AD0 high or second MPU6050
int16_t wrstax, wrstay, wrstaz;
int16_t wrstgx, wrstgy, wrstgz;

FlexSensor tmbFing;
FlexSensor indFing;
FlexSensor midFing;
FlexSensor rngFing;
FlexSensor pnkFing;

gesture currGest;
gesture GestureSet[6];

float roll, pitch;

void setup(void) {
  Serial.begin(115200);
  Wire.begin();

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  mpuHand.initialize();
  
  if (mpuHand.testConnection() == false) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  else{
    Serial.println("MPU6050 Found!");

  }

  // Set up flex sensors and MPU6050
  tmbFing.pin = 36;
  indFing.pin = 39;
  midFing.pin = 34;
  rngFing.pin = 35;
  pnkFing.pin = 32;
/* 
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ); */

  // Preset Gestures for now
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

  // Print the current gesture
  Serial.println("Current Gesture:");
  for (int i = 0; i < 5; i++) {
    Serial.print(flexion[i]);
    Serial.print(" ");
  }
  Serial.println("");

  // Check if the current gesture matches any of the preset gestures
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

  roll = atan2(handay, handaz) * 180/PI;
  pitch = atan2(-handax, sqrt(handay * handay + handaz * handaz)) * 180/PI;

/*   // Print the accelerometer data
  Serial.print("roll: ");
  Serial.println(roll, 1);
  Serial.print("pitch: ");
  Serial.println(pitch, 1);
  */

  std::string handDir = handOrient.classify(roll, pitch);
  Serial.print("hand is oriented ");
  Serial.println(handDir.c_str());

  Serial.println("");
  delay(100);
}