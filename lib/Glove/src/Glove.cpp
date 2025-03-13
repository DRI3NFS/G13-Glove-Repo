//Glove Version of Glove Header File
//Includes all functions except Gesture Bank

#include "Glove.h"
#include <Arduino.h>
#include <MPU6050.h>

//constructor
FlexSensor::FlexSensor(int pin) : pin(pin), rawValue(0), scaledVal(0), max(0), min(2000), flexState(40) { //declare what adc pin it is connected on esp32
}

//update raw value
void FlexSensor::updateRaw(){   //must be updated to automatically read from pin within header rather from main.cpp
    rawValue = analogRead(pin);
    this->autoCalibrate();
}

//scales sensor range to whatever user specifies it to be
void FlexSensor::adjustScale(int newMin, int newMax){
    scaledMax = newMax;
    scaledMin = newMin;

    scale(&scaledVal, rawValue);

    Serial.printf("%d",scaledVal);
    Serial.println("");
}

int FlexSensor::getRaw(){
    return rawValue;
}

int FlexSensor::getScaled(){
    return scaledVal;
}

//this requires actual calibration depending on user hand flex ranges
uint8_t FlexSensor::flexCheck(){
    if(scaledVal < flexState){
        return EXTD;  //this is extended
    }
    else{
        return FLEX;  //this is flexed state
    }
}

//this needs to be modified to include a dampening logic that prevents outlier values from replacing max and min
void FlexSensor::autoCalibrate() {
    //Update scale
    if (rawValue > max) {
        max = rawValue;
        return;
    }
    if (rawValue < min) {
        min = rawValue;
        return;
    }
}

void FlexSensor::scale(int* valueToScale, int input){
    int range = max - min;
    if (range == 0) range = 1;  //to eliminate divide by 0 error
    int scaledRange = scaledMax - scaledMin;

    *valueToScale = (((input - min) * (scaledRange)) / (range)) + scaledMin;
}

//the following are the key motions of the wrist: flexion, extension, radial deviation, ulnar deviation, pronation and supination
//radial and ulnar deviation would require either a magnetometer to detect so will be dropped for now
//Captures current orientation of hand sensor by calculating roll and pitch. Does not capture direction.
tiltSensor::tiltSensor(MPU6050* mpuInput) : ax(0), ay(0), az(0), pitch(0), roll(0) {
  mpu = mpuInput;
}

void tiltSensor::setAccelValues(){
    mpu->getAcceleration(&ax, &ay, &az);

    float accelx = ax/16384.0f;
    float accely = ay/16384.0f;
    float accelz = az/16384.0f;

    Serial.printf("%.2f, %.2f, %.2f,",accelx,accely,accelz);
    Serial.println("");

    //automated the conversion to pitch and roll which i assume are more useful values
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / M_PI;
    roll = atan2(ay, az) * 180 / M_PI;

    Serial.printf("roll = %f",roll);
    Serial.println("");
    Serial.printf("pitch = %f",pitch);
    Serial.println("");
}

//all of these values purely depend on my own arm's range of motion and are arbitrary
//thumb up, thumb down, palm down, palm up, point down, point up
uint8_t tiltSensor::getOrientation(){
    //Check these two first, everything else is outside these ranges
    if (pitch > 45) return FINGER_UP;   // Finger Up
    if (pitch < -45) return FINGER_DOWN; // Finger Down

    if (roll >= -135 && roll <= -45) return THUMB_UP;  // Thumb Down
    if (roll >= -45 && roll <= 45) return PALM_DOWN;    // Palm Down
    if (roll >= 45 && roll <= 135) return THUMB_DOWN;     // Thumb Up
    if ((roll >= 135 && roll <= 180) || (roll >= -180 && roll <= -135)) return PALM_UP; // Palm Up

    return NULL;
}
