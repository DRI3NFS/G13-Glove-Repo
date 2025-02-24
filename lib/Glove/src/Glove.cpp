//Glove Version of Glove Header File
//Includes all functions except Gesture Bank

#include "Glove.h"
#include <Arduino.h>

//constructor
FlexSensor::FlexSensor(int pin){ //declare what adc pin it is connected on esp32
    this->pin = pin;
};

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
}

int FlexSensor::getRaw(){
    return rawValue;
}

int FlexSensor::getScaled(){
    return scaledVal;
}

//this requires actual calibration depending on user hand flex ranges
uint8_t FlexSensor::flexCheck(){
    if(scaledVal < 40){
        return EXTD;  //this is extended
    }
    else if(scaledVal >= 40){  
        return FLEX;  //this is flexed state
    }
    else{
        return 0;     //this is an error state
    }
}

void FlexSensor::autoCalibrate() {
    if (rawValue > max) max = rawValue;
    if (rawValue < min) min = rawValue;
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
void tiltSensor::setAccelValues(int16_t MPUax, int16_t MPUay, int16_t MPUaz){
    ax = MPUax;
    ay = MPUay;
    az = MPUaz;

    //automated the conversion to pitch and roll which i assume are more useful values
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / M_PI;
    roll = atan2(ay, az) * 180 / M_PI;
}

//these values are assigned for right hand orientations with your arm pointing forward
//all of these values purely depend on my own arm's range of motion and are arbitrary
//thumb up, thumb down, palm down, palm up, point down, point up
uint8_t tiltSensor::getOrientation(){
    if(roll <= -45 && roll >= -135 && pitch >= -45 && pitch <= 45){
        orientation = THUMB_UP;
        return orientation;
    }

    else if(roll <= 45 && roll >= -45 && pitch >= -45 && pitch <= 45){
        orientation = PALM_DOWN;
        return orientation;
    }

    else if(roll <= 135 && roll >= 45 && pitch >= -45 && pitch <= 45){
        orientation = THUMB_UP;
        return orientation;
    }

    else if( ((roll <= -135 && roll >=-180)||(roll <= 180 && roll >= 135)) && pitch >= -45 && pitch <= 45){
        orientation = PALM_UP;
        return orientation;
    }

    if(pitch <= -45){
        orientation = FINGER_DOWN;
        return orientation;
    }

    else if(pitch >= 45){
        orientation = FINGER_UP;
        return orientation;
    }

    else{
        orientation = NULL;
        return orientation;
    };
}
