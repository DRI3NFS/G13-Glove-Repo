#ifndef Gesture.h
#define Gesture.h

#include <vector>
#include <string>
#include <iostream>
#include "Adafruit_Sensor.h"

//Gesture class
class gesture {
public:
void setGesture(std::string name, std::vector<int> fingerStates) {
        this->name = name;
        this->fingerStates = fingerStates;
    }

    std::vector<int> getFingerStates() {
        return fingerStates;
    }

    std::string getName() {
        return name;
    }

    std::string isThis(std::vector<int> flexion) {
        if (flexion == fingerStates) {
            return name;
        }
        else {
            return "";
        }
    }

/*
THERE SHOULD ALSO BE A SECTION HERE WHICH ACCOUNTS FOR THE GYROSCOPE DATAS WHICH MAY BE A CLASS OF ITS OWN
*/
private:
    std::string name = ""; //IF YOU WANT TO CHANGE THE DATA TYPE OF THE GESTURE, CHANGE IT HERE
    std::vector<int> fingerStates = {0, 0, 0, 0, 0};
    tilt handTilt;
    tilt wristTilt;
};

class FlexSensor {
public:
    int rawValue = 0;
    int pin;
    //manual calibration must also be implemented for when a user has to manually calibrate sensors
    void calibrateIdleMin(int idledNum){
        //this value must be adjusted within the scaled range
    }

    void calibrateIdleMax(){

    }

    //redundant function used for testing
    void autoCalibrate() {
        if (rawValue > max) max = rawValue;
        if (rawValue < min) min = rawValue;
    }

    //scales sensor range to whatever user specifies it to be
    void adjustScale(int newMax, int newMin){

    }

    //scales range of values for flex sensor between newMin and newMax (ex: scaling a value to 0-100 will adjust that value to be within that range depending on assigned raw max and min)
    int scaleValue(int newMin, int newMax) {
        int range = max - min;
        if (range == 0) range = 1;  //to eliminate divide by 0 error
        int newRange = newMax - newMin;

        scaledValue = (((rawValue - min) * newRange) / range) + newMin;

        return scaledValue;
    }

    //this requires actual calibration depending on user hand flex ranges
    int flexCheck(int scaledVal){
        if(scaledVal > 30 && scaledVal < 70){   //30 and 70 are arbitrary values, should be calibrated
            return 0;                           //this is idle state
        }
        else if(scaledVal < 30){                //30 is arbitrary, should be calibrated
            return -1;                          //this is flexed state
        }
        else{
            return 1;                           //this is extended state
        }
    }

private:
    int max = 0;
    int min = 10000;

    int scaledValue;
    int scaledMax;
    int scaledMin;

    int idleMin;
    int idleMax;
};

class tilt {
public:
    float roll;
    float pitch;
    std::string orientation = "";

    std::string classify(float roll, float pitch){
        if(pitch <= -80){
            orientation = "down";
        }
        else if(pitch >= 80){
            orientation = "up";
        }
        else if(pitch >= -10 && pitch <= 10){
            orientation = "forward";
        }
        else{};

        return orientation;
    }

    //will add code to calibrate orientations depending on user hand
};

#endif