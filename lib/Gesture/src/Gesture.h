#ifndef Gesture.h
#define Gesture.h

#include <vector>
#include <string>
#include <iostream>
#include "Adafruit_Sensor.h"

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

class tilt { //this class is gonna be removed as it is used mostly for testing the MPU6050s
//tilt class must include data from two MPU6050s: forearm and hand, most likely the commands to be included under this class are relative values between the two modules
//potentially, a new class will be made which uses the commands of tilt
//the following are the key motions of the wrist: flexion, extension, radial deviation, ulnar deviation, pronation and supination
//the goal of this class is to classify what type of movements occured on the wrist using roll and pitch values from two MPU6050s
public:
    float pitch;
    float roll;

    std::string orientation = ""; //may have to switch to different data type
    // range of motion of wrist will be relative values between the two roll and pitch values
    // a real time capture of the MPU6050 value may be required but I believe this will be called on main.cpp instead

    std::string classify(float roll, float pitch){
        //new command would involve degree of bending between the wrist and the hand
        //so something like pitchWrist - pitchHand <= an arbitrary number will be used for the argument instead
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
    //this includes a relative max and min between the angles of the palm and wrist
};

//Wrist class

class WristSensor{
public:
    //definite values for forearm and hand MPU6050s
};

//Gesture class
class gesture {
public:
    void setGesture(std::string name, std::vector<int> fingerStates) {
        this->name = name;                      //this may potentially be moved to base computer to save size
        this->fingerStates = fingerStates;      //this is most likely the value being sent to base computer
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
    //these following two classes most likely will be replaced by values which indicate the relative angles of the wrist (i.e. the degree of bending)
    //the following are the key motions of the wrist: flexion, extension, radial deviation, ulnar deviation, pronation and supination
    tilt handTilt;
    tilt wristTilt;
};

#endif