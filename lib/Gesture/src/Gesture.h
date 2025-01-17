#ifndef Gesture.h
#define Gesture.h

#include <vector>
#include <string>
#include <iostream>
#include <Arduino.h>

class FlexSensor {
public:
    //constructor
    FlexSensor(int pin);

    //update raw value
    void updateRaw(){   //must be updated to automatically read from pin within header rather from main.cpp
        rawValue = analogRead(pin);
    }

    //manual calibration must also be implemented for when a user has to manually calibrate sensors
    void calibrateIdleMin(int idledNum){
        scale(&scaledIdleMin, idledNum);
    }

    void calibrateIdleMax(int idledNum){
        scale(&scaledIdleMax, idledNum);
    }

    //ideally call this value throughout process to automatically update flex sensor range
    void autoCalibrate() {
        if (rawValue > max) max = rawValue;
        if (rawValue < min) min = rawValue;
    }

    //scales sensor range to whatever user specifies it to be
    void adjustScale(int newMin, int newMax){
        scaledMax = newMax;
        scaledMin = newMin;

        scale(&scaledVal, rawValue);
    }

    int getRaw(){
        return rawValue;
    }

    int getScaled(){
        return scaledVal;
    }

    //this requires actual calibration depending on user hand flex ranges
    int flexCheck(){
        if(scaledVal > 30 && scaledVal <60){
            return 0;                           //this is idle state
        }
        else if(scaledVal < 30){  
            return -1;                          //this is flexed state
        }
        else{
            return 1;                           //this is extended state
        }
    }

    //add a print function
    void printRaw(){
        Serial.printf("Flex at Pin %d has the value %d.", pin, rawValue);
    }

    void printScaled(){
        Serial.printf("Flex at Pin %d has the scaled value from %d to %d of %d.",
        pin,
        scaledMin,
        scaledMax,
        scaledVal);
    }

private:
    int rawValue;
    int pin;

    int max = 0;
    int min = 10000;

    int scaledVal;
    int scaledMax;
    int scaledMin;

    int scaledIdleMin;    //raw sensor value indicating idle hand state post full flexion
    int scaledIdleMax;    //raw sensor value indicating idle hand state post full extension

    void scale(int* valueToScale, int input){
        int range = max - min;
        if (range == 0) range = 1;  //to eliminate divide by 0 error
        int scaledRange = scaledMax - scaledMin;

        *valueToScale = (((input - min) * (scaledRange)) / (range)) + scaledMin;
    }
};

FlexSensor::FlexSensor(int pin){ //declare what adc pin it is connected on esp32
    this->pin = pin;
};

class tiltSensor {
//the following are the key motions of the wrist: flexion, extension, radial deviation, ulnar deviation, pronation and supination
//the goal of this class is to classify what type of movements occured on the wrist using roll and pitch values from two MPU6050s
public:
    std::string orientation;

    void setAccelValues(int16_t MPUax, int16_t MPUay, int16_t MPUaz){
        ax = MPUax;
        ay = MPUay;
        az = MPUaz;

        pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180 / PI;
        roll = atan2(-ay, az) * 180 / PI;
    }

    float getPitch(){
        return pitch;
    }

    float getRoll(){
        return roll;
    }

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

    //add a print function
    void printOrientation(){
        Serial.print("Orientation is ");
        Serial.println(orientation.c_str());
    }

    void printPitch(){
        Serial.printf("Pitch = %d\r\n", pitch);
    }

    void printRoll(){
        Serial.printf("Roll = %d\r\n", roll);
    }

    //will add code to calibrate orientations depending on user hand
    //this includes a relative max and min between the angles of the palm and wrist

private:
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    float pitch;
    float roll;
};

//Gesture class
class gesture {
public:
    gesture() {
        this->name = "";
        this->fingerStates = {0, 0, 0, 0, 0};
    }

    void setGesture(std::string name, std::vector<int> fingerNum) {
        this->name = name;                      //this is the data that will be sent to the computer
        this->fingerStates = fingerNum;      //this is most likely the value being sent to base computer
    }

    std::vector<int> getFingerStates() {
        return this->fingerStates;
    }

    std::string getName(){
        return name;
    }

    void printFingerStates(){
        Serial.print("Finger states for ");
        Serial.print(this->name.c_str());
        Serial.print(" is: ");
        for (int i = 0; i < fingerStates.size(); i++){
            Serial.printf("%d ",fingerStates[i]);
        }
        Serial.println("");
    }

    bool isThis(gesture Gesture4Comp) {
        if (fingerStates == Gesture4Comp.getFingerStates()) {
            return true;
        }
        else {
            return false;
        }
    }

/*
THERE SHOULD ALSO BE A SECTION HERE WHICH ACCOUNTS FOR THE GYROSCOPE DATAS WHICH MAY BE A CLASS OF ITS OWN
*/
private:
    //these following two classes most likely will be replaced by values which indicate the relative angles of the wrist (i.e. the degree of bending)
    //the following are the key motions of the wrist: flexion, extension, radial deviation, ulnar deviation, pronation and supination
/*     tiltSensor handTilt;
    tiltSensor wristTilt; */

    std::string name = ""; //IF YOU WANT TO CHANGE THE DATA TYPE OF THE GESTURE, CHANGE IT HERE
    std::vector<int> fingerStates = {0, 0, 0, 0, 0};

};

#endif